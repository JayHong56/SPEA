module voxel_coor_pipe #(
    parameter integer BRAM_DATA_WIDTH = 504,  // 72* 7
    parameter integer BRAM_ADDR_WIDTH = 10,  // 256 pillar * 3brams
    parameter integer BRAM_ADDR_WIDTH_PFE = 6,  // 32, PFE_CACHE 深度
    parameter integer PREFILTER_LANES = 2,
    parameter integer DRAM_DATA_WIDTH = 128 * PREFILTER_LANES,
    parameter integer DRAM_ADDR_WIDTH = 18,
    parameter integer HASH_ADDR_WIDTH = 8,  // log2(HASH_TABLE_SIZE)
    parameter [15:0] THRESHOLD_CLOSE = 16'h0100,  // 1.00m
    parameter [20-1:0] THRESHOLD_BOUDARY_X_LOW = 20'h0000,  // 0.00m
    parameter [20-1:0] THRESHOLD_BOUDARY_X_HIGH = 20'h451ec,  // 69.12m
    parameter [20-1:0] THRESHOLD_BOUDARY_Y = 20'h27ae1,  // +/- 39.68m
    parameter [15:0] THRESHOLD_BOUDARY_Z_LOW = 16'h3000,  // abs(-3)m
    parameter [15:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h1000,  // 1m
    parameter integer PRE_LAT = 0,  // 预处理固定延迟（拍数占位）
    parameter [15:0] LIFE_CYCLE = 16'd100,
    parameter integer VN_WIDTH = 5,  // 点编号宽度（hash 表输出 1..20）
    parameter [23:0] MAX_VOXEL_NUM = 24'd20,  // 每体素最多 20 点
    parameter integer BYTE_WIDTH = 9  // 1 Byte = 8 bit

) (
    input wire clk,
    input wire rst_n,

    input wire frame_end,  // 外部传入：当前帧点云结束脉冲 (1-cycle pulse)
    output wire flush_done,  // 输出：哈希表剩余有效点已全部输出完毕
    output wire hash_stall,
    // AXI-Stream in (from DRAM FIFO), 128-bit: {x_float32, y_float32, z_float32, intensity_float32}
    input wire [DRAM_DATA_WIDTH-1:0] s_axis_dram_data,
    input wire [DRAM_DATA_WIDTH/8-1:0] s_axis_dram_keep,
    input wire s_axis_dram_last,
    input wire s_axis_dram_valid,
    output wire s_axis_dram_ready,
    // BRAM Native Interface
    output reg bram_voxelpoint_wr,
    output reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen,
    output reg [BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr,
    output reg [BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata,
    // BRAM_VOXELPOINT_A
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    // BRAM_VOXELPOINT_B
    output wire bram_expire_wr_b,
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    // AXI-Stream out (expire voxel coordinates)
    output wire m_axis_expire_tvalid,
    input wire m_axis_expire_tready,
    output wire [22+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata
);

    localparam integer PT_WIDTH_I_XY = 8;
    localparam integer PT_WIDTH_F_XY = 12;
    localparam integer PT_WIDTH_I_Z = 4;
    localparam integer PT_WIDTH_F_Z = 12;
    localparam integer PT_WIDTH_I_IS = 1;
    localparam integer PT_WIDTH_F_IS = 15;

    localparam integer PT_WIDTH_XY = PT_WIDTH_I_XY + PT_WIDTH_F_XY;  // 20
    localparam integer PT_WIDTH = 16;  // 16
    localparam integer PT_WIDTH_PER = 2 * PT_WIDTH_XY + 2 * PT_WIDTH;  // 72 bit 每点
    localparam integer PT_WIDTH_FLOAT32 = 32;
    localparam integer VOXEL_WIDTH = 11;  // NOTE 根据伪图尺寸设定

    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  // 504/72 = 7
    localparam integer EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  // ceil(20/7)=3

    // BRAM clk/rst
    assign bram_voxelpoint_clk = clk;
    assign bram_voxelpoint_rst = ~rst_n;

    wire hash_req_ready;
    wire hash_req_valid;
    wire hash_fire = hash_req_valid && hash_req_ready;

    // ============================================================
    // 多路 prefilter + 单路 voxel/hash 前端
    // ============================================================
    localparam integer RAW_PT_WIDTH = 4 * PT_WIDTH_FLOAT32;  // 128 bit

    // 候选 FIFO 保存：
    // {x_fix16, y_fix16, z_fix16, intensity_float32}
    // intensity 先不转 fixed，后面单路转，节省资源
    localparam integer CAND_WIDTH = 4 * PT_WIDTH_XY + PT_WIDTH + PT_WIDTH_FLOAT32;
    // 建议用 64 或 128，必须是 2 的幂
    localparam integer CAND_FIFO_DEPTH = 32;
    localparam integer CAND_FIFO_PTR_W = $clog2(CAND_FIFO_DEPTH);
    localparam integer CAND_FIFO_CNT_W = $clog2(CAND_FIFO_DEPTH + 1);

`ifndef SYNTHESIS
    initial begin
        if (DRAM_DATA_WIDTH != RAW_PT_WIDTH * PREFILTER_LANES) begin
            $display("[ERROR][voxel_coor_pipe] DRAM_DATA_WIDTH must equal 128 * PREFILTER_LANES.");
            $stop;
        end

        if ((CAND_FIFO_DEPTH & (CAND_FIFO_DEPTH - 1)) != 0) begin
            $display("[ERROR][voxel_coor_pipe] CAND_FIFO_DEPTH must be power of 2.");
            $stop;
        end
    end
`endif

    // ------------------------------------------------------------
    // AXIS 输入握手
    // ------------------------------------------------------------
    wire s_fire;
    assign s_fire = s_axis_dram_valid && s_axis_dram_ready;

    // ------------------------------------------------------------
    // 每 lane 拆 raw point
    // 默认 lane0 在 s_axis_dram_data[127:0]
    // lane1 在 [255:128]
    // lane2 在 [383:256]
    // ------------------------------------------------------------
    wire [RAW_PT_WIDTH-1:0] lane_raw[0:PREFILTER_LANES-1];

    wire [PT_WIDTH_FLOAT32-1:0] lane_x_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_y_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_z_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_i_float32[0:PREFILTER_LANES-1];

    wire signed [PT_WIDTH_XY-1:0] lane_x_fix20_data[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH_XY-1:0] lane_y_fix20_data[0:PREFILTER_LANES-1];
    // voxel: 只用于计算 voxel_x / voxel_y，使用 ceil(+inf)
    wire signed [PT_WIDTH_XY-1:0] lane_x_fix20_voxel[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH_XY-1:0] lane_y_fix20_voxel[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH-1:0] lane_z_fix16[0:PREFILTER_LANES-1];

    wire [PT_WIDTH_XY-1:0] lane_abs_x[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_XY-1:0] lane_abs_y[0:PREFILTER_LANES-1];
    wire [PT_WIDTH-1:0] lane_abs_z[0:PREFILTER_LANES-1];

    wire lane_keep[0:PREFILTER_LANES-1];
    wire lane_present[0:PREFILTER_LANES-1];

    wire [CAND_WIDTH-1:0] lane_cand_data[0:PREFILTER_LANES-1];

    genvar gi;
    generate
        for (gi = 0; gi < PREFILTER_LANES; gi = gi + 1) begin : GEN_PREFILTER_LANE
            assign lane_raw[gi] = s_axis_dram_data[gi*RAW_PT_WIDTH+:RAW_PT_WIDTH];

            assign lane_x_float32[gi] = lane_raw[gi][127:96];
            assign lane_y_float32[gi] = lane_raw[gi][95:64];
            assign lane_z_float32[gi] = lane_raw[gi][63:32];
            assign lane_i_float32[gi] = lane_raw[gi][31:0];

            // 当前 lane 是否有完整 16-byte point
            assign lane_present[gi] = &s_axis_dram_keep[gi*(RAW_PT_WIDTH/8)+:(RAW_PT_WIDTH/8)];

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_x_data (
                .float_in        (lane_x_float32[gi]),
                .true_fixed_value(lane_x_fix20_data[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_x_voxel (
                .float_in        (lane_x_float32[gi]),
                .true_fixed_value(lane_x_fix20_voxel[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_y_data (
                .float_in        (lane_y_float32[gi]),
                .true_fixed_value(lane_y_fix20_data[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_y_voxel (
                .float_in        (lane_y_float32[gi]),
                .true_fixed_value(lane_y_fix20_voxel[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_F_Z + PT_WIDTH_I_Z),
                .FIXED_FRACTIONAL(PT_WIDTH_F_Z)
            ) u_float2fxp_z (
                .float_in        (lane_z_float32[gi]),
                .true_fixed_value(lane_z_fix16[gi]),
                .fixed_sign      ()
            );

            assign lane_abs_x[gi] = lane_x_fix20_data[gi][PT_WIDTH_XY-1] ? (~lane_x_fix20_data[gi] + 1'b1) : lane_x_fix20_data[gi];
            assign lane_abs_y[gi] = lane_y_fix20_data[gi][PT_WIDTH_XY-1] ? (~lane_y_fix20_data[gi] + 1'b1) : lane_y_fix20_data[gi];
            assign lane_abs_z[gi] = lane_z_fix16[gi][PT_WIDTH-1] ? (~lane_z_fix16[gi] + 1'b1) : lane_z_fix16[gi];

            // keep_point 判断：这里只做 x/y/z 边界，不做 voxel 坐标计算
            assign lane_keep[gi] =
                lane_present[gi] &&
                (lane_raw[gi] != {RAW_PT_WIDTH{1'b0}}) &&
                !(^lane_x_fix20_data[gi] === 1'bx) &&
                !lane_x_fix20_data[gi][PT_WIDTH_XY-1] &&
                !(lane_abs_x[gi] > THRESHOLD_BOUDARY_X_HIGH) &&
                !(lane_abs_y[gi] > THRESHOLD_BOUDARY_Y) &&
                !(lane_z_fix16[gi][PT_WIDTH-1] ? 
                    (lane_abs_z[gi] > THRESHOLD_BOUDARY_Z_LOW) :
                    (lane_abs_z[gi] > THRESHOLD_BOUDARY_Z_HIGH));

            assign lane_cand_data[gi] = {
                lane_x_fix20_data[gi],
                lane_y_fix20_data[gi],
                lane_x_fix20_voxel[gi],
                lane_y_fix20_voxel[gi],
                lane_z_fix16[gi],
                lane_i_float32[gi]
            };
        end
    endgenerate


    // ============================================================
    // Candidate FIFO：每拍最多写入 PREFILTER_LANES 个有效点
    // 每拍最多读出 1 个有效点给单路 voxel 计算
    // ============================================================
    (* ram_style = "distributed" *)
    reg [CAND_WIDTH-1:0] cand_fifo_mem[0:CAND_FIFO_DEPTH-1];

    reg [CAND_FIFO_PTR_W-1:0] cand_fifo_wr_ptr;
    reg [CAND_FIFO_PTR_W-1:0] cand_fifo_rd_ptr;
    reg [CAND_FIFO_CNT_W-1:0] cand_fifo_level;

    wire cand_fifo_empty;
    wire cand_fifo_full;

    assign cand_fifo_empty = (cand_fifo_level == {CAND_FIFO_CNT_W{1'b0}});
    assign cand_fifo_full  = (cand_fifo_level == CAND_FIFO_DEPTH[CAND_FIFO_CNT_W-1:0]);

    wire [CAND_WIDTH-1:0] cand_fifo_dout;
    assign cand_fifo_dout = cand_fifo_mem[cand_fifo_rd_ptr];

    function [CAND_FIFO_PTR_W-1:0] cand_ptr_add;
        input [CAND_FIFO_PTR_W-1:0] ptr;
        input [CAND_FIFO_CNT_W-1:0] inc;
        reg [CAND_FIFO_CNT_W:0] tmp;
        begin
            tmp = ptr + inc;
            cand_ptr_add = tmp[CAND_FIFO_PTR_W-1:0];
        end
    endfunction

    // 当前 beat 中每个 lane 是否 push
    wire lane_push[0:PREFILTER_LANES-1];

    genvar gp;
    generate
        for (gp = 0; gp < PREFILTER_LANES; gp = gp + 1) begin : GEN_LANE_PUSH
            assign lane_push[gp] = s_fire && lane_keep[gp];
        end
    endgenerate

    // 计算每个 lane 的写入偏移，以及本 beat 总 push 数
    reg [CAND_FIFO_CNT_W-1:0] lane_prefix[0:PREFILTER_LANES-1];
    reg [CAND_FIFO_CNT_W-1:0] cand_push_count;

    integer pi;
    integer pj;
    always @(*) begin
        cand_push_count = {CAND_FIFO_CNT_W{1'b0}};

        // 先统计本拍总共有多少个有效 lane
        for (pi = 0; pi < PREFILTER_LANES; pi = pi + 1) begin
            if (lane_push[pi]) begin
                cand_push_count = cand_push_count + 1'b1;
            end
        end

        // 计算每个 lane 在 FIFO 中的写入偏移
        // 高编号 lane 优先进入 FIFO：
        // PREFILTER_LANES=3 时，顺序为 lane2 -> lane1 -> lane0
        for (pi = 0; pi < PREFILTER_LANES; pi = pi + 1) begin
            lane_prefix[pi] = {CAND_FIFO_CNT_W{1'b0}};

            // 比当前 lane 编号更高、且有效的 lane，会排在它前面
            for (pj = pi + 1; pj < PREFILTER_LANES; pj = pj + 1) begin
                if (lane_push[pj]) begin
                    lane_prefix[pi] = lane_prefix[pi] + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // 单路 voxel 计算输入：从 candidate FIFO 读出的有效点
    // ============================================================
    wire signed [PT_WIDTH_XY-1:0] cand_x_fix20_data;
    wire signed [PT_WIDTH_XY-1:0] cand_y_fix20_data;
    wire signed [PT_WIDTH_XY-1:0] cand_x_fix20_voxel;
    wire signed [PT_WIDTH_XY-1:0] cand_y_fix20_voxel;
    wire signed [PT_WIDTH-1:0]    cand_z_fix16;
    wire [PT_WIDTH_FLOAT32-1:0]   cand_i_float32;

    assign cand_x_fix20_data  = cand_fifo_dout[CAND_WIDTH-1-:PT_WIDTH_XY];
    assign cand_y_fix20_data  = cand_fifo_dout[CAND_WIDTH-1-PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_x_fix20_voxel = cand_fifo_dout[CAND_WIDTH-1-2*PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_y_fix20_voxel = cand_fifo_dout[CAND_WIDTH-1-3*PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_z_fix16       = cand_fifo_dout[CAND_WIDTH-1-4*PT_WIDTH_XY-:PT_WIDTH];
    assign cand_i_float32     = cand_fifo_dout[PT_WIDTH_FLOAT32-1:0];

    // 单路 intensity float_to_fixed：只给有效点做，节省资源
    wire signed [PT_WIDTH-1:0] cand_intensity_fix16;

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_F_IS + PT_WIDTH_I_IS),
        .FIXED_FRACTIONAL(PT_WIDTH_F_IS)
    ) u_float2fxp_intensity_single (
        .float_in        (cand_i_float32),
        .true_fixed_value(cand_intensity_fix16),
        .fixed_sign      ()
    );

    // ------------------------------------------------------------
    // 单路 voxel_x / voxel_y 计算
    // ------------------------------------------------------------
    localparam integer SCALE_VOXEL_FACTOR = 20;
    localparam integer SCALE_VOXEL = 16;
    localparam signed [SCALE_VOXEL_FACTOR-1:0] MULT_FACTOR = 20'd409600;  // 6.666666 (scale = 2^16)
    localparam integer ROUND = 1;

    wire signed [PT_WIDTH_XY-1:0] bourdary_fix20_Y;
    localparam signed [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_Y_CEIL_FIX = 20'sh27ae2;  // 39.68m 但加了一个LSB
    assign bourdary_fix20_Y = THRESHOLD_BOUDARY_Y_CEIL_FIX;

    wire [PT_WIDTH_XY+SCALE_VOXEL_FACTOR-1:0] single_fxp_mul_x_out;
    wire [PT_WIDTH_XY-1:0]    single_fxp_add_y_out;
    wire [PT_WIDTH_XY+SCALE_VOXEL_FACTOR-1:0] single_fxp_mul_y_out;

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (SCALE_VOXEL_FACTOR),
        .WIFB (0),
        .WOI  (SCALE_VOXEL_FACTOR + PT_WIDTH_XY),
        .WOF  (0),
        .ROUND(ROUND)
    ) u_single_fxp_mul_voxel_x (
        .ina     (cand_x_fix20_voxel),
        .inb     (MULT_FACTOR),
        .out     (single_fxp_mul_x_out),
        .overflow()
    );

    fxp_add #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (PT_WIDTH_I_XY),
        .WIFB (PT_WIDTH_F_XY),
        .WOI  (PT_WIDTH_I_XY),
        .WOF  (PT_WIDTH_F_XY),
        .ROUND(ROUND)
    ) u_single_fxp_add_voxel_y (
        .ina     (cand_y_fix20_voxel),
        .inb     (bourdary_fix20_Y),
        .out     (single_fxp_add_y_out),
        .overflow()
    );

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (SCALE_VOXEL_FACTOR),
        .WIFB (0),
        .WOI  (SCALE_VOXEL_FACTOR + PT_WIDTH_XY),
        .WOF  (0),
        .ROUND(ROUND)
    ) u_single_fxp_mul_voxel_y (
        .ina     (single_fxp_add_y_out),
        .inb     (MULT_FACTOR),
        .out     (single_fxp_mul_y_out),
        .overflow()
    );

    wire [VOXEL_WIDTH-1:0] single_voxel_x;
    wire [VOXEL_WIDTH-1:0] single_voxel_y;

    assign single_voxel_x = single_fxp_mul_x_out[SCALE_VOXEL+VOXEL_WIDTH-1:SCALE_VOXEL];
    assign single_voxel_y = single_fxp_mul_y_out[SCALE_VOXEL+VOXEL_WIDTH-1:SCALE_VOXEL];

    wire [PT_WIDTH_PER-1:0] single_point_proc;
    assign single_point_proc = {cand_x_fix20_data, cand_y_fix20_data, cand_z_fix16, cand_intensity_fix16};


    // ============================================================
    // hash 输入 pipe1：一拍最多送 1 个有效点给 hash_table
    // ============================================================
    reg                    pipe1_valid;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_x;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_y;
    reg [PT_WIDTH_PER-1:0] pipe1_point_proc;

    assign hash_req_valid = pipe1_valid;

    wire [VOXEL_WIDTH-1:0] hash_key_x;
    wire [VOXEL_WIDTH-1:0] hash_key_y;

    assign hash_key_x = pipe1_voxel_x;
    assign hash_key_y = pipe1_voxel_y;

    // pipe1 为空，或者当前点已被 hash 接收，则可以装载下一个候选点
    wire pipe1_can_accept;
    assign pipe1_can_accept = !pipe1_valid || hash_fire;

    wire cand_fifo_pop;
    assign cand_fifo_pop = pipe1_can_accept && !cand_fifo_empty;

    // 输入 ready：必须保证 candidate FIFO 至少还有 PREFILTER_LANES 个空位
    // 同时考虑本拍可能 pop 出 1 个
    wire [CAND_FIFO_CNT_W:0] cand_fifo_space_after_pop;
    assign cand_fifo_space_after_pop = (CAND_FIFO_DEPTH - cand_fifo_level) + (cand_fifo_pop ? 1'b1 : 1'b0);

    localparam [CAND_FIFO_CNT_W:0] CAND_FIFO_DEPTH_W = CAND_FIFO_DEPTH;
    localparam [CAND_FIFO_CNT_W:0] PREFILTER_LANES_W = PREFILTER_LANES;

    wire [CAND_FIFO_CNT_W:0] cand_fifo_level_ext;
    wire [CAND_FIFO_CNT_W:0] cand_fifo_space_now;

    assign cand_fifo_level_ext = {1'b0, cand_fifo_level};
    assign cand_fifo_space_now = CAND_FIFO_DEPTH_W - cand_fifo_level_ext;

    assign s_axis_dram_ready   = (cand_fifo_space_now >= PREFILTER_LANES_W);
    // candidate FIFO 写读指针与 level 更新
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_fifo_wr_ptr <= {CAND_FIFO_PTR_W{1'b0}};
            cand_fifo_rd_ptr <= {CAND_FIFO_PTR_W{1'b0}};
            cand_fifo_level  <= {CAND_FIFO_CNT_W{1'b0}};
        end else begin
            // 多 lane 写入
            for (wi = 0; wi < PREFILTER_LANES; wi = wi + 1) begin
                if (lane_push[wi]) begin
                    cand_fifo_mem[cand_ptr_add(cand_fifo_wr_ptr, lane_prefix[wi])] <= lane_cand_data[wi];
                end
            end

            if (cand_push_count != {CAND_FIFO_CNT_W{1'b0}}) begin
                cand_fifo_wr_ptr <= cand_ptr_add(cand_fifo_wr_ptr, cand_push_count);
            end

            if (cand_fifo_pop) begin
                cand_fifo_rd_ptr <= cand_ptr_add(cand_fifo_rd_ptr, {{(CAND_FIFO_CNT_W - 1) {1'b0}}, 1'b1});
            end

            cand_fifo_level <= cand_fifo_level
                             + cand_push_count
                             - (cand_fifo_pop ? {{(CAND_FIFO_CNT_W-1){1'b0}}, 1'b1}
                                              : {CAND_FIFO_CNT_W{1'b0}});
        end
    end

    // pipe1 装载单路 voxel 结果
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_valid      <= 1'b0;
            pipe1_voxel_x    <= {VOXEL_WIDTH{1'b0}};
            pipe1_voxel_y    <= {VOXEL_WIDTH{1'b0}};
            pipe1_point_proc <= {PT_WIDTH_PER{1'b0}};
        end else begin
            if (pipe1_can_accept) begin
                pipe1_valid <= cand_fifo_pop;

                if (cand_fifo_pop) begin
                    pipe1_voxel_x    <= single_voxel_x;
                    pipe1_voxel_y    <= single_voxel_y;
                    pipe1_point_proc <= single_point_proc;
                end
            end
        end
    end


    // ============================================================
    // frame_end 延迟：等待 candidate FIFO、pipe1、hash 对齐流水排空
    // ============================================================
    reg  frame_end_pending;

    wire frontend_empty;

    reg  pproc_vld_d1;
    reg  pproc_vld_d2;
    reg  pproc_vld_d3;
    // 后面的 pproc_vld_d1/d2/d3 在下一节定义
    assign frontend_empty = cand_fifo_empty && !pipe1_valid && !pproc_vld_d1 && !pproc_vld_d2 && !pproc_vld_d3;

    wire hash_frame_end;
    assign hash_frame_end = frame_end_pending && frontend_empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_end_pending <= 1'b0;
        end else begin
            if (frame_end) begin
                frame_end_pending <= 1'b1;
            end else if (hash_frame_end) begin
                frame_end_pending <= 1'b0;
            end
        end
    end


    wire                       hash_found;
    wire                       hash_busy;
    wire                       hash_table_full;
    wire [HASH_ADDR_WIDTH-1:0] hash_out_idx;
    wire [       VN_WIDTH-1:0] hash_out_point_number;
    hash_bucket_table #(
        .COORD_WIDTH        (VOXEL_WIDTH),
        .VN_WIDTH           (VN_WIDTH),
        .LIFE_CYCLE         (LIFE_CYCLE),
        .TIMER_WIDTH        (DRAM_ADDR_WIDTH),     // NOTE max 37650
        .ADDR_WIDTH         (HASH_ADDR_WIDTH),
        .BUCKETS            (64),
        .BUCKET_SLOT_WIDTH  (2),
        .BRAM_ADDR_WIDTH    (BRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH    (BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) u_hash_table_tombstone (
        .clk                 (clk),
        .rst_n               (rst_n),
        .req_ready           (hash_req_ready),
        .req_valid           (hash_req_valid),
        .frame_end           (frame_end),
        .flush_done          (flush_done),
        .hash_stall          (hash_stall),
        .key_x               (hash_key_x),
        .key_y               (hash_key_y),
        .out_idx             (hash_out_idx),           // 0-255
        .out_point_number    (hash_out_point_number),  // 1-20
        .table_full          (hash_table_full),
        // .bram_expire_clk_a   (bram_expire_clk_a),
        // .bram_expire_rst_a   (bram_expire_rst_a),
        // .bram_expire_wr_a    (bram_expire_wr_a),
        // .bram_expire_bwen_a  (bram_expire_bwen_a),     // 32-bit Byte Enable
        .bram_expire_addr_a  (bram_expire_addr_a),     // 10-bit Address
        .bram_expire_rdata_a (bram_expire_rdata_a),    // 256-bit Data
        // .bram_expire_clk_b   (bram_expire_clk_b),
        // .bram_expire_rst_b   (bram_expire_rst_b),
        .bram_expire_wr_b    (bram_expire_wr_b),
        // .bram_expire_bwen_b  (bram_expire_bwen_b),     // 32-bit Byte Enable
        .bram_expire_addr_b  (bram_expire_addr_b),     // 10-bit Address
        .bram_expire_wrdata_b(bram_expire_wrdata_b),   // 256-bit Data
        .m_axis_expire_tready(m_axis_expire_tready),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tdata (m_axis_expire_tdata)
    );

    // ---------------------------与 hash 对齐-------------------------
    reg [PT_WIDTH_PER-1:0] pproc_d1, pproc_d2, pproc_d3;
    reg hash_req_ready_1delay, hash_req_ready_2delay;

    wire pipe_pproc_stall = hash_req_ready;
    wire pipe_bram_stall = hash_req_ready_1delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pproc_d1 <= {PT_WIDTH_PER{1'd0}};
            pproc_d2 <= {PT_WIDTH_PER{1'd0}};
            pproc_d3 <= {PT_WIDTH_PER{1'd0}};

        end else if (pipe_pproc_stall) begin  // pproc_d3需要和 hash_out_idx对齐
            // pproc_vld_d1 <= (hash_fire | u_hash_table_tombstone.kill_expired);
            pproc_d1 <= pipe1_point_proc;
            pproc_d2 <= pproc_d1;
            pproc_d3 <= pproc_d2;
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_req_ready_1delay <= 1'b0;
            hash_req_ready_2delay <= 1'b0;
        end else begin
            hash_req_ready_2delay <= hash_req_ready_1delay;
            hash_req_ready_1delay <= hash_req_ready;
        end
    end
    // ------------------------------------------------------------
    // 7) BRAM 地址/槽位计算（修正：加入 (point_number-1)>>2 与 slot 0-based）
    // ------------------------------------------------------------
    wire [VN_WIDTH-1:0] pt_idx_minus_1 = hash_out_point_number - 1'b1;
    wire [BRAM_ADDR_WIDTH-1:0] row_offset = pt_idx_minus_1 / POINTS_PER_ROW;
    wire [$clog2(POINTS_PER_ROW)-1:0] bram_slot_idx = pt_idx_minus_1 % POINTS_PER_ROW;
    wire [BRAM_ADDR_WIDTH-1:0] bram_row_addr = (hash_out_idx * EXPEND_VOXEL_ROW) + row_offset;

    localparam integer CHUNKS_PER_POINT = PT_WIDTH_PER / BYTE_WIDTH;  // 72/9 = 8
    function [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] slot_bwen_64;
        input [3:0] slot;
        reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] mask;
        begin
            mask                                          = {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            mask[slot*CHUNKS_PER_POINT+:CHUNKS_PER_POINT] = {{CHUNKS_PER_POINT{1'b1}}};
            slot_bwen_64                                  = mask;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_voxelpoint_wr     <= 1'b0;
            bram_voxelpoint_bwen   <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            bram_voxelpoint_wrdata <= {BRAM_DATA_WIDTH{1'b0}};
            bram_voxelpoint_addr   <= {BRAM_ADDR_WIDTH{1'b0}};
        end else begin
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            // 只有当 hash_busy 且我们确实在上一拍 pop 过（对齐）才写
            if (pipe_bram_stall) begin
                // hash_found 表示 hit 或者新插入成功；table_full 时 found=0
                // 同时加一个上限：每体素最多 20 点
                if ((hash_out_point_number <= MAX_VOXEL_NUM[VN_WIDTH-1:0]) && (hash_out_point_number != {VN_WIDTH{1'b0}})) begin
                    bram_voxelpoint_wr     <= 1'b1;
                    bram_voxelpoint_addr   <= bram_row_addr;
                    bram_voxelpoint_wrdata <= {POINTS_PER_ROW{pproc_d3}};
                    bram_voxelpoint_bwen   <= slot_bwen_64(bram_slot_idx);
                end
            end
        end
    end




`ifndef SYNTHESIS
    // ===================================================================
    // 仿真专用监控探针：点云坐标与 Intensity 有效范围检测
    // ===================================================================
    real dbg_pt_x, dbg_pt_y, dbg_pt_z, dbg_pt_i;
    real orig_float_x, orig_float_y, orig_float_z, orig_float_i;

    // 纯 Verilog-2001 兼容的 IEEE-754 单精度解析函数
    function real decode_float32(input [31:0] bits);
        reg        sign;
        reg [ 7:0] exp;
        reg [22:0] frac;
        begin
            sign = bits[31];
            exp  = bits[30:23];
            frac = bits[22:0];

            if (exp == 8'h00) begin
                decode_float32 = 0.0;
            end else begin
                // 公式: (-1)^sign * 2^(exp-127) * (1 + frac / 2^23)
                // 2^23 = 8388608.0
                decode_float32 = (sign ? -1.0 : 1.0) * (2.0 ** (exp - 127)) * (1.0 + ($itor(frac) / 8388608.0));
            end
        end
    endfunction

    // always @(posedge clk) begin
    //     // 当有新的有效原始点进入时，触发检测
    //     if (rst_n && point_raw_vld) begin
    //         // 1. 还原定点数的物理浮点值
    //         dbg_pt_x = $itor($signed(pt_x_fix16)) / 256.0;
    //         dbg_pt_y = $itor($signed(pt_y_fix16)) / 256.0;
    //         dbg_pt_z = $itor($signed(pt_z_fix16)) / 256.0;
    //         dbg_pt_i = $itor($signed(pt_intensity_fix16)) / 256.0;

    //         // 2. 调用手工解析函数，完美绕过 $bitstoshortreal 报错
    //         orig_float_x = decode_float32(pt_x_float32);
    //         orig_float_y = decode_float32(pt_y_float32);
    //         orig_float_z = decode_float32(pt_z_float32);
    //         orig_float_i = decode_float32(pt_intensity_float32);

    //         // 1. 监控 X 坐标: [-54.0, 54.0]
    //         if (dbg_pt_x < -54.0 || dbg_pt_x > 54.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_x OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_x, pt_x_fix16 & 16'hFFFF, orig_float_x);
    //         end

    //         // 2. 监控 Y 坐标: [-54.0, 54.0]
    //         if (dbg_pt_y < -54.0 || dbg_pt_y > 54.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_y OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_y, pt_y_fix16 & 16'hFFFF, orig_float_y);
    //         end

    //         // 3. 监控 Z 坐标: [-5.0, 3.0]
    //         if (dbg_pt_z < -5.0 || dbg_pt_z > 3.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_z OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_z, pt_z_fix16 & 16'hFFFF, orig_float_z);
    //         end

    //         // 4. 监控 Intensity: [0, 256.0]
    //         if (dbg_pt_i < 0.0 || dbg_pt_i > 256.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_intensity OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_i, pt_intensity_fix16 & 16'hFFFF, orig_float_i);
    //         end
    //     end
    // end


    // reg  [31:0] stall_cnt;  // 32位计数器，足够统计很长时间
    // 阻塞条件：有有效请求 (point_raw_vld && keep_point) 但 Hash 表未 Ready
    // wire        is_stalled = (point_raw_vld && keep_point) && (!hash_req_ready);
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         stall_cnt <= 32'd0;
    //     end else begin
    //         if (is_stalled) begin
    //             stall_cnt <= stall_cnt + 1'b1;
    //         end
    //     end
    // end

    wire test_coor = lane_y_fix20_data[0] == 20'h15c28 || lane_y_fix20_data[1] == 20'h15c28 || lane_y_fix20_data[2] == 20'h15c28;
    reg [VOXEL_WIDTH-1:0] voxel_x_test = 11'd1;
    reg [VOXEL_WIDTH-1:0] voxel_y_test = 11'd433;

    wire test_reg = (single_voxel_x == voxel_x_test) && (single_voxel_y == voxel_y_test);
    wire test_vector = (bram_voxelpoint_addr == 9'd80) && ((bram_voxelpoint_bwen == 80'h000000000000000000ff));
    // wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_x = pproc_d3_voxel[VOXEL_WIDTH*2-1:VOXEL_WIDTH];
    // wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_y = pproc_d3_voxel[VOXEL_WIDTH-1:0];
    // reg [1:0] test_reg_rise_cnt;
    // reg test_reg_d;

    // // test_reg 第三次上升沿时打印上层 BRAM 指定地址内容
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         test_reg_rise_cnt <= 2'd0;
    //         test_reg_d        <= 1'b0;
    //     end else begin
    //         test_reg_d <= test_reg;
    //         if (test_reg && !test_reg_d) begin
    //             if (test_reg_rise_cnt == 2'd2) begin
    //                 $display("[voxel_coor_pipe][%0d] TEST_REG_3RD: mem[60]=0x%0h", u_hash_table_tombstone.global_timer,
    //                          tb_voxelize.dut.u_voxelpoint_bram.mem[60]);
    //             end
    //             if (test_reg_rise_cnt != 2'd3) begin
    //                 test_reg_rise_cnt <= test_reg_rise_cnt + 1'b1;
    //             end
    //         end
    //     end
    // end

    // // 调试触发：命中指定 BRAM 写地址/掩码时，打印 pproc_d3_voxel 及其拆分坐标
    // always @(posedge clk) begin
    //     if (rst_n && test_vector) begin
    //         $display("[voxel_coor_pipe][%0d] BRAM_HIT: addr=%0d bwen=0x%020h pproc_d3_voxel=0x%0h voxel_x=%0d voxel_y=%0d",
    //                  u_hash_table_tombstone.global_timer, bram_voxelpoint_addr, bram_voxelpoint_bwen, pproc_d3_voxel,
    //                  pproc_d3_voxel_x, pproc_d3_voxel_y);
    //     end
    // end

    // // 调试断点：命中指定体素坐标时打印定点值并a中断仿真
    // always @(posedge clk) begin
    //     if (rst_n && point_raw_vld && keep_point && test_reg) begin
    //         $display(
    //             "[voxel_coor_pipe][%0t] BREAK: voxel_x=%0d voxel_y=%0d global_timer=%0d pt_x=%0f pt_y=%0f pt_z=%0f (Q8.8)",
    //             $time, voxel_x, voxel_y, u_hash_table_tombstone.global_timer, $itor($signed(pt_x_fix16)) / 256.0, $itor
    //             ($signed(pt_y_fix16)) / 256.0, $itor($signed(pt_z_fix16)) / 256.0);
    //         // $stop;
    //     end
    // end


    // wire test_reg_voxel = (pproc_d3_voxel == {voxel_x_test, voxel_y_test});

`endif

endmodule
