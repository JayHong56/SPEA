module voxel_coor_pipe #(
    parameter integer BRAM_DATA_WIDTH = 504,  // 72 * 7
    parameter integer BRAM_ADDR_WIDTH = 10,  // 256 pillar * 3 rows
    parameter integer BRAM_ADDR_WIDTH_PFE = 6,  // 32, PFE_CACHE 深度
    parameter integer DRAM_DATA_WIDTH = 128,  // 单路输入：1 point/beat, {x,y,z,intensity} float32
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
    parameter integer BYTE_WIDTH = 9  // 1 Byte = 8 bit, 当前 BRAM 写掩码按 9-bit chunk
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
    output wire bram_voxelpoint_clk,
    output wire bram_voxelpoint_rst,
    output reg bram_voxelpoint_wr,
    output reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen,
    output reg [BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr,
    output reg [BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata,
    // BRAM_VOXELPOINT_A
    output wire bram_expire_clk_a,
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    // BRAM_VOXELPOINT_B
    output wire bram_expire_clk_b,
    output wire bram_expire_rst_b,
    output wire bram_expire_wr_b,
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output wire [BRAM_DATA_WIDTH/9-1:0] bram_expire_bwen_b,
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
    localparam integer RAW_PT_WIDTH = 4 * PT_WIDTH_FLOAT32;  // 128 bit
    // BRAM clk/rst
    assign bram_voxelpoint_clk = clk;
    assign bram_voxelpoint_rst = ~rst_n;

    wire                          hash_req_ready;
    wire                          hash_req_valid;
    wire                          hash_fire = hash_req_valid && hash_req_ready;

    reg                           keep_point;
    reg  [PT_WIDTH_FLOAT32*4-1:0] point_raw_r;
    reg                           point_raw_vld;
    wire                          s_fire = s_axis_dram_valid && s_axis_dram_ready;
    // point_raw_val, keep_point, hash_fire
    // 1/0 : 当前点不在有效范围内
    // 0/? : 当前点无效，且没有新点进来，等待s_fire握手
    // 1/1/0 : 当前点有效，但 hash 表未准备好
    wire                          upstream_can_accept;
    assign s_axis_dram_ready = upstream_can_accept;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            point_raw_r   <= {PT_WIDTH_FLOAT32 * 4{1'b0}};
            point_raw_vld <= 1'b0;
        end else begin
            if (s_fire) begin
                point_raw_r   <= s_axis_dram_data;
                point_raw_vld <= 1'b1;
            end else if (upstream_can_accept) begin
                // 没有新数据进来，但当前数据已经被处理（丢弃或发往下游），清空有效位
                point_raw_vld <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (DRAM_DATA_WIDTH != RAW_PT_WIDTH) begin
            $display("[ERROR][voxel_coor_pipe] DRAM_DATA_WIDTH must equal 128 in single-lane mode.");
            $stop;
        end
    end
`endif

    // ============================================================
    // 单路 128-bit DRAM beat 预处理
    // 输入格式：s_axis_dram_data[127:96] = x_float32
    //         s_axis_dram_data[ 95:64] = y_float32
    //         s_axis_dram_data[ 63:32] = z_float32
    //         s_axis_dram_data[ 31: 0] = intensity_float32
    // ============================================================
    wire [PT_WIDTH_FLOAT32-1:0] point_x_float32;
    wire [PT_WIDTH_FLOAT32-1:0] point_y_float32;
    wire [PT_WIDTH_FLOAT32-1:0] point_z_float32;
    wire [PT_WIDTH_FLOAT32-1:0] point_i_float32;

    assign point_x_float32 = point_raw_r[127:96];
    assign point_y_float32 = point_raw_r[95:64];
    assign point_z_float32 = point_raw_r[63:32];
    assign point_i_float32 = point_raw_r[31:0];

    wire signed [PT_WIDTH_XY-1:0] point_x_fix20_data;
    wire signed [PT_WIDTH_XY-1:0] point_y_fix20_data;
    wire signed [PT_WIDTH-1:0]    point_z_fix16;
    wire signed [PT_WIDTH-1:0]    point_intensity_fix16;

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_XY),
        .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
    ) u_float2fxp_x_data (
        .float_in        (point_x_float32),
        .true_fixed_value(point_x_fix20_data),
        .fixed_sign      ()
    );

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_XY),
        .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
    ) u_float2fxp_y_data (
        .float_in        (point_y_float32),
        .true_fixed_value(point_y_fix20_data),
        .fixed_sign      ()
    );

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_F_Z + PT_WIDTH_I_Z),
        .FIXED_FRACTIONAL(PT_WIDTH_F_Z)
    ) u_float2fxp_z (
        .float_in        (point_z_float32),
        .true_fixed_value(point_z_fix16),
        .fixed_sign      ()
    );

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_F_IS + PT_WIDTH_I_IS),
        .FIXED_FRACTIONAL(PT_WIDTH_F_IS)
    ) u_float2fxp_intensity (
        .float_in        (point_i_float32),
        .true_fixed_value(point_intensity_fix16),
        .fixed_sign      ()
    );

    wire [PT_WIDTH_XY-1:0] point_abs_x;
    wire [PT_WIDTH_XY-1:0] point_abs_y;
    wire [PT_WIDTH-1:0]    point_abs_z;

    assign point_abs_x = point_x_fix20_data[PT_WIDTH_XY-1] ? (~point_x_fix20_data + 1'b1) : point_x_fix20_data;
    assign point_abs_y = point_y_fix20_data[PT_WIDTH_XY-1] ? (~point_y_fix20_data + 1'b1) : point_y_fix20_data;
    assign point_abs_z = point_z_fix16[PT_WIDTH-1] ? (~point_z_fix16 + 1'b1) : point_z_fix16;


    // keep_point（组合）
    always @(*) begin
        if (!rst_n) begin
            keep_point = 1'b0;
            // end else if (point_raw == 128'd0) begin // invaild data 
            //     keep_point <= 1'b0;
            // end else if ((ABS_pt_x_fix16 <= THRESHOLD_CLOSE) && (ABS_pt_y_fix16 <= THRESHOLD_CLOSE)) begin
            //     keep_point <= 1'b0;
        end else if (^point_x_fix20_data === 1'bx || point_raw_r == {RAW_PT_WIDTH{1'b0}}) begin
            keep_point = 1'b0;

        end else if (point_x_fix20_data[PT_WIDTH_XY-1]) begin
            keep_point = 1'b0;

        end else if (point_abs_x > THRESHOLD_BOUDARY_X_HIGH || point_abs_y > THRESHOLD_BOUDARY_Y) begin
            keep_point = 1'b0;
        end else if (point_z_fix16[PT_WIDTH-1] ? (point_abs_z > THRESHOLD_BOUDARY_Z_LOW) :
                                        (point_abs_z > THRESHOLD_BOUDARY_Z_HIGH)) begin
            keep_point = 1'b0;
        end else begin
            keep_point = 1'b1;
        end
    end

    // ------------------------------------------------------------
    // 单路 voxel_x / voxel_y 计算
    // ------------------------------------------------------------
    localparam integer SCALE_VOXEL_FACTOR = 20;
    localparam integer SCALE_VOXEL = 16;
    localparam signed [SCALE_VOXEL_FACTOR-1:0] MULT_FACTOR = 20'd409600;  // 6.25? 具体物理比例沿用原值
    localparam integer ROUND = 1;
    localparam signed [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_Y_CEIL_FIX = 20'sh27ae2;  // 39.68m 但加了一个 LSB
    // ------------------------------------------------------------
    // Fast voxel index calculation
    // MULT_FACTOR = 409600 = 25 << 14
    //
    // Q8.12 input:
    //   voxel = floor(fixed_q12 * 6.25 / 4096)
    //         = floor(fixed_q12 * 25 / 2^14)
    //
    // 不再使用通用 fxp_mul，避免 DSP + 后级 CARRY resize/round 长路径。
    // ------------------------------------------------------------

    // x: x range starts from 0, no boundary offset here.
    wire signed [PT_WIDTH_XY+5:0] voxel_x_mul25;
    wire signed [PT_WIDTH_XY+5:0] voxel_x_scaled;

    assign voxel_x_mul25 = ($signed(
        point_x_fix20_data
    ) <<< 4) + ($signed(
        point_x_fix20_data
    ) <<< 3) + $signed(
        point_x_fix20_data
    );

    assign voxel_x_scaled = voxel_x_mul25 >>> 14;


    // y: y needs +39.68m boundary offset first.
    wire signed [PT_WIDTH_XY:0] voxel_y_shift_q12;

    assign voxel_y_shift_q12 = $signed(
        {point_y_fix20_data[PT_WIDTH_XY-1], point_y_fix20_data}
    ) + $signed(
        {THRESHOLD_BOUDARY_Y_CEIL_FIX[PT_WIDTH_XY-1], THRESHOLD_BOUDARY_Y_CEIL_FIX}
    );

    wire signed [PT_WIDTH_XY+5:0] voxel_y_mul25;
    wire signed [PT_WIDTH_XY+5:0] voxel_y_scaled;

    assign voxel_y_mul25 = ($signed(
        voxel_y_shift_q12
    ) <<< 4) + ($signed(
        voxel_y_shift_q12
    ) <<< 3) + $signed(
        voxel_y_shift_q12
    );

    assign voxel_y_scaled = voxel_y_mul25 >>> 14;


    // final voxel index
    wire [VOXEL_WIDTH-1:0] single_voxel_x;
    wire [VOXEL_WIDTH-1:0] single_voxel_y;

    assign single_voxel_x = voxel_x_scaled[VOXEL_WIDTH-1:0];
    assign single_voxel_y = voxel_y_scaled[VOXEL_WIDTH-1:0];


    wire [PT_WIDTH_PER-1:0] single_point_proc;
    assign single_point_proc = {point_x_fix20_data, point_y_fix20_data, point_z_fix16, point_intensity_fix16};

    // ============================================================
    // hash 输入 pipe1：一拍最多送 1 个有效点给 hash_table
    // ============================================================
    reg                    pipe1_valid;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_x;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_y;
    reg [PT_WIDTH_PER-1:0] pipe1_point_proc;
    reg                    pipe1_keep_point;

    assign hash_req_valid = pipe1_valid;

    wire [VOXEL_WIDTH-1:0] hash_key_x;
    wire [VOXEL_WIDTH-1:0] hash_key_y;

    assign hash_key_x = pipe1_voxel_x;
    assign hash_key_y = pipe1_voxel_y;

    // pipe1 为空，或者当前点已被 hash 接收，则可以装载下一个 DRAM 点
    wire pipe1_stall = pipe1_valid && !hash_fire;
    assign upstream_can_accept = !pipe1_stall;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_valid      <= 1'b0;
            pipe1_keep_point <= 1'b0;
        end else begin
            if (!pipe1_stall) begin
                // 上游数据有效且在有效范围内，传递有效信号
                pipe1_valid      <= point_raw_vld && keep_point;
                pipe1_keep_point <= keep_point;

                // 将算好的体素坐标和点特征打入寄存器
                pipe1_voxel_x    <= single_voxel_x;
                pipe1_voxel_y    <= single_voxel_y;
                pipe1_point_proc <= single_point_proc;
            end
        end
    end

    // ============================================================
    // frame_end 延迟：等待单路前端 pipe1 和 BRAM 对齐流水排空
    // ============================================================
    reg  frame_end_pending;

    wire frontend_empty;

    reg  pproc_vld_d1;
    reg  pproc_vld_d2;
    reg  pproc_vld_d3;

    assign frontend_empty = !pipe1_valid && !pproc_vld_d1 && !pproc_vld_d2 && !pproc_vld_d3;

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
        .frame_end           (hash_frame_end),
        .flush_done          (flush_done),
        .hash_stall          (hash_stall),
        .key_x               (hash_key_x),
        .key_y               (hash_key_y),
        .out_idx             (hash_out_idx),           // 0-255
        .out_point_number    (hash_out_point_number),  // 1-20
        .table_full          (hash_table_full),
        .bram_expire_clk_a   (bram_expire_clk_a),
        // .bram_expire_rst_a   (bram_expire_rst_a),
        // .bram_expire_wr_a    (bram_expire_wr_a),
        // .bram_expire_bwen_a  (bram_expire_bwen_a),     // 32-bit Byte Enable
        .bram_expire_addr_a  (bram_expire_addr_a),     // 10-bit Address
        .bram_expire_rdata_a (bram_expire_rdata_a),    // 256-bit Data
        .bram_expire_clk_b   (bram_expire_clk_b),
        .bram_expire_rst_b   (bram_expire_rst_b),
        .bram_expire_wr_b    (bram_expire_wr_b),
        .bram_expire_bwen_b  (bram_expire_bwen_b),     // 32-bit Byte Enable
        .bram_expire_addr_b  (bram_expire_addr_b),     // 10-bit Address
        .bram_expire_wrdata_b(bram_expire_wrdata_b),   // 256-bit Data
        .m_axis_expire_tready(m_axis_expire_tready),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tdata (m_axis_expire_tdata)
    );

    // ---------------------------与 hash 对齐-------------------------
    reg [PT_WIDTH_PER-1:0] pproc_d1, pproc_d2, pproc_d3;
    reg hash_req_ready_1delay;
    reg pproc_drop_d1, pproc_drop_d2, pproc_drop_d3;

    wire pipe_pproc_stall = hash_req_ready;
    wire pipe_bram_stall = hash_req_ready_1delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pproc_d1      <= {PT_WIDTH_PER{1'd0}};
            pproc_d2      <= {PT_WIDTH_PER{1'd0}};
            pproc_d3      <= {PT_WIDTH_PER{1'd0}};
            pproc_vld_d1  <= 1'b0;
            pproc_vld_d2  <= 1'b0;
            pproc_vld_d3  <= 1'b0;
            pproc_drop_d1 <= 1'b0;
            pproc_drop_d2 <= 1'b0;
            pproc_drop_d3 <= 1'b0;
        end else if (pipe_pproc_stall) begin  // pproc_d3需要和 hash_out_idx对齐
            pproc_drop_d1 <= !pipe1_keep_point;
            pproc_drop_d2 <= pproc_drop_d1;
            pproc_drop_d3 <= pproc_drop_d2;

            pproc_vld_d1  <= hash_fire;
            pproc_vld_d2  <= pproc_vld_d1;
            pproc_vld_d3  <= pproc_vld_d2;

            pproc_d1      <= pipe1_point_proc;
            pproc_d2      <= pproc_d1;
            pproc_d3      <= pproc_d2;
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_req_ready_1delay <= 1'b0;
        end else begin
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
                if (!pproc_drop_d3 && (hash_out_point_number <= MAX_VOXEL_NUM[VN_WIDTH-1:0]) && (hash_out_point_number != {VN_WIDTH{1'b0}})) begin
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

    wire test_coor = (point_y_fix20_data == 20'h15c28);
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
