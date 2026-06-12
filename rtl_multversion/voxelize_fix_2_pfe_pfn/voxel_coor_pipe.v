module voxel_coor_pipe #(
    parameter integer        BRAM_DATA_WIDTH          = 640,       // 4 * 64
    parameter integer        BRAM_ADDR_WIDTH          = 9,         // 128pillar * 5brams
    parameter integer        BRAM_ADDR_WIDTH_PFE      = 6,         // 32, PFE_CACHE 深度
    parameter integer        DRAM_DATA_WIDTH          = 128,
    parameter integer        DRAM_ADDR_WIDTH          = 16,
    parameter integer        HASH_ADDR_WIDTH          = 8,         // log2(HASH_TABLE_SIZE)
    parameter         [15:0] THRESHOLD_CLOSE          = 16'h0100,  // 1.00m
    parameter         [15:0] THRESHOLD_BOUDARY        = 16'h3600,  // 54.00m
    parameter         [15:0] THRESHOLD_BOUDARY_Z_LOW  = 16'h5000,  // -(-5)m
    parameter         [15:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h3000,  // +3m
    parameter integer        PRE_LAT                  = 0,         // 预处理固定延迟（拍数占位）
    parameter         [15:0] LIFE_CYCLE               = 16'd100
) (
    input wire clk,
    input wire rst_n,

    input  wire                         frame_end,              // 外部传入：当前帧点云结束脉冲 (1-cycle pulse)
    output wire                         flush_done,             // 输出：哈希表剩余有效点已全部输出完毕
    output wire                         hash_stall,
    // AXI-Stream in (from DRAM FIFO)
    input  wire [  DRAM_DATA_WIDTH-1:0] s_axis_dram_data,
    input  wire [DRAM_DATA_WIDTH/8-1:0] s_axis_dram_keep,
    input  wire                         s_axis_dram_last,
    input  wire                         s_axis_dram_valid,
    output wire                         s_axis_dram_ready,
    // BRAM Native Interface
    output wire                         bram_voxelpoint_clk,
    output wire                         bram_voxelpoint_rst,
    output reg                          bram_voxelpoint_wr,
    output reg  [BRAM_DATA_WIDTH/8-1:0] bram_voxelpoint_bwen,   // 32-bit Byte Enable
    output reg  [  BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr,   // 10-bit Address
    output reg  [  BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata, // 256-bit Data

    output wire                       bram_expire_clk_a,
    // output wire                         bram_expire_rst_a,
    // output wire                         bram_expire_wr_a,
    // output wire [BRAM_DATA_WIDTH/8-1:0] bram_expire_bwen_a,   // 32-bit Byte Enable
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,  // 10-bit Address
    input  wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a, // 256-bit Data

    output wire                           bram_expire_clk_b,
    output wire                           bram_expire_rst_b,
    output wire                           bram_expire_wr_b,
    output wire [  BRAM_DATA_WIDTH/8-1:0] bram_expire_bwen_b,   // 32-bit Byte Enable
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,   // 10-bit Address
    output wire [    BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b, // 256-bit Data

    // AXI-Stream out (expire voxel coordinates)
    output wire                                m_axis_expire_tvalid,
    input  wire                                m_axis_expire_tready,
    output wire [22+BRAM_ADDR_WIDTH_PFE+5-1:0] m_axis_expire_tdata
);

    localparam integer PT_WIDTH_I_XY = 8;
    localparam integer PT_WIDTH_F_XY = 8;
    localparam integer PT_WIDTH_I_Z = 4;
    localparam integer PT_WIDTH_F_Z = 12;
    localparam integer PT_WIDTH_I_IS = 9;
    localparam integer PT_WIDTH_F_IS = 7;

    localparam integer PT_WIDTH = 16;  // 16
    localparam integer PT_WIDTH_FLOAT32 = 32;
    localparam integer VOXEL_WIDTH = 11;

    localparam [23:0] MAX_VOXEL_NUM = 24'd20;  // 每体素最多 20 点
    localparam integer VN_WIDTH = 5;  // 点编号宽度（hash 表输出 1..）
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / (4 * PT_WIDTH);  // 640/64 = 10
    localparam integer EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  // ceil(20/10)=2

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

    wire        [         PT_WIDTH_FLOAT32*4-1:0] point_raw = point_raw_r;
    // unpack
    wire        [           PT_WIDTH_FLOAT32-1:0] pt_x_float32 = point_raw[127:96];
    wire        [           PT_WIDTH_FLOAT32-1:0] pt_y_float32 = point_raw[95:64];
    wire        [           PT_WIDTH_FLOAT32-1:0] pt_z_float32 = point_raw[63:32];
    wire        [           PT_WIDTH_FLOAT32-1:0] pt_intensity_float32 = point_raw[31:0];

    // float to fixed
    wire signed [PT_WIDTH_I_XY+PT_WIDTH_F_XY-1:0] pt_x_fix16;
    wire signed [PT_WIDTH_I_XY+PT_WIDTH_F_XY-1:0] pt_y_fix16;
    wire signed [  PT_WIDTH_I_Z+PT_WIDTH_F_Z-1:0] pt_z_fix16;
    wire signed [PT_WIDTH_I_IS+PT_WIDTH_F_IS-1:0] pt_intensity_fix16;
    float_to_fixed #(
        .FIXED_WIDTH     (PT_WIDTH_F_XY + PT_WIDTH_I_XY),
        .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
    ) float2fxp_x (
        .float_in        (pt_x_float32),
        .true_fixed_value(pt_x_fix16),
        .fixed_sign      ()
    );
    float_to_fixed #(
        .FIXED_WIDTH     (PT_WIDTH_F_XY + PT_WIDTH_I_XY),
        .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
    ) float2fxp_y (
        .float_in        (pt_y_float32),
        .true_fixed_value(pt_y_fix16),
        .fixed_sign      ()
    );
    float_to_fixed #(
        .FIXED_WIDTH     (PT_WIDTH_F_Z + PT_WIDTH_I_Z),
        .FIXED_FRACTIONAL(PT_WIDTH_F_Z)
    ) float2fxp_z (
        .float_in        (pt_z_float32),
        .true_fixed_value(pt_z_fix16),
        .fixed_sign      ()
    );
    float_to_fixed #(
        .FIXED_WIDTH     (PT_WIDTH_F_IS + PT_WIDTH_I_IS),
        .FIXED_FRACTIONAL(PT_WIDTH_F_IS)
    ) float2fxp_intensity (
        .float_in        (pt_intensity_float32),
        .true_fixed_value(pt_intensity_fix16),
        .fixed_sign      ()
    );

    // 1) voxel_x / voxel_y 计算（沿用你原逻辑）
    localparam signed [1+19-1:0] MULT_FACTOR = 20'd436907;  // 6.66666 缩放倍数
    wire signed [   PT_WIDTH_I_XY+PT_WIDTH_F_XY-1:0] bourdary_fix16 = THRESHOLD_BOUDARY;  // 54 Q8.8
    wire        [   PT_WIDTH_I_XY+PT_WIDTH_F_XY-1:0] fxp_add_x_out;
    wire        [PT_WIDTH_I_XY+PT_WIDTH_F_XY+20-1:0] fxp_mul_x_out;
    localparam integer ROUND = 0;

    fxp_add #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (PT_WIDTH_I_XY),
        .WIFB (PT_WIDTH_F_XY),
        .WOI  (PT_WIDTH_I_XY),
        .WOF  (PT_WIDTH_F_XY),
        .ROUND(ROUND)
    ) fxp_add_voxel_x (
        .ina     (pt_x_fix16),
        .inb     (bourdary_fix16),
        .out     (fxp_add_x_out),
        .overflow()
    );

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (20),
        .WIFB (0),
        .WOI  (36),
        .WOF  (0),
        .ROUND(ROUND)
    ) fxp_mul_voxel_x (
        .ina     (fxp_add_x_out),
        .inb     (MULT_FACTOR),
        .out     (fxp_mul_x_out),
        .overflow()
    );

    wire [   PT_WIDTH_I_XY+PT_WIDTH_F_XY-1:0] fxp_add_y_out;
    wire [PT_WIDTH_I_XY+PT_WIDTH_F_XY+20-1:0] fxp_mul_y_out;

    fxp_add #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (PT_WIDTH_I_XY),
        .WIFB (PT_WIDTH_F_XY),
        .WOI  (PT_WIDTH_I_XY),
        .WOF  (PT_WIDTH_F_XY),
        .ROUND(ROUND)
    ) fxp_add_voxel_y (
        .ina     (pt_y_fix16),
        .inb     (bourdary_fix16),
        .out     (fxp_add_y_out),
        .overflow()
    );

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (20),
        .WIFB (0),
        .WOI  (36),
        .WOF  (0),
        .ROUND(ROUND)
    ) fxp_mul_voxel_y (
        .ina     (fxp_add_y_out),
        .inb     (MULT_FACTOR),
        .out     (fxp_mul_y_out),
        .overflow()
    );

    // 截断成体素坐标
    wire [VOXEL_WIDTH-1:0] voxel_x = fxp_mul_x_out[16+VOXEL_WIDTH-1:16];
    wire [VOXEL_WIDTH-1:0] voxel_y = fxp_mul_y_out[16+VOXEL_WIDTH-1:16];

    // abs
    wire [           15:0] ABS_pt_x_fix16 = (pt_x_fix16[15]) ? (~pt_x_fix16 + 1'b1) : pt_x_fix16;
    wire [           15:0] ABS_pt_y_fix16 = (pt_y_fix16[15]) ? (~pt_y_fix16 + 1'b1) : pt_y_fix16;
    wire [           15:0] ABS_pt_z_fix16 = (pt_z_fix16[15]) ? (~pt_z_fix16 + 1'b1) : pt_z_fix16;

    // keep_point（组合）
    always @(*) begin
        if (!rst_n) begin
            keep_point = 1'b0;
            // end else if (point_raw == 128'd0) begin // invaild data 
            //     keep_point <= 1'b0;
            // end else if ((ABS_pt_x_fix16 <= THRESHOLD_CLOSE) && (ABS_pt_y_fix16 <= THRESHOLD_CLOSE)) begin
            //     keep_point <= 1'b0;
        end else if (^pt_x_fix16 === 1'bx || point_raw == 0) begin
            keep_point = 1'b0;
        end else if (ABS_pt_x_fix16 > THRESHOLD_BOUDARY || ABS_pt_y_fix16 > THRESHOLD_BOUDARY) begin
            keep_point = 1'b0;
        end else if (pt_z_fix16[15] ? (ABS_pt_z_fix16 > THRESHOLD_BOUDARY_Z_LOW) :
                                        (ABS_pt_z_fix16 > THRESHOLD_BOUDARY_Z_HIGH)) begin
            keep_point = 1'b0;
        end else begin
            keep_point = 1'b1;
        end
    end

    // point_proc 组合值（Q8.8 拼起来）
    wire [ 4*PT_WIDTH-1:0] point_proc_comb = {pt_x_fix16, pt_y_fix16, pt_z_fix16, pt_intensity_fix16};

    // Hash 表的输入数据
    reg                    pipe1_valid;
    reg  [VOXEL_WIDTH-1:0] pipe1_voxel_x;
    reg  [VOXEL_WIDTH-1:0] pipe1_voxel_y;
    reg  [           63:0] pipe1_point_proc;
    reg                    pipe1_keep_point;
    assign hash_req_valid = pipe1_valid;
    wire [VOXEL_WIDTH-1:0] hash_key_x = pipe1_voxel_x;
    wire [VOXEL_WIDTH-1:0] hash_key_y = pipe1_voxel_y;
    // 反压握手逻辑 (Stall logic)
    // 如果 pipe1 有效数据且下游哈希表没准备好，则堵塞上游
    wire                   pipe1_stall = pipe1_valid && !hash_fire;
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
                pipe1_voxel_x    <= voxel_x;
                pipe1_voxel_y    <= voxel_y;
                pipe1_point_proc <= point_proc_comb;
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
    reg [4*PT_WIDTH-1:0] pproc_d1, pproc_d2, pproc_d3;
    reg [VOXEL_WIDTH*2-1:0] pproc_d1_voxel, pproc_d2_voxel, pproc_d3_voxel;
    reg pproc_drop_d1, pproc_drop_d2, pproc_drop_d3;
    reg hash_req_ready_1delay, hash_req_ready_2delay;

    wire pipe_pproc_stall = hash_req_ready;
    wire pipe_bram_stall = hash_req_ready_1delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pproc_d1      <= 64'd0;
            pproc_d2      <= 64'd0;
            pproc_d3      <= 64'd0;
            pproc_drop_d1 <= 1'b0;
            pproc_drop_d2 <= 1'b0;
            pproc_drop_d3 <= 1'b0;
        end else if (pipe_pproc_stall) begin  // pproc_d3需要和 hash_out_idx对齐
            // pproc_vld_d1 <= (hash_fire | u_hash_table_tombstone.kill_expired);
            pproc_drop_d1  <= !pipe1_keep_point;
            pproc_drop_d2  <= pproc_drop_d1;
            pproc_drop_d3  <= pproc_drop_d2;

            pproc_d1       <= pipe1_point_proc;
            pproc_d2       <= pproc_d1;
            pproc_d3       <= pproc_d2;

            pproc_d1_voxel <= {pipe1_voxel_x, pipe1_voxel_y};
            pproc_d2_voxel <= pproc_d1_voxel;
            pproc_d3_voxel <= pproc_d2_voxel;
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
    wire [BRAM_ADDR_WIDTH-1:0] bram_row_addr = (hash_out_idx * EXPEND_VOXEL_ROW) + ((hash_out_point_number > POINTS_PER_ROW) ? {{(BRAM_ADDR_WIDTH-1){1'b0}}, 1'b1} : {(BRAM_ADDR_WIDTH-1){1'b0}});
    wire [4:0] pt_idx_minus_1 = hash_out_point_number - {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
    wire [3:0] bram_slot_idx = (pt_idx_minus_1 >= 5'd10) ? (pt_idx_minus_1 - 5'd10) : pt_idx_minus_1[3:0];

    function [BRAM_DATA_WIDTH/8-1:0] slot_bwen_64;
        input [3:0] slot;
        reg [BRAM_DATA_WIDTH/8-1:0] mask;
        begin
            mask            = {BRAM_DATA_WIDTH / 8{1'b0}};
            mask[slot*8+:8] = 8'hFF;
            slot_bwen_64    = mask;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_voxelpoint_wr     <= 1'b0;
            bram_voxelpoint_bwen   <= {BRAM_DATA_WIDTH / 8{1'b0}};
            bram_voxelpoint_wrdata <= {POINTS_PER_ROW{1'b0}};
            bram_voxelpoint_addr   <= {BRAM_ADDR_WIDTH{1'b0}};
        end else begin
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / 8{1'b0}};
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


    reg  [31:0] stall_cnt;  // 32位计数器，足够统计很长时间
    // 阻塞条件：有有效请求 (point_raw_vld && keep_point) 但 Hash 表未 Ready
    wire        is_stalled = (point_raw_vld && keep_point) && (!hash_req_ready);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stall_cnt <= 32'd0;
        end else begin
            if (is_stalled) begin
                stall_cnt <= stall_cnt + 1'b1;
            end
        end
    end

    reg [VOXEL_WIDTH-1:0] voxel_x_test = 11'd261;
    reg [VOXEL_WIDTH-1:0] voxel_y_test = 11'd497;

    wire test_reg = (voxel_x == voxel_x_test) && (voxel_y == voxel_y_test);
    wire test_vector = (bram_voxelpoint_addr == 9'd80) && ((bram_voxelpoint_bwen == 80'h000000000000000000ff));
    wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_x = pproc_d3_voxel[VOXEL_WIDTH*2-1:VOXEL_WIDTH];
    wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_y = pproc_d3_voxel[VOXEL_WIDTH-1:0];
    reg [1:0] test_reg_rise_cnt;
    reg test_reg_d;

    // test_reg 第三次上升沿时打印上层 BRAM 指定地址内容
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            test_reg_rise_cnt <= 2'd0;
            test_reg_d        <= 1'b0;
        end else begin
            test_reg_d <= test_reg;
            if (test_reg && !test_reg_d) begin
                if (test_reg_rise_cnt == 2'd2) begin
                    $display("[voxel_coor_pipe][%0d] TEST_REG_3RD: mem[60]=0x%0h", u_hash_table_tombstone.global_timer,
                             tb_voxelize.dut.u_voxelpoint_bram.mem[60]);
                end
                if (test_reg_rise_cnt != 2'd3) begin
                    test_reg_rise_cnt <= test_reg_rise_cnt + 1'b1;
                end
            end
        end
    end

    // 调试触发：命中指定 BRAM 写地址/掩码时，打印 pproc_d3_voxel 及其拆分坐标
    always @(posedge clk) begin
        if (rst_n && test_vector) begin
            $display("[voxel_coor_pipe][%0d] BRAM_HIT: addr=%0d bwen=0x%020h pproc_d3_voxel=0x%0h voxel_x=%0d voxel_y=%0d",
                     u_hash_table_tombstone.global_timer, bram_voxelpoint_addr, bram_voxelpoint_bwen, pproc_d3_voxel,
                     pproc_d3_voxel_x, pproc_d3_voxel_y);
        end
    end

    // 调试断点：命中指定体素坐标时打印定点值并a中断仿真
    always @(posedge clk) begin
        if (rst_n && point_raw_vld && keep_point && test_reg) begin
            $display(
                "[voxel_coor_pipe][%0t] BREAK: voxel_x=%0d voxel_y=%0d global_timer=%0d pt_x=%0f pt_y=%0f pt_z=%0f (Q8.8)",
                $time, voxel_x, voxel_y, u_hash_table_tombstone.global_timer, $itor($signed(pt_x_fix16)) / 256.0, $itor
                ($signed(pt_y_fix16)) / 256.0, $itor($signed(pt_z_fix16)) / 256.0);
            // $stop;
        end
    end


    wire test_reg_voxel = (pproc_d3_voxel == {voxel_x_test, voxel_y_test});

`endif

endmodule
