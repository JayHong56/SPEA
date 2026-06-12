module pfe #(
    parameter COORD_WIDTH = 11,
    parameter PT_WIDTH = 16,
    // parameter HASH_ADDR_WIDTH = 8,  // 256
    parameter MAX_VOXEL_NUM = 12'd20,
    parameter VN_WIDTH = 5,
    parameter EXPAND_PT_DIM = 11,  // x,y,z,intensity,0,(x-mean,y-mean,z-mean,intensity-mean),(x-vmean,y-vmean,z-vmean,intensity-vmean)
    parameter integer BRAM_DATA_WIDTH = 640,  // 64bit * 【10 points per row】
    parameter integer BRAM_ADDR_WIDTH = 9,  // 256pillar * 2brams
    parameter integer BRAM_ADDR_WIDTH_PFE = 6

) (
    input  wire                                                  clk,
    input  wire                                                  rst_n,
    input  wire                                                  s_axis_expire_tvalid,
    output wire                                                  s_axis_expire_tready,
    input  wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] s_axis_expire_tdata,   // NOTE

    output wire                                    bram_pfe_clk,
    // output wire                                       bram_pfe_rst,
    output reg        [   BRAM_ADDR_WIDTH_PFE-1:0] bram_pfe_addr,           // 假设地址空间足够大
    // output reg                                        bram_pfe_en,
    input  wire       [       BRAM_DATA_WIDTH-1:0] bram_pfe_rdata,          // 255:0 ((9+7)*4 )*4points
    input  wire                                    m_axis_pfe_tready,
    output reg                                     m_axis_pfe_valid,
    output reg        [EXPAND_PT_DIM*PT_WIDTH-1:0] m_axis_pfe_data,         // 11*16 bits
    output reg                                     m_axis_pfe_last,
    output reg                                     m_axis_pfe_voxel_valid,
    output reg signed [           COORD_WIDTH-1:0] m_axis_pfe_voxel_x,
    output reg signed [           COORD_WIDTH-1:0] m_axis_pfe_voxel_y,
    output wire                                    m_axis_pfe_flush_done,   // <--- 新增
    input  wire                                    flush_done
);
    assign bram_pfe_clk = clk;
    // 注意：Verilog 切片最好用固定数值，或者确保 parameter 计算正确
    wire signed [COORD_WIDTH-1:0] axis_voxel_x = s_axis_expire_tdata[(2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire signed [COORD_WIDTH-1:0] axis_voxel_y = s_axis_expire_tdata[(COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire [BRAM_ADDR_WIDTH_PFE-1:0] axis_bram_index = s_axis_expire_tdata[(BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:BRAM_ADDR_WIDTH_PFE];
    wire [VN_WIDTH-1:0] axis_point_num_indicator = s_axis_expire_tdata[(VN_WIDTH-1) : 0];  // 1-20 points

    // -----------------------------------------------------------
    // 2. 内部存储 (Points Buffer) & 累加器
    // -----------------------------------------------------------
    // 存储该 Voxel 内的所有点，供后续处理 (Process 阶段) 使用
    localparam EXPAND_VOXEL_ROW = MAX_VOXEL_NUM >> 2;  // 20 / 4 = 5
    localparam integer PT_WIDTH_I_XY = 8;
    localparam integer PT_WIDTH_F_XY = 8;
    localparam integer PT_WIDTH_I_Z = 4;
    localparam integer PT_WIDTH_F_Z = 12;
    localparam integer PT_WIDTH_I_IS = 9;
    localparam integer PT_WIDTH_F_IS = 7;

    // --------------------扩展voxel_center点坐标维度---------------------
    // voxel_x_fix16 = voxel_x * 0.15 - 54.0 + 0.075
    localparam signed [18-1:0] COEFF_SLOPE = 18'sd9830;  // 0.15
    localparam signed [23-1:0] COEFF_OFFSET = 23'sd3534029;  // 53.925 (54.0 - 0.075 offset) in Q16

    // -----------------------------------------------------------
    // 源头安全反压控制 (In-flight tracking)
    // -----------------------------------------------------------
    reg  [2:0] inflight_cnt;
    wire       fire_in;

    // Stage2 完成一个 voxel，并成功写入 Stage3
    wire       st2_to_st3;

    // 统一的容量模型：
    // inflight_cnt  : Stage1/2 中已经接收、但还没落到 Stage3 的 voxel 数
    // 总容量 = VOX_FIFO_DEPTH + 2
    wire       pipe_stall;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inflight_cnt <= 3'd0;
        end else begin
            case ({
                fire_in, st2_to_st3
            })
                2'b10:   inflight_cnt <= inflight_cnt + 1'b1;  // 新 voxel 进入 Stage1/2
                2'b01:   inflight_cnt <= inflight_cnt - 1'b1;  // 一个 voxel 落入 Stage3
                default: inflight_cnt <= inflight_cnt;
            endcase
        end
    end


    // 常用循环变量
    integer k, i;
    // Stage 1: 地址生成流水线 (Address Generation) 【2 cycles】
    reg  [2:0] req_phase;
    reg        req_valid;
    wire       ready_to_accept = (req_phase == 3'd0) && !pipe_stall;
    assign s_axis_expire_tready = ready_to_accept;
    assign fire_in              = s_axis_expire_tvalid && ready_to_accept;

    reg [VN_WIDTH-1:0] st1_point_num;
    reg [COORD_WIDTH-1:0] st1_voxel_x, st1_voxel_y;
    reg [BRAM_ADDR_WIDTH-1:0] st1_bram_b_addr;  // 这里换成记录基准地址
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_phase     <= 3'd0;
            req_valid     <= 1'b0;
            bram_pfe_addr <= 0;
        end else begin
            if (fire_in || req_phase != 0) begin
                if (req_phase == 0) begin
                    st1_point_num   <= axis_point_num_indicator;
                    st1_voxel_x     <= axis_voxel_x;
                    st1_voxel_y     <= axis_voxel_y;
                    st1_bram_b_addr <= axis_bram_index;  // 锁存
                end

                bram_pfe_addr <= (req_phase == 3'd0) ? (axis_bram_index) : (st1_bram_b_addr + 1'b1);
                req_phase     <= (req_phase == 3'd1) ? 3'd0 : req_phase + 1'b1;
                req_valid     <= 1'b1;
            end else begin
                req_valid <= 1'b0;
            end
        end
    end

    // 将 Phase 和 Valid 信号随 BRAM 的 2 周期读取延迟进行打拍
    reg [         2:0] pipe_phase[0:3];  // 延迟 2 拍的 phase (3bits)
    reg                pipe_valid[0:3];  // 延迟 2 拍的 valid

    reg [VN_WIDTH-1:0] pipe_pnum [0:3];  // 元数据伴随流动
    reg [COORD_WIDTH-1:0] pipe_vx[0:3], pipe_vy[0:3];

    always @(posedge clk) begin
        pipe_phase[0] <= req_phase;
        pipe_phase[1] <= pipe_phase[0];
        pipe_phase[2] <= pipe_phase[1];
        pipe_phase[3] <= pipe_phase[2];

        pipe_valid[0] <= req_valid;
        pipe_valid[1] <= pipe_valid[0];
        pipe_valid[2] <= pipe_valid[1];
        pipe_valid[3] <= pipe_valid[2];

        pipe_pnum[0]  <= st1_point_num;
        pipe_pnum[1]  <= pipe_pnum[0];
        pipe_pnum[2]  <= pipe_pnum[1];
        pipe_pnum[3]  <= pipe_pnum[2];

        pipe_vx[0]    <= st1_voxel_x;
        pipe_vx[1]    <= pipe_vx[0];
        pipe_vx[2]    <= pipe_vx[1];
        pipe_vx[3]    <= pipe_vx[2];

        pipe_vy[0]    <= st1_voxel_y;
        pipe_vy[1]    <= pipe_vy[0];
        pipe_vy[2]    <= pipe_vy[1];
        pipe_vy[3]    <= pipe_vy[2];
    end

    // Stage 2: 接收数据与累加 (Fetch & Accumulate)
    reg [4*PT_WIDTH-1:0] st2_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_x_acc, sum_y_acc, sum_z_acc;
    wire [         2:0] st2_phase = pipe_phase[1];
    wire                st2_rd_valid = pipe_valid[1];
    wire [VN_WIDTH-1:0] st2_pnum = pipe_pnum[1];
    wire [         4:0] base_idx;
    reg  [        63:0] st2_point_data;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_x_step, sum_y_step, sum_z_step;
    // NOTE: Adder-Tree Type

    assign base_idx = (st2_phase == 3'd1) ? 5'd0 : 5'd10;  // phase 0 时处理前 10 个点，phase 1 时处理后 10 个点
    always @(posedge clk) begin
        if (st2_rd_valid) begin
            // phase 0 时读取前 10 个点 (idx 0~9)，phase 1 时读取后 10 个点 (idx 10~19)
            sum_x_step = 0;
            sum_y_step = 0;
            sum_z_step = 0;

            for (k = 0; k < 10; k = k + 1) begin
                if (base_idx + k < st2_pnum) begin
                    // 从 640 bits 的 BRAM 数据中截取对应的 64 bits 点数据
                    st2_point_data = bram_pfe_rdata[(k*64)+:64];

                    st2_points_buffer[base_idx+k] <= st2_point_data;

                    sum_x_step = sum_x_step + $signed(st2_point_data[63-:16]);
                    sum_y_step = sum_y_step + $signed(st2_point_data[47-:16]);
                    sum_z_step = sum_z_step + $signed(st2_point_data[31-:16]);
                end
            end

            if (st2_phase == 3'd1) begin
                sum_x_acc <= sum_x_step;
                sum_y_acc <= sum_y_step;
                sum_z_acc <= sum_z_step;
            end else begin
                sum_x_acc <= sum_x_acc + sum_x_step;
                sum_y_acc <= sum_y_acc + sum_y_step;
                sum_z_acc <= sum_z_acc + sum_z_step;
            end
        end
    end
    // Stage 3: 数据对齐与准备 (等待均值计算前置)
    // 目标：当 Stage 2 最后一个 Phase 完成后，锁存完整数据，准备喂给 DSP
    // Stage 2 -> Stage 3 完成条件：当前 voxel 两行数据都读完
    wire st2_valid = pipe_valid[3] && (pipe_phase[3] == 3'd1);  // st2的下半有值的周期
    reg  st3_valid;
    reg  st4_valid;
    wire st4_push;
    wire st4_ready_for_new = !st4_valid || st4_push;

    // Stage3 本拍是否会向 Stage4 前进
    wire st3_to_st4 = st3_valid && st4_ready_for_new;

    // Stage3 本拍是否能接收新的 Stage2 输出：
    // - st3 当前空
    // - 或者 st3 当前有效，但本拍会前推给 Stage4
    wire st3_ready_for_s2 = !st3_valid || st3_to_st4;

    // 只有真正写进 Stage3，才算 Stage1/2 里的 inflight voxel “落地”
    assign st2_to_st3 = st2_valid && st3_ready_for_s2;

    reg [VN_WIDTH-1:0] st3_pnum;
    reg [        63:0] st3_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [PT_WIDTH+VN_WIDTH-1:0] st3_sum_x, st3_sum_y, st3_sum_z;
    reg signed [COORD_WIDTH-1:0] st3_vx, st3_vy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st3_valid <= 1'b0;
        end else begin
            // 优先处理“写入新的 Stage3 数据”
            // 如果 st3_to_st4 与 st2_valid 同拍成立，就相当于 Stage3 无缝换包
            if (st2_to_st3) begin
                st3_valid <= 1'b1;
                st3_pnum  <= pipe_pnum[3];
                st3_sum_x <= sum_x_acc;
                st3_sum_y <= sum_y_acc;
                st3_sum_z <= sum_z_acc;
                st3_vx    <= pipe_vx[3];
                st3_vy    <= pipe_vy[3];

                for (i = 0; i < MAX_VOXEL_NUM; i = i + 1) begin
                    st3_points_buffer[i] <= st2_points_buffer[i];
                end
            end else if (st3_to_st4) begin
                // Stage3 数据前推到 Stage4，本拍自己清空
                st3_valid <= 1'b0;
            end
        end
    end


    // DSP 计算区 (纯组合逻辑 + 专用乘法器)
    reg signed [1+17-1:0] st3_reciprocal;  // 1.0 * 65536
    always @(*) begin
        case (st3_pnum)
            5'd1: st3_reciprocal = 18'd65536;
            5'd2: st3_reciprocal = 18'd32768;
            5'd3: st3_reciprocal = 18'd21845;
            5'd4: st3_reciprocal = 18'd16384;
            5'd5: st3_reciprocal = 18'd13107;
            5'd6: st3_reciprocal = 18'd10923;
            5'd7: st3_reciprocal = 18'd9362;
            5'd8: st3_reciprocal = 18'd8192;
            5'd9: st3_reciprocal = 18'd7282;
            5'd10: st3_reciprocal = 18'd6554;
            5'd11: st3_reciprocal = 18'd5958;
            5'd12: st3_reciprocal = 18'd5461;
            5'd13: st3_reciprocal = 18'd5041;
            5'd14: st3_reciprocal = 18'd4681;
            5'd15: st3_reciprocal = 18'd4369;
            5'd16: st3_reciprocal = 18'd4096;
            5'd17: st3_reciprocal = 18'd3855;
            5'd18: st3_reciprocal = 18'd3641;
            5'd19: st3_reciprocal = 18'd3449;
            5'd20: st3_reciprocal = 18'd3277;
            default: st3_reciprocal = 18'd0;
        endcase
    end

    wire signed [(PT_WIDTH+VN_WIDTH)+(17+1) -1:0] temp_mult_x;
    wire signed [(PT_WIDTH+VN_WIDTH)+(17+1) -1:0] temp_mult_y;
    wire signed [(PT_WIDTH+VN_WIDTH)+(17+1) -1:0] temp_mult_z;
    wire signed [        COORD_WIDTH+(17+1) -1:0] temp_mult_voxel_x;
    wire signed [        COORD_WIDTH+(17+1) -1:0] temp_mult_voxel_y;
    wire signed [        COORD_WIDTH+(17+1) -1:0] temp_sub_voxel_x;
    wire signed [        COORD_WIDTH+(17+1) -1:0] temp_sub_voxel_y;

    assign temp_sub_voxel_x = temp_mult_voxel_x - COEFF_OFFSET;  // 注意位宽防止溢出
    assign temp_sub_voxel_y = temp_mult_voxel_y - COEFF_OFFSET;
    // Cluster Center begin
    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY + VN_WIDTH),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (18),
        .WIFB (0),
        .WOI  (39),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_x (
        .ina     (st3_sum_x),
        .inb     (st3_reciprocal),
        .out     (temp_mult_x),
        .overflow()
    );
    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY + VN_WIDTH),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (18),
        .WIFB (0),
        .WOI  (39),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_y (
        .ina     (st3_sum_y),
        .inb     (st3_reciprocal),
        .out     (temp_mult_y),
        .overflow()
    );
    fxp_mul #(
        .WIIA (PT_WIDTH_I_Z + VN_WIDTH),
        .WIFA (PT_WIDTH_F_Z),
        .WIIB (18),
        .WIFB (0),
        .WOI  (39),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_z (
        .ina     (st3_sum_z),
        .inb     (st3_reciprocal),
        .out     (temp_mult_z),
        .overflow()
    );
    // Voxel Center
    fxp_mul #(
        .WIIA (COORD_WIDTH),
        .WIFA (0),
        .WIIB (18),
        .WIFB (0),
        .WOI  (29),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_vx (
        .ina     (st3_vx),
        .inb     (COEFF_SLOPE),
        .out     (temp_mult_voxel_x),
        .overflow()
    );
    fxp_mul #(
        .WIIA (COORD_WIDTH),
        .WIFA (0),
        .WIIB (18),
        .WIFB (0),
        .WOI  (29),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_vy (
        .ina     (st3_vy),
        .inb     (COEFF_SLOPE),
        .out     (temp_mult_voxel_y),
        .overflow()
    );

    // Stage 4: 结果打拍与隔离 (扇出控制核心)
    reg [VN_WIDTH-1:0] st4_pnum;
    reg [        63:0] st4_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [PT_WIDTH-1:0] st4_mean_x, st4_mean_y, st4_mean_z;
    reg signed [PT_WIDTH-1:0] st4_vx_fix16, st4_vy_fix16, st4_vz_fix16;
    reg signed [COORD_WIDTH-1:0] st4_voxel_x, st4_voxel_y;

    // Stage4 -> FIFO：只有 FIFO 不满才能 push
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st4_valid <= 1'b0;
        end else begin
            // 本拍 Stage3 -> Stage4
            if (st3_to_st4) begin
                st4_valid    <= 1'b1;
                st4_pnum     <= st3_pnum;

                // QI.F
                st4_mean_x   <= temp_mult_x[16-PT_WIDTH_F_XY+15:16-PT_WIDTH_F_XY];
                st4_mean_y   <= temp_mult_y[16-PT_WIDTH_F_XY+15:16-PT_WIDTH_F_XY];
                st4_mean_z   <= temp_mult_z[16-PT_WIDTH_F_Z+15:16-PT_WIDTH_F_Z];

                st4_vx_fix16 <= temp_sub_voxel_x[16-PT_WIDTH_F_XY+15:16-PT_WIDTH_F_XY];
                st4_vy_fix16 <= temp_sub_voxel_y[16-PT_WIDTH_F_XY+15:16-PT_WIDTH_F_XY];
                st4_vz_fix16 <= 16'hf000;  // -1.0 Q4.12

                st4_voxel_x  <= st3_vx;
                st4_voxel_y  <= st3_vy;

                for (i = 0; i < MAX_VOXEL_NUM; i = i + 1) begin
                    st4_points_buffer[i] <= st3_points_buffer[i];
                end

            end else if (st4_push) begin
                // 仅 push，没有新的 st3 数据顶进来，则清空 Stage4
                st4_valid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // 重构的 Stage 5: 4-entry Voxel FIFO (LUTRAM 友好型)
    // =========================================================================
    localparam VOX_FIFO_DEPTH = 4;
    localparam VOX_FIFO_AW = 2;  // log2(4)=2

    // 【核心改造 1】计算单个 Voxel 的总位宽，将其展平为 1D 数组
    localparam PT_VEC_W = EXPAND_PT_DIM * PT_WIDTH;  // 11 * 16 = 176 bits
    localparam VOX_VEC_W = MAX_VOXEL_NUM * PT_VEC_W;  // 20 * 176 = 3520 bits

    // 【核心改造 2】强制施加 ram_style 属性。因为深度只有 4，不加此句 Vivado 默认也会用 FF。
    (* ram_style = "distributed" *)reg        [  VOX_VEC_W-1:0] ep_fifo_data                               [0:VOX_FIFO_DEPTH-1];
    (* ram_style = "distributed" *)reg        [   VN_WIDTH-1:0] ep_fifo_pnum                               [0:VOX_FIFO_DEPTH-1];
    (* ram_style = "distributed" *)reg signed [COORD_WIDTH-1:0] ep_fifo_voxel_x                            [0:VOX_FIFO_DEPTH-1];
    (* ram_style = "distributed" *)reg signed [COORD_WIDTH-1:0] ep_fifo_voxel_y                            [0:VOX_FIFO_DEPTH-1];

    reg        [VOX_FIFO_AW-1:0] fifo_wr_ptr;
    reg        [VOX_FIFO_AW-1:0] fifo_rd_ptr;
    reg        [  VOX_FIFO_AW:0] fifo_count;  // 0~4

    wire                         fifo_empty = (fifo_count == 0);
    wire                         fifo_full = (fifo_count == VOX_FIFO_DEPTH);

    assign st4_push = st4_valid && !fifo_full;
    wire voxel_pop;
    wire [3:0] total_occupancy = inflight_cnt + (st3_valid ? 4'd1 : 4'd0) + (st4_valid ? 4'd1 : 4'd0) + fifo_count;
    assign pipe_stall = (total_occupancy >= (VOX_FIFO_DEPTH + 2));

    // always @(posedge clk) begin
    //     if (rst_n && st4_valid && fifo_full && !voxel_pop && st4_ready_for_new) begin
    //         $display("ERROR: Stage4 blocked by full FIFO without backpressure relief.");
    //         // $stop;
    //     end
    // end


    // 每个点预计算 6 路差值
    wire [PT_WIDTH-1:0] sub_x_mean       [0:MAX_VOXEL_NUM-1];
    wire [PT_WIDTH-1:0] sub_y_mean       [0:MAX_VOXEL_NUM-1];
    wire [PT_WIDTH-1:0] sub_z_mean       [0:MAX_VOXEL_NUM-1];
    wire [PT_WIDTH-1:0] sub_x_vcenter    [0:MAX_VOXEL_NUM-1];
    wire [PT_WIDTH-1:0] sub_y_vcenter    [0:MAX_VOXEL_NUM-1];
    wire [PT_WIDTH-1:0] sub_z_vcenter    [0:MAX_VOXEL_NUM-1];

    wire                sub_x_mean_ovf   [0:MAX_VOXEL_NUM-1];
    wire                sub_y_mean_ovf   [0:MAX_VOXEL_NUM-1];
    wire                sub_z_mean_ovf   [0:MAX_VOXEL_NUM-1];
    wire                sub_x_vcenter_ovf[0:MAX_VOXEL_NUM-1];
    wire                sub_y_vcenter_ovf[0:MAX_VOXEL_NUM-1];
    wire                sub_z_vcenter_ovf[0:MAX_VOXEL_NUM-1];

    genvar gi;
    generate
        for (gi = 0; gi < MAX_VOXEL_NUM; gi = gi + 1) begin : GEN_SUB_ALL

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_XY),
                .WIFA (PT_WIDTH_F_XY),
                .WIIB (PT_WIDTH_I_XY),
                .WIFB (PT_WIDTH_F_XY),
                .WOI  (PT_WIDTH_I_XY),
                .WOF  (PT_WIDTH_F_XY),
                .ROUND(1)
            ) u_sub_x_mean (
                .ina     (st4_points_buffer[gi][63-:16]),
                .inb     (st4_mean_x),
                .sub     (1'b1),
                .out     (sub_x_mean[gi]),
                .overflow(sub_x_mean_ovf[gi])
            );

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_XY),
                .WIFA (PT_WIDTH_F_XY),
                .WIIB (PT_WIDTH_I_XY),
                .WIFB (PT_WIDTH_F_XY),
                .WOI  (PT_WIDTH_I_XY),
                .WOF  (PT_WIDTH_F_XY),
                .ROUND(1)
            ) u_sub_y_mean (
                .ina     (st4_points_buffer[gi][47-:16]),
                .inb     (st4_mean_y),
                .sub     (1'b1),
                .out     (sub_y_mean[gi]),
                .overflow(sub_y_mean_ovf[gi])
            );

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_Z),
                .WIFA (PT_WIDTH_F_Z),
                .WIIB (PT_WIDTH_I_Z),
                .WIFB (PT_WIDTH_F_Z),
                .WOI  (PT_WIDTH_I_Z),
                .WOF  (PT_WIDTH_F_Z),
                .ROUND(1)
            ) u_sub_z_mean (
                .ina     (st4_points_buffer[gi][31-:16]),
                .inb     (st4_mean_z),
                .sub     (1'b1),
                .out     (sub_z_mean[gi]),
                .overflow(sub_z_mean_ovf[gi])
            );

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_XY),
                .WIFA (PT_WIDTH_F_XY),
                .WIIB (PT_WIDTH_I_XY),
                .WIFB (PT_WIDTH_F_XY),
                .WOI  (PT_WIDTH_I_XY),
                .WOF  (PT_WIDTH_F_XY),
                .ROUND(1)
            ) u_sub_x_vcenter (
                .ina     (st4_points_buffer[gi][63-:16]),
                .inb     (st4_vx_fix16),
                .sub     (1'b1),
                .out     (sub_x_vcenter[gi]),
                .overflow(sub_x_vcenter_ovf[gi])
            );

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_XY),
                .WIFA (PT_WIDTH_F_XY),
                .WIIB (PT_WIDTH_I_XY),
                .WIFB (PT_WIDTH_F_XY),
                .WOI  (PT_WIDTH_I_XY),
                .WOF  (PT_WIDTH_F_XY),
                .ROUND(1)
            ) u_sub_y_vcenter (
                .ina     (st4_points_buffer[gi][47-:16]),
                .inb     (st4_vy_fix16),
                .sub     (1'b1),
                .out     (sub_y_vcenter[gi]),
                .overflow(sub_y_vcenter_ovf[gi])
            );

            fxp_addsub #(
                .WIIA (PT_WIDTH_I_Z),
                .WIFA (PT_WIDTH_F_Z),
                .WIIB (PT_WIDTH_I_Z),
                .WIFB (PT_WIDTH_F_Z),
                .WOI  (PT_WIDTH_I_Z),
                .WOF  (PT_WIDTH_F_Z),
                .ROUND(1)
            ) u_sub_z_vcenter (
                .ina     (st4_points_buffer[gi][31-:16]),
                .inb     (st4_vz_fix16),
                .sub     (1'b1),
                .out     (sub_z_vcenter[gi]),
                .overflow(sub_z_vcenter_ovf[gi])
            );

        end
    endgenerate

    // 【核心改造 3】使用纯组合逻辑拼接超宽数据位 (3520 bits)
    reg [VOX_VEC_W-1:0] pack_st4_voxel;
    integer pi;
    always @(*) begin
        for (pi = 0; pi < MAX_VOXEL_NUM; pi = pi + 1) begin
            if (pi < st4_pnum) begin
                pack_st4_voxel[pi*PT_VEC_W+:PT_VEC_W] = {
                    st4_points_buffer[pi][63-:16],
                    st4_points_buffer[pi][47-:16],
                    st4_points_buffer[pi][31-:16],
                    st4_points_buffer[pi][15-:16],
                    16'd0,
                    sub_x_mean[pi],
                    sub_y_mean[pi],
                    sub_z_mean[pi],
                    sub_x_vcenter[pi],
                    sub_y_vcenter[pi],
                    sub_z_vcenter[pi]
                };
            end else begin
                pack_st4_voxel[pi*PT_VEC_W+:PT_VEC_W] = {PT_VEC_W{1'b0}};
            end
        end
    end

    // 【核心改造 4】独立的 RAM 写入块（绝对不能含有复位逻辑）
    always @(posedge clk) begin
        if (st4_push) begin
            ep_fifo_data[fifo_wr_ptr]    <= pack_st4_voxel;  // 一次性写入 3520 bit 单一地址
            ep_fifo_pnum[fifo_wr_ptr]    <= st4_pnum;
            ep_fifo_voxel_x[fifo_wr_ptr] <= st4_voxel_x;
            ep_fifo_voxel_y[fifo_wr_ptr] <= st4_voxel_y;
        end
    end

    // 【核心改造 5】指针与状态管理（保留复位，剥离了上面的 Data 逻辑）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wr_ptr <= {VOX_FIFO_AW{1'b0}};
            fifo_rd_ptr <= {VOX_FIFO_AW{1'b0}};
            fifo_count  <= {(VOX_FIFO_AW + 1) {1'b0}};
        end else begin
            case ({
                st4_push, voxel_pop
            })
                2'b10: begin
                    fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                    fifo_count  <= fifo_count + 1'b1;
                end
                2'b01: begin
                    fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
                    fifo_count  <= fifo_count - 1'b1;
                end
                2'b11: begin
                    fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                    fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
                end
                default: ;  // hold
            endcase
        end
    end


    // integer fi;
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         fifo_wr_ptr <= {VOX_FIFO_AW{1'b0}};
    //         fifo_rd_ptr <= {VOX_FIFO_AW{1'b0}};
    //         fifo_count  <= {(VOX_FIFO_AW + 1) {1'b0}};
    //     end else begin
    //         case ({
    //             st4_push, voxel_pop
    //         })
    //             2'b10: begin
    //                 // push only
    //                 ep_fifo_pnum[fifo_wr_ptr]    <= st4_pnum;
    //                 ep_fifo_voxel_x[fifo_wr_ptr] <= st4_voxel_x;
    //                 ep_fifo_voxel_y[fifo_wr_ptr] <= st4_voxel_y;
    //                 for (fi = 0; fi < MAX_VOXEL_NUM; fi = fi + 1) begin
    //                     if (fi < st4_pnum) begin
    //                         ep_fifo[fifo_wr_ptr][fi] <= {
    //                             st4_points_buffer[fi][63-:16],
    //                             st4_points_buffer[fi][47-:16],
    //                             st4_points_buffer[fi][31-:16],
    //                             st4_points_buffer[fi][15-:16],
    //                             16'd0,
    //                             sub_x_mean[fi],
    //                             sub_y_mean[fi],
    //                             sub_z_mean[fi],
    //                             sub_x_vcenter[fi],
    //                             sub_y_vcenter[fi],
    //                             sub_z_vcenter[fi]
    //                         };
    //                     end else begin
    //                         ep_fifo[fifo_wr_ptr][fi] <= {EXPAND_PT_DIM * PT_WIDTH{1'b0}};
    //                     end
    //                 end

    //                 fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
    //                 fifo_count  <= fifo_count + 1'b1;
    //             end

    //             2'b01: begin
    //                 // pop only
    //                 fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
    //                 fifo_count  <= fifo_count - 1'b1;
    //             end

    //             2'b11: begin
    //                 // push + pop 同时发生，count 不变
    //                 ep_fifo_pnum[fifo_wr_ptr]    <= st4_pnum;
    //                 ep_fifo_voxel_x[fifo_wr_ptr] <= st4_voxel_x;
    //                 ep_fifo_voxel_y[fifo_wr_ptr] <= st4_voxel_y;
    //                 for (fi = 0; fi < MAX_VOXEL_NUM; fi = fi + 1) begin
    //                     if (fi < st4_pnum) begin
    //                         ep_fifo[fifo_wr_ptr][fi] <= {
    //                             st4_points_buffer[fi][63-:16],
    //                             st4_points_buffer[fi][47-:16],
    //                             st4_points_buffer[fi][31-:16],
    //                             st4_points_buffer[fi][15-:16],
    //                             16'd0,
    //                             sub_x_mean[fi],
    //                             sub_y_mean[fi],
    //                             sub_z_mean[fi],
    //                             sub_x_vcenter[fi],
    //                             sub_y_vcenter[fi],
    //                             sub_z_vcenter[fi]
    //                         };
    //                     end else begin
    //                         ep_fifo[fifo_wr_ptr][fi] <= {EXPAND_PT_DIM * PT_WIDTH{1'b0}};
    //                     end
    //                 end

    //                 fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
    //                 fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
    //             end

    //             default: begin
    //                 // hold
    //             end
    //         endcase
    //     end
    // end

    // Stage 6: 基于 4-entry FIFO 的序列化输出（look-ahead rd_ptr/count 无缝切换版）
    reg [VN_WIDTH-1:0] point_cnt;
    // 【核心新增 1】判断当前是否允许数据推进一步
    // 逻辑：如果当前没有发出有效数据（!valid），或者下游发来了 ready（可以接收），状态机才往前走
    wire out_advance = (!m_axis_pfe_valid || m_axis_pfe_tready); // 当发中间点且下一个点不是最后一个点时，

    // 【核心改造 6】将当前要读取的 3520-bit 宽字整条拉出，做组合逻辑切片
    wire [VOX_VEC_W-1:0] rd_voxel_data = ep_fifo_data[fifo_rd_ptr];

    // 当前 FIFO 头信息
    wire [VN_WIDTH-1:0] cur_head_pnum = ep_fifo_pnum[fifo_rd_ptr];
    wire fire_out = m_axis_pfe_valid && m_axis_pfe_tready;
    // 当前 busy 态下，这一拍是否正在发送当前 voxel 的最后一个点
    // wire cur_last_fire = serialize_busy && (point_cnt == (cur_head_pnum - {{(VN_WIDTH - 1) {1'b0}}, 1'b1}));
    // assign voxel_pop = cur_last_fire |
    //                (!serialize_busy && !fifo_empty &&
    //                 (ep_fifo_pnum[fifo_rd_ptr] == {{(VN_WIDTH - 1){1'b0}},1'b1}));

    // 【核心修复】FIFO Pop 必须且只能在“当前 Voxel 的最后一个点真实被下游接收”的时刻发生
    assign voxel_pop = fire_out && m_axis_pfe_last;

    wire       [VOX_FIFO_AW-1:0] rd_ptr_after = fifo_rd_ptr + 1'b1;

    // wire [VOX_FIFO_AW:0] fifo_count_after =
    //     fifo_count
    //     + (st4_push ? {{VOX_FIFO_AW{1'b0}},1'b1} : {(VOX_FIFO_AW+1){1'b0}})
    //     - (cur_last_fire ? {{VOX_FIFO_AW{1'b0}},1'b1} : {(VOX_FIFO_AW+1){1'b0}});

    wire       [   VN_WIDTH-1:0] next_point_cnt = point_cnt + 1'b1;
    reg signed [COORD_WIDTH-1:0] voxel_x_test;
    reg signed [COORD_WIDTH-1:0] voxel_y_test;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            voxel_x_test <= {COORD_WIDTH{1'b0}};
            voxel_y_test <= {COORD_WIDTH{1'b0}};
        end else begin
            voxel_x_test <= ep_fifo_voxel_x[fifo_rd_ptr];
            voxel_y_test <= ep_fifo_voxel_y[fifo_rd_ptr];
        end
    end

`ifndef SYNTHESIS
    initial begin
        m_axis_pfe_data <= {EXPAND_PT_DIM * PT_WIDTH{1'b0}};
    end
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            point_cnt              <= {VN_WIDTH{1'b0}};
            m_axis_pfe_valid       <= 1'b0;
            m_axis_pfe_last        <= 1'b0;
            m_axis_pfe_voxel_valid <= 1'b0;
            m_axis_pfe_voxel_x     <= {COORD_WIDTH{1'b0}};
            m_axis_pfe_voxel_y     <= {COORD_WIDTH{1'b0}};
        end else if (out_advance) begin
            if (m_axis_pfe_valid && m_axis_pfe_last) begin
                // 【情况 A】当前 Voxel 的最后一个点刚刚被接收完！
                // 此时 voxel_pop 生效，FIFO 的 rd_ptr 将在此时钟边沿更新。
                // 强制插入 1 拍无效状态，以便等待 FIFO 数据更新并安全读取下一个 Voxel。
                // (这个无效状态正好与 PFN 第二拍的忙碌状态完美重合)
                m_axis_pfe_valid       <= 1'b0;
                m_axis_pfe_last        <= 1'b0;
                m_axis_pfe_voxel_valid <= 1'b0;

            end else if (m_axis_pfe_valid) begin
                // 【情况 B】正在发送 Voxel 的中间点，切换到下一个点
                point_cnt       <= next_point_cnt;
                m_axis_pfe_data <= rd_voxel_data[next_point_cnt*PT_VEC_W+:PT_VEC_W];
                // 提前检查切换到的这个点是否为该 Voxel 的最后一个点
                if (next_point_cnt == (cur_head_pnum - 1'b1)) begin
                    m_axis_pfe_last        <= 1'b1;
                    m_axis_pfe_voxel_valid <= 1'b1;
                    m_axis_pfe_voxel_x     <= ep_fifo_voxel_x[fifo_rd_ptr];
                    m_axis_pfe_voxel_y     <= ep_fifo_voxel_y[fifo_rd_ptr];
                end

            end else begin
                // 【情况 C】当前总线空闲，检查 FIFO 中是否有新 Voxel 任务
                if (!fifo_empty) begin
                    m_axis_pfe_valid   <= 1'b1;
                    m_axis_pfe_data    <= rd_voxel_data[0+:PT_VEC_W];
                    point_cnt          <= {VN_WIDTH{1'b0}};
                    m_axis_pfe_voxel_x <= ep_fifo_voxel_x[fifo_rd_ptr];
                    m_axis_pfe_voxel_y <= ep_fifo_voxel_y[fifo_rd_ptr];

                    if (cur_head_pnum == 1'b1) begin
                        // 单点 Voxel 特例处理
                        m_axis_pfe_last        <= 1'b1;
                        m_axis_pfe_voxel_valid <= 1'b1;
                    end else begin
                        m_axis_pfe_last        <= 1'b0;
                        m_axis_pfe_voxel_valid <= 1'b0;
                    end
                end
            end

        end
    end

    // =========================================================================
    // 帧尾排空信号传递 (Flush Propagation)
    // =========================================================================
    reg  pfe_flushing_req;
    reg  m_axis_pfe_flush_done_reg;

    // PFE 完全排空的条件：
    // 1. inflight_cnt == 0 : Stage 1 和 Stage 2 没有正在读取/累加的数据
    // 2. !st3_valid && !st4_valid : 中间 DSP 流水线段没有数据
    // 3. fifo_empty : 4-entry Voxel FIFO 已全空
    // 4. !m_axis_pfe_valid : 串行化输出总线已闲置
    wire pfe_empty = (inflight_cnt == 3'd0) && !st3_valid && !st4_valid && fifo_empty && !m_axis_pfe_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfe_flushing_req          <= 1'b0;
            m_axis_pfe_flush_done_reg <= 1'b0;
        end else begin
            // 捕获上一级的帧结束标志
            if (flush_done) begin
                pfe_flushing_req <= 1'b1;
            end

            // 当收到清空请求，且整个 PFE 完美排空后，向下一级发出 1-cycle pulse
            if (pfe_flushing_req && pfe_empty && !m_axis_pfe_flush_done_reg) begin
                m_axis_pfe_flush_done_reg <= 1'b1;
                pfe_flushing_req          <= 1'b0;
            end else begin
                m_axis_pfe_flush_done_reg <= 1'b0;
            end
        end
    end

    assign m_axis_pfe_flush_done = m_axis_pfe_flush_done_reg;



`ifndef SYNTHESIS

    // ====================================================================
    // 【独立的仿真测试模块】 抓取并打印特定 Voxel 数据
    // ====================================================================
    // synthesis translate_off
    integer        test_k;
    reg     [63:0] test_point_data;

    always @(posedge clk) begin
        // 当流水线 Stage 2 有效且坐标匹配时触发
        if (rst_n && pipe_valid[1]) begin
            if (pipe_vx[1] == 325 && pipe_vy[1] == 380) begin
                // 遍历当前周期读出的 10 个点
                for (test_k = 0; test_k < 10; test_k = test_k + 1) begin
                    // 确保当前点有效（没有超出该 Voxel 的真实点数）
                    if (base_idx + test_k < pipe_pnum[1]) begin
                        test_point_data = bram_pfe_rdata[(test_k*64)+:64];

                        $display(
                            "[Sim Time: %0t] Voxel(325, 380) HIT! Point Index: %0d | X: %0.3f, Y: %0.3f, Z: %0.3f, Intensity: %0.3f",
                            $time, (base_idx + test_k), $itor($signed(test_point_data[63-:16])) / 256.0, $itor
                            ($signed(test_point_data[47-:16])) / 256.0, $itor($signed(test_point_data[31-:16])) / 256.0,
                            $itor($signed(test_point_data[15-:16])) / 256.0);
                    end
                end
            end
        end
    end
    // synthesis translate_on
    // ====================================================================

`endif

endmodule

