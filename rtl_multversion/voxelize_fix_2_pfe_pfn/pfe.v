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
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / (4 * PT_WIDTH);  // 640 / 64 = 10
    localparam integer EXPAND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  // ceil(20/10)=2
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
    reg                            second_row_pending;
    reg  [                    2:0] req_phase;
    reg                            req_valid;
    reg  [           VN_WIDTH-1:0] st1_point_num;
    reg  [        COORD_WIDTH-1:0] st1_voxel_x;
    reg  [        COORD_WIDTH-1:0] st1_voxel_y;
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] st1_bram_b_addr;
    wire                           axis_need_two_rows = (axis_point_num_indicator > POINTS_PER_ROW);
    wire                           ready_to_accept = !second_row_pending && !pipe_stall;
    assign s_axis_expire_tready = ready_to_accept;
    assign fire_in              = s_axis_expire_tvalid && ready_to_accept;
    // 当前周期是否发起 BRAM 读请求
    wire                           issue_row0 = fire_in;
    wire                           issue_row1 = second_row_pending;
    wire                           issue_valid = issue_row0 || issue_row1;

    // 当前发起的是第几行
    wire                           issue_row_idx = issue_row1;  // 0: row0, 1: row1

    // 当前读请求是不是该 voxel 的最后一行
    wire                           issue_last = issue_row0 ? (!axis_need_two_rows) : 1'b1;

    // 当前读请求对应的元数据
    wire [           VN_WIDTH-1:0] issue_pnum = issue_row0 ? axis_point_num_indicator : st1_point_num;
    wire [        COORD_WIDTH-1:0] issue_vx = issue_row0 ? axis_voxel_x : st1_voxel_x;
    wire [        COORD_WIDTH-1:0] issue_vy = issue_row0 ? axis_voxel_y : st1_voxel_y;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_base_addr = issue_row0 ? axis_bram_index : st1_bram_b_addr;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_addr = issue_base_addr + {{(BRAM_ADDR_WIDTH_PFE - 1) {1'b0}}, issue_row_idx};
    // ------------------------------------------------------------------------
    // 关键点：如果你真的要 BRAM read latency = 1 cycle，
    // bram_pfe_addr 最好不要在 posedge 里打一拍，
    // 而是组合输出给 BRAM，让 BRAM 在当前时钟沿采到地址。
    // ------------------------------------------------------------------------
    always @(*) begin
        if (issue_valid) begin
            bram_pfe_addr = issue_addr;
        end else begin
            bram_pfe_addr = {BRAM_ADDR_WIDTH_PFE{1'b0}};
        end
    end
    // 锁存需要读第二行的 voxel 信息
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            second_row_pending <= 1'b0;
            st1_point_num      <= {VN_WIDTH{1'b0}};
            st1_voxel_x        <= {COORD_WIDTH{1'b0}};
            st1_voxel_y        <= {COORD_WIDTH{1'b0}};
            st1_bram_b_addr    <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
        end else begin
            if (fire_in) begin
                st1_point_num      <= axis_point_num_indicator;
                st1_voxel_x        <= axis_voxel_x;
                st1_voxel_y        <= axis_voxel_y;
                st1_bram_b_addr    <= axis_bram_index;

                // 只有点数超过一行容量时，下一拍才继续读 base+1
                second_row_pending <= axis_need_two_rows;
            end else if (second_row_pending) begin
                // 当前周期已经发起 row1 读请求，下周期清掉
                second_row_pending <= 1'b0;
            end
        end
    end
    // ========================================================================
    // BRAM_B read latency = 1 cycle
    //
    // cycle N   : issue_valid=1, bram_pfe_addr 有效
    // cycle N+1 : bram_pfe_rdata 有效，同时 rd_*_d1 对齐
    // ========================================================================

    reg                   rd_valid_d1;
    reg                   rd_row_idx_d1;
    reg                   rd_last_d1;
    reg [   VN_WIDTH-1:0] rd_pnum_d1;
    reg [COORD_WIDTH-1:0] rd_vx_d1;
    reg [COORD_WIDTH-1:0] rd_vy_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_valid_d1   <= 1'b0;
            rd_row_idx_d1 <= 1'b0;
            rd_last_d1    <= 1'b0;
            rd_pnum_d1    <= {VN_WIDTH{1'b0}};
            rd_vx_d1      <= {COORD_WIDTH{1'b0}};
            rd_vy_d1      <= {COORD_WIDTH{1'b0}};
        end else begin
            rd_valid_d1   <= issue_valid;
            rd_row_idx_d1 <= issue_row_idx;
            rd_last_d1    <= issue_last;
            rd_pnum_d1    <= issue_pnum;
            rd_vx_d1      <= issue_vx;
            rd_vy_d1      <= issue_vy;
        end
    end

    // ========================================================================
    // Stage 2: 接收 BRAM_B 数据并累加
    //
    // 现在 BRAM_B read latency = 1 cycle。
    // rd_valid_d1 有效时，bram_pfe_rdata 就是当前行数据。
    //
    // point_num <= 10 : 只处理 row0，row0 结束后 st2_valid
    // point_num >  10 : 处理 row0 + row1，row1 结束后 st2_valid
    // ========================================================================

    reg        [       4*PT_WIDTH-1:0] st2_points_buffer                              [0:MAX_VOXEL_NUM-1];

    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_x_acc;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_y_acc;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_z_acc;

    wire                               st2_rd_valid = rd_valid_d1;
    wire                               st2_row_idx = rd_row_idx_d1;
    wire                               st2_last_row = rd_last_d1;
    wire       [         VN_WIDTH-1:0] st2_pnum = rd_pnum_d1;
    wire       [      COORD_WIDTH-1:0] st2_vx = rd_vx_d1;
    wire       [      COORD_WIDTH-1:0] st2_vy = rd_vy_d1;

    wire       [                  4:0] base_idx = st2_row_idx ? POINTS_PER_ROW : 5'd0;

    reg        [                 63:0] st2_point_data;

    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_x_step;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_y_step;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] sum_z_step;


    // Stage2 -> Stage3 的输出寄存器
    reg                                st2_valid;
    reg        [         VN_WIDTH-1:0] st2_done_pnum;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] st2_done_sum_x;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] st2_done_sum_y;
    reg signed [PT_WIDTH+VN_WIDTH-1:0] st2_done_sum_z;
    reg signed [      COORD_WIDTH-1:0] st2_done_vx;
    reg signed [      COORD_WIDTH-1:0] st2_done_vy;


    // 注意：st2_to_st3 在后面定义。
    // 如果 Stage3 接收了当前 st2_valid，则清掉 st2_valid。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_x_acc      <= {(PT_WIDTH + VN_WIDTH) {1'b0}};
            sum_y_acc      <= {(PT_WIDTH + VN_WIDTH) {1'b0}};
            sum_z_acc      <= {(PT_WIDTH + VN_WIDTH) {1'b0}};

            st2_valid      <= 1'b0;
            st2_done_pnum  <= {VN_WIDTH{1'b0}};
            st2_done_sum_x <= {(PT_WIDTH + VN_WIDTH) {1'b0}};
            st2_done_sum_y <= {(PT_WIDTH + VN_WIDTH) {1'b0}};
            st2_done_sum_z <= {(PT_WIDTH + VN_WIDTH) {1'b0}};
            st2_done_vx    <= {COORD_WIDTH{1'b0}};
            st2_done_vy    <= {COORD_WIDTH{1'b0}};
        end else begin
            if (st2_to_st3) begin
                st2_valid <= 1'b0;
            end

            if (st2_rd_valid) begin
                sum_x_step = 0;
                sum_y_step = 0;
                sum_z_step = 0;

                for (k = 0; k < POINTS_PER_ROW; k = k + 1) begin
                    if (base_idx + k < st2_pnum) begin
                        st2_point_data = bram_pfe_rdata[(k*64)+:64];

                        st2_points_buffer[base_idx+k] <= st2_point_data;

                        sum_x_step = sum_x_step + $signed(st2_point_data[63-:16]);
                        sum_y_step = sum_y_step + $signed(st2_point_data[47-:16]);
                        sum_z_step = sum_z_step + $signed(st2_point_data[31-:16]);
                    end
                end

                // row0：重新开始累加
                // row1：接着 row0 的累加结果继续加
                if (st2_row_idx == 1'b0) begin
                    sum_x_acc <= sum_x_step;
                    sum_y_acc <= sum_y_step;
                    sum_z_acc <= sum_z_step;
                end else begin
                    sum_x_acc <= sum_x_acc + sum_x_step;
                    sum_y_acc <= sum_y_acc + sum_y_step;
                    sum_z_acc <= sum_z_acc + sum_z_step;
                end

                // 当前行是该 voxel 的最后一行，产生一个完整 voxel 给 Stage3
                if (st2_last_row) begin
                    st2_valid     <= 1'b1;
                    st2_done_pnum <= st2_pnum;
                    st2_done_vx   <= st2_vx;
                    st2_done_vy   <= st2_vy;

                    if (st2_row_idx == 1'b0) begin
                        // 只有一行
                        st2_done_sum_x <= sum_x_step;
                        st2_done_sum_y <= sum_y_step;
                        st2_done_sum_z <= sum_z_step;
                    end else begin
                        // 两行，当前是第二行
                        st2_done_sum_x <= sum_x_acc + sum_x_step;
                        st2_done_sum_y <= sum_y_acc + sum_y_step;
                        st2_done_sum_z <= sum_z_acc + sum_z_step;
                    end
                end
            end
        end
    end


    // Stage 3: 数据对齐与准备 (等待均值计算前置)
    // 目标：当 Stage 2 最后一个 Phase 完成后，锁存完整数据，准备喂给 DSP
    // Stage 2 -> Stage 3 完成条件：当前 voxel 两行数据都读完
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
                st3_pnum  <= st2_done_pnum;
                st3_sum_x <= st2_done_sum_x;
                st3_sum_y <= st2_done_sum_y;
                st3_sum_z <= st2_done_sum_z;
                st3_vx    <= st2_done_vx;
                st3_vy    <= st2_done_vy;

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
    wire [VN_WIDTH-1:0] cur_head_pnum = ep_fifo_pnum[fifo_rd_ptr];
    wire fire_out = m_axis_pfe_valid && m_axis_pfe_tready;
    wire [VOX_FIFO_AW-1:0] rd_ptr_after = fifo_rd_ptr + 1'b1;

    // 当前 busy 态下，这一拍是否正在发送当前 voxel 的最后一个点
    // wire cur_last_fire = serialize_busy && (point_cnt == (cur_head_pnum - {{(VN_WIDTH - 1) {1'b0}}, 1'b1}));
    // assign voxel_pop = cur_last_fire |
    //                (!serialize_busy && !fifo_empty &&
    //                 (ep_fifo_pnum[fifo_rd_ptr] == {{(VN_WIDTH - 1){1'b0}},1'b1}));

    // ============================================================
    // Look-ahead next voxel
    //
    // 用于在当前 voxel last 点被接收后，下一拍直接输出下一个 voxel。
    // ============================================================
    wire [VOX_VEC_W-1:0] rd_voxel_data_next = ep_fifo_data[rd_ptr_after];
    wire [VN_WIDTH-1:0] next_exist_pnum = ep_fifo_pnum[rd_ptr_after];
    wire signed [COORD_WIDTH-1:0] next_exist_voxel_x = ep_fifo_voxel_x[rd_ptr_after];
    wire signed [COORD_WIDTH-1:0] next_exist_voxel_y = ep_fifo_voxel_y[rd_ptr_after];

    // 当前 FIFO 里除了正在输出的 current voxel 之外，是否已经有下一个 voxel
    wire has_existing_next = (fifo_count > 1);

    // 如果当前 FIFO 只有 current voxel，但 Stage4 这一拍正好 push 新 voxel，
    // 那么可以用 bypass 直接输出新 voxel 的 point0，避免多一拍。
    wire has_bypass_next = (fifo_count == 1) && st4_push;
    // 当前 voxel pop 后，下一拍是否有新 voxel 可以继续输出
    wire has_next_after_pop = has_existing_next || has_bypass_next;

    // 选择下一个 voxel 的数据来源：
    // 1. 如果 FIFO 里本来就有下一个 voxel，用 rd_ptr_after 读。
    // 2. 如果是同拍 st4_push 进来的新 voxel，用 pack_st4_voxel bypass。
    wire [VOX_VEC_W-1:0] next_voxel_data = has_existing_next ? rd_voxel_data_next : pack_st4_voxel;
    wire [VN_WIDTH-1:0] next_voxel_pnum = has_existing_next ? next_exist_pnum : st4_pnum;
    wire signed [COORD_WIDTH-1:0] next_voxel_x = has_existing_next ? next_exist_voxel_x : st4_voxel_x;
    wire signed [COORD_WIDTH-1:0] next_voxel_y = has_existing_next ? next_exist_voxel_y : st4_voxel_y;



    // 【核心修复】FIFO Pop 必须且只能在“当前 Voxel 的最后一个点真实被下游接收”的时刻发生
    assign voxel_pop = fire_out && m_axis_pfe_last;
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
        m_axis_pfe_data        <= {EXPAND_PT_DIM * PT_WIDTH{1'b0}};

    end else if (out_advance) begin

        // 默认情况下 voxel_valid 拉低；
        // 只有当前输出的是某 voxel 最后一个点时才拉高。
        m_axis_pfe_voxel_valid <= 1'b0;

        // ================================================================
        // 情况 A：
        // 当前正在输出，并且当前点是该 voxel 最后一个点。
        //
        // 如果这个 last 点被下游接收，则当前 voxel pop。
        // 此时：
        //   - 如果后面还有下一个 voxel，下一拍直接输出下一个 voxel 的 point0
        //   - 如果后面没有，则 valid 拉低
        // ================================================================
        if (m_axis_pfe_valid && m_axis_pfe_last) begin

            if (has_next_after_pop) begin
                // 直接切到下一个 voxel 的 point0，不插入 bubble
                m_axis_pfe_valid   <= 1'b1;
                m_axis_pfe_data    <= next_voxel_data[0+:PT_VEC_W];
                point_cnt          <= {VN_WIDTH{1'b0}};

                m_axis_pfe_voxel_x <= next_voxel_x;
                 m_axis_pfe_voxel_y <= next_voxel_y;

                if (next_voxel_pnum == {{(VN_WIDTH-1){1'b0}}, 1'b1}) begin
                    // 下一个 voxel 只有 1 个点，point0 同时也是 last
                    m_axis_pfe_last        <= 1'b1;
                    m_axis_pfe_voxel_valid <= 1'b1;
                end else begin
                    m_axis_pfe_last        <= 1'b0;
                    m_axis_pfe_voxel_valid <= 1'b0;
                end

            end else begin
                // 没有下一个 voxel，才真正空闲
                m_axis_pfe_valid       <= 1'b0;
                m_axis_pfe_last        <= 1'b0;
                m_axis_pfe_voxel_valid <= 1'b0;
            end

        end

        // ================================================================
        // 情况 B：
        // 当前正在输出一个 voxel 的中间点。
        // 下游 ready 后，切换到下一个点。
        // ================================================================
        else if (m_axis_pfe_valid) begin
            point_cnt       <= next_point_cnt;
            m_axis_pfe_data <= rd_voxel_data[next_point_cnt*PT_VEC_W+:PT_VEC_W];

            m_axis_pfe_voxel_x <= ep_fifo_voxel_x[fifo_rd_ptr];
            m_axis_pfe_voxel_y <= ep_fifo_voxel_y[fifo_rd_ptr];

            if (next_point_cnt == (cur_head_pnum - {{(VN_WIDTH-1){1'b0}}, 1'b1})) begin
                m_axis_pfe_last        <= 1'b1;
                m_axis_pfe_voxel_valid <= 1'b1;
            end else begin
                m_axis_pfe_last        <= 1'b0;
                m_axis_pfe_voxel_valid <= 1'b0;
            end
        end

        // ================================================================
        // 情况 C：
        // 当前输出空闲。
        //
        // 如果 FIFO 里有 voxel，则启动输出。
        // 如果 FIFO 为空但 Stage4 本拍正好 push，也可以 bypass 直接启动输出。
        // ================================================================
        else begin
            if (!fifo_empty) begin
                m_axis_pfe_valid   <= 1'b1;
                m_axis_pfe_data    <= rd_voxel_data[0+:PT_VEC_W];
                point_cnt          <= {VN_WIDTH{1'b0}};

                m_axis_pfe_voxel_x <= ep_fifo_voxel_x[fifo_rd_ptr];
                m_axis_pfe_voxel_y <= ep_fifo_voxel_y[fifo_rd_ptr];

                if (cur_head_pnum == {{(VN_WIDTH-1){1'b0}}, 1'b1}) begin
                    m_axis_pfe_last        <= 1'b1;
                    m_axis_pfe_voxel_valid <= 1'b1;
                end else begin
                    m_axis_pfe_last        <= 1'b0;
                    m_axis_pfe_voxel_valid <= 1'b0;
                end

            end else if (st4_push) begin
                // FIFO 原本为空，但本拍 Stage4 正好推入一个新 voxel。
                // 这里直接 bypass 输出它的 point0。
                m_axis_pfe_valid   <= 1'b1;
                m_axis_pfe_data    <= pack_st4_voxel[0+:PT_VEC_W];
                point_cnt          <= {VN_WIDTH{1'b0}};

                m_axis_pfe_voxel_x <= st4_voxel_x;
                m_axis_pfe_voxel_y <= st4_voxel_y;

                if (st4_pnum == {{(VN_WIDTH-1){1'b0}}, 1'b1}) begin
                    m_axis_pfe_last        <= 1'b1;
                    m_axis_pfe_voxel_valid <= 1'b1;
                end else begin
                    m_axis_pfe_last        <= 1'b0;
                    m_axis_pfe_voxel_valid <= 1'b0;
                end

            end else begin
                m_axis_pfe_valid       <= 1'b0;
                m_axis_pfe_last        <= 1'b0;
                m_axis_pfe_voxel_valid <= 1'b0;
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
        if (rst_n && st2_rd_valid) begin
            if (st2_vx == 325 && st2_vy == 380) begin
                for (test_k = 0; test_k < POINTS_PER_ROW; test_k = test_k + 1) begin
                    if (base_idx + test_k < st2_pnum) begin
                        test_point_data = bram_pfe_rdata[(test_k*64)+:64];

                        $display(
                            "[Sim Time: %0t] Voxel(325, 380) HIT! Point Index: %0d | X: %0.3f, Y: %0.3f, Z: %0.3f, Intensity: %0.3f",
                            $time, (base_idx + test_k), $itor($signed(test_point_data[63-:16])) / 256.0, $itor
                            ($signed(test_point_data[47-:16])) / 256.0, $itor($signed(test_point_data[31-:16])) / 4096.0,
                            $itor($signed(test_point_data[15-:16])) / 128.0);
                    end
                end
            end
        end
    end
    // synthesis translate_on
    // ====================================================================

`endif

endmodule

