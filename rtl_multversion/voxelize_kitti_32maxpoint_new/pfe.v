
module pfe #(
    parameter COORD_WIDTH = 11,
    parameter PT_WIDTH = 16,
    parameter PT_WIDTH_PER = 72,
    // parameter HASH_ADDR_WIDTH = 8,  // 256
    parameter MAX_VOXEL_NUM = 12'd100,
    parameter VN_WIDTH = 7,
    parameter EXPAND_PT_DIM = 9,  // PFN interface remains 9*16: {x_vcenter,y_vcenter,z,intensity,x_cluster,y_cluster,z_cluster,x_vcenter,y_vcenter}
    parameter integer BRAM_DATA_WIDTH = 576,  // 72bit * 8 points per row
    parameter integer BRAM_ADDR_WIDTH = 10,  // 256pillar * 2brams
    parameter integer BRAM_ADDR_WIDTH_PFE = 12
) (
    input wire clk,
    input wire rst_n,
    input wire s_axis_expire_tvalid,
    output wire s_axis_expire_tready,
    input wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] s_axis_expire_tdata,  // NOTE
    // pfe
    // output wire                                       bram_pfe_rst,
    output reg [BRAM_ADDR_WIDTH_PFE-1:0] bram_pfe_addr,  // 假设地址空间足够大
    // output reg                                        bram_pfe_en,
    input  wire       [                           BRAM_DATA_WIDTH-1:0] bram_pfe_rdata,          // 72bit point: {x[19:0] Q8.12, y[19:0] Q8.12, z[15:0] Q4.12, intensity[15:0] Q1.15}
    input wire m_axis_pfe_tready,
    output reg m_axis_pfe_valid,
    output reg [EXPAND_PT_DIM*PT_WIDTH-1:0] m_axis_pfe_data,  // 9*16 bits to PFN
    output reg m_axis_pfe_last,
    output reg m_axis_pfe_voxel_valid,
    output reg signed [COORD_WIDTH-1:0] m_axis_pfe_voxel_x,
    output reg signed [COORD_WIDTH-1:0] m_axis_pfe_voxel_y,
    output wire m_axis_pfe_flush_done,  // <--- 新增
    input wire flush_done
);
    // 注意：Verilog 切片最好用固定数值，或者确保 parameter 计算正确
    wire signed [COORD_WIDTH-1:0] axis_voxel_x = s_axis_expire_tdata[(2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire signed [COORD_WIDTH-1:0] axis_voxel_y = s_axis_expire_tdata[(COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire [BRAM_ADDR_WIDTH_PFE-1:0] axis_bram_index = s_axis_expire_tdata[(BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:BRAM_ADDR_WIDTH_PFE];
    wire [VN_WIDTH-1:0] axis_point_num_indicator = s_axis_expire_tdata[(VN_WIDTH-1) : 0];  // 1-32 points

    // -----------------------------------------------------------
    // 2. 内部存储 (Points Buffer) & 累加器
    // -----------------------------------------------------------
    // 存储该 Voxel 内的所有点，供后续处理 (Process 阶段) 使用
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  // 576 / 72 = 8
    // BRAM_B 中每个逻辑 voxel 固定保留 EXPAND_VOXEL_ROW 行；100 点时为 13 行。
    localparam integer EXPAND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;
    localparam integer ROW_IDX_WIDTH = (EXPAND_VOXEL_ROW <= 2) ? 1 : $clog2(EXPAND_VOXEL_ROW);
    localparam [VN_WIDTH-1:0] POINTS_PER_ROW_V = POINTS_PER_ROW;

    // BRAM point format: {x_q812[19:0], y_q812[19:0], z_q412[15:0], intensity_q115[15:0]} = 72 bit.
    // PFN interface remains 9 * 16 bit. x/y derived features are converted to signed Q1.15.
    localparam integer PT_WIDTH_XY = 20;
    localparam integer PT_WIDTH_Z = 16;
    localparam integer PT_WIDTH_IS = 16;
    localparam integer PT_WIDTH_I_XY = 8;
    localparam integer PT_WIDTH_F_XY = 12;
    localparam integer PT_WIDTH_I_XY_OUT = 1;
    localparam integer PT_WIDTH_F_XY_OUT = 15;
    localparam integer PT_WIDTH_I_Z = 4;
    localparam integer PT_WIDTH_F_Z = 12;
    localparam integer PT_WIDTH_I_IS = 1;
    localparam integer PT_WIDTH_F_IS = 15;

    localparam integer SUM_XY_WIDTH = PT_WIDTH_XY + VN_WIDTH;
    localparam integer SUM_Z_WIDTH = PT_WIDTH_Z + VN_WIDTH;

    function signed [PT_WIDTH-1:0] q812_to_q115_sat;
        input signed [PT_WIDTH_XY-1:0] val_q812;
        reg signed [PT_WIDTH_XY+3-1:0] val_ext;
        reg signed [PT_WIDTH_XY+3-1:0] val_q115_wide;
        begin
            // Q8.12 -> Q1.15: multiply raw integer by 2^(15-12) = 8.
            // Sign-extend first; otherwise a 20-bit left-shift would wrap before saturation.
            val_ext       = {{3{val_q812[PT_WIDTH_XY-1]}}, val_q812};
            val_q115_wide = val_ext <<< (PT_WIDTH_F_XY_OUT - PT_WIDTH_F_XY);
            if (val_q115_wide > 23'sd32767) begin
                q812_to_q115_sat = 16'sh7fff;
            end else if (val_q115_wide < -23'sd32768) begin
                q812_to_q115_sat = 16'sh8000;
            end else begin
                q812_to_q115_sat = val_q115_wide[PT_WIDTH-1:0];
            end
        end
    endfunction

    // --------------------扩展voxel_center点坐标维度---------------------
    // voxel_x_fix16 = voxel_x * 0.16 - 0.0 + 0.08
    localparam integer ST3_SCALE = 1 + 17;
    localparam signed [ST3_SCALE-1:0] COEFF_SLOPE = 18'sd10486;  // 0.16
    localparam signed [ST3_SCALE-1:0] COEFF_OFFSET_X = 18'sd5243;  // +0.08 (-0.000 + 0.08 offset) in Q16
    localparam signed [ST3_SCALE+$clog2(40)-1:0] COEFF_OFFSET_Y = 24'sd2595226;  // -39.60 (-39.68 + 0.08 offset) in Q16

    // -----------------------------------------------------------
    // 源头安全反压控制 (In-flight tracking)
    // -----------------------------------------------------------
    wire       pfe_frame_clear;  // 整个 PFE 内部的“软复位”，在每帧结束时由外部控制信号触发，清空所有 inflight voxel 的状态。
    reg [2:0] inflight_cnt;
    wire fire_in;

    // Stage2 完成一个 voxel，并成功写入 Stage3
    wire st2_to_st3;

    // 统一的容量模型：
    // inflight_cnt  : Stage1/2 中已经接收、但还没落到 Stage3 的 voxel 数
    // 取消 ep_fifo 后，总容量只剩 Stage3 + Stage4 两个完整 pillar 槽位。
    wire pipe_stall;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inflight_cnt <= 3'd0;

        end else if (pfe_frame_clear) begin
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
    // Stage 1: 地址生成流水线 (Address Generation)
    // 支持每个 voxel 读取 1..13 行。
    reg more_row_pending;
    reg [ROW_IDX_WIDTH-1:0] next_row_idx;
    reg [ROW_IDX_WIDTH:0] st1_total_rows;
    reg [VN_WIDTH-1:0] st1_point_num;
    reg [COORD_WIDTH-1:0] st1_voxel_x;
    reg [COORD_WIDTH-1:0] st1_voxel_y;
    reg [BRAM_ADDR_WIDTH_PFE-1:0] st1_bram_b_addr;

    wire [ROW_IDX_WIDTH:0] axis_need_rows = (axis_point_num_indicator + POINTS_PER_ROW_V - 1'b1) / POINTS_PER_ROW;
    wire ready_to_accept = !more_row_pending && !pipe_stall;
    assign s_axis_expire_tready = ready_to_accept;
    assign fire_in              = s_axis_expire_tvalid && ready_to_accept;

    // 当前周期是否发起 BRAM 读请求
    wire issue_row0 = fire_in;
    wire issue_more_row = more_row_pending;
    wire issue_valid = issue_row0 || issue_more_row;

    // 当前发起的是第几行：0/1/2
    wire [ROW_IDX_WIDTH-1:0] issue_row_idx = issue_row0 ? {ROW_IDX_WIDTH{1'b0}} : next_row_idx;

    // 当前读请求是不是该 voxel 的最后一行
    wire issue_last = issue_row0 ? (axis_need_rows == {{ROW_IDX_WIDTH{1'b0}}, 1'b1}) : (next_row_idx == (st1_total_rows - 1'b1));

    // 当前读请求对应的元数据
    wire [VN_WIDTH-1:0] issue_pnum = issue_row0 ? axis_point_num_indicator : st1_point_num;
    wire [COORD_WIDTH-1:0] issue_vx = issue_row0 ? axis_voxel_x : st1_voxel_x;
    wire [COORD_WIDTH-1:0] issue_vy = issue_row0 ? axis_voxel_y : st1_voxel_y;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_base_addr = issue_row0 ? axis_bram_index : st1_bram_b_addr;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_addr =
        issue_base_addr + {{(BRAM_ADDR_WIDTH_PFE - ROW_IDX_WIDTH) {1'b0}}, issue_row_idx};
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
    // 锁存需要继续读 row1/row2 的 voxel 信息
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            more_row_pending <= 1'b0;
            next_row_idx     <= {ROW_IDX_WIDTH{1'b0}};
            st1_total_rows   <= 3'd0;

        end else if (pfe_frame_clear) begin
            more_row_pending <= 1'b0;
            next_row_idx     <= {ROW_IDX_WIDTH{1'b0}};
            st1_total_rows   <= 3'd0;

        end else begin
            if (fire_in) begin
                st1_point_num   <= axis_point_num_indicator;
                st1_voxel_x     <= axis_voxel_x;
                st1_voxel_y     <= axis_voxel_y;
                st1_bram_b_addr <= axis_bram_index;
                st1_total_rows  <= axis_need_rows;

                if (axis_need_rows == {{ROW_IDX_WIDTH{1'b0}}, 1'b1}) begin
                    more_row_pending <= 1'b0;
                    next_row_idx     <= {ROW_IDX_WIDTH{1'b0}};
                end else begin
                    more_row_pending <= 1'b1;
                    next_row_idx     <= {{(ROW_IDX_WIDTH - 1) {1'b0}}, 1'b1};
                end
            end else if (more_row_pending) begin
                if (next_row_idx == (st1_total_rows - 1'b1)) begin
                    // 当前周期已经发起最后一行读请求，下周期清掉
                    more_row_pending <= 1'b0;
                    next_row_idx     <= {ROW_IDX_WIDTH{1'b0}};
                end else begin
                    // 当前周期发起 row1，下一周期继续 row2
                    next_row_idx <= next_row_idx + 1'b1;
                end
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
    reg [ROW_IDX_WIDTH-1:0] rd_row_idx_d1;
    reg                   rd_last_d1;
    reg [   VN_WIDTH-1:0] rd_pnum_d1;
    reg [COORD_WIDTH-1:0] rd_vx_d1;
    reg [COORD_WIDTH-1:0] rd_vy_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_valid_d1   <= 1'b0;
            rd_row_idx_d1 <= {ROW_IDX_WIDTH{1'b0}};
            rd_last_d1    <= 1'b0;

        end else if (pfe_frame_clear) begin
            rd_valid_d1   <= 1'b0;
            rd_row_idx_d1 <= {ROW_IDX_WIDTH{1'b0}};
            rd_last_d1    <= 1'b0;

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
    // point_num <= 8  : 只处理 row0，row0 结束后 st2_valid
    // point_num <= 16 : 处理 row0 + row1，row1 结束后 st2_valid
    // point_num <= 24 : 处理 row0 + row1 + row2，row2 结束后 st2_valid
    // point_num <= 32 : 处理 row0 + row1 + row2 + row3，row3 结束后 st2_valid
    // ========================================================================

    reg        [PT_WIDTH_PER-1:0] st2_points_buffer                       [0:MAX_VOXEL_NUM-1];

    reg signed [SUM_XY_WIDTH-1:0] sum_x_acc;
    reg signed [SUM_XY_WIDTH-1:0] sum_y_acc;
    reg signed [ SUM_Z_WIDTH-1:0] sum_z_acc;

    wire                          st2_rd_valid = rd_valid_d1;
    wire       [ROW_IDX_WIDTH-1:0] st2_row_idx = rd_row_idx_d1;
    wire                          st2_last_row = rd_last_d1;
    wire       [    VN_WIDTH-1:0] st2_pnum = rd_pnum_d1;
    wire       [ COORD_WIDTH-1:0] st2_vx = rd_vx_d1;
    wire       [ COORD_WIDTH-1:0] st2_vy = rd_vy_d1;

    wire       [    VN_WIDTH-1:0] base_idx = st2_row_idx * POINTS_PER_ROW;

    reg        [PT_WIDTH_PER-1:0] st2_point_data;

    reg signed [SUM_XY_WIDTH-1:0] sum_x_step;
    reg signed [SUM_XY_WIDTH-1:0] sum_y_step;
    reg signed [ SUM_Z_WIDTH-1:0] sum_z_step;


    // Stage2 -> Stage3 的输出寄存器
    reg                           st2_valid;
    reg        [    VN_WIDTH-1:0] st2_done_pnum;
    reg signed [SUM_XY_WIDTH-1:0] st2_done_sum_x;
    reg signed [SUM_XY_WIDTH-1:0] st2_done_sum_y;
    reg signed [ SUM_Z_WIDTH-1:0] st2_done_sum_z;
    reg signed [ COORD_WIDTH-1:0] st2_done_vx;
    reg signed [ COORD_WIDTH-1:0] st2_done_vy;


    // 注意：st2_to_st3 在后面定义。
    // 如果 Stage3 接收了当前 st2_valid，则清掉 st2_valid。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st2_valid <= 1'b0;

        end else if (pfe_frame_clear) begin
            st2_valid <= 1'b0;

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
                        st2_point_data = bram_pfe_rdata[(k*PT_WIDTH_PER)+:PT_WIDTH_PER];

                        st2_points_buffer[base_idx+k] <= st2_point_data;

                        sum_x_step = sum_x_step + $signed(st2_point_data[71-:20]);
                        sum_y_step = sum_y_step + $signed(st2_point_data[51-:20]);
                        sum_z_step = sum_z_step + $signed(st2_point_data[31-:16]);
                    end
                end

                // row0：重新开始累加
                // row1/row2：接着前面行的累加结果继续加
                if (st2_row_idx == {ROW_IDX_WIDTH{1'b0}}) begin
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

                    if (st2_row_idx == {ROW_IDX_WIDTH{1'b0}}) begin
                        // 只有一行
                        st2_done_sum_x <= sum_x_step;
                        st2_done_sum_y <= sum_y_step;
                        st2_done_sum_z <= sum_z_step;
                    end else begin
                        // 多行，当前是最后一行：sum_x_acc/y/z_acc 已经包含前面所有行
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
    // Stage 2 -> Stage 3 完成条件：当前 voxel 的有效行数据都读完
    reg  st3_valid;
    reg  st4_valid;

    // FIFO 已取消：Stage4 直接作为当前待输出 pillar 的 holding buffer。
    // 只有当前 pillar 的 last 点真正被 PFN 接收后，Stage4 才允许释放/接收下一包。
    wire fire_out = m_axis_pfe_valid && m_axis_pfe_tready;
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
    reg [PT_WIDTH_PER-1:0] st3_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [SUM_XY_WIDTH-1:0] st3_sum_x, st3_sum_y;
    reg signed [SUM_Z_WIDTH-1:0] st3_sum_z;
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
    reg signed [ST3_SCALE-1:0] st3_reciprocal;  // 1.0 * 65536
    always @(*) begin
        case (st3_pnum)
            7'd1: st3_reciprocal = 18'd65536;
            7'd2: st3_reciprocal = 18'd32768;
            7'd3: st3_reciprocal = 18'd21845;
            7'd4: st3_reciprocal = 18'd16384;
            7'd5: st3_reciprocal = 18'd13107;
            7'd6: st3_reciprocal = 18'd10923;
            7'd7: st3_reciprocal = 18'd9362;
            7'd8: st3_reciprocal = 18'd8192;
            7'd9: st3_reciprocal = 18'd7282;
            7'd10: st3_reciprocal = 18'd6554;
            7'd11: st3_reciprocal = 18'd5958;
            7'd12: st3_reciprocal = 18'd5461;
            7'd13: st3_reciprocal = 18'd5041;
            7'd14: st3_reciprocal = 18'd4681;
            7'd15: st3_reciprocal = 18'd4369;
            7'd16: st3_reciprocal = 18'd4096;
            7'd17: st3_reciprocal = 18'd3855;
            7'd18: st3_reciprocal = 18'd3641;
            7'd19: st3_reciprocal = 18'd3449;
            7'd20: st3_reciprocal = 18'd3277;
            7'd21: st3_reciprocal = 18'd3121;
            7'd22: st3_reciprocal = 18'd2979;
            7'd23: st3_reciprocal = 18'd2849;
            7'd24: st3_reciprocal = 18'd2731;
            7'd25: st3_reciprocal = 18'd2621;
            7'd26: st3_reciprocal = 18'd2521;
            7'd27: st3_reciprocal = 18'd2427;
            7'd28: st3_reciprocal = 18'd2341;
            7'd29: st3_reciprocal = 18'd2260;
            7'd30: st3_reciprocal = 18'd2185;
            7'd31: st3_reciprocal = 18'd2114;
            7'd32: st3_reciprocal = 18'd2048;
            7'd33: st3_reciprocal = 18'd1986;
            7'd34: st3_reciprocal = 18'd1928;
            7'd35: st3_reciprocal = 18'd1872;
            7'd36: st3_reciprocal = 18'd1820;
            7'd37: st3_reciprocal = 18'd1771;
            7'd38: st3_reciprocal = 18'd1725;
            7'd39: st3_reciprocal = 18'd1680;
            7'd40: st3_reciprocal = 18'd1638;
            7'd41: st3_reciprocal = 18'd1598;
            7'd42: st3_reciprocal = 18'd1560;
            7'd43: st3_reciprocal = 18'd1524;
            7'd44: st3_reciprocal = 18'd1489;
            7'd45: st3_reciprocal = 18'd1456;
            7'd46: st3_reciprocal = 18'd1425;
            7'd47: st3_reciprocal = 18'd1394;
            7'd48: st3_reciprocal = 18'd1365;
            7'd49: st3_reciprocal = 18'd1337;
            7'd50: st3_reciprocal = 18'd1311;
            7'd51: st3_reciprocal = 18'd1285;
            7'd52: st3_reciprocal = 18'd1260;
            7'd53: st3_reciprocal = 18'd1237;
            7'd54: st3_reciprocal = 18'd1214;
            7'd55: st3_reciprocal = 18'd1192;
            7'd56: st3_reciprocal = 18'd1170;
            7'd57: st3_reciprocal = 18'd1150;
            7'd58: st3_reciprocal = 18'd1130;
            7'd59: st3_reciprocal = 18'd1111;
            7'd60: st3_reciprocal = 18'd1092;
            7'd61: st3_reciprocal = 18'd1074;
            7'd62: st3_reciprocal = 18'd1057;
            7'd63: st3_reciprocal = 18'd1040;
            7'd64: st3_reciprocal = 18'd1024;
            7'd65: st3_reciprocal = 18'd1008;
            7'd66: st3_reciprocal = 18'd993;
            7'd67: st3_reciprocal = 18'd978;
            7'd68: st3_reciprocal = 18'd964;
            7'd69: st3_reciprocal = 18'd950;
            7'd70: st3_reciprocal = 18'd936;
            7'd71: st3_reciprocal = 18'd923;
            7'd72: st3_reciprocal = 18'd910;
            7'd73: st3_reciprocal = 18'd898;
            7'd74: st3_reciprocal = 18'd886;
            7'd75: st3_reciprocal = 18'd874;
            7'd76: st3_reciprocal = 18'd862;
            7'd77: st3_reciprocal = 18'd851;
            7'd78: st3_reciprocal = 18'd840;
            7'd79: st3_reciprocal = 18'd830;
            7'd80: st3_reciprocal = 18'd819;
            7'd81: st3_reciprocal = 18'd809;
            7'd82: st3_reciprocal = 18'd799;
            7'd83: st3_reciprocal = 18'd790;
            7'd84: st3_reciprocal = 18'd780;
            7'd85: st3_reciprocal = 18'd771;
            7'd86: st3_reciprocal = 18'd762;
            7'd87: st3_reciprocal = 18'd753;
            7'd88: st3_reciprocal = 18'd745;
            7'd89: st3_reciprocal = 18'd736;
            7'd90: st3_reciprocal = 18'd728;
            7'd91: st3_reciprocal = 18'd720;
            7'd92: st3_reciprocal = 18'd712;
            7'd93: st3_reciprocal = 18'd705;
            7'd94: st3_reciprocal = 18'd697;
            7'd95: st3_reciprocal = 18'd690;
            7'd96: st3_reciprocal = 18'd683;
            7'd97: st3_reciprocal = 18'd676;
            7'd98: st3_reciprocal = 18'd669;
            7'd99: st3_reciprocal = 18'd662;
            7'd100: st3_reciprocal = 18'd655;
            default: st3_reciprocal = 18'd0;
        endcase
    end

    wire signed [SUM_XY_WIDTH+ST3_SCALE-1:0] temp_mult_x;
    wire signed [SUM_XY_WIDTH+ST3_SCALE-1:0] temp_mult_y;
    wire signed [SUM_Z_WIDTH +ST3_SCALE-1:0] temp_mult_z;
    wire signed [COORD_WIDTH+ST3_SCALE -1:0] temp_mult_voxel_x;
    wire signed [COORD_WIDTH+ST3_SCALE -1:0] temp_mult_voxel_y;
    wire signed [COORD_WIDTH+ST3_SCALE -1:0] temp_sub_voxel_x;
    wire signed [COORD_WIDTH+ST3_SCALE -1:0] temp_sub_voxel_y;

    wire signed [           PT_WIDTH_XY-1:0] meanx_center_q812;
    wire signed [           PT_WIDTH_XY-1:0] meany_center_q812;
    wire signed [            PT_WIDTH_Z-1:0] meanz_center_q412;

    assign meanx_center_q812 = temp_mult_x[16-PT_WIDTH_F_XY+PT_WIDTH_XY-1:16-PT_WIDTH_F_XY];
    assign meany_center_q812 = temp_mult_y[16-PT_WIDTH_F_XY+PT_WIDTH_XY-1:16-PT_WIDTH_F_XY];
    assign meanz_center_q412 = temp_mult_z[16-PT_WIDTH_F_Z+PT_WIDTH_Z-1:16-PT_WIDTH_F_Z];

    // Cluster Center begin
    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY + VN_WIDTH),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (ST3_SCALE),
        .WIFB (0),
        .WOI  (SUM_XY_WIDTH + ST3_SCALE),
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
        .WIIB (ST3_SCALE),
        .WIFB (0),
        .WOI  (SUM_XY_WIDTH + ST3_SCALE),
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
        .WIIB (ST3_SCALE),
        .WIFB (0),
        .WOI  (SUM_Z_WIDTH + ST3_SCALE),
        .WOF  (0),
        .ROUND(1)
    ) fxp_mul_z (
        .ina     (st3_sum_z),
        .inb     (st3_reciprocal),
        .out     (temp_mult_z),
        .overflow()
    );
    // Voxel Center
    // ============================================================
    // Voxel center fast constant multiply
    // COEFF_SLOPE = 10486 = 2^13 + 2^11 + 2^8 - 2^4 + 2^2 + 2^1
    // temp_mult_voxel_* keeps the same Q16 meaning as old fxp_mul_vx/vy.
    // ============================================================

    wire signed [COORD_WIDTH+ST3_SCALE-1:0] st3_vx_ext;
    wire signed [COORD_WIDTH+ST3_SCALE-1:0] st3_vy_ext;

    assign st3_vx_ext = {{ST3_SCALE{st3_vx[COORD_WIDTH-1]}}, st3_vx};
    assign st3_vy_ext = {{ST3_SCALE{st3_vy[COORD_WIDTH-1]}}, st3_vy};

    wire signed [COORD_WIDTH+ST3_SCALE-1:0] temp_mult_voxel_x_fast;
    wire signed [COORD_WIDTH+ST3_SCALE-1:0] temp_mult_voxel_y_fast;

    assign temp_mult_voxel_x_fast = (st3_vx_ext <<< 13) + (st3_vx_ext <<< 11) + (st3_vx_ext <<< 8) - (st3_vx_ext <<< 4) + (st3_vx_ext <<< 2) + (st3_vx_ext <<< 1);

    assign temp_mult_voxel_y_fast = (st3_vy_ext <<< 13) + (st3_vy_ext <<< 11) + (st3_vy_ext <<< 8) - (st3_vy_ext <<< 4) + (st3_vy_ext <<< 2) + (st3_vy_ext <<< 1);

    assign temp_sub_voxel_x = temp_mult_voxel_x_fast + COEFF_OFFSET_X;
    assign temp_sub_voxel_y = temp_mult_voxel_y_fast - COEFF_OFFSET_Y;
    wire signed [PT_WIDTH_XY-1:0] vx_center_q812;
    wire signed [PT_WIDTH_XY-1:0] vy_center_q812;

    assign vx_center_q812 = temp_sub_voxel_x[16-PT_WIDTH_F_XY+PT_WIDTH_XY-1:16-PT_WIDTH_F_XY];
    assign vy_center_q812 = temp_sub_voxel_y[16-PT_WIDTH_F_XY+PT_WIDTH_XY-1:16-PT_WIDTH_F_XY];

    // Stage 4: 结果打拍与隔离 (扇出控制核心)
    reg [VN_WIDTH-1:0] st4_pnum;
    reg [PT_WIDTH_PER-1:0] st4_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [PT_WIDTH_XY-1:0] st4_mean_x, st4_mean_y;
    reg signed [PT_WIDTH_Z-1:0] st4_mean_z;
    reg signed [PT_WIDTH_XY-1:0] st4_vx_fixxy, st4_vy_fixxy;
    reg signed [COORD_WIDTH-1:0] st4_voxel_x, st4_voxel_y;

    // Stage4 现在直接作为输出 holding buffer；st4_push 表示当前 pillar 已完整输出。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st4_valid <= 1'b0;

        end else if (pfe_frame_clear) begin
            st4_valid <= 1'b0;

        end else begin
            // 本拍 Stage3 -> Stage4
            if (st3_to_st4) begin
                st4_valid    <= 1'b1;
                st4_pnum     <= st3_pnum;

                // QI.F
                st4_mean_x   <= meanx_center_q812;
                st4_mean_y   <= meany_center_q812;
                st4_mean_z   <= meanz_center_q412;

                st4_vx_fixxy <= vx_center_q812;
                st4_vy_fixxy <= vy_center_q812;

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
    // Stage 5: Optimized current-point feature generation
    // =========================================================================
    // 优化点：
    // 1) 不再对 32 个点并行实例化 fxp_addsub；
    // 2) 不再生成 3584bit pack_st4_voxel；
    // 3) serializer 要输出哪个点，就只对当前 point_cnt 对应的 1 个点做特征计算。
    //
    // 输出格式保持不变：
    // {x_vcenter, y_vcenter, z, intensity, x_cluster, y_cluster, z_cluster, x_vcenter, y_vcenter}
    localparam PT_VEC_W = EXPAND_PT_DIM * PT_WIDTH;  // 9 * 16 = 144 bits, PFN interface

    // 定制 Q8.12 - Q8.12，替代通用 fxp_addsub。
    // 行为等价于先扩展到更宽 signed diff，再饱和回 20-bit signed Q8.12。
    function signed [PT_WIDTH_XY-1:0] sub_q812_sat;
        input signed [PT_WIDTH_XY-1:0] a_q812;
        input signed [PT_WIDTH_XY-1:0] b_q812;
        reg signed [21:0] diff_q812;
        begin
            diff_q812 = {{2{a_q812[PT_WIDTH_XY-1]}}, a_q812} - {{2{b_q812[PT_WIDTH_XY-1]}}, b_q812};

            if (diff_q812 > 22'sd524287) begin
                sub_q812_sat = 20'sh7ffff;
            end else if (diff_q812 < -22'sd524288) begin
                sub_q812_sat = 20'sh80000;
            end else begin
                sub_q812_sat = diff_q812[PT_WIDTH_XY-1:0];
            end
        end
    endfunction

    // 定制 Q4.12 - Q4.12，替代通用 fxp_addsub。
    // 饱和回 16-bit signed Q4.12。
    function signed [PT_WIDTH_Z-1:0] sub_q412_sat;
        input signed [PT_WIDTH_Z-1:0] a_q412;
        input signed [PT_WIDTH_Z-1:0] b_q412;
        reg signed [17:0] diff_q412;
        begin
            diff_q412 = {{2{a_q412[PT_WIDTH_Z-1]}}, a_q412} - {{2{b_q412[PT_WIDTH_Z-1]}}, b_q412};

            if (diff_q412 > 18'sd32767) begin
                sub_q412_sat = 16'sh7fff;
            end else if (diff_q412 < -18'sd32768) begin
                sub_q412_sat = 16'sh8000;
            end else begin
                sub_q412_sat = diff_q412[PT_WIDTH_Z-1:0];
            end
        end
    endfunction

    // FIFO 取消后，Stage4 本身就是唯一输出 holding buffer。
    // st4_push 的含义从“写入 FIFO”改为“当前 Stage4 pillar 已被完整发送”。
    // 注意：不能在 point0 装载到 m_axis_pfe_data 的同拍释放 Stage4；必须等 valid&&ready&&last。
    assign st4_push = st4_valid && fire_out && m_axis_pfe_last;

    // 无 FIFO 后总容量只剩 Stage3 + Stage4 两个完整 pillar 槽位。
    // inflight_cnt 仍代表 Stage1/2 中已接收但尚未落到 Stage3 的 pillar。
    wire [3:0] total_occupancy = inflight_cnt + (st3_valid ? 4'd1 : 4'd0) + (st4_valid ? 4'd1 : 4'd0);
    assign pipe_stall = (total_occupancy >= 4'd2);

    // Stage6 在 out_advance 时会装载“下一个要输出的点”。
    // - 当前 valid 且不是 last：装载 next_point_cnt；
    // - 当前 idle 且 st4_valid：装载 point0；
    // 其他情况该组合结果不会被使用。
    wire [VN_WIDTH-1:0] next_point_cnt;
    wire [VN_WIDTH-1:0] feature_point_idx = (m_axis_pfe_valid && !m_axis_pfe_last) ? next_point_cnt : {VN_WIDTH{1'b0}};

    wire [PT_WIDTH_PER-1:0] cur_point_raw = st4_points_buffer[feature_point_idx];

    wire signed [PT_WIDTH_XY-1:0] cur_x_q812 = cur_point_raw[71-:20];
    wire signed [PT_WIDTH_XY-1:0] cur_y_q812 = cur_point_raw[51-:20];
    wire signed [PT_WIDTH_Z-1:0] cur_z_q412 = cur_point_raw[31-:16];
    wire [PT_WIDTH_IS-1:0] cur_i_q115 = cur_point_raw[15-:16];

    wire signed [PT_WIDTH_XY-1:0] cur_x_cluster_q812 = sub_q812_sat(cur_x_q812, st4_mean_x);
    wire signed [PT_WIDTH_XY-1:0] cur_y_cluster_q812 = sub_q812_sat(cur_y_q812, st4_mean_y);
    wire signed [PT_WIDTH_Z-1:0] cur_z_cluster_q412 = sub_q412_sat(cur_z_q412, st4_mean_z);
    wire signed [PT_WIDTH_XY-1:0] cur_x_vcenter_q812 = sub_q812_sat(cur_x_q812, st4_vx_fixxy);
    wire signed [PT_WIDTH_XY-1:0] cur_y_vcenter_q812 = sub_q812_sat(cur_y_q812, st4_vy_fixxy);

    wire [PT_WIDTH-1:0] cur_x_cluster_q115 = q812_to_q115_sat(cur_x_cluster_q812);
    wire [PT_WIDTH-1:0] cur_y_cluster_q115 = q812_to_q115_sat(cur_y_cluster_q812);
    wire [PT_WIDTH-1:0] cur_x_vcenter_q115 = q812_to_q115_sat(cur_x_vcenter_q812);
    wire [PT_WIDTH-1:0] cur_y_vcenter_q115 = q812_to_q115_sat(cur_y_vcenter_q812);

    wire [PT_VEC_W-1:0] cur_pfe_data9 = {
        cur_x_vcenter_q115,
        cur_y_vcenter_q115,
        cur_z_q412,
        cur_i_q115,
        cur_x_cluster_q115,
        cur_y_cluster_q115,
        cur_z_cluster_q412,
        cur_x_vcenter_q115,
        cur_y_vcenter_q115
    };

    // Stage 6: 无 FIFO 逐点序列化输出
    // 当前正在发送的完整 pillar 直接来自 Stage4。
    reg [VN_WIDTH-1:0] point_cnt;

    // 如果当前没有 valid，或者当前 valid 已被下游 ready 接收，才推进输出状态。
    assign next_point_cnt = point_cnt + 1'b1;
    wire out_advance = (!m_axis_pfe_valid || m_axis_pfe_tready);
    wire [VN_WIDTH-1:0] st4_last_idx = st4_pnum - {{(VN_WIDTH - 1) {1'b0}}, 1'b1};

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

        end else if (pfe_frame_clear) begin
            point_cnt              <= {VN_WIDTH{1'b0}};
            m_axis_pfe_valid       <= 1'b0;
            m_axis_pfe_last        <= 1'b0;
            m_axis_pfe_voxel_valid <= 1'b0;
            m_axis_pfe_voxel_x     <= {COORD_WIDTH{1'b0}};
            m_axis_pfe_voxel_y     <= {COORD_WIDTH{1'b0}};

        end else if (out_advance) begin
            // 默认拉低；last 点输出时再拉高，并在 ready 反压期间保持。
            m_axis_pfe_voxel_valid <= 1'b0;

            if (m_axis_pfe_valid) begin
                // 当前点已经被 PFN 接收。
                if (m_axis_pfe_last) begin
                    // 当前 pillar 已完整发送。FIFO 已取消，因此这里不做 look-ahead 直切。
                    // 如果 Stage3 同拍顶进 Stage4，新 pillar 下一拍以后再启动输出。
                    m_axis_pfe_valid       <= 1'b0;
                    m_axis_pfe_last        <= 1'b0;
                    m_axis_pfe_voxel_valid <= 1'b0;
                    point_cnt              <= {VN_WIDTH{1'b0}};
                end else begin
                    // 继续发送当前 Stage4 pillar 的下一个点。
                    point_cnt          <= next_point_cnt;
                    m_axis_pfe_data    <= cur_pfe_data9;
                    m_axis_pfe_voxel_x <= st4_voxel_x;
                    m_axis_pfe_voxel_y <= st4_voxel_y;

                    if (next_point_cnt == st4_last_idx) begin
                        m_axis_pfe_last        <= 1'b1;
                        m_axis_pfe_voxel_valid <= 1'b1;
                    end else begin
                        m_axis_pfe_last        <= 1'b0;
                        m_axis_pfe_voxel_valid <= 1'b0;
                    end
                end
            end else begin
                // 当前输出空闲，如果 Stage4 有完整 pillar，则启动 point0。
                if (st4_valid) begin
                    m_axis_pfe_valid   <= 1'b1;
                    m_axis_pfe_data    <= cur_pfe_data9;
                    point_cnt          <= {VN_WIDTH{1'b0}};
                    m_axis_pfe_voxel_x <= st4_voxel_x;
                    m_axis_pfe_voxel_y <= st4_voxel_y;

                    if (st4_pnum == {{(VN_WIDTH - 1) {1'b0}}, 1'b1}) begin
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
                    point_cnt              <= {VN_WIDTH{1'b0}};
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
    // 2. !st3_valid && !st4_valid : 中间完整 pillar 槽位没有数据
    // 3. !m_axis_pfe_valid : 串行化输出总线已闲置
    wire pfe_empty = (inflight_cnt == 3'd0) && !st3_valid && !st4_valid && !m_axis_pfe_valid;

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
    assign pfe_frame_clear = m_axis_pfe_flush_done_reg;

`ifndef SYNTHESIS
    wire                       test_pfe = (issue_vx == 11'd60 && issue_vy == 11'd116) ? 1'b1 : 1'b0;
    // ====================================================================
    // 【独立的仿真测试模块】 抓取并打印特定 Voxel 数据
    // ====================================================================
    // synthesis translate_off
    integer                    test_k;
    reg     [PT_WIDTH_PER-1:0] test_point_data;

    always @(posedge clk) begin
        // 当流水线 Stage 2 有效且坐标匹配时触发
        if (rst_n && st2_rd_valid) begin
            if (st2_vx == 325 && st2_vy == 380) begin
                for (test_k = 0; test_k < POINTS_PER_ROW; test_k = test_k + 1) begin
                    if (base_idx + test_k < st2_pnum) begin
                        test_point_data = bram_pfe_rdata[(test_k*PT_WIDTH_PER)+:PT_WIDTH_PER];

                        $display(
                            "[Sim Time: %0t] Voxel(325, 380) HIT! Point Index: %0d | X: %0.3f, Y: %0.3f, Z: %0.3f, Intensity: %0.3f",
                            $time, (base_idx + test_k), $itor($signed(test_point_data[71-:20])) / 4096.0, $itor
                            ($signed(test_point_data[51-:20])) / 4096.0, $itor($signed(test_point_data[31-:16])) / 4096.0,
                            $itor($signed(test_point_data[15-:16])) / 32768.0);
                    end
                end
            end
        end
    end
    // synthesis translate_on
    // ====================================================================

`endif


`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n && st2_valid && !st2_to_st3 && st2_rd_valid && st2_last_row) begin
            $display("[ERROR][pfe] Stage2 overwrite risk: st2_valid is held but a new completed voxel arrives.");
            $stop;
        end
    end
`endif


endmodule
