module pfn_layer #(
    parameter EXPAND_PT_DIM   = 11,
    parameter COORD_WIDTH     = 11,
    parameter OUT_PT_DIM      = 48,
    parameter PT_WIDTH        = 16,
    parameter MEM_WEIGHT_FILE = "NOTHING",
    parameter MEM_BIAS_FILE   = "NOTHING",
    parameter WEIGHT_WIDTH    = 16,
    parameter ACC_WIDTH       = 32
) (
    input wire clk,
    input wire rst_n,

    output wire                                     s_axis_pfe_tready,
    input  wire                                     s_axis_pfe_valid,
    input  wire        [EXPAND_PT_DIM*PT_WIDTH-1:0] s_axis_pfe_data,
    input  wire                                     s_axis_pfe_last,
    input  wire                                     s_axis_pfe_voxel_valid,
    input  wire signed [           COORD_WIDTH-1:0] s_axis_pfe_voxel_x,
    input  wire signed [           COORD_WIDTH-1:0] s_axis_pfe_voxel_y,
    input  wire                                     s_axis_pfe_flush_done,
    // output
    output wire                                     m_axis_pfn_voxel_valid_out,
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_x,
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_y,
    output reg                                      m_axis_pfn_valid,
    input  wire                                     m_axis_pfn_tready,
    output reg         [   OUT_PT_DIM*PT_WIDTH-1:0] m_axis_pfn_data,
    output wire                                     m_axis_pfn_flush_done
);
    assign m_axis_pfn_voxel_valid_out = m_axis_pfn_tready & m_axis_pfn_valid;
    localparam HALF_OUT_DIM = OUT_PT_DIM / 2;  // 24
    // 1. 权重与偏置 ROM 
    reg signed [EXPAND_PT_DIM*WEIGHT_WIDTH-1:0] weight_row_rom[0:OUT_PT_DIM-1];
    reg signed [                 ACC_WIDTH-1:0] bias_rom      [0:OUT_PT_DIM-1];

    initial begin
        if (MEM_WEIGHT_FILE != "NOTHING") begin
            $display("Loading weight from %s", MEM_WEIGHT_FILE);
            $readmemh(MEM_WEIGHT_FILE, weight_row_rom);
        end
        if (MEM_BIAS_FILE != "NOTHING") begin
            $display("Loading bias from %s", MEM_BIAS_FILE);
            $readmemh(MEM_BIAS_FILE, bias_rom);
        end
    end

    // =======================================================
    // 全局流水线暂停控制 (反压逻辑核心)
    // =======================================================
    // 如果下游不 ready，且当前正在输出有效数据，则冻结整条流水线
    wire pipe_ready = m_axis_pfn_tready || !m_axis_pfn_valid;

    // =======================================================
    // 级联流水线 Stage 1: 输入锁存与乘法运算 (打一拍)
    // 修复：乘法器操作数提前 Mux，强制复用 DSP 硬件
    // =======================================================
    reg  phase;  // 0: 处理通道 0~23, 1: 处理通道 24~47

    // 输入不仅要满足 phase==0，还要保证当前流水线没有被反压冻结
    assign s_axis_pfe_tready = (phase == 0) && pipe_ready;

    // 输入数据的第一级元数据锁存
    reg        [EXPAND_PT_DIM*PT_WIDTH-1:0] latched_pfe_data;
    reg                                     latched_last;
    reg                                     latched_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_x;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_y;

    // 伴随数据向下游流动的控制信号
    reg                                     st2_valid;
    reg                                     st2_phase;
    reg                                     st2_last;
    reg                                     st2_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_x;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_y;

    // --- 核心修改：新增组合逻辑 Mux 提取操作数 ---
    reg signed [              PT_WIDTH-1:0] op_data             [0:EXPAND_PT_DIM-1];
    reg signed [          WEIGHT_WIDTH-1:0] op_weight           [ 0:HALF_OUT_DIM-1] [0:EXPAND_PT_DIM-1];
    reg signed [             ACC_WIDTH-1:0] op_bias             [ 0:HALF_OUT_DIM-1];

    integer i_k, i_j;  // 组合逻辑专用循环变量
    always @(*) begin
        // 1. Mux 选择乘法器的 数据输入 (11个维度)
        for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
            op_data[i_k] = (phase == 0) ? $signed(s_axis_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]) :
                $signed(latched_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]);
        end

        // 2. Mux 选择乘法器的 权重输入 和 加法器的 Bias 输入 (24个输出通道)
        for (i_j = 0; i_j < HALF_OUT_DIM; i_j = i_j + 1) begin
            op_bias[i_j] = (phase == 0) ? ($signed(bias_rom[i_j]) <<< 8) : ($signed(bias_rom[i_j+HALF_OUT_DIM]) <<< 8);

            for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
                op_weight[i_j][i_k] = (phase == 0) ? $signed(weight_row_rom[i_j][i_k*WEIGHT_WIDTH+:WEIGHT_WIDTH]) :
                    $signed(weight_row_rom[i_j+HALF_OUT_DIM][i_k*WEIGHT_WIDTH+:WEIGHT_WIDTH]);
            end
        end
    end

    // --- 核心流水线寄存器 (Pipeline Registers) ---
    (* use_dsp = "yes" *)reg signed [ACC_WIDTH-1:0] st2_mult[0:HALF_OUT_DIM-1] [0:EXPAND_PT_DIM-1];
    reg signed [ACC_WIDTH-1:0] st2_bias[0:HALF_OUT_DIM-1];

    integer j, k;  // 时序逻辑专用循环变量
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase               <= 1'b0;
            st2_valid           <= 1'b0;
            st2_phase           <= 1'b0;
            latched_last        <= 1'b0;
            latched_voxel_valid <= 1'b0;
        end else if (pipe_ready) begin  // 全局停滞控制
            if (phase == 0) begin
                if (s_axis_pfe_valid) begin
                    phase               <= 1'b1;  // 下一拍处理后半区

                    // 锁存供 Phase 1 用的数据
                    latched_pfe_data    <= s_axis_pfe_data;
                    latched_last        <= s_axis_pfe_last;
                    latched_voxel_valid <= s_axis_pfe_voxel_valid;
                    latched_voxel_x     <= s_axis_pfe_voxel_x;
                    latched_voxel_y     <= s_axis_pfe_voxel_y;

                    // 传递控制信号
                    st2_valid           <= 1'b1;
                    st2_phase           <= 1'b0;
                    st2_last            <= s_axis_pfe_last;
                    st2_voxel_valid     <= s_axis_pfe_voxel_valid;
                    st2_voxel_x         <= s_axis_pfe_voxel_x;
                    st2_voxel_y         <= s_axis_pfe_voxel_y;

                    // --- 统一乘法运算：硬件将被完美复用 ---
                    for (j = 0; j < HALF_OUT_DIM; j = j + 1) begin
                        st2_bias[j] <= op_bias[j];
                        for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                            st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                        end
                    end
                end else begin
                    st2_valid <= 1'b0;
                end
            end else begin
                // phase == 1 时
                phase           <= 1'b0;  // 一点处理完毕，回到接收新点状态

                // 传递控制信号
                st2_valid       <= 1'b1;
                st2_phase       <= 1'b1;
                st2_last        <= latched_last;
                st2_voxel_valid <= latched_voxel_valid;
                st2_voxel_x     <= latched_voxel_x;
                st2_voxel_y     <= latched_voxel_y;

                // --- 统一乘法运算：这里代码和 Phase 0 完全一致！ ---
                for (j = 0; j < HALF_OUT_DIM; j = j + 1) begin
                    st2_bias[j] <= op_bias[j];
                    for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                        st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                    end
                end
            end
        end
    end

    // =======================================================
    // 级联流水线 Stage 2: 强制加法树 (打一拍)
    // =======================================================
    reg                          st3_valid;
    reg                          st3_phase;
    reg                          st3_last;
    reg                          st3_voxel_valid;
    reg signed [COORD_WIDTH-1:0] st3_voxel_x;
    reg signed [COORD_WIDTH-1:0] st3_voxel_y;

    reg signed [  ACC_WIDTH-1:0] st3_mac_tree    [0:HALF_OUT_DIM-1];
    integer                      j_1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st3_valid       <= 1'b0;
            st3_phase       <= 1'b0;
            st3_last        <= 1'b0;
            st3_voxel_valid <= 1'b0;
            st3_voxel_x     <= {COORD_WIDTH{1'b0}};
            st3_voxel_y     <= {COORD_WIDTH{1'b0}};
        end else if (pipe_ready) begin  // 全局停滞控制
            // 1. 无脑传递控制信号 (把 st2 的状态向后推一拍)
            st3_valid       <= st2_valid;
            st3_phase       <= st2_phase;
            st3_last        <= st2_last;
            st3_voxel_valid <= st2_voxel_valid;
            st3_voxel_x     <= st2_voxel_x;
            st3_voxel_y     <= st2_voxel_y;

            // 2. 数据计算：当有有效数据时，计算 4 级加法树并打入寄存器
            if (st2_valid) begin
                for (j_1 = 0; j_1 < HALF_OUT_DIM; j_1 = j_1 + 1) begin
                    st3_mac_tree[j_1] <= (
                    // 左半树
                    (
                            (st2_bias[j_1]      + st2_mult[j_1][0]) + 
                            (st2_mult[j_1][1]   + st2_mult[j_1][2])
                        ) + (
                            (st2_mult[j_1][3]   + st2_mult[j_1][4]) + 
                             st2_mult[j_1][5]
                        )
                    ) + (
                    // 右半树
                    ((st2_mult[j_1][6] + st2_mult[j_1][7]) + (st2_mult[j_1][8] + st2_mult[j_1][9])) + (st2_mult[j_1][10]));
                end
            end
        end
    end

    // =======================================================
    // 级联流水线 Stage 3: 去量化与 ReLU (纯组合逻辑)
    // 组合逻辑会随着打过拍的 st3 自动挂起，无需改动
    // =======================================================
    reg        [PT_WIDTH-1:0] relu_comb   [0:HALF_OUT_DIM-1];
    reg signed [        63:0] dequant_temp[0:HALF_OUT_DIM-1];
    integer                   j_2;
    always @(*) begin
        for (j_2 = 0; j_2 < HALF_OUT_DIM; j_2 = j_2 + 1) begin
            dequant_temp[j_2] = st3_mac_tree[j_2] * 994;
            if (st3_mac_tree[j_2] < 0) relu_comb[j_2] = 0;
            else relu_comb[j_2] = dequant_temp[j_2][PT_WIDTH+20-1 : 20];
        end
    end
    // =======================================================
    // 级联流水线 Stage 4: Max Pool 与 数据输出
    // =======================================================
    reg signed [PT_WIDTH-1:0] max_pool_regs[0:OUT_PT_DIM-1];

    integer                   o;
`ifndef SYNTHESIS
    initial begin
        for (o = 0; o < OUT_PT_DIM; o = o + 1) begin
            max_pool_regs[o] = 0;
        end
    end
`endif
    integer oc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_pfn_valid   <= 1'b0;
            // m_axis_pfn_voxel_valid <= 1'b0;
            m_axis_pfn_voxel_x <= {COORD_WIDTH{1'b0}};
            m_axis_pfn_voxel_y <= {COORD_WIDTH{1'b0}};
        end else if (pipe_ready) begin  // 全局停滞控制
            // 默认拉低单周期脉冲
            m_axis_pfn_valid <= 1'b0;
            // m_axis_pfn_voxel_valid <= 1'b0;

            if (st3_valid) begin
                if (st3_phase == 0) begin
                    // 【对应前半区计算完毕】：更新前半区 (0~23) 的 Max Pool
                    for (oc = 0; oc < HALF_OUT_DIM; oc = oc + 1) begin
                        if (relu_comb[oc] > max_pool_regs[oc]) begin
                            max_pool_regs[oc] <= relu_comb[oc];
                        end
                    end
                    // =======================================================
                    // 【第一拍】：提前一个周期拉高 Voxel 信号
                    // =======================================================

                end else begin
                    // 【对应后半区计算完毕】：更新后半区 (24~47) 的 Max Pool
                    for (oc = 0; oc < HALF_OUT_DIM; oc = oc + 1) begin
                        if (relu_comb[oc] > max_pool_regs[oc+HALF_OUT_DIM]) begin
                            max_pool_regs[oc+HALF_OUT_DIM] <= relu_comb[oc];
                        end
                    end

                    // =======================================================
                    // 【第二拍】：输出 pfn_valid，并让 voxel_valid 持续保持！
                    // =======================================================
                    if (st3_last) begin
                        m_axis_pfn_valid   <= 1'b1;
                        // m_axis_pfn_voxel_valid <= st3_voxel_valid;
                        m_axis_pfn_voxel_x <= st3_voxel_x;
                        m_axis_pfn_voxel_y <= st3_voxel_y;

                        for (oc = 0; oc < HALF_OUT_DIM; oc = oc + 1) begin
                            // 1. 前半区数据直接从 max_pool_regs 取出
                            m_axis_pfn_data[oc*PT_WIDTH+:PT_WIDTH] <= max_pool_regs[oc];

                            // 2. 后半区数据：用三目运算符挑出最大值抢时间输出
                            m_axis_pfn_data[(oc + HALF_OUT_DIM)*PT_WIDTH +: PT_WIDTH] <= 
                                (relu_comb[oc] > max_pool_regs[oc + HALF_OUT_DIM]) ? relu_comb[oc] : max_pool_regs[oc + HALF_OUT_DIM];

                            // 3. 结算完毕，清空该 Voxel 的所有缓存池
                            max_pool_regs[oc] <= 0;
                            max_pool_regs[oc+HALF_OUT_DIM] <= 0;
                        end
                    end
                end
            end
        end
    end
    // =========================================================================
    // 帧尾排空信号传递 (Flush Propagation)
    // =========================================================================
    reg  pfn_flushing_req;
    reg  m_axis_pfn_flush_done_reg;

    wire pfn_empty = (phase == 1'b0) && !st2_valid && !st3_valid && !m_axis_pfn_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfn_flushing_req          <= 1'b0;
            m_axis_pfn_flush_done_reg <= 1'b0;
        end else begin
            if (s_axis_pfe_flush_done) begin
                pfn_flushing_req <= 1'b1;
            end

            // pfn_empty 自然已经包含了管线停滞的状态（如果有残留数据就不会 empty）
            if (pfn_flushing_req && pfn_empty && !m_axis_pfn_flush_done_reg) begin
                m_axis_pfn_flush_done_reg <= 1'b1;
                pfn_flushing_req          <= 1'b0;
            end else begin
                m_axis_pfn_flush_done_reg <= 1'b0;
            end
        end
    end

    assign m_axis_pfn_flush_done = m_axis_pfn_flush_done_reg;

    // =======================================================
    // 仿真测试探针区
    // =======================================================
`ifndef SYNTHESIS
    integer csv_fd;
    integer pfe_csv_fd;
    integer pfe_i;
    integer out_i;
    real    pfe_q88_val;
    real    out_q88_val;

    integer pt_cnt;
    integer out_pt_cnt;

    initial begin
        pt_cnt     = 0;
        out_pt_cnt = 0;

        csv_fd     = $fopen("pfn_layer_out.csv", "w");
        if (csv_fd != 0) begin
            $fwrite(csv_fd, "time,voxel_x,voxel_y,pt_cnt");
            for (out_i = 0; out_i < OUT_PT_DIM; out_i = out_i + 1) begin
                $fwrite(csv_fd, ",dim%0d", out_i);
            end
            $fwrite(csv_fd, "\n");
        end

        pfe_csv_fd = $fopen("pfn_layer_pfe_input_dec.csv", "w");
        if (pfe_csv_fd != 0) begin
            $fwrite(pfe_csv_fd, "time,voxel_x,voxel_y");
            for (pfe_i = EXPAND_PT_DIM - 1; pfe_i >= 0; pfe_i = pfe_i - 1) begin
                $fwrite(pfe_csv_fd, ",dim%0d", pfe_i);
            end
            $fwrite(pfe_csv_fd, "\n");
        end
    end

    // Simulation log: input
    always @(posedge clk) begin
        if (rst_n && s_axis_pfe_valid && s_axis_pfe_tready) begin
            if ((s_axis_pfe_voxel_x == 11'sd287) && (s_axis_pfe_voxel_y == 11'sd368)) begin
                $display("[%0t] DBG voxel(287,368) s_axis_pfe_data=0x%0h", $time, s_axis_pfe_data);
            end

            if (pfe_csv_fd != 0) begin
                $fwrite(pfe_csv_fd, "%0t,%0d,%0d", $time, s_axis_pfe_voxel_x, s_axis_pfe_voxel_y);
                for (pfe_i = EXPAND_PT_DIM - 1; pfe_i >= 0; pfe_i = pfe_i - 1) begin
                    if (pfe_i == 7) begin
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 128.0;
                    end else if (pfe_i == 8 || pfe_i == 3 || pfe_i == 0) begin
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 4096.0;
                    end else begin
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 256.0;
                    end
                    $fwrite(pfe_csv_fd, ",%0.6f", pfe_q88_val);
                end
                $fwrite(pfe_csv_fd, "\n");
            end

            if (s_axis_pfe_last) begin
                out_pt_cnt <= pt_cnt + 1;
                pt_cnt     <= 0;
            end else begin
                pt_cnt <= pt_cnt + 1;
            end
        end
    end

    // Simulation log: output (修复：加入 && m_axis_pfn_tready，防止反压时重复写入)
    always @(posedge clk) begin
        if (rst_n && m_axis_pfn_valid && m_axis_pfn_tready) begin
            if (csv_fd != 0) begin
                $fwrite(csv_fd, "%0t,%0d,%0d,%0d", $time, m_axis_pfn_voxel_x, m_axis_pfn_voxel_y, out_pt_cnt);
                for (out_i = 0; out_i < OUT_PT_DIM; out_i = out_i + 1) begin
                    out_q88_val = $itor($signed(m_axis_pfn_data[out_i*PT_WIDTH+:PT_WIDTH])) / 256.0;
                    $fwrite(csv_fd, ",%0.6f", out_q88_val);
                end
                $fwrite(csv_fd, "\n");
            end
        end
    end
`endif

endmodule
