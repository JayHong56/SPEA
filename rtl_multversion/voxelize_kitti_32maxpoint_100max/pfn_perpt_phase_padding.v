module pfn_perpt_phase_padding #(
    parameter         EXPAND_PT_DIM      = 9,
    parameter         COORD_WIDTH        = 11,
    parameter         OUT_PT_DIM         = 64,
    parameter         PT_WIDTH           = 16,
    parameter         MEM_WEIGHT_FILE    = "NOTHING",
    parameter         MEM_BIAS_FILE      = "NOTHING",
    parameter         MEM_BIAS_RELU_FILE = "NOTHING",
    parameter         WEIGHT_WIDTH       = 16,
    parameter         ACC_WIDTH          = 32,
    // Software PFN uses a fixed point dimension M=MAX_VOXEL_NUM and lets zero padding
    // points participate in max-pool. If a voxel has fewer valid points, hardware
    // must compare the valid-point max with ReLU(bias) to match that behavior.
    parameter integer MAX_VOXEL_NUM      = 32
) (
    input  wire                                     clk,
    input  wire                                     rst_n,
    output wire                                     s_axis_pfe_tready,
    input  wire                                     s_axis_pfe_valid,
    input  wire        [EXPAND_PT_DIM*PT_WIDTH-1:0] s_axis_pfe_data,
    input  wire                                     s_axis_pfe_last,
    input  wire                                     s_axis_pfe_voxel_valid,
    input  wire signed [           COORD_WIDTH-1:0] s_axis_pfe_voxel_x,
    input  wire signed [           COORD_WIDTH-1:0] s_axis_pfe_voxel_y,
    input  wire                                     s_axis_pfe_flush_done,
    // output
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_x,
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_y,
    output reg                                      m_axis_pfn_valid,
    input  wire                                     m_axis_pfn_tready,
    output reg         [   OUT_PT_DIM*PT_WIDTH-1:0] m_axis_pfn_data,
    output wire                                     m_axis_pfn_flush_done
);
    localparam integer TDM_FACTOR = 2;
    localparam integer OUT_CHUNK_DIM = (OUT_PT_DIM + TDM_FACTOR - 1) / TDM_FACTOR;  // ceil(64/3)=22
    localparam integer PHASE_W = (TDM_FACTOR <= 2) ? 1 : $clog2(TDM_FACTOR);
    localparam integer LAST_PHASE = TDM_FACTOR - 1;
    localparam integer LAST_CHUNK_BASE = LAST_PHASE * OUT_CHUNK_DIM;  // 2*22=44


    localparam integer DEQUANT_SHIFT = 23;
    localparam signed [16-1:0] DEQUANT_MUL = 16'sd109;  // 3482 = round(2^28 / 77089.47481872393)，其�?256 是量化后的小数放大倍数 (Q8.8)
    localparam integer PT_CNT_WIDTH = (MAX_VOXEL_NUM <= 1) ? 1 : $clog2(MAX_VOXEL_NUM + 1);
    localparam [PT_CNT_WIDTH-1:0] MAX_VOXEL_NUM_CNT = MAX_VOXEL_NUM;

`ifndef SYNTHESIS
    initial begin
        if (OUT_CHUNK_DIM * TDM_FACTOR < OUT_PT_DIM) begin
            $display("[ERROR][pfn_layer] OUT_CHUNK_DIM too small.");
            $stop;
        end
    end
`endif  // 1. 权重与偏�?ROM 

    wire                                        pfn_frame_clear;
    reg signed [EXPAND_PT_DIM*WEIGHT_WIDTH-1:0] weight_row_rom  [0:OUT_PT_DIM-1];
    reg signed [                 ACC_WIDTH-1:0] bias_rom        [0:OUT_PT_DIM-1];
    reg        [                  PT_WIDTH-1:0] bias_relu_rom   [0:OUT_PT_DIM-1];
    initial begin
        if (MEM_WEIGHT_FILE != "NOTHING") begin
            $display("Loading weight from %s", MEM_WEIGHT_FILE);
            $readmemh(MEM_WEIGHT_FILE, weight_row_rom);
        end
        if (MEM_BIAS_FILE != "NOTHING") begin
            $display("Loading bias from %s", MEM_BIAS_FILE);
            $readmemh(MEM_BIAS_FILE, bias_rom);
        end
        if (MEM_BIAS_RELU_FILE != "NOTHING") begin
            $display("Loading bias relu from %s", MEM_BIAS_RELU_FILE);
            $readmemh(MEM_BIAS_RELU_FILE, bias_relu_rom);
        end
    end

    // =======================================================
    // 全局流水线暂停控�?(反压逻辑核心)
    // =======================================================
    // 如果下游�?ready，且当前正在输出有效数据，则冻结整条流水�?
    wire pipe_ready = m_axis_pfn_tready || !m_axis_pfn_valid;

    // =======================================================
    // 级联流水�?Stage 1: 输入锁存与乘法运�?(打一�?
    // 修复：乘法器操作数提�?Mux，强制复�?DSP 硬件
    // =======================================================
    reg [PHASE_W-1:0] phase;  // 0,1,2,3 四个时分复用阶段

    assign s_axis_pfe_tready = (phase == {PHASE_W{1'b0}}) && pipe_ready;

    wire pfe_fire = s_axis_pfe_valid && s_axis_pfe_tready;

    // 当前 voxel 已经接收的有效点数。当前点进入时的点数�?cur_voxel_pt_cnt + 1�?
    // 该计数用于在输出时判断是否存�?padding zero 点�?
    reg [PT_CNT_WIDTH-1:0] cur_voxel_pt_cnt;
    wire [PT_CNT_WIDTH-1:0] this_point_cnt;
    assign this_point_cnt = cur_voxel_pt_cnt + {{(PT_CNT_WIDTH - 1) {1'b0}}, 1'b1};

    // 输入数据的第一级元数据锁存
    reg        [EXPAND_PT_DIM*PT_WIDTH-1:0] latched_pfe_data;
    reg                                     latched_last;
    reg                                     latched_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_x;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_y;
    reg        [          PT_CNT_WIDTH-1:0] latched_voxel_pt_cnt;

    // 伴随数据向下游流动的控制信号
    reg                                     st2_valid;
    reg        [               PHASE_W-1:0] st2_phase;
    reg                                     st2_last;
    reg                                     st2_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_x;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_y;
    reg        [          PT_CNT_WIDTH-1:0] st2_voxel_pt_cnt;

    // --- 核心修改：新增组合逻辑 Mux 提取操作�?---
    reg signed [              PT_WIDTH-1:0] op_data              [0:EXPAND_PT_DIM-1];
    reg signed [          WEIGHT_WIDTH-1:0] op_weight            [0:OUT_CHUNK_DIM-1] [0:EXPAND_PT_DIM-1];
    reg signed [             ACC_WIDTH-1:0] op_bias              [0:OUT_CHUNK_DIM-1];
    integer i_k, i_j;
    integer sel_oc;

    always @(*) begin
        // 1. 数据输入选择�?
        // phase 0 使用当前 s_axis_pfe_data�?
        // phase 1/2/3 使用锁存�?latched_pfe_data�?
        for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
            op_data[i_k] = (phase == {PHASE_W{1'b0}}) ? $signed(s_axis_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]) :
                $signed(latched_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]);
        end

        // 2. 当前 phase 对应的输出通道范围�?
        // phase 0 -> 0  ~ 11
        // phase 1 -> 12 ~ 23
        // phase 2 -> 24 ~ 35
        // phase 3 -> 36 ~ 47
        for (i_j = 0; i_j < OUT_CHUNK_DIM; i_j = i_j + 1) begin
            sel_oc = phase * OUT_CHUNK_DIM + i_j;

            if (sel_oc < OUT_PT_DIM) begin
                op_bias[i_j] = $signed(bias_rom[sel_oc]);

                for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
                    op_weight[i_j][i_k] = $signed(weight_row_rom[sel_oc][i_k*WEIGHT_WIDTH+:WEIGHT_WIDTH]);
                end
            end else begin
                op_bias[i_j] = {ACC_WIDTH{1'b0}};

                for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
                    op_weight[i_j][i_k] = {WEIGHT_WIDTH{1'b0}};
                end
            end
        end
    end

    // --- 核心流水线寄存器 (Pipeline Registers) ---
    (* use_dsp = "yes" *) reg signed [ACC_WIDTH-1:0] st2_mult[0:OUT_CHUNK_DIM-1][0:EXPAND_PT_DIM-1];
    // reg signed [ACC_WIDTH-1:0] st2_bias[0:OUT_CHUNK_DIM-1];
    integer j, k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase               <= {PHASE_W{1'b0}};
            st2_valid           <= 1'b0;
            st2_phase           <= {PHASE_W{1'b0}};
            st2_last            <= 1'b0;
            st2_voxel_valid     <= 1'b0;

            latched_last        <= 1'b0;
            latched_voxel_valid <= 1'b0;

        end else if (pfn_frame_clear) begin
            phase               <= {PHASE_W{1'b0}};
            st2_valid           <= 1'b0;
            st2_phase           <= {PHASE_W{1'b0}};
            st2_last            <= 1'b0;
            st2_voxel_valid     <= 1'b0;

            latched_last        <= 1'b0;
            latched_voxel_valid <= 1'b0;

        end else if (pipe_ready) begin
            // 只在真正接受 PFE 点时更新当前 voxel 点数�?
            if (pfe_fire) begin
                if (s_axis_pfe_last) begin
                    cur_voxel_pt_cnt <= {PT_CNT_WIDTH{1'b0}};
                end else begin
                    cur_voxel_pt_cnt <= this_point_cnt;
                end
            end

            // --------------------------------------------------
            // phase 0：接收新点，并计算通道 0~11
            // --------------------------------------------------
            if (phase == {PHASE_W{1'b0}}) begin
                if (s_axis_pfe_valid) begin
                    phase                <= phase + 1'b1;

                    // 锁存该点，供 phase 1/2/3 使用
                    latched_pfe_data     <= s_axis_pfe_data;
                    latched_last         <= s_axis_pfe_last;
                    latched_voxel_valid  <= s_axis_pfe_voxel_valid;
                    latched_voxel_x      <= s_axis_pfe_voxel_x;
                    latched_voxel_y      <= s_axis_pfe_voxel_y;
                    latched_voxel_pt_cnt <= this_point_cnt;

                    // 控制信号传�?
                    st2_valid            <= 1'b1;
                    st2_phase            <= {PHASE_W{1'b0}};
                    st2_last             <= s_axis_pfe_last;
                    st2_voxel_valid      <= s_axis_pfe_voxel_valid;
                    st2_voxel_x          <= s_axis_pfe_voxel_x;
                    st2_voxel_y          <= s_axis_pfe_voxel_y;
                    st2_voxel_pt_cnt     <= this_point_cnt;

                    // 计算当前 phase 对应�?12 个输出通道
                    for (j = 0; j < OUT_CHUNK_DIM; j = j + 1) begin
                        // st2_bias[j] <= op_bias[j];
                        for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                            st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                        end
                    end

                end else begin
                    st2_valid <= 1'b0;
                end

            end else begin
                // --------------------------------------------------
                // phase 1/2/3：继续处理锁存的同一个点
                // --------------------------------------------------

                if (phase == LAST_PHASE[PHASE_W-1:0]) begin
                    phase <= {PHASE_W{1'b0}};
                end else begin
                    phase <= phase + 1'b1;
                end

                st2_valid        <= 1'b1;
                st2_phase        <= phase;
                st2_last         <= latched_last;
                st2_voxel_valid  <= latched_voxel_valid;
                st2_voxel_x      <= latched_voxel_x;
                st2_voxel_y      <= latched_voxel_y;
                st2_voxel_pt_cnt <= latched_voxel_pt_cnt;

                for (j = 0; j < OUT_CHUNK_DIM; j = j + 1) begin
                    // st2_bias[j] <= op_bias[j];
                    for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                        st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                    end
                end
            end
        end
    end
    // =======================================================
    // 级联流水�?Stage 2: 强制加法�?(打一�?
    // =======================================================
    reg                           st3_valid;
    reg        [     PHASE_W-1:0] st3_phase;
    reg                           st3_last;
    reg                           st3_voxel_valid;
    reg signed [ COORD_WIDTH-1:0] st3_voxel_x;
    reg signed [ COORD_WIDTH-1:0] st3_voxel_y;
    reg        [PT_CNT_WIDTH-1:0] st3_voxel_pt_cnt;

    reg signed [   ACC_WIDTH-1:0] st3_mac_tree     [0:OUT_CHUNK_DIM-1];
    integer                       j_1;
    integer                       st3_sel_oc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st3_valid       <= 1'b0;
            st3_phase       <= {PHASE_W{1'b0}};
            st3_last        <= 1'b0;
            st3_voxel_valid <= 1'b0;

        end else if (pfn_frame_clear) begin
            st3_valid       <= 1'b0;
            st3_phase       <= {PHASE_W{1'b0}};
            st3_last        <= 1'b0;
            st3_voxel_valid <= 1'b0;

        end else if (pipe_ready) begin  // 全局停滞控制
            // 1. 无脑传递控制信�?(�?st2 的状态向后推一�?
            st3_valid        <= st2_valid;
            st3_phase        <= st2_phase;
            st3_last         <= st2_last;
            st3_voxel_valid  <= st2_voxel_valid;
            st3_voxel_x      <= st2_voxel_x;
            st3_voxel_y      <= st2_voxel_y;
            st3_voxel_pt_cnt <= st2_voxel_pt_cnt;

            // 2. 数据计算：当有有效数据时，计�?4 级加法树并打入寄存器
            if (st2_valid) begin
                for (j_1 = 0; j_1 < OUT_CHUNK_DIM; j_1 = j_1 + 1) begin
                    st3_sel_oc = st2_phase * OUT_CHUNK_DIM + j_1;

                    if (st3_sel_oc < OUT_PT_DIM) begin
                        st3_mac_tree[j_1] <= (
                        (
                            (bias_rom[st3_sel_oc] + st2_mult[j_1][0]) +
                            (st2_mult[j_1][1] + st2_mult[j_1][2])
                        ) + (
                            (st2_mult[j_1][3] + st2_mult[j_1][4]) +
                            (st2_mult[j_1][5] + st2_mult[j_1][6])
                        )
                    ) + (
                        st2_mult[j_1][7] + st2_mult[j_1][8]
                    );
                    end else begin
                        st3_mac_tree[j_1] <= {ACC_WIDTH{1'b0}};
                    end
                end
            end
        end
    end

    // =======================================================
    // 级联流水�?Stage 3: 去量化与 ReLU (纯组合逻辑)
    // 组合逻辑会随着打过拍的 st3 自动挂起，无需改动
    // =======================================================
    reg        [PT_WIDTH-1:0] relu_comb   [0:OUT_CHUNK_DIM-1];
    reg signed [        63:0] dequant_temp[0:OUT_CHUNK_DIM-1];
    integer                   j_2;
    always @(*) begin
        for (j_2 = 0; j_2 < OUT_CHUNK_DIM; j_2 = j_2 + 1) begin
            dequant_temp[j_2] = st3_mac_tree[j_2] * DEQUANT_MUL;
            if (st3_mac_tree[j_2] < 0) begin
                relu_comb[j_2] = 0;
            end else begin
                relu_comb[j_2] = dequant_temp[j_2][PT_WIDTH+DEQUANT_SHIFT-1 : DEQUANT_SHIFT];
            end
        end
    end


    wire has_padding_point;
    assign has_padding_point = (st3_voxel_pt_cnt < MAX_VOXEL_NUM_CNT);

    // =======================================================
    // 级联流水�?Stage 4: Max Pool �?数据输出
    // =======================================================
    reg     [PT_WIDTH-1:0] pad_max_regs [0:OUT_PT_DIM-1];
    reg     [PT_WIDTH-1:0] max_pool_regs[0:OUT_PT_DIM-1];

    integer                o;
`ifndef SYNTHESIS
    initial begin
        for (o = 0; o < OUT_PT_DIM; o = o + 1) begin
            pad_max_regs[o]  = 0;
            max_pool_regs[o] = 0;
        end
    end
`endif

    integer oc;
    integer oc_idx;
    reg [PT_WIDTH-1:0] valid_next_max;
    reg [PT_WIDTH-1:0] pad_next_max;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_pfn_valid <= 1'b0;

            for (oc = 0; oc < OUT_PT_DIM; oc = oc + 1) begin
                max_pool_regs[oc] <= {PT_WIDTH{1'b0}};
                pad_max_regs[oc]  <= bias_relu_rom[oc];
            end

        end else if (pfn_frame_clear) begin
            m_axis_pfn_valid <= 1'b0;

        end else if (pipe_ready) begin
            // 默认拉低单周�?valid
            m_axis_pfn_valid <= 1'b0;

            if (st3_valid) begin

                // --------------------------------------------------
                // 如果当前是最后一�?phase，并且当前点�?voxel 的最后一个点�?
                // 则本拍完成最�?phase �?max，并输出整个 PFN feature�?
                // --------------------------------------------------
                if ((st3_phase == LAST_PHASE[PHASE_W-1:0]) && st3_last) begin
                    m_axis_pfn_valid   <= 1'b1;
                    m_axis_pfn_voxel_x <= st3_voxel_x;
                    m_axis_pfn_voxel_y <= st3_voxel_y;

                    // Output previous phases. Their padding-aware max was already
                    // accumulated into pad_max_regs when each phase was processed.
                    // max_pool_regs remains the valid-point-only max.
                    for (oc = 0; oc < LAST_CHUNK_BASE; oc = oc + 1) begin
                        if (has_padding_point) begin
                            m_axis_pfn_data[oc*PT_WIDTH+:PT_WIDTH] <= pad_max_regs[oc];
                        end else begin
                            m_axis_pfn_data[oc*PT_WIDTH+:PT_WIDTH] <= max_pool_regs[oc];
                        end

                        max_pool_regs[oc] <= {PT_WIDTH{1'b0}};
                        pad_max_regs[oc]  <= bias_relu_rom[oc];
                    end

                    // 输出最后一�?phase 的通道：当�?relu_comb 还没写入 max_pool_regs�?
                    // 所以先比较 valid-point max，再根据需要加�?padding bias baseline�?
                    for (oc = 0; oc < OUT_CHUNK_DIM; oc = oc + 1) begin
                        oc_idx = LAST_CHUNK_BASE + oc;

                        if (oc_idx < OUT_PT_DIM) begin
                            if (relu_comb[oc] > max_pool_regs[oc_idx]) begin
                                valid_next_max = relu_comb[oc];
                            end else begin
                                valid_next_max = max_pool_regs[oc_idx];
                            end

                            if (valid_next_max > bias_relu_rom[oc_idx]) begin
                                pad_next_max = valid_next_max;
                            end else begin
                                pad_next_max = bias_relu_rom[oc_idx];
                            end

                            if (has_padding_point) begin
                                m_axis_pfn_data[oc_idx*PT_WIDTH+:PT_WIDTH] <= pad_next_max;
                            end else begin
                                m_axis_pfn_data[oc_idx*PT_WIDTH+:PT_WIDTH] <= valid_next_max;
                            end

                            max_pool_regs[oc_idx] <= {PT_WIDTH{1'b0}};
                            pad_max_regs[oc_idx]  <= bias_relu_rom[oc_idx];
                        end
                    end

                end else begin
                    // --------------------------------------------------
                    // Normal case: update the current phase chunk and
                    // keep the phase padding-aware max in pad_max_regs.
                    // --------------------------------------------------
                    for (oc = 0; oc < OUT_CHUNK_DIM; oc = oc + 1) begin
                        oc_idx = st3_phase * OUT_CHUNK_DIM + oc;

                        if (oc_idx < OUT_PT_DIM) begin
                            if (relu_comb[oc] > max_pool_regs[oc_idx]) begin
                                valid_next_max = relu_comb[oc];
                            end else begin
                                valid_next_max = max_pool_regs[oc_idx];
                            end

                            max_pool_regs[oc_idx] <= valid_next_max;

                            if (valid_next_max > bias_relu_rom[oc_idx]) begin
                                pad_next_max = valid_next_max;
                            end else begin
                                pad_next_max = bias_relu_rom[oc_idx];
                            end

                            pad_max_regs[oc_idx] <= pad_next_max;
                        end
                    end
                end
            end
        end
    end
    // =========================================================================
    // 帧尾排空信号传�?(Flush Propagation)
    // =========================================================================
    reg  pfn_flushing_req;
    reg  m_axis_pfn_flush_done_reg;

    wire pfn_empty = (phase == {PHASE_W{1'b0}}) && !st2_valid && !st3_valid && !m_axis_pfn_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfn_flushing_req          <= 1'b0;
            m_axis_pfn_flush_done_reg <= 1'b0;
        end else begin
            if (s_axis_pfe_flush_done) begin
                pfn_flushing_req <= 1'b1;
            end

            if (pfn_flushing_req && pfn_empty && !m_axis_pfn_flush_done_reg) begin
                m_axis_pfn_flush_done_reg <= 1'b1;
                pfn_flushing_req          <= 1'b0;
            end else begin
                m_axis_pfn_flush_done_reg <= 1'b0;
            end
        end
    end

    assign m_axis_pfn_flush_done = m_axis_pfn_flush_done_reg;
    assign pfn_frame_clear = m_axis_pfn_flush_done_reg;
    // =======================================================
    // 仿真测试探针�?
    // =======================================================
`ifndef SYNTHESIS
    wire test_pfn = s_axis_pfe_voxel_y == 11'sd141 && s_axis_pfe_voxel_x == 11'sd87;
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
            // if ((s_axis_pfe_voxel_x == 11'sd287) && (s_axis_pfe_voxel_y == 11'sd368)) begin
            //     $display("[%0t] DBG voxel(287,368) s_axis_pfe_data=0x%0h", $time, s_axis_pfe_data);
            // end

            if (pfe_csv_fd != 0) begin
                $fwrite(pfe_csv_fd, "%0t,%0d,%0d", $time, s_axis_pfe_voxel_x, s_axis_pfe_voxel_y);
                for (pfe_i = EXPAND_PT_DIM - 1; pfe_i >= 0; pfe_i = pfe_i - 1) begin
                    if (pfe_i == 2 || pfe_i == 6) begin
                        // z, z-mean: Q4.12
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 4096.0;
                    end else begin
                        // others: Q1.15
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 32768.0;
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

    // Simulation log: output (修复：加�?&& m_axis_pfn_tready，防止反压时重复写入)
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
