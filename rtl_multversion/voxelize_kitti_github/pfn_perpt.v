module pfn_layer #(
    parameter         EXPAND_PT_DIM      = 9,
    parameter         COORD_WIDTH        = 11,
    parameter         OUT_PT_DIM         = 64,
    parameter         PT_WIDTH           = 16,
    parameter         MEM_WEIGHT_FILE    = "NOTHING",
    parameter         MEM_BIAS_FILE      = "NOTHING",
    parameter         MEM_BIAS_RELU_FILE = "NOTHING",
    parameter         WEIGHT_WIDTH       = 16,
    parameter         ACC_WIDTH          = 32,
    
    
    
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
    
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_x,
    output reg signed  [           COORD_WIDTH-1:0] m_axis_pfn_voxel_y,
    output reg                                      m_axis_pfn_valid,
    input  wire                                     m_axis_pfn_tready,
    output reg         [   OUT_PT_DIM*PT_WIDTH-1:0] m_axis_pfn_data,
    output wire                                     m_axis_pfn_flush_done
);
    localparam integer TDM_FACTOR = 4;
    localparam integer OUT_CHUNK_DIM = (OUT_PT_DIM + TDM_FACTOR - 1) / TDM_FACTOR;  
    localparam integer PHASE_W = (TDM_FACTOR <= 2) ? 1 : $clog2(TDM_FACTOR);
    localparam integer LAST_PHASE = TDM_FACTOR - 1;
    localparam integer LAST_CHUNK_BASE = LAST_PHASE * OUT_CHUNK_DIM;  


    localparam integer DEQUANT_SHIFT = 23;
    localparam signed [16-1:0] DEQUANT_MUL = 16'sd109;  
    localparam integer PT_CNT_WIDTH = (MAX_VOXEL_NUM <= 1) ? 1 : $clog2(MAX_VOXEL_NUM + 1);
    localparam [PT_CNT_WIDTH-1:0] MAX_VOXEL_NUM_CNT = MAX_VOXEL_NUM;

`ifndef SYNTHESIS
    initial begin
        if (OUT_CHUNK_DIM * TDM_FACTOR < OUT_PT_DIM) begin
            $display("[ERROR][pfn_layer] OUT_CHUNK_DIM too small.");
            $stop;
        end
    end
`endif  

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

    
    
    
    
    wire pipe_ready = m_axis_pfn_tready || !m_axis_pfn_valid;

    
    
    
    
    reg [PHASE_W-1:0] phase;  

    assign s_axis_pfe_tready = (phase == {PHASE_W{1'b0}}) && pipe_ready;

    wire pfe_fire = s_axis_pfe_valid && s_axis_pfe_tready;

    
    
    reg [PT_CNT_WIDTH-1:0] cur_voxel_pt_cnt;
    wire [PT_CNT_WIDTH-1:0] this_point_cnt;
    assign this_point_cnt = cur_voxel_pt_cnt + {{(PT_CNT_WIDTH - 1) {1'b0}}, 1'b1};

    
    reg        [EXPAND_PT_DIM*PT_WIDTH-1:0] latched_pfe_data;
    reg                                     latched_last;
    reg                                     latched_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_x;
    reg signed [           COORD_WIDTH-1:0] latched_voxel_y;
    reg        [          PT_CNT_WIDTH-1:0] latched_voxel_pt_cnt;

    
    reg                                     st2_valid;
    reg        [               PHASE_W-1:0] st2_phase;
    reg                                     st2_last;
    reg                                     st2_voxel_valid;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_x;
    reg signed [           COORD_WIDTH-1:0] st2_voxel_y;
    reg        [          PT_CNT_WIDTH-1:0] st2_voxel_pt_cnt;

    
    reg signed [              PT_WIDTH-1:0] op_data              [0:EXPAND_PT_DIM-1];
    reg signed [          WEIGHT_WIDTH-1:0] op_weight            [0:OUT_CHUNK_DIM-1] [0:EXPAND_PT_DIM-1];
    reg signed [             ACC_WIDTH-1:0] op_bias              [0:OUT_CHUNK_DIM-1];
    integer i_k, i_j;
    integer sel_oc;

    always @(*) begin
        
        
        
        for (i_k = 0; i_k < EXPAND_PT_DIM; i_k = i_k + 1) begin
            op_data[i_k] = (phase == {PHASE_W{1'b0}}) ? $signed(s_axis_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]) :
                $signed(latched_pfe_data[i_k*PT_WIDTH+:PT_WIDTH]);
        end

        
        
        
        
        
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

    
    (* use_dsp = "yes" *) reg signed [ACC_WIDTH-1:0] st2_mult[0:OUT_CHUNK_DIM-1][0:EXPAND_PT_DIM-1];
    
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
            
            if (pfe_fire) begin
                if (s_axis_pfe_last) begin
                    cur_voxel_pt_cnt <= {PT_CNT_WIDTH{1'b0}};
                end else begin
                    cur_voxel_pt_cnt <= this_point_cnt;
                end
            end

            
            
            
            if (phase == {PHASE_W{1'b0}}) begin
                if (s_axis_pfe_valid) begin
                    phase                <= phase + 1'b1;

                    
                    latched_pfe_data     <= s_axis_pfe_data;
                    latched_last         <= s_axis_pfe_last;
                    latched_voxel_valid  <= s_axis_pfe_voxel_valid;
                    latched_voxel_x      <= s_axis_pfe_voxel_x;
                    latched_voxel_y      <= s_axis_pfe_voxel_y;
                    latched_voxel_pt_cnt <= this_point_cnt;

                    
                    st2_valid            <= 1'b1;
                    st2_phase            <= {PHASE_W{1'b0}};
                    st2_last             <= s_axis_pfe_last;
                    st2_voxel_valid      <= s_axis_pfe_voxel_valid;
                    st2_voxel_x          <= s_axis_pfe_voxel_x;
                    st2_voxel_y          <= s_axis_pfe_voxel_y;
                    st2_voxel_pt_cnt     <= this_point_cnt;

                    
                    for (j = 0; j < OUT_CHUNK_DIM; j = j + 1) begin
                        
                        for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                            st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                        end
                    end

                end else begin
                    st2_valid <= 1'b0;
                end

            end else begin
                
                
                

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
                    
                    for (k = 0; k < EXPAND_PT_DIM; k = k + 1) begin
                        st2_mult[j][k] <= op_data[k] * op_weight[j][k];
                    end
                end
            end
        end
    end
    
    
    
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

        end else if (pipe_ready) begin  
            
            st3_valid        <= st2_valid;
            st3_phase        <= st2_phase;
            st3_last         <= st2_last;
            st3_voxel_valid  <= st2_voxel_valid;
            st3_voxel_x      <= st2_voxel_x;
            st3_voxel_y      <= st2_voxel_y;
            st3_voxel_pt_cnt <= st2_voxel_pt_cnt;

            
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

    
    
    
    
    
    
    reg     [PT_WIDTH-1:0] max_pool_regs[0:OUT_PT_DIM-1];

    integer                o;
`ifndef SYNTHESIS
    initial begin
        for (o = 0; o < OUT_PT_DIM; o = o + 1) begin
            max_pool_regs[o] = 0;
        end
    end
`endif

    integer oc;
    integer oc_idx;
    reg [PT_WIDTH-1:0] valid_next_max;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_pfn_valid <= 1'b0;

            for (oc = 0; oc < OUT_PT_DIM; oc = oc + 1) begin
                max_pool_regs[oc] <= {PT_WIDTH{1'b0}};
            end

        end else if (pfn_frame_clear) begin
            m_axis_pfn_valid <= 1'b0;

        end else if (pipe_ready) begin
            
            m_axis_pfn_valid <= 1'b0;

            if (st3_valid) begin

                
                
                
                
                if ((st3_phase == LAST_PHASE[PHASE_W-1:0]) && st3_last) begin
                    m_axis_pfn_valid   <= 1'b1;
                    m_axis_pfn_voxel_x <= st3_voxel_x;
                    m_axis_pfn_voxel_y <= st3_voxel_y;

                    
                    
                    
                    for (oc = 0; oc < LAST_CHUNK_BASE; oc = oc + 1) begin
                        if (has_padding_point && (bias_relu_rom[oc] > max_pool_regs[oc])) begin
                            m_axis_pfn_data[oc*PT_WIDTH+:PT_WIDTH] <= bias_relu_rom[oc];
                        end else begin
                            m_axis_pfn_data[oc*PT_WIDTH+:PT_WIDTH] <= max_pool_regs[oc];
                        end

                        max_pool_regs[oc] <= {PT_WIDTH{1'b0}};
                    end

                    
                    
                    for (oc = 0; oc < OUT_CHUNK_DIM; oc = oc + 1) begin
                        oc_idx = LAST_CHUNK_BASE + oc;

                        if (oc_idx < OUT_PT_DIM) begin
                            if (relu_comb[oc] > max_pool_regs[oc_idx]) begin
                                valid_next_max = relu_comb[oc];
                            end else begin
                                valid_next_max = max_pool_regs[oc_idx];
                            end

                            if (has_padding_point && (bias_relu_rom[oc_idx] > valid_next_max)) begin
                                m_axis_pfn_data[oc_idx*PT_WIDTH+:PT_WIDTH] <= bias_relu_rom[oc_idx];
                            end else begin
                                m_axis_pfn_data[oc_idx*PT_WIDTH+:PT_WIDTH] <= valid_next_max;
                            end

                            max_pool_regs[oc_idx] <= {PT_WIDTH{1'b0}};
                        end
                    end

                end else begin
                    
                    
                    
                    
                    for (oc = 0; oc < OUT_CHUNK_DIM; oc = oc + 1) begin
                        oc_idx = st3_phase * OUT_CHUNK_DIM + oc;

                        if (oc_idx < OUT_PT_DIM) begin
                            if (relu_comb[oc] > max_pool_regs[oc_idx]) begin
                                valid_next_max = relu_comb[oc];
                            end else begin
                                valid_next_max = max_pool_regs[oc_idx];
                            end

                            max_pool_regs[oc_idx] <= valid_next_max;
                        end
                    end
                end
            end
        end
    end
    
    
    
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
    
    
    
`ifndef SYNTHESIS
    wire test_pfn = s_axis_pfe_voxel_y == 11'sd38 && s_axis_pfe_voxel_x == 11'sd247;
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

    
    always @(posedge clk) begin
        if (rst_n && s_axis_pfe_valid && s_axis_pfe_tready) begin
            
            
            

            if (pfe_csv_fd != 0) begin
                $fwrite(pfe_csv_fd, "%0t,%0d,%0d", $time, s_axis_pfe_voxel_x, s_axis_pfe_voxel_y);
                for (pfe_i = EXPAND_PT_DIM - 1; pfe_i >= 0; pfe_i = pfe_i - 1) begin
                    if (pfe_i == 2 || pfe_i == 6) begin
                        
                        pfe_q88_val = $itor($signed(s_axis_pfe_data[pfe_i*PT_WIDTH+:PT_WIDTH])) / 4096.0;
                    end else begin
                        
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
