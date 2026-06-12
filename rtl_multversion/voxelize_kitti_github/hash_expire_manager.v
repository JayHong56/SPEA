module hash_expire_manager #(
    parameter COORD_WIDTH = 11,
    parameter VN_WIDTH = 6,
    parameter TIMER_WIDTH = 18,
    parameter PT_WIDTH_PER = 72,  
    parameter ADDR_WIDTH = 8,  
    parameter LIFE_CYCLE = 16'd100,
    parameter TABLE_SIZE = 256,
    parameter BUCKET_AW = 6,
    parameter BUCKET_SLOT_WIDTH = 2,  
    parameter ENTRY_WIDTH = 2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + TIMER_WIDTH,
    parameter integer BRAM_DATA_WIDTH = 576,  
    parameter integer BRAM_ADDR_WIDTH = 10,  
    parameter integer BRAM_ADDR_WIDTH_PFE = 8,
    parameter integer BYTE_WIDTH = 9
) (
    input wire clk,
    input wire rst_n,
    input wire write_commit,  
    input wire [ADDR_WIDTH-1:0] write_addr,  
    input wire [TIMER_WIDTH-1:0] time_now,  
    input wire frame_end,  
    output wire flush_done,  
    output reg [BUCKET_AW-1:0] a_addr_shadow,
    output reg [BUCKET_SLOT_WIDTH-1:0] a_we_shadow,
    input wire [ENTRY_WIDTH-1:0] a_rdata_shadow,
    output reg [ENTRY_WIDTH-1:0] b_wdata,
    output reg [BUCKET_AW-1:0] b_addr,
    output reg [BUCKET_SLOT_WIDTH-1:0] b_we,
    output reg kill_valid,
    output wire kill_expired,
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    output reg bram_expire_wr_b,
    output reg [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output reg [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    
    output wire m_axis_expire_tvalid,
    input wire m_axis_expire_tready,
    output wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata,
    
    output wire hash_stall,
    output wire expire_blocking
);
    
    localparam ST_EMPTY = 2'b00;
    localparam ST_OCCU = 2'b01;
    localparam ST_TOMB = 2'b10;
    localparam ST_DRAIN = 2'b11;

    localparam integer EW = 2 * COORD_WIDTH + BRAM_ADDR_WIDTH_PFE + VN_WIDTH;
    localparam integer DEPTH = 32;
    wire                       fifo_in_ready;
    wire                       fifo_in_valid;
    wire [             EW-1:0] fifo_in_data;
    wire                       fifo_out_valid;
    wire [             EW-1:0] fifo_out_data;
    wire [$clog2(DEPTH+1)-1:0] fifo_out_level;
    hash_expire_fifo_sync #(  
        .WIDTH(EW),
        .DEPTH(DEPTH)
    ) u_hash_expire_fifo_sync (
        .clk  (clk),
        .rst_n(rst_n),

        .in_valid(fifo_in_valid),
        .in_ready(fifo_in_ready),
        .in_data (fifo_in_data),

        .out_valid(fifo_out_valid),
        .out_ready(fifo_out_ready),
        .out_data (fifo_out_data),

        .level(fifo_out_level)
    );
    assign fifo_out_ready = m_axis_expire_tready;
    assign m_axis_expire_tvalid = fifo_out_valid;
    assign m_axis_expire_tdata = fifo_out_data;
    wire fifo_full = (fifo_out_level == DEPTH);
    

    
    
    
    
    
    
    
    localparam integer NOTIFY_DEPTH = 4;
    localparam integer NOTIFY_AW = 2;

    reg [EW-1:0] notify_mem[0:NOTIFY_DEPTH-1];

    reg [NOTIFY_AW-1:0] notify_wr_ptr;
    reg [NOTIFY_AW-1:0] notify_rd_ptr;
    reg [$clog2(NOTIFY_DEPTH+1)-1:0] notify_count;

    wire notify_empty = (notify_count == 0);
    wire notify_full = (notify_count == NOTIFY_DEPTH);

    
    wire notify_push;
    wire [EW-1:0] notify_push_data;

    
    assign fifo_in_valid = !notify_empty;
    assign fifo_in_data  = notify_mem[notify_rd_ptr];

    wire notify_pop = fifo_in_valid && fifo_in_ready;

    
    
    wire notify_has_room = (notify_count <= NOTIFY_DEPTH - 2);

    always @(posedge clk) begin
        if (notify_push && !notify_full) begin
            notify_mem[notify_wr_ptr] <= notify_push_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            notify_wr_ptr <= {NOTIFY_AW{1'b0}};
            notify_rd_ptr <= {NOTIFY_AW{1'b0}};
            notify_count  <= {($clog2(NOTIFY_DEPTH + 1)) {1'b0}};

        end else if (flush_done) begin
            notify_wr_ptr <= {NOTIFY_AW{1'b0}};
            notify_rd_ptr <= {NOTIFY_AW{1'b0}};
            notify_count  <= {($clog2(NOTIFY_DEPTH + 1)) {1'b0}};

        end else begin
            if (notify_push && !notify_full) begin
                notify_wr_ptr <= notify_wr_ptr + 1'b1;
            end

            if (notify_pop) begin
                notify_rd_ptr <= notify_rd_ptr + 1'b1;
            end

            case ({
                notify_push && !notify_full, notify_pop
            })
                2'b10:   notify_count <= notify_count + 1'b1;
                2'b01:   notify_count <= notify_count - 1'b1;
                2'b11:   notify_count <= notify_count;
                default: notify_count <= notify_count;
            endcase
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n) begin
            if (notify_push && notify_full) begin
                $display("[ERROR][hash_expire_manager] notify FIFO overflow at time %0t", $time);
                $stop;
            end
        end
    end
`endif

    
    reg  hash_stall_reg;
    wire expire_fifo_almost_full = (fifo_out_level >= (DEPTH / 2));
    wire expired_fifo_empty = (fifo_out_level == 0);
    wire task_almost_full;
    wire task_almost_empty;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_stall_reg <= 1'b0;

        end else if (flush_done) begin
            hash_stall_reg <= 1'b0;

        end else begin
            if (expire_fifo_almost_full || task_almost_full) begin
                
                hash_stall_reg <= 1'b1;
            end else if (expired_fifo_empty && task_almost_empty) begin
                
                hash_stall_reg <= 1'b0;
            end
        end
    end
    assign hash_stall = hash_stall_reg;

    localparam integer DQ_DEPTH = LIFE_CYCLE;
    (* ram_style = "distributed" *) reg [ADDR_WIDTH-1:0] dq_mem[0:DQ_DEPTH-1];
    reg [$clog2(DQ_DEPTH)-1:0] dq_wr_ptr, dq_rd_ptr;
    reg [$clog2(DQ_DEPTH+1)-1:0] dq_count;
    reg                          cand_valid;
    reg [        ADDR_WIDTH-1:0] cand_addr;
    reg                          cand_pending_valid;
    reg [        ADDR_WIDTH-1:0] cand_pending_addr;
    reg [      TIMER_WIDTH-1:0] cand_pending_time;

    
    
    
    always @(posedge clk) begin
        if (write_commit) begin
            dq_mem[dq_wr_ptr] <= write_addr;
        end
    end

    
    
    
    wire [ADDR_WIDTH-1:0] dq_rdata = dq_mem[dq_rd_ptr];

    integer k;
    
    
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dq_wr_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_rd_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_count   <= {($clog2(DQ_DEPTH + 1)) {1'b0}};
            cand_valid <= 1'b0;
            cand_addr  <= {ADDR_WIDTH{1'b0}};
        end else if (frame_end || flush_done) begin
            
            dq_wr_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_rd_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_count   <= {($clog2(DQ_DEPTH + 1)) {1'b0}};
            cand_valid <= 1'b0;
            cand_addr  <= {ADDR_WIDTH{1'b0}};
            
        end else if (write_commit) begin

            dq_wr_ptr <= (dq_wr_ptr == DQ_DEPTH - 1) ? {($clog2(DQ_DEPTH)) {1'b0}} : (dq_wr_ptr + 1'b1);

            if (dq_count == DQ_DEPTH - 1) begin
                cand_addr  <= dq_rdata;  
                cand_valid <= 1'b1;
                dq_rd_ptr  <= (dq_rd_ptr == DQ_DEPTH - 1) ? {($clog2(DQ_DEPTH)) {1'b0}} : (dq_rd_ptr + 1'b1);
            end else begin
                dq_count   <= dq_count + 1'b1;
                cand_valid <= 1'b0;
            end

        end else begin
            cand_valid <= 1'b0;
            cand_addr  <= {ADDR_WIDTH{1'b0}};
        end
    end

    
    
    
    
    
    
    
    
    
    reg s0_valid;
    reg [ADDR_WIDTH-1:0] s0_addr;
    reg [TIMER_WIDTH-1:0] s0_time;
    reg s1_valid;
    reg s1_force_expire;  
    reg [ADDR_WIDTH-1:0] s1_addr;
    reg [TIMER_WIDTH-1:0] s1_time;
    reg [BUCKET_SLOT_WIDTH-1:0] s0_we;
    reg s0_force_expire;  

    wire [1:0] r_status;
    wire signed [COORD_WIDTH-1:0] r_key_x;
    wire signed [COORD_WIDTH-1:0] r_key_y;
    wire [VN_WIDTH-1:0] r_pn;
    wire [TIMER_WIDTH-1:0] r_ts;
    assign {r_status, r_key_x, r_key_y, r_pn, r_ts} = a_rdata_shadow;


    reg flushing;
    reg [ADDR_WIDTH:0] flush_cnt;  
    
    wire flush_run = flushing && (flush_cnt < TABLE_SIZE) && !hash_stall;

    wire normal_cand_valid = cand_pending_valid || cand_valid;
    wire [ADDR_WIDTH-1:0] normal_cand_addr = cand_pending_valid ? cand_pending_addr : cand_addr;
    wire [TIMER_WIDTH-1:0] normal_cand_time = cand_pending_valid ? cand_pending_time : time_now;

    
    wire eff_cand_valid = flush_run ? 1'b1 : normal_cand_valid;
    wire [ADDR_WIDTH-1:0] eff_cand_addr = flush_run ? flush_cnt[ADDR_WIDTH-1:0] : normal_cand_addr;
    wire [TIMER_WIDTH-1:0] eff_cand_time = flush_run ? time_now : normal_cand_time;
    
    
    wire pipe_advance_raw = flush_run | normal_cand_valid | s0_valid | s1_valid;
    wire expire_block;
    wire pipe_advance = pipe_advance_raw && !expire_block;
    assign expire_blocking = expire_block;

    wire normal_cand_consumed = pipe_advance && !flush_run && normal_cand_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_pending_valid <= 1'b0;
            cand_pending_addr  <= {ADDR_WIDTH{1'b0}};
            cand_pending_time  <= {TIMER_WIDTH{1'b0}};
        end else if (frame_end || flush_done) begin
            cand_pending_valid <= 1'b0;
            cand_pending_addr  <= {ADDR_WIDTH{1'b0}};
            cand_pending_time  <= {TIMER_WIDTH{1'b0}};
        end else if (cand_pending_valid) begin
            if (normal_cand_consumed) begin
                cand_pending_valid <= cand_valid;
                cand_pending_addr  <= cand_addr;
                cand_pending_time  <= time_now;
            end
        end else if (cand_valid && !normal_cand_consumed) begin
            cand_pending_valid <= 1'b1;
            cand_pending_addr  <= cand_addr;
            cand_pending_time  <= time_now;
        end
    end

    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_addr_shadow   <= {BUCKET_AW{1'b0}};
            a_we_shadow     <= {BUCKET_SLOT_WIDTH{1'b0}};

            s0_valid        <= 1'b0;
            s0_addr         <= {ADDR_WIDTH{1'b0}};
            s0_time         <= {TIMER_WIDTH{1'b0}};
            s0_we           <= {BUCKET_SLOT_WIDTH{1'b0}};
            s0_force_expire <= 1'b0;

            s1_valid        <= 1'b0;
            s1_addr         <= {ADDR_WIDTH{1'b0}};
            s1_time         <= {TIMER_WIDTH{1'b0}};
            s1_force_expire <= 1'b0;

        end else if (flush_done) begin
            a_addr_shadow   <= {BUCKET_AW{1'b0}};
            a_we_shadow     <= {BUCKET_SLOT_WIDTH{1'b0}};

            s0_valid        <= 1'b0;
            s0_addr         <= {ADDR_WIDTH{1'b0}};
            s0_time         <= {TIMER_WIDTH{1'b0}};
            s0_we           <= {BUCKET_SLOT_WIDTH{1'b0}};
            s0_force_expire <= 1'b0;

            s1_valid        <= 1'b0;
            s1_addr         <= {ADDR_WIDTH{1'b0}};
            s1_time         <= {TIMER_WIDTH{1'b0}};
            s1_force_expire <= 1'b0;

        end else if (pipe_advance) begin
            s1_force_expire <= s0_force_expire;  

            s1_valid        <= s0_valid;
            s1_addr         <= s0_addr;
            s1_time         <= s0_time;
            a_we_shadow     <= s0_we;  

            s0_valid        <= eff_cand_valid;
            s0_addr         <= eff_cand_addr;
            s0_time         <= eff_cand_time;
            s0_force_expire <= flush_run;  


            if (eff_cand_valid) begin
                a_addr_shadow <= eff_cand_addr[(ADDR_WIDTH-1)-:BUCKET_AW];
                s0_we <= eff_cand_addr[BUCKET_SLOT_WIDTH-1:0];
            end
        end
    end

    localparam [23:0] MAX_VOXEL_NUM = 24'd32;
    
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  
    
    
    localparam integer EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;
    localparam integer RESERVED_VOXEL_ROW = EXPEND_VOXEL_ROW;
    localparam [VN_WIDTH-1:0] POINTS_PER_ROW_V = POINTS_PER_ROW;
    localparam [VN_WIDTH-1:0] TWO_POINTS_PER_ROW_V = 2 * POINTS_PER_ROW;
    localparam [VN_WIDTH-1:0] THREE_POINTS_PER_ROW_V = 3 * POINTS_PER_ROW;
    localparam [BRAM_ADDR_WIDTH_PFE-1:0] RESERVED_VOXEL_ROW_V = RESERVED_VOXEL_ROW;


    wire [TIMER_WIDTH-1:0] ts_reg = (s1_valid) ? (s1_time - r_ts) : {TIMER_WIDTH{1'b0}};
    wire expired = (r_status == ST_OCCU) && (s1_force_expire || ts_reg >= LIFE_CYCLE);
    assign kill_expired = expired;
    
    

    
    
    
    
    
    
    localparam integer TASK_DEPTH = 16;
    localparam integer TASK_AW = 4;

    localparam TASK_WIDTH = ADDR_WIDTH + 2 * COORD_WIDTH + VN_WIDTH;
    reg [TASK_WIDTH-1:0] task_fifo_mem[0:TASK_DEPTH-1];  
    reg [TASK_AW-1:0] task_wr_ptr;
    reg [TASK_AW-1:0] task_rd_ptr;
    reg [$clog2(TASK_DEPTH+1)-1:0] task_count;

    wire task_empty = (task_count == 0);
    wire task_full = (task_count == TASK_DEPTH);

    wire [TASK_WIDTH-1:0] task_dout = task_fifo_mem[task_rd_ptr];
    
    wire [ADDR_WIDTH-1:0] task_addr = task_dout[TASK_WIDTH-1-:ADDR_WIDTH];
    wire signed [COORD_WIDTH-1:0] task_kx = task_dout[2*COORD_WIDTH+VN_WIDTH-1-:COORD_WIDTH];
    wire signed [COORD_WIDTH-1:0] task_ky = task_dout[COORD_WIDTH+VN_WIDTH-1-:COORD_WIDTH];
    wire [VN_WIDTH-1:0] task_pn = task_dout[VN_WIDTH-1-:VN_WIDTH];
    
    wire [2:0] task_copy_rows = (task_pn <= POINTS_PER_ROW_V) ? 3'd1 :
                                (task_pn <= TWO_POINTS_PER_ROW_V) ? 3'd2 :
                                (task_pn <= THREE_POINTS_PER_ROW_V) ? 3'd3 : 3'd4;

    
    
    
    reg copy_st;
    localparam ST_IDLE = 1'b0;
    localparam ST_COPY = 1'b1;
    
    reg  [    BRAM_ADDR_WIDTH-1:0] fsm_read_addr;
    reg                            fsm_read_en;
    
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] fsm_write_addr;
    
    
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] task_base_ptr;
    
    reg  [         ADDR_WIDTH-1:0] fsm_task_addr;
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] fsm_task_base_ptr;
    reg  [         ADDR_WIDTH-1:0] fsm_release_addr;

    
    
    reg  [                 EW-1:0] fsm_notify_data;
    reg                            fsm_read_last;
    reg  [                 EW-1:0] fsm_task_notify_data;
    reg  [                    2:0] fsm_copy_rows;  
    reg  [                    1:0] fsm_next_row_idx;  

    
    
    wire                           task_accept = (copy_st == ST_IDLE) && (!task_empty) && notify_has_room;

    
    wire                           task_pop = task_accept;
    
    wire                           task_pop_safe = task_pop && !task_empty;

    wire                           task_can_push = !task_full || task_pop;
    wire                           release_valid;
    wire                           expire_accept = s1_valid && expired && task_can_push && !notify_push;
    wire                           task_push = expire_accept;

    assign expire_block = s1_valid && expired && (!task_can_push || notify_push);
    assign task_almost_full = (task_count >= TASK_DEPTH - 3);
    assign task_almost_empty = (task_count <= 3);
    
    always @(posedge clk) begin
        if (task_push) begin
            task_fifo_mem[task_wr_ptr] <= {s1_addr, r_key_x, r_key_y, r_pn};
        end
    end

    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            task_wr_ptr <= {TASK_AW{1'b0}};
            task_rd_ptr <= {TASK_AW{1'b0}};
            task_count  <= {($clog2(TASK_DEPTH + 1)) {1'b0}};

        end else if (flush_done) begin
            task_wr_ptr <= {TASK_AW{1'b0}};
            task_rd_ptr <= {TASK_AW{1'b0}};
            task_count  <= {($clog2(TASK_DEPTH + 1)) {1'b0}};

        end else begin
            if (task_push) begin
                task_wr_ptr <= task_wr_ptr + 1'b1;
            end

            if (task_pop_safe) begin
                task_rd_ptr <= task_rd_ptr + 1'b1;
            end

            case ({
                task_push, task_pop_safe
            })
                2'b10:   task_count <= task_count + 1'b1;
                2'b01:   task_count <= task_count - 1'b1;
                2'b11:   task_count <= task_count;
                default: task_count <= task_count;
            endcase
        end
    end
    
    
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            copy_st           <= ST_IDLE;

            fsm_read_en       <= 1'b0;
            fsm_read_addr     <= {BRAM_ADDR_WIDTH{1'b0}};
            fsm_write_addr    <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last     <= 1'b0;

            fsm_task_addr     <= {ADDR_WIDTH{1'b0}};
            fsm_task_base_ptr <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_release_addr  <= {ADDR_WIDTH{1'b0}};
            fsm_copy_rows     <= 3'd0;
            fsm_next_row_idx  <= 2'd0;

            task_base_ptr     <= {BRAM_ADDR_WIDTH_PFE{1'b0}};

        end else if (flush_done) begin
            copy_st           <= ST_IDLE;

            fsm_read_en       <= 1'b0;
            fsm_read_addr     <= {BRAM_ADDR_WIDTH{1'b0}};
            fsm_write_addr    <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last     <= 1'b0;

            fsm_task_addr     <= {ADDR_WIDTH{1'b0}};
            fsm_task_base_ptr <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_release_addr  <= {ADDR_WIDTH{1'b0}};
            fsm_copy_rows     <= 3'd0;
            fsm_next_row_idx  <= 2'd0;

            task_base_ptr     <= {BRAM_ADDR_WIDTH_PFE{1'b0}};


        end else begin
            
            fsm_read_en     <= 1'b0;
            fsm_read_last   <= 1'b0;
            fsm_notify_data <= {EW{1'b0}};

            case (copy_st)

                
                
                
                
                
                ST_IDLE: begin
                    if (task_accept) begin
                        
                        fsm_task_addr        <= task_addr;
                        fsm_task_base_ptr    <= task_base_ptr;
                        fsm_release_addr     <= task_addr;
                        fsm_task_notify_data <= {task_kx, task_ky, task_base_ptr, task_pn};
                        fsm_copy_rows        <= task_copy_rows;
                        fsm_next_row_idx     <= 2'd1;

                        
                        fsm_read_en          <= 1'b1;
                        fsm_read_addr        <= task_addr * EXPEND_VOXEL_ROW;

                        
                        fsm_write_addr       <= task_base_ptr;

                        
                        if (task_copy_rows == 3'd1) begin
                            fsm_read_last   <= 1'b1;
                            fsm_notify_data <= {task_kx, task_ky, task_base_ptr, task_pn};
                            copy_st         <= ST_IDLE;
                        end else begin
                            fsm_read_last   <= 1'b0;
                            fsm_notify_data <= {EW{1'b0}};
                            copy_st         <= ST_COPY;
                        end

                        
                        task_base_ptr <= task_base_ptr + RESERVED_VOXEL_ROW_V;
                    end
                end

                
                
                
                
                
                
                ST_COPY: begin
                    if ((fsm_next_row_idx != (fsm_copy_rows - 1'b1)) || notify_has_room) begin
                        fsm_read_en <= 1'b1;

                        
                        fsm_read_addr <= (fsm_task_addr * EXPEND_VOXEL_ROW) + fsm_next_row_idx;
                        fsm_release_addr <= fsm_task_addr;

                        
                        fsm_write_addr <= fsm_task_base_ptr + {{(BRAM_ADDR_WIDTH_PFE - 2) {1'b0}}, fsm_next_row_idx};

                        if (fsm_next_row_idx == (fsm_copy_rows - 1'b1)) begin
                            
                            fsm_read_last   <= 1'b1;
                            fsm_notify_data <= fsm_task_notify_data;
                            copy_st         <= ST_IDLE;
                        end else begin
                            
                            fsm_read_last    <= 1'b0;
                            fsm_notify_data  <= {EW{1'b0}};
                            fsm_next_row_idx <= fsm_next_row_idx + 1'b1;
                            copy_st          <= ST_COPY;
                        end
                    end else begin
                        
                        copy_st <= ST_COPY;
                    end
                end

                default: begin
                    copy_st <= ST_IDLE;
                end
            endcase
        end
    end



    assign bram_expire_addr_a = fsm_read_addr;
    
    
    
    
    
    

    reg                           fsm_read_en_d1;
    reg [BRAM_ADDR_WIDTH_PFE-1:0] fsm_write_addr_d1;
    reg                           fsm_read_last_d1;
    reg [                 EW-1:0] fsm_notify_data_d1;
    reg [         ADDR_WIDTH-1:0] fsm_release_addr_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_read_en_d1      <= 1'b0;
            fsm_write_addr_d1   <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last_d1    <= 1'b0;
            fsm_release_addr_d1 <= {ADDR_WIDTH{1'b0}};

            bram_expire_wr_b    <= 1'b0;
            bram_expire_addr_b  <= {BRAM_ADDR_WIDTH_PFE{1'b0}};

        end else if (flush_done) begin
            fsm_read_en_d1      <= 1'b0;
            fsm_write_addr_d1   <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last_d1    <= 1'b0;
            fsm_release_addr_d1 <= {ADDR_WIDTH{1'b0}};

            bram_expire_wr_b    <= 1'b0;
            bram_expire_addr_b  <= {BRAM_ADDR_WIDTH_PFE{1'b0}};

        end else begin
            fsm_read_en_d1      <= fsm_read_en;
            fsm_write_addr_d1   <= fsm_write_addr;
            fsm_read_last_d1    <= fsm_read_last;
            fsm_notify_data_d1  <= fsm_notify_data;
            fsm_release_addr_d1 <= fsm_release_addr;

            if (fsm_read_en_d1) begin
                bram_expire_wr_b     <= 1'b1;
                bram_expire_addr_b   <= fsm_write_addr_d1;
                bram_expire_wrdata_b <= bram_expire_rdata_a;
            end else begin
                bram_expire_wr_b <= 1'b0;
            end
        end
    end

    
    
    
    
    assign notify_push      = fsm_read_en_d1 && fsm_read_last_d1;
    assign notify_push_data = fsm_notify_data_d1;
    assign release_valid    = notify_push;

    
    
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kill_valid <= 1'b0;
            b_we       <= {BUCKET_SLOT_WIDTH{1'b0}};
            b_addr     <= {BUCKET_AW{1'b0}};

        end else if (flush_done) begin
            kill_valid <= 1'b0;
            b_we       <= {BUCKET_SLOT_WIDTH{1'b0}};
            b_addr     <= {BUCKET_AW{1'b0}};

        end else begin
            if (release_valid) begin
                kill_valid <= 1'b1;
                b_we       <= fsm_release_addr_d1[BUCKET_SLOT_WIDTH-1:0];
                b_addr     <= fsm_release_addr_d1[ADDR_WIDTH-1-:BUCKET_AW];
                b_wdata    <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
            end else if (expire_accept) begin  
                kill_valid <= 1'b1;
                b_we       <= s1_addr[BUCKET_SLOT_WIDTH-1:0];
                b_addr     <= s1_addr[ADDR_WIDTH-1-:BUCKET_AW];
                b_wdata    <= {ST_DRAIN, r_key_x, r_key_y, r_pn, r_ts};
            end else begin
                kill_valid <= 1'b0;
                b_we       <= {BUCKET_SLOT_WIDTH{1'b0}};
            end
        end
    end


    
    
    

    
    
    
    
    
    
    wire all_drained = (flush_cnt == TABLE_SIZE) &&
                        (!s0_force_expire && !s1_force_expire) &&
                        task_empty &&
                        (copy_st == ST_IDLE) &&
                        (!fsm_read_en) &&
                        (!fsm_read_en_d1) &&
                        (!bram_expire_wr_b) &&
                        notify_empty &&
                        (fifo_out_level == 0 && !fifo_out_valid);
    
    assign flush_done = flushing && all_drained;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flushing  <= 1'b0;
            flush_cnt <= {(ADDR_WIDTH + 1) {1'b0}};
        end else begin
            if (frame_end && !flushing) begin
                flushing  <= 1'b1;
                flush_cnt <= {(ADDR_WIDTH + 1) {1'b0}};
            end else if (flushing) begin
                if (flush_cnt < TABLE_SIZE) begin
                    if (!hash_stall && !expire_block) begin
                        flush_cnt <= flush_cnt + 1'b1;
                    end
                end else if (all_drained) begin
                    flushing  <= 1'b0;
                    flush_cnt <= {(ADDR_WIDTH + 1) {1'b0}};
                end
            end
        end
    end


endmodule
