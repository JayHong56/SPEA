module voxel_coor_pipe #(
    parameter integer BRAM_DATA_WIDTH = 576,  
    parameter integer BRAM_ADDR_WIDTH = 10,  
    parameter integer BRAM_ADDR_WIDTH_PFE = 8,  
    parameter integer DRAM_DATA_WIDTH = 128,  
    parameter integer DRAM_ADDR_WIDTH = 18,
    parameter integer HASH_ADDR_WIDTH = 8,  
    parameter [15:0] THRESHOLD_CLOSE = 16'h0100,  
    parameter [20-1:0] THRESHOLD_BOUDARY_X_LOW = 20'h0000,  
    parameter [20-1:0] THRESHOLD_BOUDARY_X_HIGH = 20'h451ec,  
    parameter [20-1:0] THRESHOLD_BOUDARY_Y = 20'h27ae1,  
    parameter [15:0] THRESHOLD_BOUDARY_Z_LOW = 16'h3000,  
    parameter [15:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h1000,  
    parameter integer PRE_LAT = 0,  
    parameter [15:0] LIFE_CYCLE = 16'd100,
    parameter integer VN_WIDTH = 6,  
    parameter [23:0] MAX_VOXEL_NUM = 24'd32,  
    parameter integer BYTE_WIDTH = 9  
) (
    input wire clk,
    input wire rst_n,

    input wire frame_end,  
    output wire flush_done,  
    output wire hash_stall,
    
    input wire [DRAM_DATA_WIDTH-1:0] s_axis_dram_data,
    input wire [DRAM_DATA_WIDTH/8-1:0] s_axis_dram_keep,
    input wire s_axis_dram_last,
    input wire s_axis_dram_valid,
    output wire s_axis_dram_ready,
    
    output reg bram_voxelpoint_wr,
    output reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen,
    output reg [BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr,
    output reg [BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata,
    
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    
    output wire bram_expire_wr_b,
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    
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

    localparam integer PT_WIDTH_XY = PT_WIDTH_I_XY + PT_WIDTH_F_XY;  
    localparam integer PT_WIDTH = 16;  
    localparam integer PT_WIDTH_PER = 2 * PT_WIDTH_XY + 2 * PT_WIDTH;  
    localparam integer PT_WIDTH_FLOAT32 = 32;
    localparam integer VOXEL_WIDTH = 11;  

    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  
    localparam integer EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  
    localparam integer RAW_PT_WIDTH = 4 * PT_WIDTH_FLOAT32;  
    localparam [VN_WIDTH-1:0] MAX_VOXEL_POINTS = MAX_VOXEL_NUM[VN_WIDTH-1:0];
    

    wire                          hash_req_ready;
    wire                          hash_req_valid;
    wire                          hash_fire = hash_req_valid && hash_req_ready;

    reg                           keep_point;
    reg  [PT_WIDTH_FLOAT32*4-1:0] point_raw_r;
    reg                           point_raw_vld;
    wire                          s_fire = s_axis_dram_valid && s_axis_dram_ready;
    
    
    
    
    wire                          upstream_can_accept;
    assign s_axis_dram_ready = upstream_can_accept && !flush_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            point_raw_vld <= 1'b0;
        end else begin
            if (s_fire) begin
                point_raw_r   <= s_axis_dram_data;
                point_raw_vld <= 1'b1;
            end else if (upstream_can_accept) begin
                
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


    
    always @(*) begin
        if (!rst_n) begin
            keep_point = 1'b0;
            
            
            
            
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

    
    
    
    localparam integer SCALE_VOXEL_FACTOR = 20;
    localparam integer SCALE_VOXEL = 16;
    localparam signed [SCALE_VOXEL_FACTOR-1:0] MULT_FACTOR = 20'd409600;  
    localparam integer ROUND = 1;
    localparam signed [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_Y_CEIL_FIX = 20'sh27ae2;  
    
    
    
    
    
    
    
    
    
    

    
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


    
    wire [VOXEL_WIDTH-1:0] single_voxel_x;
    wire [VOXEL_WIDTH-1:0] single_voxel_y;

    assign single_voxel_x = voxel_x_scaled[VOXEL_WIDTH-1:0];
    assign single_voxel_y = voxel_y_scaled[VOXEL_WIDTH-1:0];


    wire [PT_WIDTH_PER-1:0] single_point_proc;
    assign single_point_proc = {point_x_fix20_data, point_y_fix20_data, point_z_fix16, point_intensity_fix16};

    
    
    
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

    
    wire pipe1_stall = pipe1_valid && !hash_fire;
    assign upstream_can_accept = !pipe1_stall;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_valid      <= 1'b0;
            pipe1_keep_point <= 1'b0;
        end else begin
            if (!pipe1_stall) begin
                
                pipe1_valid      <= point_raw_vld && keep_point;
                pipe1_keep_point <= keep_point;

                
                pipe1_voxel_x    <= single_voxel_x;
                pipe1_voxel_y    <= single_voxel_y;
                pipe1_point_proc <= single_point_proc;
            end
        end
    end

    
    
    
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
        .TIMER_WIDTH        (DRAM_ADDR_WIDTH),     
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
        .out_idx             (hash_out_idx),           
        .out_point_number    (hash_out_point_number),  
        .table_full          (hash_table_full),
        .bram_expire_addr_a  (bram_expire_addr_a),     
        .bram_expire_rdata_a (bram_expire_rdata_a),    
        .bram_expire_wr_b    (bram_expire_wr_b),
        .bram_expire_addr_b  (bram_expire_addr_b),     
        .bram_expire_wrdata_b(bram_expire_wrdata_b),   
        .m_axis_expire_tready(m_axis_expire_tready),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tdata (m_axis_expire_tdata)
    );

    
    reg [PT_WIDTH_PER-1:0] pproc_d1, pproc_d2, pproc_d3;
    reg hash_req_ready_1delay;
    reg pproc_drop_d1, pproc_drop_d2, pproc_drop_d3;
    reg [5:0] full_voxel_lfsr;
    reg [BRAM_ADDR_WIDTH-1:0] full_rand_row_d1, full_rand_row_d2, full_rand_row_d3;
    reg [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_d1, full_rand_slot_d2, full_rand_slot_d3;
    reg [5:0] full_rand_value_d1, full_rand_value_d2, full_rand_value_d3;
    reg [7:0] full_point_count[0:(1 << HASH_ADDR_WIDTH)-1];
    integer full_cnt_i;

    wire full_voxel_lfsr_feedback = full_voxel_lfsr[5] ^ full_voxel_lfsr[4];

    function [4:0] full_rand_choice_32;
        input [5:0] lfsr_state;
        reg [5:0] nonzero_state;
        reg [5:0] choice6;
        begin
            nonzero_state = (lfsr_state == 6'd0) ? 6'd1 : lfsr_state;
            if (nonzero_state <= 6'd32) begin
                choice6 = nonzero_state - 6'd1;
            end else begin
                choice6 = nonzero_state - 6'd33;
            end
            full_rand_choice_32 = choice6[4:0];
        end
    endfunction

    function [BRAM_ADDR_WIDTH-1:0] full_rand_row_offset;
        input [4:0] choice;
        begin
            if (choice < 5'd8) begin
                full_rand_row_offset = {BRAM_ADDR_WIDTH{1'b0}};
            end else if (choice < 5'd16) begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 1) {1'b0}}, 1'b1};
            end else if (choice < 5'd24) begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 2) {1'b0}}, 2'd2};
            end else begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 2) {1'b0}}, 2'd3};
            end
        end
    endfunction

    function [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_idx;
        input [4:0] choice;
        begin
            case (choice)
                5'd0, 5'd8, 5'd16, 5'd24: full_rand_slot_idx = 3'd0;
                5'd1, 5'd9, 5'd17, 5'd25: full_rand_slot_idx = 3'd1;
                5'd2, 5'd10, 5'd18, 5'd26: full_rand_slot_idx = 3'd2;
                5'd3, 5'd11, 5'd19, 5'd27: full_rand_slot_idx = 3'd3;
                5'd4, 5'd12, 5'd20, 5'd28: full_rand_slot_idx = 3'd4;
                5'd5, 5'd13, 5'd21, 5'd29: full_rand_slot_idx = 3'd5;
                5'd6, 5'd14, 5'd22, 5'd30: full_rand_slot_idx = 3'd6;
                5'd7, 5'd15, 5'd23, 5'd31: full_rand_slot_idx = 3'd7;
                default: full_rand_slot_idx = 3'd0;
            endcase
        end
    endfunction

    wire [4:0] full_rand_choice = full_rand_choice_32(full_voxel_lfsr);
    wire [BRAM_ADDR_WIDTH-1:0] full_rand_row_next = full_rand_row_offset(full_rand_choice);
    wire [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_next = full_rand_slot_idx(full_rand_choice);

    function [5:0] full_keep_threshold;
        input [7:0] total_count;
        begin
            case (total_count)
                8'd0, 8'd1, 8'd2, 8'd3, 8'd4, 8'd5, 8'd6, 8'd7,
                8'd8, 8'd9, 8'd10, 8'd11, 8'd12, 8'd13, 8'd14, 8'd15,
                8'd16, 8'd17, 8'd18, 8'd19, 8'd20, 8'd21, 8'd22, 8'd23,
                8'd24, 8'd25, 8'd26, 8'd27, 8'd28, 8'd29, 8'd30, 8'd31,
                8'd32:
                full_keep_threshold = 6'd63;
                8'd33: full_keep_threshold = 6'd62;
                8'd34: full_keep_threshold = 6'd60;
                8'd35: full_keep_threshold = 6'd58;
                8'd36: full_keep_threshold = 6'd57;
                8'd37: full_keep_threshold = 6'd55;
                8'd38: full_keep_threshold = 6'd54;
                8'd39: full_keep_threshold = 6'd52;
                8'd40: full_keep_threshold = 6'd51;
                8'd41: full_keep_threshold = 6'd50;
                8'd42: full_keep_threshold = 6'd48;
                8'd43: full_keep_threshold = 6'd47;
                8'd44: full_keep_threshold = 6'd46;
                8'd45: full_keep_threshold = 6'd45;
                8'd46: full_keep_threshold = 6'd44;
                8'd47: full_keep_threshold = 6'd43;
                8'd48: full_keep_threshold = 6'd42;
                8'd49: full_keep_threshold = 6'd41;
                8'd50: full_keep_threshold = 6'd40;
                8'd51: full_keep_threshold = 6'd40;
                8'd52: full_keep_threshold = 6'd39;
                8'd53: full_keep_threshold = 6'd38;
                8'd54: full_keep_threshold = 6'd37;
                8'd55: full_keep_threshold = 6'd37;
                8'd56: full_keep_threshold = 6'd36;
                8'd57: full_keep_threshold = 6'd35;
                8'd58: full_keep_threshold = 6'd35;
                8'd59: full_keep_threshold = 6'd34;
                8'd60: full_keep_threshold = 6'd34;
                8'd61: full_keep_threshold = 6'd33;
                8'd62: full_keep_threshold = 6'd33;
                8'd63, 8'd64: full_keep_threshold = 6'd32;
                default: begin
                    if (total_count <= 8'd84) begin
                        full_keep_threshold = 6'd24;
                    end else if (total_count <= 8'd126) begin
                        full_keep_threshold = 6'd16;
                    end else if (total_count <= 8'd180) begin
                        full_keep_threshold = 6'd11;
                    end else begin
                        full_keep_threshold = 6'd8;
                    end
                end
            endcase
        end
    endfunction



    wire pipe_pproc_stall = hash_req_ready;
    wire pipe_bram_stall = hash_req_ready_1delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pproc_vld_d1 <= 1'b0;
            pproc_vld_d2 <= 1'b0;
            pproc_vld_d3 <= 1'b0;
            pproc_drop_d1 <= 1'b0;
            pproc_drop_d2 <= 1'b0;
            pproc_drop_d3 <= 1'b0;
            full_rand_row_d1 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_row_d2 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_row_d3 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_slot_d1 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_slot_d2 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_slot_d3 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_value_d1 <= 6'd1;
            full_rand_value_d2 <= 6'd1;
            full_rand_value_d3 <= 6'd1;
        end else if (pipe_pproc_stall) begin  
            pproc_drop_d1      <= !pipe1_keep_point;
            pproc_drop_d2      <= pproc_drop_d1;
            pproc_drop_d3      <= pproc_drop_d2;

            full_rand_row_d1   <= full_rand_row_next;
            full_rand_row_d2   <= full_rand_row_d1;
            full_rand_row_d3   <= full_rand_row_d2;
            full_rand_slot_d1  <= full_rand_slot_next;
            full_rand_slot_d2  <= full_rand_slot_d1;
            full_rand_slot_d3  <= full_rand_slot_d2;
            full_rand_value_d1 <= full_voxel_lfsr;
            full_rand_value_d2 <= full_rand_value_d1;
            full_rand_value_d3 <= full_rand_value_d2;

            pproc_vld_d1       <= hash_fire;
            pproc_vld_d2       <= pproc_vld_d1;
            pproc_vld_d3       <= pproc_vld_d2;

            pproc_d1           <= pipe1_point_proc;
            pproc_d2           <= pproc_d1;
            pproc_d3           <= pproc_d2;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            full_voxel_lfsr <= 6'h2d;
        end else if (hash_fire) begin
            full_voxel_lfsr <= {full_voxel_lfsr[4:0], full_voxel_lfsr_feedback};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_req_ready_1delay <= 1'b0;
        end else begin
            hash_req_ready_1delay <= hash_req_ready;
        end
    end
    
    
    
    wire [VN_WIDTH-1:0] pt_idx_minus_1 = hash_out_point_number - 1'b1;
    wire [BRAM_ADDR_WIDTH-1:0] row_offset = pt_idx_minus_1 / POINTS_PER_ROW;
    wire [$clog2(POINTS_PER_ROW)-1:0] bram_slot_idx = pt_idx_minus_1 % POINTS_PER_ROW;
    wire [BRAM_ADDR_WIDTH-1:0] bram_row_addr = (hash_out_idx * EXPEND_VOXEL_ROW) + row_offset;
    wire hash_point_number_valid = (hash_out_point_number <= MAX_VOXEL_POINTS) && (hash_out_point_number != {VN_WIDTH{1'b0}});
    wire [7:0] max_voxel_points_u8 = {{(8 - VN_WIDTH) {1'b0}}, MAX_VOXEL_POINTS};
    wire [7:0] hash_point_number_u8 = {{(8 - VN_WIDTH) {1'b0}}, hash_out_point_number};
    wire [7:0] full_point_count_cur = full_point_count[hash_out_idx];
    wire hash_point_number_full = (hash_out_point_number == MAX_VOXEL_POINTS) && (full_point_count_cur >= max_voxel_points_u8);
    wire [7:0] full_point_count_inc = (full_point_count_cur == 8'hff) ? 8'hff : (full_point_count_cur + 8'd1);
    wire [7:0] full_point_count_next = hash_point_number_full ? full_point_count_inc : hash_point_number_u8;
    wire [5:0] full_keep_threshold_sel = full_keep_threshold(full_point_count_next);
    wire full_voxel_random_write = !hash_point_number_full || (full_rand_value_d3 <= full_keep_threshold_sel);
    wire [BRAM_ADDR_WIDTH-1:0] bram_write_addr_sel =
        hash_point_number_full ? ((hash_out_idx * EXPEND_VOXEL_ROW) + full_rand_row_d3) : bram_row_addr;
    wire [$clog2(POINTS_PER_ROW)-1:0] bram_slot_idx_sel = hash_point_number_full ? full_rand_slot_d3 : bram_slot_idx;

    localparam integer CHUNKS_PER_POINT = PT_WIDTH_PER / BYTE_WIDTH;  
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
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            bram_voxelpoint_addr <= {BRAM_ADDR_WIDTH{1'b0}};
            for (full_cnt_i = 0; full_cnt_i < (1 << HASH_ADDR_WIDTH); full_cnt_i = full_cnt_i + 1) begin
                full_point_count[full_cnt_i] <= 8'd0;
            end
        end else begin
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            
            if (pipe_bram_stall) begin
                
                
                if (!pproc_drop_d3 && hash_point_number_valid) begin
                    full_point_count[hash_out_idx] <= full_point_count_next;
                    if (full_voxel_random_write) begin
                        bram_voxelpoint_wr     <= 1'b1;
                        bram_voxelpoint_addr   <= bram_write_addr_sel;
                        bram_voxelpoint_wrdata <= {POINTS_PER_ROW{pproc_d3}};
                        bram_voxelpoint_bwen   <= slot_bwen_64(bram_slot_idx_sel);
                    end
                end
            end
        end
    end




`ifndef SYNTHESIS
    
    
    
    real dbg_pt_x, dbg_pt_y, dbg_pt_z, dbg_pt_i;
    real orig_float_x, orig_float_y, orig_float_z, orig_float_i;

    
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
                
                
                decode_float32 = (sign ? -1.0 : 1.0) * (2.0 ** (exp - 127)) * (1.0 + ($itor(frac) / 8388608.0));
            end
        end
    endfunction

    
    
    
    
    
    
    
    

    
    
    
    
    

    
    
    
    
    
    

    
    
    
    
    
    

    
    
    
    
    
    

    
    
    
    
    
    
    
    


    
    
    
    
    
    
    
    
    
    
    
    

    wire test_coor = (point_y_fix20_data == 20'h15c28);
    reg [VOXEL_WIDTH-1:0] voxel_x_test = 11'd38;
    reg [VOXEL_WIDTH-1:0] voxel_y_test = 11'd247;

    wire test_reg = (single_voxel_x == voxel_x_test) && (single_voxel_y == voxel_y_test);
    wire test_vector = (bram_voxelpoint_addr == 9'd80) && ((bram_voxelpoint_bwen == 80'h000000000000000000ff));
    
    
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    
    
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    
    


    

`endif

endmodule
