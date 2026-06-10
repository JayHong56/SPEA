
module pfe #(
    parameter COORD_WIDTH = 11,
    parameter PT_WIDTH = 16,
    parameter PT_WIDTH_PER = 72,
    
    parameter MAX_VOXEL_NUM = 12'd32,
    parameter VN_WIDTH = 6,
    parameter EXPAND_PT_DIM = 9,  
    parameter integer BRAM_DATA_WIDTH = 576,  
    parameter integer BRAM_ADDR_WIDTH = 10,  
    parameter integer BRAM_ADDR_WIDTH_PFE = 8
) (
    input wire clk,
    input wire rst_n,
    input wire s_axis_expire_tvalid,
    output wire s_axis_expire_tready,
    input wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] s_axis_expire_tdata,  
    
    
    output reg [BRAM_ADDR_WIDTH_PFE-1:0] bram_pfe_addr,  
    
    input  wire       [                           BRAM_DATA_WIDTH-1:0] bram_pfe_rdata,          
    input wire m_axis_pfe_tready,
    output reg m_axis_pfe_valid,
    output reg [EXPAND_PT_DIM*PT_WIDTH-1:0] m_axis_pfe_data,  
    output reg m_axis_pfe_last,
    output reg m_axis_pfe_voxel_valid,
    output reg signed [COORD_WIDTH-1:0] m_axis_pfe_voxel_x,
    output reg signed [COORD_WIDTH-1:0] m_axis_pfe_voxel_y,
    output wire m_axis_pfe_flush_done,  
    input wire flush_done
);
    
    wire signed [COORD_WIDTH-1:0] axis_voxel_x = s_axis_expire_tdata[(2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire signed [COORD_WIDTH-1:0] axis_voxel_y = s_axis_expire_tdata[(COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:COORD_WIDTH];
    wire [BRAM_ADDR_WIDTH_PFE-1:0] axis_bram_index = s_axis_expire_tdata[(BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1)-:BRAM_ADDR_WIDTH_PFE];
    wire [VN_WIDTH-1:0] axis_point_num_indicator = s_axis_expire_tdata[(VN_WIDTH-1) : 0];  

    
    
    
    
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  
    
    localparam integer EXPAND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  
    localparam [VN_WIDTH-1:0] POINTS_PER_ROW_V = POINTS_PER_ROW;
    localparam [VN_WIDTH-1:0] TWO_POINTS_PER_ROW_V = 2 * POINTS_PER_ROW;
    localparam [VN_WIDTH-1:0] THREE_POINTS_PER_ROW_V = 3 * POINTS_PER_ROW;

    
    
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

    
    
    localparam integer ST3_SCALE = 1 + 17;
    localparam signed [ST3_SCALE-1:0] COEFF_SLOPE = 18'sd10486;  
    localparam signed [ST3_SCALE-1:0] COEFF_OFFSET_X = 18'sd5243;  
    localparam signed [ST3_SCALE+$clog2(40)-1:0] COEFF_OFFSET_Y = 24'sd2595226;  

    
    
    
    wire       pfe_frame_clear;  
    reg [2:0] inflight_cnt;
    wire fire_in;

    
    wire st2_to_st3;

    
    
    
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
                2'b10:   inflight_cnt <= inflight_cnt + 1'b1;  
                2'b01:   inflight_cnt <= inflight_cnt - 1'b1;  
                default: inflight_cnt <= inflight_cnt;
            endcase
        end
    end

    
    integer k, i;
    
    
    reg more_row_pending;
    reg [1:0] next_row_idx;
    reg [2:0] st1_total_rows;
    reg [VN_WIDTH-1:0] st1_point_num;
    reg [COORD_WIDTH-1:0] st1_voxel_x;
    reg [COORD_WIDTH-1:0] st1_voxel_y;
    reg [BRAM_ADDR_WIDTH_PFE-1:0] st1_bram_b_addr;

    wire [                    2:0] axis_need_rows = (axis_point_num_indicator <= POINTS_PER_ROW_V) ? 3'd1 :
                                                    (axis_point_num_indicator <= TWO_POINTS_PER_ROW_V) ? 3'd2 :
                                                    (axis_point_num_indicator <= THREE_POINTS_PER_ROW_V) ? 3'd3 : 3'd4;
    wire ready_to_accept = !more_row_pending && !pipe_stall;
    assign s_axis_expire_tready = ready_to_accept;
    assign fire_in              = s_axis_expire_tvalid && ready_to_accept;

    
    wire issue_row0 = fire_in;
    wire issue_more_row = more_row_pending;
    wire issue_valid = issue_row0 || issue_more_row;

    
    wire [1:0] issue_row_idx = issue_row0 ? 2'd0 : next_row_idx;

    
    wire issue_last = issue_row0 ? (axis_need_rows == 3'd1) : (next_row_idx == (st1_total_rows - 1'b1));

    
    wire [VN_WIDTH-1:0] issue_pnum = issue_row0 ? axis_point_num_indicator : st1_point_num;
    wire [COORD_WIDTH-1:0] issue_vx = issue_row0 ? axis_voxel_x : st1_voxel_x;
    wire [COORD_WIDTH-1:0] issue_vy = issue_row0 ? axis_voxel_y : st1_voxel_y;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_base_addr = issue_row0 ? axis_bram_index : st1_bram_b_addr;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] issue_addr = issue_base_addr + {{(BRAM_ADDR_WIDTH_PFE - 2) {1'b0}}, issue_row_idx};
    
    
    
    
    
    always @(*) begin
        if (issue_valid) begin
            bram_pfe_addr = issue_addr;
        end else begin
            bram_pfe_addr = {BRAM_ADDR_WIDTH_PFE{1'b0}};
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            more_row_pending <= 1'b0;
            next_row_idx     <= 2'd0;
            st1_total_rows   <= 3'd0;

        end else if (pfe_frame_clear) begin
            more_row_pending <= 1'b0;
            next_row_idx     <= 2'd0;
            st1_total_rows   <= 3'd0;

        end else begin
            if (fire_in) begin
                st1_point_num   <= axis_point_num_indicator;
                st1_voxel_x     <= axis_voxel_x;
                st1_voxel_y     <= axis_voxel_y;
                st1_bram_b_addr <= axis_bram_index;
                st1_total_rows  <= axis_need_rows;

                if (axis_need_rows == 3'd1) begin
                    more_row_pending <= 1'b0;
                    next_row_idx     <= 2'd0;
                end else begin
                    more_row_pending <= 1'b1;
                    next_row_idx     <= 2'd1;
                end
            end else if (more_row_pending) begin
                if (next_row_idx == (st1_total_rows - 1'b1)) begin
                    
                    more_row_pending <= 1'b0;
                    next_row_idx     <= 2'd0;
                end else begin
                    
                    next_row_idx <= next_row_idx + 1'b1;
                end
            end
        end
    end
    
    
    
    
    
    

    reg                   rd_valid_d1;
    reg [            1:0] rd_row_idx_d1;
    reg                   rd_last_d1;
    reg [   VN_WIDTH-1:0] rd_pnum_d1;
    reg [COORD_WIDTH-1:0] rd_vx_d1;
    reg [COORD_WIDTH-1:0] rd_vy_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_valid_d1   <= 1'b0;
            rd_row_idx_d1 <= 2'd0;
            rd_last_d1    <= 1'b0;

        end else if (pfe_frame_clear) begin
            rd_valid_d1   <= 1'b0;
            rd_row_idx_d1 <= 2'd0;
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

    
    
    
    
    
    
    
    
    
    
    

    reg        [PT_WIDTH_PER-1:0] st2_points_buffer                       [0:MAX_VOXEL_NUM-1];

    reg signed [SUM_XY_WIDTH-1:0] sum_x_acc;
    reg signed [SUM_XY_WIDTH-1:0] sum_y_acc;
    reg signed [ SUM_Z_WIDTH-1:0] sum_z_acc;

    wire                          st2_rd_valid = rd_valid_d1;
    wire       [             1:0] st2_row_idx = rd_row_idx_d1;
    wire                          st2_last_row = rd_last_d1;
    wire       [    VN_WIDTH-1:0] st2_pnum = rd_pnum_d1;
    wire       [ COORD_WIDTH-1:0] st2_vx = rd_vx_d1;
    wire       [ COORD_WIDTH-1:0] st2_vy = rd_vy_d1;

    wire       [    VN_WIDTH-1:0] base_idx = st2_row_idx * POINTS_PER_ROW;

    reg        [PT_WIDTH_PER-1:0] st2_point_data;

    reg signed [SUM_XY_WIDTH-1:0] sum_x_step;
    reg signed [SUM_XY_WIDTH-1:0] sum_y_step;
    reg signed [ SUM_Z_WIDTH-1:0] sum_z_step;


    
    reg                           st2_valid;
    reg        [    VN_WIDTH-1:0] st2_done_pnum;
    reg signed [SUM_XY_WIDTH-1:0] st2_done_sum_x;
    reg signed [SUM_XY_WIDTH-1:0] st2_done_sum_y;
    reg signed [ SUM_Z_WIDTH-1:0] st2_done_sum_z;
    reg signed [ COORD_WIDTH-1:0] st2_done_vx;
    reg signed [ COORD_WIDTH-1:0] st2_done_vy;


    
    
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

                
                
                if (st2_row_idx == 2'd0) begin
                    sum_x_acc <= sum_x_step;
                    sum_y_acc <= sum_y_step;
                    sum_z_acc <= sum_z_step;
                end else begin
                    sum_x_acc <= sum_x_acc + sum_x_step;
                    sum_y_acc <= sum_y_acc + sum_y_step;
                    sum_z_acc <= sum_z_acc + sum_z_step;
                end

                
                if (st2_last_row) begin
                    st2_valid     <= 1'b1;
                    st2_done_pnum <= st2_pnum;
                    st2_done_vx   <= st2_vx;
                    st2_done_vy   <= st2_vy;

                    if (st2_row_idx == 2'd0) begin
                        
                        st2_done_sum_x <= sum_x_step;
                        st2_done_sum_y <= sum_y_step;
                        st2_done_sum_z <= sum_z_step;
                    end else begin
                        
                        st2_done_sum_x <= sum_x_acc + sum_x_step;
                        st2_done_sum_y <= sum_y_acc + sum_y_step;
                        st2_done_sum_z <= sum_z_acc + sum_z_step;
                    end
                end
            end
        end
    end


    
    
    
    reg  st3_valid;
    reg  st4_valid;

    
    
    wire fire_out = m_axis_pfe_valid && m_axis_pfe_tready;
    wire st4_push;
    wire st4_ready_for_new = !st4_valid || st4_push;

    
    wire st3_to_st4 = st3_valid && st4_ready_for_new;

    
    
    
    wire st3_ready_for_s2 = !st3_valid || st3_to_st4;

    
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
                
                st3_valid <= 1'b0;
            end
        end
    end


    
    reg signed [ST3_SCALE-1:0] st3_reciprocal;  
    always @(*) begin
        case (st3_pnum)
            6'd1: st3_reciprocal = 18'd65536;
            6'd2: st3_reciprocal = 18'd32768;
            6'd3: st3_reciprocal = 18'd21845;
            6'd4: st3_reciprocal = 18'd16384;
            6'd5: st3_reciprocal = 18'd13107;
            6'd6: st3_reciprocal = 18'd10923;
            6'd7: st3_reciprocal = 18'd9362;
            6'd8: st3_reciprocal = 18'd8192;
            6'd9: st3_reciprocal = 18'd7282;
            6'd10: st3_reciprocal = 18'd6554;
            6'd11: st3_reciprocal = 18'd5958;
            6'd12: st3_reciprocal = 18'd5461;
            6'd13: st3_reciprocal = 18'd5041;
            6'd14: st3_reciprocal = 18'd4681;
            6'd15: st3_reciprocal = 18'd4369;
            6'd16: st3_reciprocal = 18'd4096;
            6'd17: st3_reciprocal = 18'd3855;
            6'd18: st3_reciprocal = 18'd3641;
            6'd19: st3_reciprocal = 18'd3449;
            6'd20: st3_reciprocal = 18'd3277;
            6'd21: st3_reciprocal = 18'd3121;
            6'd22: st3_reciprocal = 18'd2979;
            6'd23: st3_reciprocal = 18'd2850;
            6'd24: st3_reciprocal = 18'd2731;
            6'd25: st3_reciprocal = 18'd2621;
            6'd26: st3_reciprocal = 18'd2521;
            6'd27: st3_reciprocal = 18'd2427;
            6'd28: st3_reciprocal = 18'd2341;
            6'd29: st3_reciprocal = 18'd2260;
            6'd30: st3_reciprocal = 18'd2185;
            6'd31: st3_reciprocal = 18'd2114;
            6'd32: st3_reciprocal = 18'd2048;
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

    
    reg [VN_WIDTH-1:0] st4_pnum;
    reg [PT_WIDTH_PER-1:0] st4_points_buffer[0:MAX_VOXEL_NUM-1];
    reg signed [PT_WIDTH_XY-1:0] st4_mean_x, st4_mean_y;
    reg signed [PT_WIDTH_Z-1:0] st4_mean_z;
    reg signed [PT_WIDTH_XY-1:0] st4_vx_fixxy, st4_vy_fixxy;
    reg signed [COORD_WIDTH-1:0] st4_voxel_x, st4_voxel_y;

    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st4_valid <= 1'b0;

        end else if (pfe_frame_clear) begin
            st4_valid <= 1'b0;

        end else begin
            
            if (st3_to_st4) begin
                st4_valid    <= 1'b1;
                st4_pnum     <= st3_pnum;

                
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
                
                st4_valid <= 1'b0;
            end
        end
    end

    
    
    
    
    
    
    
    
    
    
    localparam PT_VEC_W = EXPAND_PT_DIM * PT_WIDTH;  

    
    
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

    
    
    
    assign st4_push = st4_valid && fire_out && m_axis_pfe_last;

    
    
    wire [3:0] total_occupancy = inflight_cnt + (st3_valid ? 4'd1 : 4'd0) + (st4_valid ? 4'd1 : 4'd0);
    assign pipe_stall = (total_occupancy >= 4'd2);

    
    
    
    
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

    
    
    reg [VN_WIDTH-1:0] point_cnt;

    
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
            
            m_axis_pfe_voxel_valid <= 1'b0;

            if (m_axis_pfe_valid) begin
                
                if (m_axis_pfe_last) begin
                    
                    
                    m_axis_pfe_valid       <= 1'b0;
                    m_axis_pfe_last        <= 1'b0;
                    m_axis_pfe_voxel_valid <= 1'b0;
                    point_cnt              <= {VN_WIDTH{1'b0}};
                end else begin
                    
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

    
    
    
    reg  pfe_flushing_req;
    reg  m_axis_pfe_flush_done_reg;

    
    
    
    
    wire pfe_empty = (inflight_cnt == 3'd0) && !st3_valid && !st4_valid && !m_axis_pfe_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfe_flushing_req          <= 1'b0;
            m_axis_pfe_flush_done_reg <= 1'b0;
        end else begin
            
            if (flush_done) begin
                pfe_flushing_req <= 1'b1;
            end

            
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
    
    
    
    
    integer                    test_k;
    reg     [PT_WIDTH_PER-1:0] test_point_data;

    always @(posedge clk) begin
        
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
