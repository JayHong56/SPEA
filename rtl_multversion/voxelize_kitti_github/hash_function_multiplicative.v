















module hash_func_multiplicative #(
    parameter integer COORD_WIDTH = 11,
    parameter integer BUCKET_AW   = 6,   
    parameter integer SEED        = 0    
) (
    input  wire signed [COORD_WIDTH-1:0] key_x,
    input  wire signed [COORD_WIDTH-1:0] key_y,
    output wire        [  BUCKET_AW-1:0] hash_out
);

    localparam integer KEY_W = 2 * COORD_WIDTH;

    
    
    wire [KEY_W-1:0] hash_key;
    assign hash_key = {key_x[COORD_WIDTH-1:0], key_y[COORD_WIDTH-1:0]};

    
    
    
    
    
    
    
    
    
    function [KEY_W-1:0] h3_mask;
        input integer seed;
        input integer bit_id;
        reg [7:0] sel;
        begin
            sel = {seed[3:0], bit_id[3:0]};
            case (sel)
                
                8'h00: h3_mask = 22'b0111000110110001110111;  
                8'h01: h3_mask = 22'b1001110101001011111110;  
                8'h02: h3_mask = 22'b0110110001010101110001;  
                8'h03: h3_mask = 22'b0001100101110011111110;  
                8'h04: h3_mask = 22'b1100011110010000001101;  
                8'h05: h3_mask = 22'b1010000111011000100111;  

                
                8'h10: h3_mask = 22'b1110000101111000110110;  
                8'h11: h3_mask = 22'b1001000101110010110101;  
                8'h12: h3_mask = 22'b1100110111101110010011;  
                8'h13: h3_mask = 22'b0100011100010001000111;  
                8'h14: h3_mask = 22'b1000010100011110010110;  
                8'h15: h3_mask = 22'b0010110010011011001111;  

                
                8'h20: h3_mask = 22'b0001001101110101001101;  
                8'h21: h3_mask = 22'b0111111010001001111110;  
                8'h22: h3_mask = 22'b1110011011001101101001;  
                8'h23: h3_mask = 22'b0001101011100000110000;  
                8'h24: h3_mask = 22'b0110001010000101010011;  
                8'h25: h3_mask = 22'b0000001101111001111010;  

                
                8'h30: h3_mask = 22'b0111111001010111100110;  
                8'h31: h3_mask = 22'b1011001001100010110101;  
                8'h32: h3_mask = 22'b1011100010110110000010;  
                8'h33: h3_mask = 22'b0010011010010110100101;  
                8'h34: h3_mask = 22'b0101011111010101101000;  
                8'h35: h3_mask = 22'b0011110000000001111000;  

                
                
                default: h3_mask = 22'b1010011010110101101001;  
            endcase
        end
    endfunction

    genvar j;
    generate
        for (j = 0; j < BUCKET_AW; j = j + 1) begin : g_h3_hash
            assign hash_out[j] = ^(hash_key & h3_mask(SEED, j));
        end
    endgenerate

endmodule
