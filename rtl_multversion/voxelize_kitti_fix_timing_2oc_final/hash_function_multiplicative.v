// -----------------------------------------------------------------------------
// Drop-in replacement for the original multiplicative hash function.
//
// NOTE:
//   The module name is intentionally kept as "hash_func_multiplicative" so that
//   existing instantiations in hash_bucket_table do not need to be modified.
//   Internally, this version implements a Class-H3-style hash:
//       hash_out[j] = parity( {key_x,key_y} & MASK[SEED][j] )
//   This is a pure LUT implementation: no DSP multipliers are inferred.
//
// Recommended/default configuration for the current design:
//   COORD_WIDTH = 11
//   BUCKET_AW   = 6       // 64 buckets
//   SEED        = 0..3    // four independent hash choices
// -----------------------------------------------------------------------------

module hash_func_multiplicative #(
    parameter integer  COORD_WIDTH = 11,
    parameter integer BUCKET_AW   = 6,  // 64 buckets when BUCKET_AW = 6
    parameter integer SEED        = 0   // seed 0/1/2/3 for four independent H3 masks
) (
    input  wire signed [COORD_WIDTH-1:0] key_x,
    input  wire signed [COORD_WIDTH-1:0] key_y,
    output wire        [  BUCKET_AW-1:0] hash_out
);

    localparam integer KEY_W = 2 * COORD_WIDTH;

    // Hash key used by the H3 matrix. The signed attribute of key_x/key_y is
    // irrelevant here; we hash their bit patterns directly.
    wire [KEY_W-1:0] hash_key;
    assign hash_key = {key_x[COORD_WIDTH-1:0], key_y[COORD_WIDTH-1:0]};

    // -------------------------------------------------------------------------
    // Fixed random binary matrix masks.
    // For COORD_WIDTH=11 and BUCKET_AW=6, these are 4 x 6 masks of 22 bits.
    // Each output bit computes parity(hash_key & mask).
    //
    // If KEY_W is wider than 22, these constants are zero-extended by Verilog;
    // if KEY_W is narrower, they are truncated. The current accelerator uses
    // KEY_W=22, which is the intended configuration.
    // -------------------------------------------------------------------------
    function [KEY_W-1:0] h3_mask;
        input integer seed;
        input integer bit_id;
        reg [7:0] sel;
        begin
            sel = {seed[3:0], bit_id[3:0]};
            case (sel)
                // SEED = 0
                8'h00: h3_mask = 22'b0111000110110001110111; // 0x1C6C77
                8'h01: h3_mask = 22'b1001110101001011111110; // 0x2752FE
                8'h02: h3_mask = 22'b0110110001010101110001; // 0x1B1571
                8'h03: h3_mask = 22'b0001100101110011111110; // 0x065CFE
                8'h04: h3_mask = 22'b1100011110010000001101; // 0x31E40D
                8'h05: h3_mask = 22'b1010000111011000100111; // 0x287627

                // SEED = 1
                8'h10: h3_mask = 22'b1110000101111000110110; // 0x385E36
                8'h11: h3_mask = 22'b1001000101110010110101; // 0x245CB5
                8'h12: h3_mask = 22'b1100110111101110010011; // 0x337B93
                8'h13: h3_mask = 22'b0100011100010001000111; // 0x11C447
                8'h14: h3_mask = 22'b1000010100011110010110; // 0x214796
                8'h15: h3_mask = 22'b0010110010011011001111; // 0x0B26CF

                // SEED = 2
                8'h20: h3_mask = 22'b0001001101110101001101; // 0x04DD4D
                8'h21: h3_mask = 22'b0111111010001001111110; // 0x1FA27E
                8'h22: h3_mask = 22'b1110011011001101101001; // 0x39B369
                8'h23: h3_mask = 22'b0001101011100000110000; // 0x06B830
                8'h24: h3_mask = 22'b0110001010000101010011; // 0x18A153
                8'h25: h3_mask = 22'b0000001101111001111010; // 0x00DE7A

                // SEED = 3
                8'h30: h3_mask = 22'b0111111001010111100110; // 0x1F95E6
                8'h31: h3_mask = 22'b1011001001100010110101; // 0x2C98B5
                8'h32: h3_mask = 22'b1011100010110110000010; // 0x2E2D82
                8'h33: h3_mask = 22'b0010011010010110100101; // 0x09A5A5
                8'h34: h3_mask = 22'b0101011111010101101000; // 0x15F568
                8'h35: h3_mask = 22'b0011110000000001111000; // 0x0F0078

                // Fallback for unsupported SEED/BIT combinations.
                // The present design only uses SEED=0..3 and BUCKET_AW=6.
                default: h3_mask = 22'b1010011010110101101001; // 0x29AD69
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
