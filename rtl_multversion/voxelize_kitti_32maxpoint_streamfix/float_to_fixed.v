`timescale 1ps / 1ps

// ============================================================
// Fast truncation-only float32 -> signed fixed-point converter.
// Use this for data path values that are stored/compared.
// No ROUND_MODE, no nearest, no rounding adder.
// ============================================================
module float_to_fixed_data #(
    parameter integer FIXED_WIDTH      = 16,  // must not be > 32
    parameter integer FIXED_FRACTIONAL = 8
) (
    input  wire [31:0]            float_in,
    output wire                   fixed_sign,
    output wire [FIXED_WIDTH-1:0] true_fixed_value
);

    localparam integer FIXED_INTEGER = FIXED_WIDTH - FIXED_FRACTIONAL;
    localparam [7:0] EXP_OVERFLOW = 8'd127 + FIXED_INTEGER - 1;

    wire [7:0]  float_exp;
    wire [22:0] float_mantissa;

    assign {fixed_sign, float_exp, float_mantissa} = float_in;

    wire [31:0] working_out = {1'b1, float_mantissa, 8'h0};

    wire [7:0] shift_dist = EXP_OVERFLOW - float_exp;
    wire [4:0] trunc_shift_dist = (|shift_dist[7:5]) ? 5'b11111 : shift_dist[4:0];
    wire [31:0] shifted_out = working_out >> trunc_shift_dist;

    wire is_zero_or_denorm = (float_exp == 8'd0);
    wire is_overflow       = (float_exp >= EXP_OVERFLOW);

    wire [FIXED_WIDTH-1:0] POS_SAT     = {1'b0, {(FIXED_WIDTH-1){1'b1}}};
    wire [FIXED_WIDTH-1:0] NEG_MAG_SAT = {1'b1, {(FIXED_WIDTH-1){1'b0}}};

    // Magnitude after truncation. MSB is kept 0 for non-overflow positive magnitude.
    wire [FIXED_WIDTH-1:0] fixed_mag_trunc = {1'b0, shifted_out[30 : 32-FIXED_WIDTH]};

    wire [FIXED_WIDTH-1:0] fixed_mag =
        is_zero_or_denorm ? {FIXED_WIDTH{1'b0}} :
        is_overflow       ? (fixed_sign ? NEG_MAG_SAT : POS_SAT) :
                            fixed_mag_trunc;

    assign true_fixed_value = fixed_sign ?
                              (~fixed_mag + {{(FIXED_WIDTH-1){1'b0}}, 1'b1}) :
                              fixed_mag;

endmodule


// ============================================================
// Fast ceil(+inf)-only float32 -> signed fixed-point converter.
// Use this only for voxel-index coordinate conversion.
// ROUND_MODE=1 nearest has been removed.
// Behavior:
//   positive number: ceil to next fixed LSB if discarded bits are nonzero
//   negative number: trunc magnitude then negate, equivalent to ceil(+inf)
// ============================================================
module float_to_fixed_voxel_ceil #(
    parameter integer FIXED_WIDTH      = 16,  // intended <= 31 for discarded-bit OR; current coor uses 20
    parameter integer FIXED_FRACTIONAL = 8
) (
    input  wire [31:0]            float_in,
    output wire                   fixed_sign,
    output wire [FIXED_WIDTH-1:0] true_fixed_value
);

    localparam integer FIXED_INTEGER = FIXED_WIDTH - FIXED_FRACTIONAL;
    localparam [7:0] EXP_OVERFLOW = 8'd127 + FIXED_INTEGER - 1;

    wire [7:0]  float_exp;
    wire [22:0] float_mantissa;

    assign {fixed_sign, float_exp, float_mantissa} = float_in;

    wire [31:0] working_out = {1'b1, float_mantissa, 8'h0};

    wire [7:0] shift_dist = EXP_OVERFLOW - float_exp;
    wire [4:0] trunc_shift_dist = (|shift_dist[7:5]) ? 5'b11111 : shift_dist[4:0];
    wire [31:0] shifted_out = working_out >> trunc_shift_dist;

    wire is_zero_or_denorm = (float_exp == 8'd0);
    wire is_overflow       = (float_exp >= EXP_OVERFLOW);

    wire [FIXED_WIDTH-1:0] POS_SAT     = {1'b0, {(FIXED_WIDTH-1){1'b1}}};
    wire [FIXED_WIDTH-1:0] NEG_MAG_SAT = {1'b1, {(FIXED_WIDTH-1){1'b0}}};

    wire [FIXED_WIDTH-1:0] fixed_mag_trunc = {1'b0, shifted_out[30 : 32-FIXED_WIDTH]};

    wire discarded_nonzero;
    generate
        if (FIXED_WIDTH >= 32) begin : G_NO_DISCARD_BITS
            assign discarded_nonzero = 1'b0;
        end else begin : G_HAS_DISCARD_BITS
            assign discarded_nonzero = |shifted_out[31-FIXED_WIDTH : 0];
        end
    endgenerate

    // ceil(+inf): positive values increment when fractional discarded bits exist.
    // Negative values do not increment magnitude; trunc magnitude + negate already moves toward +inf.
    wire ceil_inc = (!fixed_sign) && discarded_nonzero;

    wire [FIXED_WIDTH:0] pos_mag_ceil_ext =
        {1'b0, fixed_mag_trunc} + {{FIXED_WIDTH{1'b0}}, ceil_inc};

    wire pos_round_overflow = pos_mag_ceil_ext[FIXED_WIDTH] || pos_mag_ceil_ext[FIXED_WIDTH-1];

    wire [FIXED_WIDTH-1:0] fixed_mag_pos_ceil =
        pos_round_overflow ? POS_SAT : pos_mag_ceil_ext[FIXED_WIDTH-1:0];

    wire [FIXED_WIDTH-1:0] fixed_mag_ceil =
        fixed_sign ? fixed_mag_trunc : fixed_mag_pos_ceil;

    wire [FIXED_WIDTH-1:0] fixed_mag =
        is_zero_or_denorm ? {FIXED_WIDTH{1'b0}} :
        is_overflow       ? (fixed_sign ? NEG_MAG_SAT : POS_SAT) :
                            fixed_mag_ceil;

    assign true_fixed_value = fixed_sign ?
                              (~fixed_mag + {{(FIXED_WIDTH-1){1'b0}}, 1'b1}) :
                              fixed_mag;

endmodule


