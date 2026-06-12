`timescale 1ps / 1ps

module float_to_fixed #(
    parameter FIXED_WIDTH      = 16,  // must not be > 32
    parameter FIXED_FRACTIONAL = 8
) (
    float_in,
    fixed_sign,
    true_fixed_value
);

    input [31:0] float_in;
    output fixed_sign;
    output [FIXED_WIDTH-1:0] true_fixed_value;

    wire [ 7:0] float_exp;
    wire [22:0] float_mantissa;
    wire [15:0] fixed_mag;
    assign {fixed_sign, float_exp, float_mantissa} = float_in;

    wire [31:0] working_out = {1'b1, float_mantissa, 8'h0};

    wire [7:0] shift_dist = 8'd127 + (FIXED_WIDTH - FIXED_FRACTIONAL) - 1 - float_exp;
    wire [4:0] trunc_shift_dist = (|shift_dist[7:5]) ? 5'b11111 : shift_dist[4:0];

    wire [31:0] shifted_out = working_out >> trunc_shift_dist;

    // 新增：上溢饱和判断 (当阶码 >= 134 时，说明整数部分绝对 >= 128，溢出！)
    wire is_overflow = (float_exp >= 8'd127 + (FIXED_WIDTH - FIXED_FRACTIONAL) - 1);

    // 真正的饱和幅值：如果溢出，强制给到 15 位的最大正数 15'h7FFF
    assign fixed_mag = is_overflow ? 16'h7FFF : {1'b0, shifted_out[30:32-FIXED_WIDTH]};

    assign true_fixed_value = fixed_sign ? -fixed_mag : fixed_mag;

endmodule
