module hash_func_multiplicative #(
    parameter COORD_WIDTH = 11,
    parameter BUCKET_AW   = 6,   // 64 Buckets
    parameter SEED        = 0    // 新增：种子参数 (0, 1, 2, 3)
) (
    input  wire signed [COORD_WIDTH-1:0] key_x,
    input  wire signed [COORD_WIDTH-1:0] key_y,
    output wire        [  BUCKET_AW-1:0] hash_out
);

    // ============================================================
    // 1. 输入扰动 (Input Perturbation)
    // 根据 SEED 改变输入的位模式，制造“雪崩效应”
    // ============================================================
    reg signed [COORD_WIDTH-1:0] mod_x;
    reg signed [COORD_WIDTH-1:0] mod_y;

    always @(*) begin
        case (SEED)
            0: begin
                // 模式 0: 原始输入
                mod_x = key_x;
                mod_y = key_y;
            end
            1: begin
                // 模式 1: X 取反 (改变了符号和数值，哈希值巨变)
                mod_x = ~key_x;
                mod_y = key_y;
            end
            2: begin
                // 模式 2: Y 取反
                mod_x = key_x;
                mod_y = ~key_y;
            end
            3: begin
                // 模式 3: X/Y 异或不同掩码 (0x5A = 01011010)
                // 强制打乱输入位的规律
                mod_x = key_x ^ 8'h5A;
                mod_y = key_y ^ 8'hA5;
            end
            default: begin
                mod_x = key_x;
                mod_y = key_y;
            end
        endcase
    end

    // ============================================================
    // 2. 乘法核心 (保持不变，利用 DSP48)
    // ============================================================
    localparam signed [24:0] PRIME_X = 25'd10368889;
    localparam signed [24:0] PRIME_Y = 25'd10000169;

    wire signed [35:0] mult_x;
    wire signed [35:0] mult_y;

    // 使用扰动后的 mod_x / mod_y 进行乘法
    assign mult_x = mod_x * PRIME_X;
    assign mult_y = mod_y * PRIME_Y;

    // ============================================================
    // 3. 混合与输出
    // ============================================================
    wire [35:0] mixed = mult_x ^ mult_y;

    // 提取高位 (根据你代码中的 24 bit 起始位置)
    assign hash_out = mixed[24-:BUCKET_AW];

endmodule
