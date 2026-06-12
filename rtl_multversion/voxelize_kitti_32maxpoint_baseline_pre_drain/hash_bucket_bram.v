`timescale 1ns / 1ps

// ==========================================================
// LUTRAM 1R1W memory core
// - One synchronous read port
// - One synchronous write port
// - Single clock
// - Intended for small hash bucket tables
// ==========================================================
module hash_bucket_lutram_1r1w #(
    parameter integer WIDTH            = 64,
    parameter integer DEPTH            = 64,
    parameter integer AW               = $clog2(DEPTH),

    // 0: read-old on same-cycle read/write same address
    // 1: read-new by explicit bypass
    parameter integer FORWARD_ON_WRITE = 0
) (
    input  wire             clk,
    input  wire             rst_n,

    // Read port
    input  wire [AW-1:0]    r_addr,
    input  wire             r_en,
    output reg  [WIDTH-1:0] r_data,

    // Write port
    input  wire [AW-1:0]    w_addr,
    input  wire             w_en,
    input  wire [WIDTH-1:0] w_data
);

    // 强制 Vivado 优先推断 LUTRAM / distributed RAM
    (* ram_style = "distributed" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    integer i;

`ifndef SYNTHESIS
    initial begin
        r_data = {WIDTH{1'b0}};
        for (i = 0; i < DEPTH; i = i + 1) begin
            mem[i] = {WIDTH{1'b0}};
        end
    end
`endif

    always @(posedge clk) begin
        if (!rst_n) begin
            r_data <= {WIDTH{1'b0}};
        end else begin
            // 同步读
            if (r_en) begin
                if (FORWARD_ON_WRITE && w_en && (w_addr == r_addr)) begin
                    r_data <= w_data;
                end else begin
                    r_data <= mem[r_addr];
                end
            end

            // 同步写
            if (w_en) begin
                mem[w_addr] <= w_data;
            end
        end
    end

endmodule


// ==========================================================
// Drop-in replacement for original hash_bucket_bram
//
// Original module looked like true dual-port RAM.
// This replacement is intentionally specialized:
//
//   Port A: read-only
//   Port B: write-only
//
// In your current hash_bucket_array / hash_bucket_array_shadow,
// A port write is unused, and B port read is unused.
// Therefore this version is suitable and much easier to infer as LUTRAM.
// ==========================================================
module hash_bucket_bram #(
    parameter integer WIDTH     = 64,
    parameter integer DEPTH     = 64,
    parameter integer AW        = $clog2(DEPTH),
    parameter         NUM_BYTES = WIDTH / 8
) (
    input wire clk,
    input wire rst_n,

    // Port A: read-only in this LUTRAM-specialized version
    input  wire [AW-1:0]    a_addr,
    input  wire             a_en,
    output wire [WIDTH-1:0] a_rdata,

    // Port B: write-only in this LUTRAM-specialized version
    input  wire [AW-1:0]    b_addr,
    input  wire             b_we,
    input  wire [WIDTH-1:0] b_wdata
);
    hash_bucket_lutram_1r1w #(
        .WIDTH           (WIDTH),
        .DEPTH           (DEPTH),
        .AW              (AW),

        // 保持和你原始 BRAM 模板更接近的语义：
        // 同周期读写同地址时，读到旧值。
        .FORWARD_ON_WRITE(0)
    ) u_lutram (
        .clk   (clk),
        .rst_n (rst_n),

        .r_addr(a_addr),
        .r_en  (a_en),
        .r_data(a_rdata),

        .w_addr(b_addr),
        .w_en  (b_we),
        .w_data(b_wdata)
    );

endmodule