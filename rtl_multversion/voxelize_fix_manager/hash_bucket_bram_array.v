module hash_bucket_array #(
    parameter ENTRY_WIDTH = 45,
    parameter BUCKETS     = 64,
    parameter BUCKET_AW   = 6
) (
    input wire clk,

    // Port A
    input wire [BUCKET_AW-1:0] a_addr_0,
    input wire [BUCKET_AW-1:0] a_addr_1,
    input wire [BUCKET_AW-1:0] a_addr_2,
    input wire [BUCKET_AW-1:0] a_addr_3,

    input  wire                   we0_a,
    input  wire                   we1_a,
    input  wire                   we2_a,
    input  wire                   we3_a,
    input  wire                   en0_a,
    input  wire                   en1_a,
    input  wire                   en2_a,
    input  wire                   en3_a,
    input  wire [ENTRY_WIDTH-1:0] wdata0_a,
    input  wire [ENTRY_WIDTH-1:0] wdata1_a,
    input  wire [ENTRY_WIDTH-1:0] wdata2_a,
    input  wire [ENTRY_WIDTH-1:0] wdata3_a,
    output wire [ENTRY_WIDTH-1:0] rdata0_a,
    output wire [ENTRY_WIDTH-1:0] rdata1_a,
    output wire [ENTRY_WIDTH-1:0] rdata2_a,
    output wire [ENTRY_WIDTH-1:0] rdata3_a,

    // Port B
    input  wire [  BUCKET_AW-1:0] b_addr,
    input  wire                   we0_b,
    input  wire                   we1_b,
    input  wire                   we2_b,
    input  wire                   we3_b,
    input  wire [ENTRY_WIDTH-1:0] wdata0_b,
    input  wire [ENTRY_WIDTH-1:0] wdata1_b,
    input  wire [ENTRY_WIDTH-1:0] wdata2_b,
    input  wire [ENTRY_WIDTH-1:0] wdata3_b,
    output wire [ENTRY_WIDTH-1:0] rdata0_b,
    output wire [ENTRY_WIDTH-1:0] rdata1_b,
    output wire [ENTRY_WIDTH-1:0] rdata2_b,
    output wire [ENTRY_WIDTH-1:0] rdata3_b
);

    hash_bucket_bram #(
        .WIDTH(ENTRY_WIDTH),
        .DEPTH(BUCKETS)
    ) u_hash_bucket_bram0 (
        .clk_b  (clk),
        .clk_a  (clk),
        .a_addr (a_addr_0),
        .a_we   (we0_a),
        .a_en   (en0_a),
        .a_wdata(wdata0_a),
        .a_rdata(rdata0_a),
        .b_addr (b_addr),
        .b_we   (we0_b),
        .b_en   (1'b1),
        .b_wdata(wdata0_b),
        .b_rdata(rdata0_b)
    );

    hash_bucket_bram #(
        .WIDTH(ENTRY_WIDTH),
        .DEPTH(BUCKETS)
    ) u_hash_bucket_bram1 (
        .clk_b  (clk),
        .clk_a  (clk),
        .a_addr (a_addr_1),
        .a_we   (we1_a),
        .a_en   (en1_a),
        .a_wdata(wdata1_a),
        .a_rdata(rdata1_a),
        .b_addr (b_addr),
        .b_we   (we1_b),
        .b_en   (1'b1),
        .b_wdata(wdata1_b),
        .b_rdata(rdata1_b)
    );

    hash_bucket_bram #(
        .WIDTH(ENTRY_WIDTH),
        .DEPTH(BUCKETS)
    ) u_hash_bucket_bram2 (
        .clk_b  (clk),
        .clk_a  (clk),
        .a_addr (a_addr_2),
        .a_we   (we2_a),
        .a_en   (en2_a),
        .a_wdata(wdata2_a),
        .a_rdata(rdata2_a),
        .b_addr (b_addr),
        .b_we   (we2_b),
        .b_en   (1'b1),
        .b_wdata(wdata2_b),
        .b_rdata(rdata2_b)
    );

    hash_bucket_bram #(
        .WIDTH(ENTRY_WIDTH),
        .DEPTH(BUCKETS)
    ) u_hash_bucket_bram3 (
        .clk_b  (clk),
        .clk_a  (clk),
        .a_addr (a_addr_3),
        .a_we   (we3_a),
        .a_en   (en3_a),
        .a_wdata(wdata3_a),
        .a_rdata(rdata3_a),
        .b_addr (b_addr),
        .b_we   (we3_b),
        .b_en   (1'b1),
        .b_wdata(wdata3_b),
        .b_rdata(rdata3_b)
    );

endmodule
