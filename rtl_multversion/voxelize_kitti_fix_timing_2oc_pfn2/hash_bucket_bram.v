module hash_bucket_bram #(
    parameter integer WIDTH     = 64,
    parameter integer DEPTH     = 64,
    parameter integer AW        = $clog2(DEPTH),
    parameter         NUM_BYTES = WIDTH / 8
) (
    input wire clk_a,
    input wire clk_b,
    input wire rst_n,

    // Port A
    input  wire [   AW-1:0] a_addr,
    input  wire             a_en,
    input  wire             a_we,
    input  wire [WIDTH-1:0] a_wdata,
    output reg  [WIDTH-1:0] a_rdata,

    // Port B
    input  wire [   AW-1:0] b_addr,
    input  wire             b_en,
    input  wire             b_we,
    input  wire [WIDTH-1:0] b_wdata,
    output reg  [WIDTH-1:0] b_rdata
);
    (* ram_style="block" *) reg [WIDTH-1:0] mem[0:DEPTH-1];
    integer idx;
`ifndef SYNTHESIS
    initial begin
        for (idx = 0; idx < (1 << AW); idx = idx + 1) begin
            mem[idx] = {WIDTH{1'b0}};
        end
    end
`endif

    // 循环变量
    integer i;
    // Port A
    always @(posedge clk_a) begin

        if (a_en) begin
            a_rdata <= mem[a_addr];

            if (a_we) begin
                a_rdata <= mem[a_addr];
                mem[a_addr] <= a_wdata;
            end
        end
    end

    // Port B
    always @(posedge clk_b) begin
        if (b_en) begin
            b_rdata <= mem[b_addr];

            if (b_we) begin
                b_rdata <= mem[b_addr];
                mem[b_addr] <= b_wdata;
            end
        end
    end

endmodule
