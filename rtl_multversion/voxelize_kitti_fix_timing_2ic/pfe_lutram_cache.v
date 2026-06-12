module pfe_lutram_cache #(
    parameter integer DATA_WIDTH = 504,
    parameter integer ADDR_WIDTH = 7
) (
    input  wire                  clk,

    input  wire                  wr_en,
    input  wire [ADDR_WIDTH-1:0] wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,

    input  wire                  rd_en,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [DATA_WIDTH-1:0] rd_data
);

    (* ram_style = "distributed" *)
    reg [DATA_WIDTH-1:0] mem [0:(1 << ADDR_WIDTH)-1];

    always @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end

        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end
    end

endmodule