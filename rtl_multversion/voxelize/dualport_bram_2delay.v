/*
  Modified to support Byte-Enable (Byte-Write-Enable).
  Adapted from http://danstrother.com/2010/09/11/inferring-rams-in-fpgas/
*/

`timescale 1ps / 1ps

module dualport_bram_2delay #(
    parameter DATA_WIDTH      = 256,
    parameter ADDR_WIDTH      = 10,
    parameter MEM_FILE        = "NOTHING",
    parameter MEM_FILE_LENGTH = 0,
    parameter NUM_BYTES       = DATA_WIDTH / 8
) (
    // Port A
    input  wire                    a_clk,
    input  wire                    a_wr,    // write enable
    input  wire                    a_en,    // bram enable for port A
    input  wire [ NUM_BYTES - 1:0] a_bwen,
    input  wire [ADDR_WIDTH - 1:0] a_addr,
    input  wire [DATA_WIDTH - 1:0] a_din,
    output reg  [DATA_WIDTH - 1:0] a_dout,

    // Port B
    input  wire                    b_clk,
    input  wire                    b_wr,    // write enable
    input  wire                    b_en,
    input  wire [ NUM_BYTES - 1:0] b_bwen,
    input  wire [ADDR_WIDTH - 1:0] b_addr,
    input  wire [DATA_WIDTH - 1:0] b_din,
    output reg  [DATA_WIDTH - 1:0] b_dout
);

    (* ram_style = "bram" *) reg [DATA_WIDTH - 1:0] mem_2delay[(1 << ADDR_WIDTH) - 1:0];

    // Two-cycle read pipeline registers
    reg [DATA_WIDTH - 1:0] a_dout_d1;
    reg [DATA_WIDTH - 1:0] b_dout_d1;
    reg                    a_en_d1;
    reg                    b_en_d1;

    // Initialization
    generate
        if (MEM_FILE != "NOTHING") begin
            initial begin
                $readmemh(MEM_FILE, mem_2delay, 0, MEM_FILE_LENGTH - 1);
            end
        end
    endgenerate

    integer idx;
    initial begin
        for (idx = 0; idx < (1 << ADDR_WIDTH); idx = idx + 1) begin
            mem_2delay[idx] = {DATA_WIDTH{1'b0}};
        end
    end

    // 循环变量
    integer i;

    // Port A
    always @(posedge a_clk) begin

        a_en_d1 <= a_en;

        if (a_en_d1) begin
            a_dout <= a_dout_d1;
        end

        if (a_en) begin
            a_dout_d1 <= mem_2delay[a_addr];

            if (a_wr) begin
                for (i = 0; i < NUM_BYTES; i = i + 1) begin
                    if (a_bwen[i]) begin
                        mem_2delay[a_addr][i*8+:8] <= a_din[i*8+:8];
                    end
                end
            end
        end
    end

    // Port B
    always @(posedge b_clk) begin

        b_en_d1 <= b_en;

        if (b_en_d1) begin
            b_dout <= b_dout_d1;
        end


        if (b_en) begin
            b_dout_d1 <= mem_2delay[b_addr];

            if (b_wr) begin
                for (i = 0; i < NUM_BYTES; i = i + 1) begin
                    if (b_bwen[i]) begin
                        mem_2delay[b_addr][i*8+:8] <= b_din[i*8+:8];
                    end
                end
            end
        end
    end

endmodule
