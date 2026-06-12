/*
  Modified to support 9-bit Chunk-Write-Enable.
  Optimized for Xilinx 36K BRAM architecture to achieve 100% utilization.
*/

`timescale 1ps / 1ps

module dualport_bram #(
    parameter DATA_WIDTH      = 504,
    parameter ADDR_WIDTH      = 10,
    parameter MEM_FILE        = "NOTHING",
    parameter MEM_FILE_LENGTH = 0,
    // 核心修改 1：计算 9-bit 块的数量 (504 / 9 = 56)
    parameter NUM_CHUNKS      = DATA_WIDTH / 9 
) (
    // Port A
    input  wire                    a_clk,
    input  wire                    a_wr,    // write enable
    input  wire                    a_en,    // bram enable for port A
    input  wire [NUM_CHUNKS - 1:0] a_bwen,  // 核心修改 2：位宽变为 56 bit
    input  wire [ADDR_WIDTH - 1:0] a_addr,
    input  wire [DATA_WIDTH - 1:0] a_din,
    output reg  [DATA_WIDTH - 1:0] a_dout,

    // Port B
    input  wire                    b_clk,
    input  wire                    b_wr,    // write enable
    input  wire                    b_en,
    input  wire [NUM_CHUNKS - 1:0] b_bwen,  // 核心修改 2：位宽变为 56 bit
    input  wire [ADDR_WIDTH - 1:0] b_addr,
    input  wire [DATA_WIDTH - 1:0] b_din,
    output reg  [DATA_WIDTH - 1:0] b_dout
);

    // Shared Memory
    (* ram_style = "bram" *) reg [DATA_WIDTH - 1:0] mem[(1 << ADDR_WIDTH) - 1:0];

    // Initialization
    generate
        if (MEM_FILE != "NOTHING") begin
            initial begin
                $readmemh(MEM_FILE, mem, 0, MEM_FILE_LENGTH - 1);
            end
        end
    endgenerate

    integer idx;
    initial begin
        for (idx = 0; idx < (1 << ADDR_WIDTH); idx = idx + 1) begin
            mem[idx] = {DATA_WIDTH{1'b0}};
        end
    end

    // 循环变量
    integer i;

    // Port A
    always @(posedge a_clk) begin
        if (a_en) begin
            a_dout <= mem[a_addr];

            if (a_wr) begin
                a_dout <= mem[a_addr];
                // 核心修改 3：按 9-bit 步进切片
                for (i = 0; i < NUM_CHUNKS; i = i + 1) begin
                    if (a_bwen[i]) begin
                        mem[a_addr][i*9+:9] <= a_din[i*9+:9];
                    end
                end
            end
        end
    end

    // Port B
    always @(posedge b_clk) begin
        if (b_en) begin
            b_dout <= mem[b_addr];

            if (b_wr) begin
                b_dout <= mem[b_addr];
                // 核心修改 3：按 9-bit 步进切片
                for (i = 0; i < NUM_CHUNKS; i = i + 1) begin
                    if (b_bwen[i]) begin
                        mem[b_addr][i*9+:9] <= b_din[i*9+:9];
                    end
                end
            end
        end
    end

endmodule