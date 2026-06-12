













`timescale 1ps / 1ps

module voxel_sdp_bram #(
    parameter integer DATA_WIDTH      = 576,
    parameter integer ADDR_WIDTH      = 10,
    parameter         MEM_FILE        = "NOTHING",
    parameter integer MEM_FILE_LENGTH = 0,
    parameter integer CHUNK_WIDTH     = 9,
    parameter integer NUM_CHUNKS      = DATA_WIDTH / CHUNK_WIDTH
) (
    input wire clk,

    
    input  wire                    wr_en,
    input  wire [NUM_CHUNKS-1:0]   wr_bwen,
    input  wire [ADDR_WIDTH-1:0]   wr_addr,
    input  wire [DATA_WIDTH-1:0]   wr_data,

    
    input  wire                    rd_en,
    input  wire [ADDR_WIDTH-1:0]   rd_addr,
    output reg  [DATA_WIDTH-1:0]   rd_data
);

    localparam integer DEPTH = (1 << ADDR_WIDTH);

    (* ram_style = "bram" *)
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    integer i;

    generate
        if (MEM_FILE != "NOTHING") begin : g_init_file
            initial begin
                $readmemh(MEM_FILE, mem, 0, MEM_FILE_LENGTH - 1);
            end
        end else begin : g_init_zero
            integer idx;
            initial begin
                for (idx = 0; idx < DEPTH; idx = idx + 1) begin
                    mem[idx] = {DATA_WIDTH{1'b0}};
                end
            end
        end
    endgenerate

    always @(posedge clk) begin
        
        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end

        
        if (wr_en) begin
            for (i = 0; i < NUM_CHUNKS; i = i + 1) begin
                if (wr_bwen[i]) begin
                    mem[wr_addr][i*CHUNK_WIDTH +: CHUNK_WIDTH] <=
                        wr_data[i*CHUNK_WIDTH +: CHUNK_WIDTH];
                end
            end
        end
    end

endmodule
