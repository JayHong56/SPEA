





























`timescale 1ps / 1ps

module dram #(
    parameter DATA_WIDTH      = 128,
    parameter ADDR_WIDTH      = 16,         
    parameter MEM_FILE        = "NOTHING",
    parameter MEM_FILE_LENGTH = -1

) (
    input clk,
    input rst_n,

    input                           en,
    input                           we,
    input      [(ADDR_WIDTH - 1):0] write_address,
    input      [(ADDR_WIDTH - 1):0] read_address,
    input      [(DATA_WIDTH - 1):0] data_in,
    output reg [(DATA_WIDTH - 1):0] data_out
);


    
    (* ram_style="block" *) reg [(DATA_WIDTH - 1):0] mem[0:((1 << ADDR_WIDTH) - 1)];  
    reg [(ADDR_WIDTH - 1):0] read_address_reg;

    initial begin
        if (MEM_FILE != "NOTHING") begin
            $display("Loading file...");
            $readmemb(MEM_FILE, mem, 0, MEM_FILE_LENGTH - 1);
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            data_out <= 0;
        end else begin
            if (en) begin
                data_out <= mem[read_address];
                if (we) begin
                    mem[write_address] <= data_in;
                    data_out <= data_in;
                end
            end
        end
    end

endmodule
