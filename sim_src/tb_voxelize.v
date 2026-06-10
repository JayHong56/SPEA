`timescale 1ns / 1ps

module tb_voxelize;

    reg clk;
    reg rst_n;

    
    voxelize_3oc dut (
        .clk_p  (clk),
        .clk_n  (~clk),
        .rst_n(rst_n)
    );

    
    initial begin
        clk = 1'b0;
        forever #2.5 clk = ~clk;
    end

    
    initial begin
        rst_n = 1'b0;
        #100;
        rst_n = 1'b1;
    end

    
    initial begin
        
        
        

        
        @(posedge rst_n);
        #100000;

    end

endmodule
