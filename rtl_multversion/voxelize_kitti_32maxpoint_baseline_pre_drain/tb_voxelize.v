`timescale 1ns / 1ps

module tb_voxelize;

    reg clk;
    reg rst_n;

    // DUT
    voxelize_1oc dut (
        .clk_p  (clk),
        .clk_n  (~clk),
        .rst_n(rst_n)
    );

    // 100MHz clock: 10ns period
    initial begin
        clk = 1'b0;
        forever #2.5 clk = ~clk;
    end

    // reset
    initial begin
        rst_n = 1'b0;
        #100;
        rst_n = 1'b1;
    end

    // run control
    initial begin
        // 可选：波形（Icarus/部分仿真器可用）
        // $dumpfile("tb_voxelize.vcd");
        // $dumpvars(0, tb_voxelize);

        // 等待复位释放后跑一段时间
        @(posedge rst_n);
        #100000;

    end

endmodule
