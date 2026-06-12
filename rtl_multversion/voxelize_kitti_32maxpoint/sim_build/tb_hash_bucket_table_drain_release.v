`timescale 1ns / 1ps

module tb_hash_bucket_table_drain_release;
    localparam COORD_WIDTH = 11;
    localparam VN_WIDTH = 6;
    localparam BRAM_DATA_WIDTH = 576;
    localparam BRAM_ADDR_WIDTH = 10;
    localparam BRAM_ADDR_WIDTH_PFE = 8;
    localparam ENTRY_WIDTH = 2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + 18;
    localparam ST_TOMB = 2'b10;
    localparam ST_DRAIN = 2'b11;

    reg clk;
    reg rst_n;
    reg req_valid;
    wire req_ready;
    reg frame_end;
    wire flush_done;
    reg signed [COORD_WIDTH-1:0] key_x;
    reg signed [COORD_WIDTH-1:0] key_y;
    wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a;
    wire bram_expire_wr_b;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b;
    wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b;
    wire m_axis_expire_tvalid;
    wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata;

    hash_bucket_table #(
        .COORD_WIDTH(COORD_WIDTH),
        .VN_WIDTH(VN_WIDTH),
        .LIFE_CYCLE(16'd1000),
        .TIMER_WIDTH(18),
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH(BRAM_ADDR_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_valid),
        .req_ready(req_ready),
        .frame_end(frame_end),
        .flush_done(flush_done),
        .key_x(key_x),
        .key_y(key_y),
        .hash_stall(),
        .out_idx(),
        .out_point_number(),
        .table_full(),
        .bram_expire_addr_a(bram_expire_addr_a),
        .bram_expire_rdata_a({BRAM_DATA_WIDTH{1'b0}}),
        .bram_expire_wr_b(bram_expire_wr_b),
        .bram_expire_addr_b(bram_expire_addr_b),
        .bram_expire_wrdata_b(bram_expire_wrdata_b),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tready(1'b1),
        .m_axis_expire_tdata(m_axis_expire_tdata)
    );

    integer timeout_count;
    integer drain_seen;
    integer tomb_seen;
    integer order_error;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task send_point;
        begin
            @(posedge clk);
            while (!req_ready) begin
                @(posedge clk);
            end
            req_valid <= 1'b1;
            @(posedge clk);
            req_valid <= 1'b0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drain_seen <= 0;
            tomb_seen <= 0;
            order_error <= 0;
        end else if (dut.kill_valid) begin
            if (dut.kill_wdata[ENTRY_WIDTH-1 -: 2] == ST_DRAIN) begin
                drain_seen <= drain_seen + 1;
            end
            if (dut.kill_wdata[ENTRY_WIDTH-1 -: 2] == ST_TOMB) begin
                if (drain_seen == 0) begin
                    order_error <= 1;
                end
                tomb_seen <= tomb_seen + 1;
            end
        end
    end

    initial begin
        rst_n = 1'b0;
        req_valid = 1'b0;
        frame_end = 1'b0;
        key_x = 11'sd5;
        key_y = -11'sd2;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        send_point();
        repeat (20) @(posedge clk);

        frame_end = 1'b1;
        @(posedge clk);
        frame_end = 1'b0;

        timeout_count = 0;
        while (!flush_done && timeout_count < 1000) begin
            timeout_count = timeout_count + 1;
            @(posedge clk);
        end
        repeat (10) @(posedge clk);

        if (order_error) begin
            $display("FAIL: TOMB appeared before DRAIN");
            $finish;
        end
        if (drain_seen !== 1) begin
            $display("FAIL: drain_seen=%0d expected 1", drain_seen);
            $finish;
        end
        if (tomb_seen !== 1) begin
            $display("FAIL: tomb_seen=%0d expected 1", tomb_seen);
            $finish;
        end
        $display("PASS: DRAIN then TOMB sequence observed");
        $finish;
    end
endmodule
