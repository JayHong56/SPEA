`timescale 1ns / 1ps

module tb_hash_bucket_table_cam_full;
    localparam COORD_WIDTH = 11;
    localparam VN_WIDTH = 7;
    localparam BRAM_DATA_WIDTH = 576;
    localparam BRAM_ADDR_WIDTH = 10;
    localparam BRAM_ADDR_WIDTH_PFE = 12;

    reg clk;
    reg rst_n;
    reg req_valid;
    wire req_ready;
    reg frame_end;
    reg signed [COORD_WIDTH-1:0] key_x;
    reg signed [COORD_WIDTH-1:0] key_y;
    wire hash_stall;
    wire out_valid;
    wire [7:0] out_idx;
    wire [VN_WIDTH-1:0] out_point_number;
    wire table_full;
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
        .MAX_VOXEL_NUM(16'd100),
        .ENTRY_POINT_CAP(16'd32),
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH(BRAM_ADDR_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_valid),
        .req_ready(req_ready),
        .frame_end(frame_end),
        .flush_done(),
        .key_x(key_x),
        .key_y(key_y),
        .hash_stall(hash_stall),
        .out_valid(out_valid),
        .out_idx(out_idx),
        .out_point_number(out_point_number),
        .table_full(table_full),
        .bram_expire_addr_a(bram_expire_addr_a),
        .bram_expire_rdata_a({BRAM_DATA_WIDTH{1'b0}}),
        .bram_expire_wr_b(bram_expire_wr_b),
        .bram_expire_addr_b(bram_expire_addr_b),
        .bram_expire_wrdata_b(bram_expire_wrdata_b),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tready(1'b1),
        .m_axis_expire_tdata(m_axis_expire_tdata)
    );

    integer n;
    integer out_count;
    integer drop_count;
    integer max_pn_seen;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task send_point;
        input signed [COORD_WIDTH-1:0] x;
        input signed [COORD_WIDTH-1:0] y;
        begin
            @(posedge clk);
            while (!req_ready) begin
                @(posedge clk);
            end
            key_x <= x;
            key_y <= y;
            req_valid <= 1'b1;
            @(posedge clk);
            req_valid <= 1'b0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_count   <= 0;
            drop_count  <= 0;
            max_pn_seen <= 0;
        end else begin
            if (out_valid) begin
                out_count <= out_count + 1;
                if (out_point_number > max_pn_seen) begin
                    max_pn_seen <= out_point_number;
                end
            end
            if (table_full) begin
                drop_count <= drop_count + 1;
            end
        end
    end

    initial begin
        rst_n = 1'b0;
        req_valid = 1'b0;
        frame_end = 1'b0;
        key_x = {COORD_WIDTH{1'b0}};
        key_y = {COORD_WIDTH{1'b0}};

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (n = 0; n < 105; n = n + 1) begin
            send_point(11'sd10, -11'sd3);
        end

        repeat (80) @(posedge clk);

        if (out_count !== 100) begin
            $display("FAIL: out_count=%0d expected 100", out_count);
            $finish;
        end
        if (drop_count !== 5) begin
            $display("FAIL: drop_count=%0d expected 5", drop_count);
            $finish;
        end
        if (max_pn_seen > 32) begin
            $display("FAIL: max_pn_seen=%0d expected <=32", max_pn_seen);
            $finish;
        end

        $display("PASS: out_count=%0d drop_count=%0d max_pn_seen=%0d", out_count, drop_count, max_pn_seen);
        $finish;
    end
endmodule
