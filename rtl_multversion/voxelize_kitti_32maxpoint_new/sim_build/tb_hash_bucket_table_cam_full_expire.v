`timescale 1ns / 1ps

module tb_hash_bucket_table_cam_full_expire;
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
    wire flush_done;
    reg signed [COORD_WIDTH-1:0] key_x;
    reg signed [COORD_WIDTH-1:0] key_y;
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
        .flush_done(flush_done),
        .key_x(key_x),
        .key_y(key_y),
        .hash_stall(),
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
    integer expire_event_count;
    integer kill_write_count;
    integer timeout_count;
    integer seen_flush_done;
    reg count_kills;
    reg [VN_WIDTH-1:0] last_expire_pn;

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
            expire_event_count <= 0;
            kill_write_count   <= 0;
            last_expire_pn     <= {VN_WIDTH{1'b0}};
        end else begin
            if (m_axis_expire_tvalid) begin
                expire_event_count <= expire_event_count + 1;
                last_expire_pn <= m_axis_expire_tdata[VN_WIDTH-1:0];
            end
            if (count_kills && (dut.we0_b || dut.we1_b || dut.we2_b || dut.we3_b) && dut.kill_write) begin
                kill_write_count <= kill_write_count + 1;
            end
        end
    end

    initial begin
        rst_n = 1'b0;
        req_valid = 1'b0;
        frame_end = 1'b0;
        count_kills = 1'b0;
        key_x = {COORD_WIDTH{1'b0}};
        key_y = {COORD_WIDTH{1'b0}};

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (n = 0; n < 65; n = n + 1) begin
            send_point(11'sd21, 11'sd7);
        end

        repeat (40) @(posedge clk);
        count_kills = 1'b1;
        frame_end = 1'b1;
        @(posedge clk);
        frame_end = 1'b0;

        timeout_count = 0;
        seen_flush_done = 0;
        while (!seen_flush_done && timeout_count < 2000) begin
            timeout_count = timeout_count + 1;
            @(posedge clk);
            if (flush_done) begin
                seen_flush_done = 1;
            end
        end
        repeat (10) @(posedge clk);

        if (!seen_flush_done) begin
            $display("FAIL: flush_done timeout flush_cnt=%0d flushing=%0d s0_force=%0d s1_force=%0d task_count=%0d copy_st=%0d fsm_read_en=%0d fsm_read_en_d1=%0d bram_expire_wr_b=%0d notify_count=%0d fifo_level=%0d fifo_valid=%0d kill_clear_busy=%0d extra_kill_valid=%0d kill_valid=%0d kill_chain_valid=%0d",
                     dut.u_hash_expire_manager.flush_cnt,
                     dut.u_hash_expire_manager.flushing,
                     dut.u_hash_expire_manager.s0_force_expire,
                     dut.u_hash_expire_manager.s1_force_expire,
                     dut.u_hash_expire_manager.task_count,
                     dut.u_hash_expire_manager.copy_st,
                     dut.u_hash_expire_manager.fsm_read_en,
                     dut.u_hash_expire_manager.fsm_read_en_d1,
                     dut.u_hash_expire_manager.bram_expire_wr_b,
                     dut.u_hash_expire_manager.notify_count,
                     dut.u_hash_expire_manager.fifo_out_level,
                     dut.u_hash_expire_manager.fifo_out_valid,
                     dut.kill_clear_busy,
                     dut.extra_kill_valid,
                     dut.kill_valid,
                     dut.kill_chain_valid);
            $finish;
        end
        if (expire_event_count !== 1) begin
            $display("FAIL: expire_event_count=%0d expected 1", expire_event_count);
            $finish;
        end
        if (last_expire_pn !== 7'd65) begin
            $display("FAIL: last_expire_pn=%0d expected 65", last_expire_pn);
            $finish;
        end
        if (kill_write_count !== 3) begin
            $display("FAIL: kill_write_count=%0d expected 3", kill_write_count);
            $finish;
        end

        $display("PASS: expire_event_count=%0d last_expire_pn=%0d kill_write_count=%0d",
                 expire_event_count, last_expire_pn, kill_write_count);
        $finish;
    end
endmodule
