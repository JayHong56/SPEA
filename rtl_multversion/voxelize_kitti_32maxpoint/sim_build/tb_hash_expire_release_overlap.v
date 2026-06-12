`timescale 1ns / 1ps

module tb_hash_expire_release_overlap;
    localparam COORD_WIDTH = 11;
    localparam VN_WIDTH = 6;
    localparam TIMER_WIDTH = 18;
    localparam ADDR_WIDTH = 8;
    localparam LIFE_CYCLE = 4;
    localparam TABLE_SIZE = 256;
    localparam BUCKET_AW = 6;
    localparam BUCKET_SLOT_WIDTH = 2;
    localparam ENTRY_WIDTH = 2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + TIMER_WIDTH;
    localparam BRAM_DATA_WIDTH = 576;
    localparam BRAM_ADDR_WIDTH = 10;
    localparam BRAM_ADDR_WIDTH_PFE = 8;

    localparam ST_OCCU = 2'b01;
    localparam ST_TOMB = 2'b10;
    localparam ST_DRAIN = 2'b11;

    reg clk;
    reg rst_n;
    reg write_commit;
    reg [ADDR_WIDTH-1:0] write_addr;
    reg [TIMER_WIDTH-1:0] time_now;
    reg frame_end;

    wire flush_done;
    wire [BUCKET_AW-1:0] a_addr_shadow;
    wire [BUCKET_SLOT_WIDTH-1:0] a_we_shadow;
    wire [ENTRY_WIDTH-1:0] a_rdata_shadow;
    wire [ENTRY_WIDTH-1:0] b_wdata;
    wire [BUCKET_AW-1:0] b_addr;
    wire [BUCKET_SLOT_WIDTH-1:0] b_we;
    wire kill_valid;
    wire kill_expired;
    wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a;
    wire bram_expire_wr_b;
    wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b;
    wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b;
    wire m_axis_expire_tvalid;
    wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata;
    wire hash_stall;

    wire [ADDR_WIDTH-1:0] shadow_slot = {a_addr_shadow, a_we_shadow};
    wire signed [COORD_WIDTH-1:0] shadow_x = $signed({3'b000, shadow_slot});
    wire signed [COORD_WIDTH-1:0] shadow_y = $signed({3'b000, shadow_slot}) + 11'sd100;

    assign a_rdata_shadow = {ST_OCCU, shadow_x, shadow_y, 6'd1, {TIMER_WIDTH{1'b0}}};

    hash_expire_manager #(
        .COORD_WIDTH(COORD_WIDTH),
        .VN_WIDTH(VN_WIDTH),
        .TIMER_WIDTH(TIMER_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .LIFE_CYCLE(LIFE_CYCLE),
        .TABLE_SIZE(TABLE_SIZE),
        .BUCKET_AW(BUCKET_AW),
        .BUCKET_SLOT_WIDTH(BUCKET_SLOT_WIDTH),
        .ENTRY_WIDTH(ENTRY_WIDTH),
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH(BRAM_ADDR_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .write_commit(write_commit),
        .write_addr(write_addr),
        .time_now(time_now),
        .frame_end(frame_end),
        .flush_done(flush_done),
        .a_addr_shadow(a_addr_shadow),
        .a_we_shadow(a_we_shadow),
        .a_rdata_shadow(a_rdata_shadow),
        .b_wdata(b_wdata),
        .b_addr(b_addr),
        .b_we(b_we),
        .kill_valid(kill_valid),
        .kill_expired(kill_expired),
        .bram_expire_addr_a(bram_expire_addr_a),
        .bram_expire_rdata_a({BRAM_DATA_WIDTH{1'b1}}),
        .bram_expire_wr_b(bram_expire_wr_b),
        .bram_expire_addr_b(bram_expire_addr_b),
        .bram_expire_wrdata_b(bram_expire_wrdata_b),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tready(1'b1),
        .m_axis_expire_tdata(m_axis_expire_tdata),
        .hash_stall(hash_stall)
    );

    integer i;
    integer drain_seen;
    integer tomb_seen;
    integer release_expired_overlap_seen;
    integer release_kill_overlap_seen;
    integer drain_to_tomb_overlap_seen;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drain_seen <= 0;
            tomb_seen <= 0;
            release_expired_overlap_seen <= 0;
            release_kill_overlap_seen <= 0;
            drain_to_tomb_overlap_seen <= 0;
        end else begin
            if (dut.release_valid && dut.s1_valid && dut.expired) begin
                release_expired_overlap_seen <= release_expired_overlap_seen + 1;
                if (dut.expire_accept) begin
                    $display("FAIL: expire_accept asserted during release/expired overlap at %0t", $time);
                    $finish;
                end
                if (!dut.expire_block) begin
                    $display("FAIL: expire_block missing during release/expired overlap at %0t", $time);
                    $finish;
                end
                if (dut.pipe_advance) begin
                    $display("FAIL: expire pipeline advanced during release/expired overlap at %0t", $time);
                    $finish;
                end
            end

            if (dut.release_valid && kill_valid) begin
                release_kill_overlap_seen <= release_kill_overlap_seen + 1;
            end

            if (kill_valid && b_wdata[ENTRY_WIDTH-1-:2] == ST_DRAIN) begin
                drain_seen <= drain_seen + 1;
            end
            if (kill_valid && b_wdata[ENTRY_WIDTH-1-:2] == ST_TOMB) begin
                tomb_seen <= tomb_seen + 1;
            end

            if (dut.release_valid && kill_valid && b_wdata[ENTRY_WIDTH-1-:2] == ST_DRAIN) begin
                #1;
                if (!(kill_valid && b_wdata[ENTRY_WIDTH-1-:2] == ST_TOMB)) begin
                    $display("FAIL: release did not update next kill output to TOMB at %0t", $time);
                    $finish;
                end
                drain_to_tomb_overlap_seen <= drain_to_tomb_overlap_seen + 1;
            end
        end
    end

    initial begin
        rst_n = 1'b0;
        write_commit = 1'b0;
        write_addr = {ADDR_WIDTH{1'b0}};
        time_now = 18'd100;
        frame_end = 1'b0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (i = 0; i < 28; i = i + 1) begin
            @(posedge clk);
            write_commit <= 1'b1;
            write_addr <= i[ADDR_WIDTH-1:0];
            time_now <= 18'd100 + i[TIMER_WIDTH-1:0];
        end

        @(posedge clk);
        write_commit <= 1'b0;

        repeat (80) @(posedge clk);

        if (drain_seen == 0) begin
            $display("FAIL: no ST_DRAIN kill write observed");
            $finish;
        end
        if (tomb_seen == 0) begin
            $display("FAIL: no ST_TOMB release write observed");
            $finish;
        end
        if (release_expired_overlap_seen == 0) begin
            $display("FAIL: release_valid/expired overlap was not exercised");
            $finish;
        end
        if (release_kill_overlap_seen == 0) begin
            $display("FAIL: release_valid/kill_valid overlap was not exercised");
            $finish;
        end
        if (drain_to_tomb_overlap_seen == 0) begin
            $display("FAIL: old DRAIN kill plus release-to-TOMB overlap was not exercised");
            $finish;
        end

        $display("PASS: release overlap timing checked. DRAIN=%0d TOMB=%0d rel_exp=%0d rel_kill=%0d drain_to_tomb=%0d",
                 drain_seen, tomb_seen, release_expired_overlap_seen, release_kill_overlap_seen, drain_to_tomb_overlap_seen);
        $finish;
    end
endmodule
