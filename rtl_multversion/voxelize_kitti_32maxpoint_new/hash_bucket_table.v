module hash_bucket_table #(
    parameter         COORD_WIDTH         = 11,
    parameter         VN_WIDTH            = 7,                                      // point number width
    parameter         LIFE_CYCLE          = 16'd100,
    parameter         TIMER_WIDTH         = 18,
    parameter         ADDR_WIDTH          = 8,                                      // 9 -> 512 depths
    parameter         MAX_VOXEL_NUM       = 16'd100,
    parameter         ENTRY_POINT_CAP     = 16'd32,
    parameter         BUCKETS             = 64,
    parameter         BUCKET_SLOT_WIDTH   = 2,                                      // 4 slots per bucket
    parameter         TABLE_SIZE          = BUCKETS * (1'b1 << BUCKET_SLOT_WIDTH),  // 256 slots
    parameter integer BRAM_DATA_WIDTH     = 576,                                    // 8 * 72
    parameter integer BRAM_ADDR_WIDTH     = 10,                                     // 256 pillars * 4 rows
    parameter integer BRAM_ADDR_WIDTH_PFE = 12,                                     // PFE cache row address
    parameter integer BYTE_WIDTH          = 9
) (
    input wire clk,
    input wire rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire frame_end,  // 外部传入：当前帧点云结束脉冲 (1-cycle pulse)
    output wire flush_done,  // 输出：哈希表剩余有效点已全部输出完毕
    input wire signed [COORD_WIDTH-1:0] key_x,
    input wire signed [COORD_WIDTH-1:0] key_y,
    output wire hash_stall,
    output reg out_valid,
    output reg [ADDR_WIDTH-1:0] out_idx,  // 0..255 = {bucket[5:0], way[1:0]}
    output reg [VN_WIDTH-1:0] out_point_number,
    output reg table_full,
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    output wire bram_expire_wr_b,
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    // axi-stream 
    output wire m_axis_expire_tvalid,
    input wire m_axis_expire_tready,
    output wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata
);
    localparam ST_EMPTY = 2'b00;
    localparam ST_OCCU = 2'b01;
    localparam ST_TOMB = 2'b10;
    localparam BUCKET_AW = $clog2(BUCKETS);
    localparam ENTRY_WIDTH = 2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + TIMER_WIDTH;
    localparam integer CAM_DEPTH = 16;
    localparam CAM_AW = $clog2(CAM_DEPTH);
    localparam integer MAX_CHAIN = (MAX_VOXEL_NUM + ENTRY_POINT_CAP - 1) / ENTRY_POINT_CAP;

    reg [BUCKET_AW-1:0] a_addr_0, a_addr_1, a_addr_2, a_addr_3;
    reg [BUCKET_AW-1:0] a_addr_0_backup, a_addr_1_backup, a_addr_2_backup, a_addr_3_backup;
    wire [BUCKET_AW-1:0] a_addr_shadow;
    reg [ENTRY_WIDTH-1:0] wdata0_b, wdata1_b, wdata2_b, wdata3_b;
    reg we0_b, we1_b, we2_b, we3_b;
    reg  [        BUCKET_AW-1:0] shared_write_addr_bucket;
    wire [BUCKET_SLOT_WIDTH-1:0] shared_write_addr_slot = we0_b ? 2'd0 : we1_b ? 2'd1 : we2_b ? 2'd2 : we3_b ? 2'd3 : 2'd0;
    wire [       ADDR_WIDTH-1:0] shared_write_addr = {shared_write_addr_bucket, shared_write_addr_slot};
    wire [ENTRY_WIDTH-1:0] rdata0_a, rdata1_a, rdata2_a, rdata3_a;
    wire en0_a, en1_a, en2_a, en3_a;
    wire [BUCKET_AW-1:0] a_addr_0_main;
    wire [BUCKET_AW-1:0] a_addr_1_main;
    wire [BUCKET_AW-1:0] a_addr_2_main;
    wire [BUCKET_AW-1:0] a_addr_3_main;
    hash_bucket_array #(
        .ENTRY_WIDTH(ENTRY_WIDTH),
        .BUCKETS    (BUCKETS),
        .BUCKET_AW  (BUCKET_AW)
    ) u_hash_bucket_array (
        .clk     (clk),
        .rst_n   (rst_n),
        // Port A
        .a_addr_0(a_addr_0_main),
        .a_addr_1(a_addr_1_main),
        .a_addr_2(a_addr_2_main),
        .a_addr_3(a_addr_3_main),
        .en0_a   (en0_a),
        .en1_a   (en1_a),
        .en2_a   (en2_a),
        .en3_a   (en3_a),
        .rdata0_a(rdata0_a),
        .rdata1_a(rdata1_a),
        .rdata2_a(rdata2_a),
        .rdata3_a(rdata3_a),
        // Port B
        .b_addr  (shared_write_addr_bucket),
        .we0_b   (we0_b),
        .we1_b   (we1_b),
        .we2_b   (we2_b),
        .we3_b   (we3_b),
        .wdata0_b(wdata0_b),
        .wdata1_b(wdata1_b),
        .wdata2_b(wdata2_b),
        .wdata3_b(wdata3_b)
    );
    
    reg we0_a_shadow, we1_a_shadow, we2_a_shadow, we3_a_shadow;
    wire [ENTRY_WIDTH-1:0] rdata0_a_shadow, rdata1_a_shadow, rdata2_a_shadow, rdata3_a_shadow;
    hash_bucket_array_shadow #(
        .ENTRY_WIDTH(ENTRY_WIDTH),
        .BUCKETS    (BUCKETS),
        .BUCKET_AW  (BUCKET_AW)
    ) u_hash_bucket_array_shadow (
        .clk     (clk),
        .rst_n   (rst_n),
        // Port A
        .a_addr  (a_addr_shadow),
        .en0_a   (1'b1),
        .en1_a   (1'b1),
        .en2_a   (1'b1),
        .en3_a   (1'b1),
        .rdata0_a(rdata0_a_shadow),
        .rdata1_a(rdata1_a_shadow),
        .rdata2_a(rdata2_a_shadow),
        .rdata3_a(rdata3_a_shadow),
        // Port B
        .b_addr  (shared_write_addr_bucket),
        .we0_b   (we0_b),
        .we1_b   (we1_b),
        .we2_b   (we2_b),
        .we3_b   (we3_b),
        .wdata0_b(wdata0_b),
        .wdata1_b(wdata1_b),
        .wdata2_b(wdata2_b),
        .wdata3_b(wdata3_b)
    );

    wire [BUCKET_AW-1:0] bucket_idx_0;
    wire [BUCKET_AW-1:0] bucket_idx_1;
    wire [BUCKET_AW-1:0] bucket_idx_2;
    wire [BUCKET_AW-1:0] bucket_idx_3;
    // Hash 0: 原始
    hash_func_multiplicative #(
        .COORD_WIDTH(COORD_WIDTH),
        .BUCKET_AW  (BUCKET_AW),
        .SEED       (0)             // <--- 种子 0
    ) u_hash_func_0 (
        .key_x   (key_x),
        .key_y   (key_y),
        .hash_out(bucket_idx_0)
    );

    // Hash 1: X 取反
    hash_func_multiplicative #(
        .COORD_WIDTH(COORD_WIDTH),
        .BUCKET_AW  (BUCKET_AW),
        .SEED       (1)             // <--- 种子 1
    ) u_hash_func_1 (
        .key_x   (key_x),
        .key_y   (key_y),
        .hash_out(bucket_idx_1)
    );

    // Hash 2: Y 取反
    hash_func_multiplicative #(
        .COORD_WIDTH(COORD_WIDTH),
        .BUCKET_AW  (BUCKET_AW),
        .SEED       (2)             // <--- 种子 2
    ) u_hash_func_2 (
        .key_x   (key_x),
        .key_y   (key_y),
        .hash_out(bucket_idx_2)
    );

    // Hash 3: 异或扰动
    hash_func_multiplicative #(
        .COORD_WIDTH(COORD_WIDTH),
        .BUCKET_AW  (BUCKET_AW),
        .SEED       (3)             // <--- 种子 3
    ) u_hash_func_3 (
        .key_x   (key_x),
        .key_y   (key_y),
        .hash_out(bucket_idx_3)
    );

    reg s0_valid;
    reg signed [COORD_WIDTH-1:0] s0_x, s0_y;
    reg [BUCKET_AW-1:0] s0_bucket_0, s0_bucket_1, s0_bucket_2, s0_bucket_3;
    reg s1_valid;
    reg signed [COORD_WIDTH-1:0] s1_x, s1_y;
    reg [BUCKET_AW-1:0] s1_bucket_0, s1_bucket_1, s1_bucket_2, s1_bucket_3;

    wire kill_valid;
    wire kill_expired;
    reg kill_write;
    wire action_hit_base;
    wire action_hit_cam;
    wire action_hit_cam_full;
    wire action_free;
    wire action_alloc_regular_free;
    wire action_alloc_spill_free;
    wire action_drop_full;
    wire action_kill;
    wire action_full;

    // action_hit/free 可以在 expired 有效，无法在valid、write有效
    // 因为s0 s1状态机只能在write运转
    wire did_write = (action_hit_base | action_hit_cam | action_hit_cam_full | action_free) & (!hash_stall);
    // wire did_write = busy & (!kill_valid) & (!kill_write);
    wire [ENTRY_WIDTH-1:0] main_write_data = (we0_b) ? wdata0_b :
                                        (we1_b) ? wdata1_b :
                                        (we2_b) ? wdata2_b :
                                        (we3_b) ? wdata3_b : {ENTRY_WIDTH{1'b0}};

    reg [TIMER_WIDTH-1:0] global_timer;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            global_timer <= {TIMER_WIDTH{1'b0}};
        end else if (flush_done) begin
            global_timer <= {TIMER_WIDTH{1'b0}};
        end else if (did_write) begin
            global_timer <= global_timer + 1'b1;
        end
    end

    function [ENTRY_WIDTH-1:0] make_entry;
        input [1:0] st;
        input signed [COORD_WIDTH-1:0] kx;
        input signed [COORD_WIDTH-1:0] ky;
        input [VN_WIDTH-1:0] pn;
        input [TIMER_WIDTH-1:0] ts;
        begin
            make_entry = {st, kx, ky, pn, ts};
        end
    endfunction

    // action_full crush
    reg is_probing;
    reg probe_wait;
    wire kill_stall;
    reg [BUCKET_AW-1:0] probe_offset;
    reg probe_spill_mode;
    reg probe_spill_chain_exists;
    reg [ADDR_WIDTH-1:0] probe_spill_root_idx;
    wire action_true_full = action_full && (probe_offset == {BUCKET_AW{1'b1}});  // 探测了一整圈依旧满
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_probing   <= 1'b0;
            probe_wait   <= 1'b0;
            probe_offset <= {BUCKET_AW{1'b0}};
            probe_spill_mode <= 1'b0;
            probe_spill_chain_exists <= 1'b0;
            probe_spill_root_idx <= {ADDR_WIDTH{1'b0}};

        end else if (flush_done) begin
            is_probing   <= 1'b0;
            probe_wait   <= 1'b0;
            probe_offset <= {BUCKET_AW{1'b0}};
            probe_spill_mode <= 1'b0;
            probe_spill_chain_exists <= 1'b0;
            probe_spill_root_idx <= {ADDR_WIDTH{1'b0}};

        end else begin
            if (kill_stall | hash_stall) begin
                // 被 kill 打断，复位探测状态
                // is_probing   <= 1'b0;
                // probe_wait   <= 1'b0;
                // probe_offset <= {BUCKET_AW{1'b0}};
                is_probing   <= is_probing;
                probe_wait   <= probe_wait;
                probe_offset <= probe_offset;
            end else if (is_probing) begin
                if (probe_wait) begin
                    probe_wait <= 1'b0;  // BRAM 读取周期等待结束
                end else begin
                    if (action_free) begin
                        // 探测成功，退出探测模式
                        is_probing   <= 1'b0;
                        probe_offset <= {BUCKET_AW{1'b0}};
                        probe_spill_mode <= 1'b0;
                        probe_spill_chain_exists <= 1'b0;
                        probe_spill_root_idx <= {ADDR_WIDTH{1'b0}};
                    end else begin
                        if (probe_offset == {BUCKET_AW{1'b1}}) begin
                            // 已经搜遍全表，真满，放弃探测
                            is_probing   <= 1'b0;
                            probe_offset <= {BUCKET_AW{1'b0}};
                            probe_spill_mode <= 1'b0;
                            probe_spill_chain_exists <= 1'b0;
                            probe_spill_root_idx <= {ADDR_WIDTH{1'b0}};
                        end else begin
                            // 仍满，继续探测下一个地址
                            probe_wait   <= 1'b1;
                            probe_offset <= probe_offset + 1'b1;
                        end
                    end
                end

            end else if (action_full && probe_offset != {BUCKET_AW{1'b1}}) begin
                // 首次触发 action_full，启动探测
                is_probing   <= 1'b1;
                probe_wait   <= 1'b1;
                probe_offset <= 1'b1;
                probe_spill_mode <= spill_alloc_need;
                probe_spill_chain_exists <= cam_full_hit;
                probe_spill_root_idx <= spill_root_idx;
            end
        end
    end

    // 动态叠加当前的偏移量，获取真实正在评价的 Bucket
    wire [  BUCKET_AW-1:0] eval_bucket_0 = s1_bucket_0 + probe_offset;
    wire [  BUCKET_AW-1:0] eval_bucket_1 = s1_bucket_1 + probe_offset;
    wire [  BUCKET_AW-1:0] eval_bucket_2 = s1_bucket_2 + probe_offset;
    wire [  BUCKET_AW-1:0] eval_bucket_3 = s1_bucket_3 + probe_offset;
    // ---------------- bypass (forwarding) ----------------
    // 旁路命中：主要解决连续查询同一个(x,y)点时，前一个写回还未完成但后一个查询已经到达s1阶段的情况
    wire [ENTRY_WIDTH-1:0] next_bp_data;
    wire [ ADDR_WIDTH-1:0] next_bp_addr;

    // 2级旁路寄存器
    reg bp1_valid, bp2_valid;
    reg [ADDR_WIDTH-1:0] bp1_addr, bp2_addr;
    reg [ENTRY_WIDTH-1:0] bp1_data, bp2_data;

    // 检查正在评价的地址是否命中旁路缓存
    wire                          match_bp1_0 = bp1_valid && ({eval_bucket_0, 2'd0} == bp1_addr);
    wire                          match_bp1_1 = bp1_valid && ({eval_bucket_1, 2'd1} == bp1_addr);
    wire                          match_bp1_2 = bp1_valid && ({eval_bucket_2, 2'd2} == bp1_addr);
    wire                          match_bp1_3 = bp1_valid && ({eval_bucket_3, 2'd3} == bp1_addr);
    wire                          match_bp2_0 = bp2_valid && ({eval_bucket_0, 2'd0} == bp2_addr);
    wire                          match_bp2_1 = bp2_valid && ({eval_bucket_1, 2'd1} == bp2_addr);
    wire                          match_bp2_2 = bp2_valid && ({eval_bucket_2, 2'd2} == bp2_addr);
    wire                          match_bp2_3 = bp2_valid && ({eval_bucket_3, 2'd3} == bp2_addr);

    // 将读口数据在命中时无缝替换为旁路缓存数据（bp1 为最新，优先级 > bp2）
    wire        [ENTRY_WIDTH-1:0] rdata0_a_eff = match_bp1_0 ? bp1_data : match_bp2_0 ? bp2_data : rdata0_a;
    wire        [ENTRY_WIDTH-1:0] rdata1_a_eff = match_bp1_1 ? bp1_data : match_bp2_1 ? bp2_data : rdata1_a;
    wire        [ENTRY_WIDTH-1:0] rdata2_a_eff = match_bp1_2 ? bp1_data : match_bp2_2 ? bp2_data : rdata2_a;
    wire        [ENTRY_WIDTH-1:0] rdata3_a_eff = match_bp1_3 ? bp1_data : match_bp2_3 ? bp2_data : rdata3_a;


    wire        [            1:0] st                                                                        [0:3];
    wire signed [COORD_WIDTH-1:0] kx                                                                        [0:3];
    wire signed [COORD_WIDTH-1:0] ky                                                                        [0:3];
    wire        [   VN_WIDTH-1:0] pn                                                                        [0:3];
    wire        [TIMER_WIDTH-1:0] ts                                                                        [0:3];

    assign {st[0], kx[0], ky[0], pn[0], ts[0]} = rdata0_a_eff;
    assign {st[1], kx[1], ky[1], pn[1], ts[1]} = rdata1_a_eff;
    assign {st[2], kx[2], ky[2], pn[2], ts[2]} = rdata2_a_eff;
    assign {st[3], kx[3], ky[3], pn[3], ts[3]} = rdata3_a_eff;


    // shadow array select signal
    wire [BUCKET_SLOT_WIDTH-1:0] a_we_shadow;
    wire [ENTRY_WIDTH-1:0] a_rdata_shadow = (a_we_shadow == 2'd0) ? rdata0_a_shadow :
                                    (a_we_shadow == 2'd1) ? rdata1_a_shadow :
                                    (a_we_shadow == 2'd2) ? rdata2_a_shadow :
                                    (a_we_shadow == 2'd3) ? rdata3_a_shadow : {ENTRY_WIDTH{1'b0}};

    wire [BUCKET_AW-1:0] kill_addr_bucket;
    wire [BUCKET_SLOT_WIDTH-1:0] kill_we_bucket;
    wire [ENTRY_WIDTH-1:0] kill_wdata;
    wire [ADDR_WIDTH-1:0] kill_addr = {kill_addr_bucket, kill_we_bucket};
    wire [ADDR_WIDTH-1:0] kill_write_addr;
    wire signed [COORD_WIDTH-1:0] expire_lookup_key_x;
    wire signed [COORD_WIDTH-1:0] expire_lookup_key_y;
    wire [ADDR_WIDTH-1:0] expire_lookup_addr;
    wire signed [COORD_WIDTH-1:0] kill_key_x;
    wire signed [COORD_WIDTH-1:0] kill_key_y;
    wire [ADDR_WIDTH-1:0] kill_addr_flat;
    wire expire_chain_valid;
    wire [2:0] expire_chain_count;
    wire [ADDR_WIDTH-1:0] expire_chain_addr0;
    wire [ADDR_WIDTH-1:0] expire_chain_addr1;
    wire [ADDR_WIDTH-1:0] expire_chain_addr2;
    wire [ADDR_WIDTH-1:0] expire_chain_addr3;
    wire [VN_WIDTH-1:0] expire_chain_pn0;
    wire [VN_WIDTH-1:0] expire_chain_pn1;
    wire [VN_WIDTH-1:0] expire_chain_pn2;
    wire [VN_WIDTH-1:0] expire_chain_pn3;
    wire [VN_WIDTH-1:0] expire_chain_total_pn;
    wire [TIMER_WIDTH-1:0] expire_chain_last_ts;
    wire kill_chain_valid;
    wire [2:0] kill_chain_count;
    wire [ADDR_WIDTH-1:0] kill_chain_addr0;
    wire [ADDR_WIDTH-1:0] kill_chain_addr1;
    wire [ADDR_WIDTH-1:0] kill_chain_addr2;
    wire [ADDR_WIDTH-1:0] kill_chain_addr3;
    reg extra_kill_valid;
    reg [ADDR_WIDTH-1:0] extra_kill_addr;
    reg [2:0] extra_kill_count;
    reg [ADDR_WIDTH-1:0] extra_kill_q0, extra_kill_q1, extra_kill_q2;
    wire kill_clear_busy = extra_kill_valid | (kill_valid && kill_chain_valid);
    hash_expire_manager #(
        .COORD_WIDTH        (COORD_WIDTH),
        .VN_WIDTH           (VN_WIDTH),
        .TIMER_WIDTH        (TIMER_WIDTH),
        .ADDR_WIDTH         (ADDR_WIDTH),
        .LIFE_CYCLE         (LIFE_CYCLE),
        .MAX_VOXEL_NUM      (MAX_VOXEL_NUM),
        .ENTRY_POINT_CAP    (ENTRY_POINT_CAP),
        .TABLE_SIZE         (TABLE_SIZE),
        .BUCKET_AW          (BUCKET_AW),
        .BUCKET_SLOT_WIDTH  (BUCKET_SLOT_WIDTH),
        .ENTRY_WIDTH        (2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + TIMER_WIDTH),
        .BRAM_ADDR_WIDTH    (BRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH    (BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE),
        .BYTE_WIDTH         (BYTE_WIDTH)
    ) u_hash_expire_manager (
        .clk                 (clk),
        .rst_n               (rst_n),
        .write_commit        (did_write),
        .write_addr          (kill_write_addr),       // 0-255
        .kill_clear_busy     (kill_clear_busy),
        .expire_lookup_key_x (expire_lookup_key_x),
        .expire_lookup_key_y (expire_lookup_key_y),
        .expire_lookup_addr  (expire_lookup_addr),
        .expire_chain_valid  (expire_chain_valid),
        .expire_chain_count  (expire_chain_count),
        .expire_chain_addr0  (expire_chain_addr0),
        .expire_chain_addr1  (expire_chain_addr1),
        .expire_chain_addr2  (expire_chain_addr2),
        .expire_chain_addr3  (expire_chain_addr3),
        .expire_chain_pn0    (expire_chain_pn0),
        .expire_chain_pn1    (expire_chain_pn1),
        .expire_chain_pn2    (expire_chain_pn2),
        .expire_chain_pn3    (expire_chain_pn3),
        .expire_chain_total_pn(expire_chain_total_pn),
        .expire_chain_last_ts(expire_chain_last_ts),
        .time_now            (global_timer),
        .frame_end           (frame_end),             // <--- 接进来
        .flush_done          (flush_done),            // <--- 接出去
        .a_addr_shadow       (a_addr_shadow),
        .a_we_shadow         (a_we_shadow),
        .a_rdata_shadow      (a_rdata_shadow),
        .b_wdata             (kill_wdata),
        .b_addr              (kill_addr_bucket),
        .b_we                (kill_we_bucket),
        .kill_valid          (kill_valid),
        .kill_key_x          (kill_key_x),
        .kill_key_y          (kill_key_y),
        .kill_addr_flat      (kill_addr_flat),
        .kill_chain_valid    (kill_chain_valid),
        .kill_chain_count    (kill_chain_count),
        .kill_chain_addr0    (kill_chain_addr0),
        .kill_chain_addr1    (kill_chain_addr1),
        .kill_chain_addr2    (kill_chain_addr2),
        .kill_chain_addr3    (kill_chain_addr3),
        .kill_expired        (kill_expired),
        .bram_expire_addr_a  (bram_expire_addr_a),    // 10-bit Address
        .bram_expire_rdata_a (bram_expire_rdata_a),   // 576-bit Data
        .bram_expire_wr_b    (bram_expire_wr_b),
        .bram_expire_addr_b  (bram_expire_addr_b),    // 10-bit Address
        .bram_expire_wrdata_b(bram_expire_wrdata_b),  // 576-bit Data
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tready(m_axis_expire_tready),
        .m_axis_expire_tdata (m_axis_expire_tdata),
        .hash_stall          (hash_stall)
    );

    integer i;
    reg hit;
    reg [1:0] hit_way;
    reg free_found;
    reg [1:0] free_way;
    reg [1:0] free_way_rr;
    reg free_found_rr;
    reg [1:0] rr_ptr;
    wire [3:0] free_mask = {
        (st[3] == ST_EMPTY || st[3] == ST_TOMB),
        (st[2] == ST_EMPTY || st[2] == ST_TOMB),
        (st[1] == ST_EMPTY || st[1] == ST_TOMB),
        (st[0] == ST_EMPTY || st[0] == ST_TOMB)
    };

    // 新增：明确绑定敏感列表的 MUX，彻底杜绝仿真器装死
    wire [BUCKET_AW-1:0] hit_eval_bucket = 
        (hit_way == 2'd0) ? eval_bucket_0 :
        (hit_way == 2'd1) ? eval_bucket_1 :
        (hit_way == 2'd2) ? eval_bucket_2 : eval_bucket_3;

    wire [BUCKET_AW-1:0] free_eval_bucket = 
        (free_way == 2'd0) ? eval_bucket_0 :
        (free_way == 2'd1) ? eval_bucket_1 :
        (free_way == 2'd2) ? eval_bucket_2 : eval_bucket_3;

    // =========================================================================
    // 新增：内部 TLB 地址映射表 (CAM 结构)
    // =========================================================================
    reg cam_valid[0:CAM_DEPTH-1];
    reg signed [COORD_WIDTH-1:0] cam_kx[0:CAM_DEPTH-1];
    reg signed [COORD_WIDTH-1:0] cam_ky[0:CAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] cam_mapped_idx[0:CAM_DEPTH-1];  // 记录真实的 BRAM 物理地址
    reg [VN_WIDTH-1:0] cam_pn[0:CAM_DEPTH-1];  // 缓存点数，避免 1 周期 BRAM 读延迟

    reg cam_hit;
    reg [CAM_AW-1:0] cam_hit_idx;
    reg cam_free_found;
    reg [CAM_AW-1:0] cam_free_idx;

    integer j;
    always @(*) begin
        cam_hit        = 1'b0;
        cam_hit_idx    = {CAM_AW{1'b0}};
        cam_free_found = 1'b0;
        cam_free_idx   = {CAM_AW{1'b0}};

        for (j = 0; j < CAM_DEPTH; j = j + 1) begin  // 并行搜索 CAM 表
            if (cam_valid[j]) begin
                if ((cam_kx[j] == s1_x) && (cam_ky[j] == s1_y)) begin
                    cam_hit     = 1'b1;
                    cam_hit_idx = j[CAM_AW-1:0];
                end
            end else if (!cam_free_found) begin
                cam_free_found = 1'b1;
                cam_free_idx   = j[CAM_AW-1:0];
            end
        end
    end
    // 如果 CAM 满了被迫淘汰，直接用 global_timer 末尾几位做伪随机覆盖 (Pseudo-Random)
    wire [CAM_AW-1:0] target_cam_idx = cam_free_found ? cam_free_idx : global_timer[CAM_AW-1:0];

    // cam_full：同一个逻辑 pillar 超过单个物理 entry(32点) 后的链式元数据。
    reg cam_full_valid[0:CAM_DEPTH-1];
    reg signed [COORD_WIDTH-1:0] cam_full_kx[0:CAM_DEPTH-1];
    reg signed [COORD_WIDTH-1:0] cam_full_ky[0:CAM_DEPTH-1];
    reg [2:0] cam_full_seg_count[0:CAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] cam_full_idx0[0:CAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] cam_full_idx1[0:CAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] cam_full_idx2[0:CAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] cam_full_idx3[0:CAM_DEPTH-1];
    reg [VN_WIDTH-1:0] cam_full_pn0[0:CAM_DEPTH-1];
    reg [VN_WIDTH-1:0] cam_full_pn1[0:CAM_DEPTH-1];
    reg [VN_WIDTH-1:0] cam_full_pn2[0:CAM_DEPTH-1];
    reg [VN_WIDTH-1:0] cam_full_pn3[0:CAM_DEPTH-1];
    reg [VN_WIDTH-1:0] cam_full_total_pn[0:CAM_DEPTH-1];
    reg [TIMER_WIDTH-1:0] cam_full_last_ts[0:CAM_DEPTH-1];

    reg cam_full_hit;
    reg [CAM_AW-1:0] cam_full_hit_idx;
    reg cam_full_free_found;
    reg [CAM_AW-1:0] cam_full_free_idx;
    reg expire_chain_hit;
    reg [CAM_AW-1:0] expire_chain_hit_idx;

    integer cf_i;
    always @(*) begin
        cam_full_hit        = 1'b0;
        cam_full_hit_idx    = {CAM_AW{1'b0}};
        cam_full_free_found = 1'b0;
        cam_full_free_idx   = {CAM_AW{1'b0}};
        expire_chain_hit    = 1'b0;
        expire_chain_hit_idx = {CAM_AW{1'b0}};

        for (cf_i = 0; cf_i < CAM_DEPTH; cf_i = cf_i + 1) begin
            if (cam_full_valid[cf_i]) begin
                if ((cam_full_kx[cf_i] == s1_x) && (cam_full_ky[cf_i] == s1_y)) begin
                    cam_full_hit     = 1'b1;
                    cam_full_hit_idx = cf_i[CAM_AW-1:0];
                end
                if ((cam_full_kx[cf_i] == expire_lookup_key_x) && (cam_full_ky[cf_i] == expire_lookup_key_y)) begin
                    expire_chain_hit     = 1'b1;
                    expire_chain_hit_idx = cf_i[CAM_AW-1:0];
                end
            end else if (!cam_full_free_found) begin
                cam_full_free_found = 1'b1;
                cam_full_free_idx   = cf_i[CAM_AW-1:0];
            end
        end
    end

    wire [1:0] cam_full_tail_seg = cam_full_seg_count[cam_full_hit_idx][1:0] - 1'b1;
    wire [ADDR_WIDTH-1:0] cam_full_tail_idx =
        (cam_full_tail_seg == 2'd0) ? cam_full_idx0[cam_full_hit_idx] :
        (cam_full_tail_seg == 2'd1) ? cam_full_idx1[cam_full_hit_idx] :
        (cam_full_tail_seg == 2'd2) ? cam_full_idx2[cam_full_hit_idx] :
                                      cam_full_idx3[cam_full_hit_idx];
    wire [VN_WIDTH-1:0] cam_full_tail_pn =
        (cam_full_tail_seg == 2'd0) ? cam_full_pn0[cam_full_hit_idx] :
        (cam_full_tail_seg == 2'd1) ? cam_full_pn1[cam_full_hit_idx] :
        (cam_full_tail_seg == 2'd2) ? cam_full_pn2[cam_full_hit_idx] :
                                      cam_full_pn3[cam_full_hit_idx];

    assign expire_chain_valid   = expire_chain_hit;
    assign expire_chain_count   = cam_full_seg_count[expire_chain_hit_idx];
    assign expire_chain_addr0   = cam_full_idx0[expire_chain_hit_idx];
    assign expire_chain_addr1   = cam_full_idx1[expire_chain_hit_idx];
    assign expire_chain_addr2   = cam_full_idx2[expire_chain_hit_idx];
    assign expire_chain_addr3   = cam_full_idx3[expire_chain_hit_idx];
    assign expire_chain_pn0     = cam_full_pn0[expire_chain_hit_idx];
    assign expire_chain_pn1     = cam_full_pn1[expire_chain_hit_idx];
    assign expire_chain_pn2     = cam_full_pn2[expire_chain_hit_idx];
    assign expire_chain_pn3     = cam_full_pn3[expire_chain_hit_idx];
    assign expire_chain_total_pn = cam_full_total_pn[expire_chain_hit_idx];
    assign expire_chain_last_ts = cam_full_last_ts[expire_chain_hit_idx];

    wire hit_full = hit && (pn[hit_way] >= ENTRY_POINT_CAP[VN_WIDTH-1:0]);
    wire cam_hit_full = cam_hit && (cam_pn[cam_hit_idx] >= ENTRY_POINT_CAP[VN_WIDTH-1:0]);
    wire cam_full_tail_full = cam_full_hit && (cam_full_tail_pn >= ENTRY_POINT_CAP[VN_WIDTH-1:0]);
    wire cam_full_at_logical_cap = cam_full_hit && (cam_full_total_pn[cam_full_hit_idx] >= MAX_VOXEL_NUM[VN_WIDTH-1:0]);
    wire cam_full_has_seg_room = cam_full_hit && (cam_full_seg_count[cam_full_hit_idx] < MAX_CHAIN);
    wire cam_full_tail_writable = cam_full_hit && !cam_full_at_logical_cap && !cam_full_tail_full;
    wire first_spill_need = !cam_full_hit && ((hit && hit_full) || (!hit && cam_hit && cam_hit_full));
    wire chain_spill_need = cam_full_hit && !cam_full_at_logical_cap && cam_full_tail_full && cam_full_has_seg_room;
    wire spill_alloc_need = (is_probing && probe_spill_mode) || first_spill_need || chain_spill_need;
    wire regular_alloc_need = !cam_full_hit && !hit && !cam_hit && !probe_spill_mode;
    wire spill_chain_exists = (is_probing && probe_spill_mode) ? probe_spill_chain_exists : cam_full_hit;
    wire spill_meta_available = spill_chain_exists || cam_full_free_found;
    wire [ADDR_WIDTH-1:0] active_primary_idx = hit ? {hit_eval_bucket, hit_way} : cam_mapped_idx[cam_hit_idx];
    wire [ADDR_WIDTH-1:0] spill_root_idx = (is_probing && probe_spill_mode) ? probe_spill_root_idx :
                                           cam_full_hit ? cam_full_idx0[cam_full_hit_idx] : active_primary_idx;

    assign next_bp_data = action_hit_base ? make_entry(
        ST_OCCU, s1_x, s1_y, pn[hit_way] + 1'b1, global_timer
    ) : action_hit_cam ? make_entry(
        ST_OCCU,
        s1_x,
        s1_y,
        cam_pn[cam_hit_idx] + 1'b1,
        global_timer
    ) : action_hit_cam_full ? make_entry(
        ST_OCCU,
        s1_x,
        s1_y,
        cam_full_tail_pn + 1'b1,
        global_timer
    ) : action_free ? make_entry(
        ST_OCCU, s1_x, s1_y, {{(VN_WIDTH - 1) {1'b0}}, 1'b1}, global_timer
    ) : {ENTRY_WIDTH{1'b0}};

    assign next_bp_addr = action_hit_base ? {hit_eval_bucket, hit_way} : 
                          action_hit_cam  ? cam_mapped_idx[cam_hit_idx] : 
                          action_hit_cam_full ? cam_full_tail_idx :
                          action_free     ? {free_eval_bucket, free_way} : 
                          {ADDR_WIDTH{1'b0}};

`ifndef SYNTHESIS
    integer fp, fp2;  // 文件句柄
    initial begin
        // 1. 打开文件 ( "w" 表示覆盖写入, "a" 表示追加 )
        fp  = $fopen("ssimulation_result.csv", "w");
        fp2 = $fopen("ssimulation_result_killed.csv", "w");

        // 2. 写入表头 (可选)
        $fwrite(fp, "Key_X, Key_Y, Bucket_ID, Total_Load\n");
        // $fwrite(fp, "")
    end
`endif
    // -------- actions for current s1 (combinational) --------
    assign kill_stall = kill_expired | kill_valid | kill_write | extra_kill_valid;
    wire full_stall = (action_full | is_probing) & (!action_free);
    wire pipe_stall = hash_stall | kill_stall | (full_stall & (!action_true_full));
    assign req_ready = !pipe_stall;  // 已修复：添加kill_write时，不能够正确阻塞前面的模块

    // -------- actions for current s1 (combinational) --------
    // action_hit/free 可以在 expired 有效（因为，无法在valid、write有效
    assign action_hit_base = s1_valid && (!cam_full_hit) && hit && !hit_full && !kill_stall && (!probe_wait) && !hash_stall;
    assign action_hit_cam = s1_valid && (!cam_full_hit) && (!hit) && cam_hit && !cam_hit_full && !kill_stall && (!probe_wait) && !hash_stall;
    assign action_hit_cam_full = s1_valid && cam_full_tail_writable && !kill_stall && (!probe_wait) && !hash_stall;

    assign action_alloc_regular_free = s1_valid && regular_alloc_need && free_found && !kill_stall && (!probe_wait) && !hash_stall;
    assign action_alloc_spill_free = s1_valid && spill_alloc_need && spill_meta_available && free_found && !kill_stall && (!probe_wait) && !hash_stall;
    assign action_free = action_alloc_regular_free | action_alloc_spill_free;
    assign action_full = s1_valid && ((regular_alloc_need) || (spill_alloc_need && spill_meta_available)) &&
                         (!free_found) && !kill_stall && (!probe_wait) && !hash_stall;
    assign action_drop_full = s1_valid && !kill_stall && (!probe_wait) && !hash_stall &&
                              ((cam_full_at_logical_cap) ||
                               (spill_alloc_need && !spill_meta_available) ||
                               (cam_full_hit && cam_full_tail_full && !cam_full_has_seg_room));
    assign action_kill = kill_valid;

    assign kill_write_addr = next_bp_addr;

    // 针对bram的读延时，在kill_expired 和 action_full 的时候进行锁存 
    assign en0_a = (kill_stall | action_full | hash_stall) ? 1'b0 : 1'b1;  // 只有在 s1 阶段且不 stall 时才使能读端口
    assign en1_a = (kill_stall | action_full | hash_stall) ? 1'b0 : 1'b1;
    assign en2_a = (kill_stall | action_full | hash_stall) ? 1'b0 : 1'b1;
    assign en3_a = (kill_stall | action_full | hash_stall) ? 1'b0 : 1'b1;

    reg [2:0] chain_extra_count;
    reg [ADDR_WIDTH-1:0] chain_extra_q0, chain_extra_q1, chain_extra_q2;
    task push_extra_kill_addr;
        input [ADDR_WIDTH-1:0] addr;
        begin
            case (chain_extra_count)
                3'd0: chain_extra_q0 = addr;
                3'd1: chain_extra_q1 = addr;
                default: chain_extra_q2 = addr;
            endcase
            chain_extra_count = chain_extra_count + 1'b1;
        end
    endtask

    always @(*) begin
        chain_extra_count = 3'd0;
        chain_extra_q0    = {ADDR_WIDTH{1'b0}};
        chain_extra_q1    = {ADDR_WIDTH{1'b0}};
        chain_extra_q2    = {ADDR_WIDTH{1'b0}};
        if (kill_chain_valid) begin
            if ((kill_chain_count > 3'd0) && (kill_chain_addr0 != kill_addr)) begin
                push_extra_kill_addr(kill_chain_addr0);
            end
            if ((kill_chain_count > 3'd1) && (kill_chain_addr1 != kill_addr)) begin
                push_extra_kill_addr(kill_chain_addr1);
            end
            if ((kill_chain_count > 3'd2) && (kill_chain_addr2 != kill_addr)) begin
                push_extra_kill_addr(kill_chain_addr2);
            end
            if ((kill_chain_count > 3'd3) && (kill_chain_addr3 != kill_addr)) begin
                push_extra_kill_addr(kill_chain_addr3);
            end
        end
    end

    reg busy;
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kill_write               <= 1'b0;
            we0_b                    <= 1'b0;
            we1_b                    <= 1'b0;
            we2_b                    <= 1'b0;
            we3_b                    <= 1'b0;
            busy                     <= 1'b0;
            table_full               <= 1'b0;
            rr_ptr                   <= 2'd0;
            out_valid                <= 1'b0;
            extra_kill_valid         <= 1'b0;
            extra_kill_count         <= 3'd0;

            shared_write_addr_bucket <= {BUCKET_AW{1'b0}};
            out_idx                  <= {ADDR_WIDTH{1'b0}};
            out_point_number         <= {VN_WIDTH{1'b0}};

            for (k = 0; k < CAM_DEPTH; k = k + 1) begin
                cam_valid[k]      <= 1'b0;
                cam_full_valid[k] <= 1'b0;
            end

        end else if (flush_done) begin
            kill_write               <= 1'b0;
            we0_b                    <= 1'b0;
            we1_b                    <= 1'b0;
            we2_b                    <= 1'b0;
            we3_b                    <= 1'b0;
            busy                     <= 1'b0;
            table_full               <= 1'b0;
            rr_ptr                   <= 2'd0;
            out_valid                <= 1'b0;
            extra_kill_valid         <= 1'b0;
            extra_kill_count         <= 3'd0;

            shared_write_addr_bucket <= {BUCKET_AW{1'b0}};
            out_idx                  <= {ADDR_WIDTH{1'b0}};
            out_point_number         <= {VN_WIDTH{1'b0}};

            for (k = 0; k < CAM_DEPTH; k = k + 1) begin
                cam_valid[k]      <= 1'b0;
                cam_full_valid[k] <= 1'b0;
            end

        end else begin
            // defaults
            we0_b      <= 0;
            we1_b      <= 0;
            we2_b      <= 0;
            we3_b      <= 0;
            busy       <= 0;
            table_full <= 0;
            out_valid  <= 1'b0;

            if (extra_kill_valid) begin
                kill_write               <= 1'b1;
                we0_b                    <= extra_kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd0;
                we1_b                    <= extra_kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd1;
                we2_b                    <= extra_kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd2;
                we3_b                    <= extra_kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd3;
                shared_write_addr_bucket <= extra_kill_addr[(ADDR_WIDTH-1)-:BUCKET_AW];
                case (extra_kill_addr[BUCKET_SLOT_WIDTH-1:0])
                    2'd0: wdata0_b <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
                    2'd1: wdata1_b <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
                    2'd2: wdata2_b <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
                    2'd3: wdata3_b <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
                endcase
                for (k = 0; k < CAM_DEPTH; k = k + 1) begin
                    if (cam_valid[k] && cam_mapped_idx[k] == extra_kill_addr) begin
                        cam_valid[k] <= 1'b0;
                    end
                end
                if (extra_kill_count > 3'd1) begin
                    extra_kill_addr  <= extra_kill_q0;
                    extra_kill_q0    <= extra_kill_q1;
                    extra_kill_q1    <= extra_kill_q2;
                    extra_kill_count <= extra_kill_count - 1'b1;
                end else begin
                    extra_kill_valid <= 1'b0;
                    extra_kill_count <= 3'd0;
                end

            end else if (action_hit_base) begin
                busy                     <= 1'b1;
                out_valid                <= 1'b1;
                out_idx                  <= {hit_eval_bucket, hit_way};
                shared_write_addr_bucket <= hit_eval_bucket;

                out_point_number         <= pn[hit_way] + 1'b1;
                case (hit_way)
                    2'd0: begin
                        we0_b <= 1;
                        wdata0_b <= make_entry(ST_OCCU, s1_x, s1_y, pn[0] + 1'b1, global_timer);
                    end
                    2'd1: begin
                        we1_b <= 1;
                        wdata1_b <= make_entry(ST_OCCU, s1_x, s1_y, pn[1] + 1'b1, global_timer);
                    end
                    2'd2: begin
                        we2_b <= 1;
                        wdata2_b <= make_entry(ST_OCCU, s1_x, s1_y, pn[2] + 1'b1, global_timer);
                    end
                    2'd3: begin
                        we3_b <= 1;
                        wdata3_b <= make_entry(ST_OCCU, s1_x, s1_y, pn[3] + 1'b1, global_timer);
                    end
                endcase

            end else if (action_hit_cam) begin
                busy <= 1'b1;
                out_valid <= 1'b1;
                out_idx <= cam_mapped_idx[cam_hit_idx];
                shared_write_addr_bucket <= cam_mapped_idx[cam_hit_idx][ADDR_WIDTH-1 : BUCKET_SLOT_WIDTH];

                // 先行计算出新的点数
                out_point_number <= cam_pn[cam_hit_idx] + 1'b1;

                // 透写 (Write-Through) 强制刷新到 BRAM
                case (cam_mapped_idx[cam_hit_idx][BUCKET_SLOT_WIDTH-1:0])
                    2'd0: begin
                        we0_b <= 1;
                        wdata0_b <= make_entry(
                            ST_OCCU,
                            s1_x,
                            s1_y,
                            cam_pn[cam_hit_idx] + 1'b1,
                            global_timer
                        );
                    end
                    2'd1: begin
                        we1_b <= 1;
                        wdata1_b <= make_entry(
                            ST_OCCU,
                            s1_x,
                            s1_y,
                            cam_pn[cam_hit_idx] + 1'b1,
                            global_timer
                        );
                    end
                    2'd2: begin
                        we2_b <= 1;
                        wdata2_b <= make_entry(
                            ST_OCCU,
                            s1_x,
                            s1_y,
                            cam_pn[cam_hit_idx] + 1'b1,
                            global_timer
                        );
                    end
                    2'd3: begin
                        we3_b <= 1;
                        wdata3_b <= make_entry(
                            ST_OCCU,
                            s1_x,
                            s1_y,
                            cam_pn[cam_hit_idx] + 1'b1,
                            global_timer
                        );
                    end
                endcase

                // 更新 TLB 缓存内的点数与时间
                cam_pn[cam_hit_idx] <= cam_pn[cam_hit_idx] + 1'b1;

            end else if (action_hit_cam_full) begin
                busy                     <= 1'b1;
                out_valid                <= 1'b1;
                out_idx                  <= cam_full_tail_idx;
                shared_write_addr_bucket <= cam_full_tail_idx[ADDR_WIDTH-1 : BUCKET_SLOT_WIDTH];
                out_point_number         <= cam_full_tail_pn + 1'b1;
                case (cam_full_tail_idx[BUCKET_SLOT_WIDTH-1:0])
                    2'd0: begin
                        we0_b    <= 1;
                        wdata0_b <= make_entry(ST_OCCU, s1_x, s1_y, cam_full_tail_pn + 1'b1, global_timer);
                    end
                    2'd1: begin
                        we1_b    <= 1;
                        wdata1_b <= make_entry(ST_OCCU, s1_x, s1_y, cam_full_tail_pn + 1'b1, global_timer);
                    end
                    2'd2: begin
                        we2_b    <= 1;
                        wdata2_b <= make_entry(ST_OCCU, s1_x, s1_y, cam_full_tail_pn + 1'b1, global_timer);
                    end
                    2'd3: begin
                        we3_b    <= 1;
                        wdata3_b <= make_entry(ST_OCCU, s1_x, s1_y, cam_full_tail_pn + 1'b1, global_timer);
                    end
                endcase
                cam_full_total_pn[cam_full_hit_idx] <= cam_full_total_pn[cam_full_hit_idx] + 1'b1;
                cam_full_last_ts[cam_full_hit_idx]  <= global_timer;
                case (cam_full_tail_seg)
                    2'd0: cam_full_pn0[cam_full_hit_idx] <= cam_full_pn0[cam_full_hit_idx] + 1'b1;
                    2'd1: cam_full_pn1[cam_full_hit_idx] <= cam_full_pn1[cam_full_hit_idx] + 1'b1;
                    2'd2: cam_full_pn2[cam_full_hit_idx] <= cam_full_pn2[cam_full_hit_idx] + 1'b1;
                    default: cam_full_pn3[cam_full_hit_idx] <= cam_full_pn3[cam_full_hit_idx] + 1'b1;
                endcase

            end else if (action_free) begin
                busy                     <= 1'b1;
                out_valid                <= 1'b1;
                out_idx                  <= {free_eval_bucket, free_way};
                shared_write_addr_bucket <= free_eval_bucket;
                out_point_number         <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                case (free_way)
                    2'd0: begin
                        we0_b    <= 1;
                        wdata0_b <= make_entry(ST_OCCU, s1_x, s1_y, {{(VN_WIDTH - 1) {1'b0}}, 1'b1}, global_timer);
                    end
                    2'd1: begin
                        we1_b    <= 1;
                        wdata1_b <= make_entry(ST_OCCU, s1_x, s1_y, {{(VN_WIDTH - 1) {1'b0}}, 1'b1}, global_timer);
                    end
                    2'd2: begin
                        we2_b    <= 1;
                        wdata2_b <= make_entry(ST_OCCU, s1_x, s1_y, {{(VN_WIDTH - 1) {1'b0}}, 1'b1}, global_timer);
                    end
                    2'd3: begin
                        we3_b    <= 1;
                        wdata3_b <= make_entry(ST_OCCU, s1_x, s1_y, {{(VN_WIDTH - 1) {1'b0}}, 1'b1}, global_timer);
                    end
                endcase
                rr_ptr <= rr_ptr + 2'd1;
                if (action_alloc_regular_free && is_probing) begin
                    cam_valid[target_cam_idx]      <= 1'b1;
                    cam_kx[target_cam_idx]         <= s1_x;
                    cam_ky[target_cam_idx]         <= s1_y;
                    cam_mapped_idx[target_cam_idx] <= {free_eval_bucket, free_way};
                    cam_pn[target_cam_idx]         <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                end
                if (action_alloc_spill_free) begin
                    if (spill_chain_exists) begin
                        case (cam_full_seg_count[cam_full_hit_idx])
                            3'd1: begin
                                cam_full_idx1[cam_full_hit_idx] <= {free_eval_bucket, free_way};
                                cam_full_pn1[cam_full_hit_idx]  <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                            end
                            3'd2: begin
                                cam_full_idx2[cam_full_hit_idx] <= {free_eval_bucket, free_way};
                                cam_full_pn2[cam_full_hit_idx]  <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                            end
                            default: begin
                                cam_full_idx3[cam_full_hit_idx] <= {free_eval_bucket, free_way};
                                cam_full_pn3[cam_full_hit_idx]  <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                            end
                        endcase
                        cam_full_seg_count[cam_full_hit_idx] <= cam_full_seg_count[cam_full_hit_idx] + 1'b1;
                        cam_full_total_pn[cam_full_hit_idx]  <= cam_full_total_pn[cam_full_hit_idx] + 1'b1;
                        cam_full_last_ts[cam_full_hit_idx]   <= global_timer;
                    end else begin
                        cam_full_valid[cam_full_free_idx]     <= 1'b1;
                        cam_full_kx[cam_full_free_idx]        <= s1_x;
                        cam_full_ky[cam_full_free_idx]        <= s1_y;
                        cam_full_seg_count[cam_full_free_idx] <= 3'd2;
                        cam_full_idx0[cam_full_free_idx]      <= spill_root_idx;
                        cam_full_idx1[cam_full_free_idx]      <= {free_eval_bucket, free_way};
                        cam_full_idx2[cam_full_free_idx]      <= {ADDR_WIDTH{1'b0}};
                        cam_full_idx3[cam_full_free_idx]      <= {ADDR_WIDTH{1'b0}};
                        cam_full_pn0[cam_full_free_idx]       <= ENTRY_POINT_CAP[VN_WIDTH-1:0];
                        cam_full_pn1[cam_full_free_idx]       <= {{(VN_WIDTH - 1) {1'b0}}, 1'b1};
                        cam_full_pn2[cam_full_free_idx]       <= {VN_WIDTH{1'b0}};
                        cam_full_pn3[cam_full_free_idx]       <= {VN_WIDTH{1'b0}};
                        cam_full_total_pn[cam_full_free_idx]  <= ENTRY_POINT_CAP[VN_WIDTH-1:0] + 1'b1;
                        cam_full_last_ts[cam_full_free_idx]   <= global_timer;
                    end
                end

            end else if (action_true_full) begin
                table_full <= 1'b1;
            end else if (action_drop_full) begin
                table_full <= 1'b1;
            end

            if (action_kill) begin
                kill_write               <= 1'b1;
                we0_b                    <= kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd0;
                we1_b                    <= kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd1;
                we2_b                    <= kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd2;
                we3_b                    <= kill_addr[BUCKET_SLOT_WIDTH-1:0] == 2'd3;
                shared_write_addr_bucket <= kill_addr[(ADDR_WIDTH-1)-:BUCKET_AW];
                case (kill_addr[BUCKET_SLOT_WIDTH-1:0])
                    2'd0: wdata0_b <= kill_wdata;
                    2'd1: wdata1_b <= kill_wdata;
                    2'd2: wdata2_b <= kill_wdata;
                    2'd3: wdata3_b <= kill_wdata;
                endcase
                // 同步追踪：防悬空指针，如果废弃的 BRAM 地址在 TLB 有映射，直接置其无效
                for (k = 0; k < CAM_DEPTH; k = k + 1) begin
                    if (cam_valid[k] && cam_mapped_idx[k] == kill_addr) begin
                        cam_valid[k] <= 1'b0;
                    end
                    if (cam_full_valid[k] && (cam_full_kx[k] == kill_key_x) && (cam_full_ky[k] == kill_key_y)) begin
                        cam_full_valid[k] <= 1'b0;
                    end
                end
                if (chain_extra_count != 3'd0) begin
                    extra_kill_valid <= 1'b1;
                    extra_kill_addr  <= chain_extra_q0;
                    extra_kill_q0    <= chain_extra_q1;
                    extra_kill_q1    <= chain_extra_q2;
                    extra_kill_count <= chain_extra_count;
                end
            end else if (!extra_kill_valid) begin
                // 当前 s1 请求结束
                kill_write <= 1'b0;
            end

            if (action_full) begin
                a_addr_0_backup <= a_addr_0;
                a_addr_1_backup <= a_addr_1;
                a_addr_2_backup <= a_addr_2;
                a_addr_3_backup <= a_addr_3;
            end
        end
    end

    assign a_addr_0_main = (is_probing && action_free) ? a_addr_0_backup : a_addr_0;
    assign a_addr_1_main = (is_probing && action_free) ? a_addr_1_backup : a_addr_1;
    assign a_addr_2_main = (is_probing && action_free) ? a_addr_2_backup : a_addr_2;
    assign a_addr_3_main = (is_probing && action_free) ? a_addr_3_backup : a_addr_3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid <= 1'b0;
            s1_valid <= 1'b0;

        end else if (flush_done) begin
            s0_valid <= 1'b0;
            s1_valid <= 1'b0;

        end else begin
            if (!pipe_stall) begin
                s1_valid    <= s0_valid;
                s1_x        <= s0_x;
                s1_y        <= s0_y;
                s1_bucket_0 <= s0_bucket_0;
                s1_bucket_1 <= s0_bucket_1;
                s1_bucket_2 <= s0_bucket_2;
                s1_bucket_3 <= s0_bucket_3;

                s0_valid    <= req_valid;
                if (req_valid) begin
                    // 寄存
                    s0_x        <= key_x;
                    s0_y        <= key_y;
                    s0_bucket_0 <= bucket_idx_0;
                    s0_bucket_1 <= bucket_idx_1;
                    s0_bucket_2 <= bucket_idx_2;
                    s0_bucket_3 <= bucket_idx_3;
                    // 请求读取
                    a_addr_0    <= bucket_idx_0;  // issue read to all 4 BRAMs
                    a_addr_1    <= bucket_idx_1;
                    a_addr_2    <= bucket_idx_2;
                    a_addr_3    <= bucket_idx_3;
                end

            end else if (full_stall && !action_true_full && !kill_stall && !hash_stall) begin
                // 探测模式：冻结时将地址进行累加探测
                // 注意：此时 state machine 内部也在对 probe_offset 累加
                a_addr_0 <= s1_bucket_0 + probe_offset + 1'b1;
                a_addr_1 <= s1_bucket_1 + probe_offset + 1'b1;
                a_addr_2 <= s1_bucket_2 + probe_offset + 1'b1;
                a_addr_3 <= s1_bucket_3 + probe_offset + 1'b1;
            end
        end
    end


    always @(*) begin
        hit     = 1'b0;
        hit_way = 2'b0;

        for (i = 0; i < 4; i = i + 1) begin
            if (!hit && st[i] == ST_OCCU && kx[i] == s1_x && ky[i] == s1_y) begin
                hit     = 1'b1;
                hit_way = i[1:0];
            end
        end

        free_found_rr = 1'b0;
        free_way_rr   = 2'd0;

        case (rr_ptr)
            2'd0: begin
                if (free_mask[0]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd0;
                end else if (free_mask[1]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd1;
                end else if (free_mask[2]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd2;
                end else if (free_mask[3]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd3;
                end
            end
            2'd1: begin
                if (free_mask[1]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd1;
                end else if (free_mask[2]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd2;
                end else if (free_mask[3]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd3;
                end else if (free_mask[0]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd0;
                end
            end
            2'd2: begin
                if (free_mask[2]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd2;
                end else if (free_mask[3]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd3;
                end else if (free_mask[0]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd0;
                end else if (free_mask[1]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd1;
                end
            end
            2'd3: begin
                if (free_mask[3]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd3;
                end else if (free_mask[0]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd0;
                end else if (free_mask[1]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd1;
                end else if (free_mask[2]) begin
                    free_found_rr = 1'b1;
                    free_way_rr   = 2'd2;
                end
            end
        endcase

        free_found = free_found_rr;
        free_way   = free_way_rr;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bp1_valid <= 1'b0;
            bp2_valid <= 1'b0;

        end else if (flush_done) begin
            bp1_valid <= 1'b0;
            bp2_valid <= 1'b0;

        end else if (!kill_stall && !hash_stall) begin
            // Stage 1: 捕获本周期的主路径有效写入
            if (did_write) begin
                bp1_valid <= 1'b1;
                bp1_addr  <= next_bp_addr;
                bp1_data  <= next_bp_data;
            end else begin
                bp1_valid <= 1'b0;  // 随时间推移，数据下推到Stage 2
            end

            // Stage 2: 捕获上一周期的主路径写入
            bp2_valid <= bp1_valid;
            bp2_addr  <= bp1_addr;
            bp2_data  <= bp1_data;

            // NOTE： 探究当连续两个点之间，会不会出现针对这个点的kill
            if (action_kill) begin
                if (bp1_valid && (bp1_addr == kill_addr)) bp1_valid <= 1'b0;
                if (bp2_valid && (bp2_addr == kill_addr)) bp2_valid <= 1'b0;
            end
        end
    end


`ifndef SYNTHESIS

    reg [31:0] hash_stall_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_stall_counter <= 0;
        end else if (hash_stall) begin
            hash_stall_counter <= hash_stall_counter + 1;
        end
    end

    wire kill_test = kill_addr == 8'd158;
    wire [31:0] current_total_load;
    hash_load_monitor #(
        .BUCKETS   (BUCKETS),
        .BUCKET_AW (BUCKET_AW),
        .MAX_SLOTS (4),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_hash_load_monitor (
        .clk  (clk),
        .rst_n(rst_n),

        // 【关键修复】：只捕捉真正分配新槽位的动作，避免将 hit 更新算作负载
        .alloc_valid (action_free),
        .alloc_bucket(shared_write_addr_bucket),

        // 捕捉由于 expired 产生的清理动作
        .kill_valid(kill_write),
        .kill_addr (kill_write_addr), // 这里应接入传给 manager 的真实写入墓碑地址

        .debug_addr        (),
        .debug_load        (),
        .total_full_buckets(),
        .total_load        (current_total_load)
    );

    reg [           15:0] delay_1cycle_temp_0;
    reg [           15:0] delay_1cycle_temp_1;
    reg [COORD_WIDTH-1:0] delay_1cycle_temp_2;
    reg [COORD_WIDTH-1:0] delay_1cycle_temp_3;
    reg [           31:0] cycle_count = 0;
    always @(posedge clk) begin
        // 3. 设置触发条件：比如每当有一个有效的写操作时记录
        if (did_write) begin
            // %0t: 时间, %d: 十进制, %h: 十六进制
            // 这里的信号名要换成你模块里实际的信号
            // $fwrite(fp, "%d, %d, %d, %d\n", global_timer, debug_s1_x, debug_s1_y, out_idx);
            // $fwrite(fp, "%d, %d, %d, voxel_point: %d\n", debug_s1_x, debug_s1_y, out_idx, u_voxelpoint_bram.mem);
            $fwrite(fp, "%d, %d, %d\n", s1_x, s1_y, kill_write_addr);

        end

        if (kill_valid) begin
            $fwrite(fp2, "KILL, global_timer: %d, addr: %d, ts_reg: %d, r_pn: %d, x: %d, y: %d\n", global_timer, kill_addr,
                    delay_1cycle_temp_0, delay_1cycle_temp_1, delay_1cycle_temp_2, delay_1cycle_temp_3);
            // cycle_count <= cycle_count + 1;
        end
    end

    always @(posedge clk) begin
        if (u_hash_expire_manager.expired) begin
            delay_1cycle_temp_0 <= u_hash_expire_manager.ts_reg;
            delay_1cycle_temp_1 <= u_hash_expire_manager.r_pn;
            delay_1cycle_temp_2 <= u_hash_expire_manager.r_key_x;
            delay_1cycle_temp_3 <= u_hash_expire_manager.r_key_y;
        end
    end



    // ===================================================================
    // 2. 负载随时间变化的监控与打印逻辑
    // ===================================================================
    // ===================================================================
    // 2. 负载随时间变化的监控与打印逻辑
    // ===================================================================
    integer fp_load;
    reg [TIMER_WIDTH-1:0] global_timer_d1;
    reg [31:0] collision_count;

    // ----------------------------------------------------
    // 【新增】：组合逻辑动态计算 cam_load 
    // ----------------------------------------------------
    reg [4:0] cam_load;  // CAM_DEPTH=16，最大值16需要5位
    integer c_i;
    always @(*) begin
        cam_load = 0;
        for (c_i = 0; c_i < CAM_DEPTH; c_i = c_i + 1) begin
            if (cam_valid[c_i]) begin
                cam_load = cam_load + 1'b1;
            end
        end
    end
    // ----------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            collision_count <= 32'd0;
        end else if (flush_done) begin
            collision_count <= 32'd0;
        end else if (action_free && is_probing) begin
            collision_count <= collision_count + 32'd1;
        end
    end

    initial begin
        // 新建一个独立的 csv 文件用于专门绘制负载曲线
        fp_load = $fopen("hash_load_over_time_mulhash.csv", "w");
        $fwrite(fp_load, "Global_Timer,Total_Load,cam_load,collision_count\n");
    end

    always @(posedge clk) begin
        if (rst_n) begin
            global_timer_d1 <= global_timer;

            // 核心修改 2：检测 global_timer 发生跳变的时刻
            if (global_timer != global_timer_d1) begin
                // 在控制台终端打印 (可选，如果不希望刷屏可以注释掉)
                // $display("[LOAD MONITOR] Timer: %0d | Hash Table Load: %0d / %0d", global_timer, current_total_load,
                //          TABLE_SIZE);

                // 核心修改 3：写入专用的 CSV 文件
                $fwrite(fp_load, "%0d, %0d, %0d, %0d\n", global_timer, current_total_load, cam_load, collision_count);
            end
        end
    end
`endif
endmodule
