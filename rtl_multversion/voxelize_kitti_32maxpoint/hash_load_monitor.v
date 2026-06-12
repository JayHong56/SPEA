module hash_load_monitor #(
    parameter BUCKETS    = 64,
    parameter BUCKET_AW  = 6,
    parameter MAX_SLOTS  = 4,
    parameter ADDR_WIDTH = 8  // 对应 BUCKET_AW + BUCKET_SLOT_WIDTH
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clear,

    // 分配新槽位事件 (顶层请接入 action_free)
    input  wire                  alloc_valid,
    input  wire [BUCKET_AW-1:0]  alloc_bucket,

    // 释放旧槽位事件 (顶层请接入 kill_write)
    input  wire                  kill_valid,
    input  wire [ADDR_WIDTH-1:0] kill_addr,

    // 实时监控输出
    output reg  [BUCKET_AW-1:0]  debug_addr,         // 最近发生变动的 Bucket ID
    output reg  [7:0]            debug_load,         // 该 Bucket 变动后的槽位数
    output reg  [31:0]           total_full_buckets, // 当前已完全满载的 Bucket 数量
    output reg  [31:0]           total_load          // 哈希表当前总占用槽位数
);

    // 记录每个 Bucket 当前已用的槽位数量
    reg [$clog2(MAX_SLOTS+1)-1:0] bucket_load [0:BUCKETS-1];

    // 从 kill_addr 提取对应的 Bucket ID
    wire [BUCKET_AW-1:0] kill_bucket = kill_addr[ADDR_WIDTH-1 -: BUCKET_AW];

    // 满桶状态判断逻辑
    wire alloc_makes_full    = alloc_valid && (bucket_load[alloc_bucket] == MAX_SLOTS - 1);
    wire kill_makes_not_full = kill_valid  && (bucket_load[kill_bucket] == MAX_SLOTS);
    wire same_bucket_conflict = (alloc_valid && kill_valid && (alloc_bucket == kill_bucket));

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            total_load         <= 0;
            total_full_buckets <= 0;
            debug_addr         <= 0;
            debug_load         <= 0;
            for (i = 0; i < BUCKETS; i = i + 1) begin
                bucket_load[i] <= 0;
            end
        end else begin
            
            // ==========================================
            // 1. 全局总容量更新 (Total Load)
            // ==========================================
            if (alloc_valid && !kill_valid) begin
                if (total_load < BUCKETS * MAX_SLOTS) begin
                    total_load <= total_load + 1'b1;
                end
            end else if (!alloc_valid && kill_valid) begin
                if (total_load != 0) begin
                    total_load <= total_load - 1'b1;
                end
            end
            // 如果 alloc 和 kill 同时发生，total_load 保持不变

            // ==========================================
            // 2. Bucket 级负载与 Full 状态更新
            // ==========================================
            if (same_bucket_conflict) begin
                // 同一个 Bucket 进出一个，负载和 Full 状态均不改变
                debug_addr <= alloc_bucket;
                debug_load <= bucket_load[alloc_bucket]; 
            end else begin
                // 处理 Alloc
                if (alloc_valid) begin
                    if (bucket_load[alloc_bucket] < MAX_SLOTS) begin
                        bucket_load[alloc_bucket] <= bucket_load[alloc_bucket] + 1'b1;
                    end
                    debug_addr <= alloc_bucket;
                    debug_load <= (bucket_load[alloc_bucket] < MAX_SLOTS) ? bucket_load[alloc_bucket] + 1'b1 : bucket_load[alloc_bucket];
                end
                
                // 处理 Kill
                if (kill_valid) begin
                    if (bucket_load[kill_bucket] != 0) begin
                        bucket_load[kill_bucket] <= bucket_load[kill_bucket] - 1'b1;
                    end
                    if (!alloc_valid) begin // 只有单纯 kill 时才把 debug 视角切给它
                        debug_addr <= kill_bucket;
                        debug_load <= (bucket_load[kill_bucket] != 0) ? bucket_load[kill_bucket] - 1'b1 : bucket_load[kill_bucket];
                    end
                end

                // 处理 Full Buckets 统计量
                if (alloc_makes_full && kill_makes_not_full) begin
                    // 一个桶满了，另一个桶不满了，总满桶数互相抵消，保持不变
                    total_full_buckets <= total_full_buckets;
                end else if (alloc_makes_full) begin
                    total_full_buckets <= total_full_buckets + 1'b1;
                end else if (kill_makes_not_full) begin
                    if (total_full_buckets != 0) begin
                        total_full_buckets <= total_full_buckets - 1'b1;
                    end
                end
            end
        end
    end

endmodule
