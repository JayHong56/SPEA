module hash_expire_manager #(
    parameter         COORD_WIDTH         = 11,
    parameter         VN_WIDTH            = 5,
    parameter         TIMER_WIDTH         = 16,
    parameter         ADDR_WIDTH          = 8,                                                       // 8 -> 256 slots
    parameter         LIFE_CYCLE          = 16'd100,
    parameter         TABLE_SIZE          = 256,
    parameter         BUCKET_AW           = 6,
    parameter         BUCKET_SLOT_WIDTH   = 2,                                                       // 4 slots per bucket
    parameter         ENTRY_WIDTH         = 2 + COORD_WIDTH + COORD_WIDTH + VN_WIDTH + TIMER_WIDTH,
    parameter integer BRAM_DATA_WIDTH     = 640,                                                     // 10 * 64
    parameter integer BRAM_ADDR_WIDTH     = 9,                                                       // 256pillar * 2brams
    parameter integer BRAM_ADDR_WIDTH_PFE = 6
) (
    input wire clk,
    input wire rst_n,
    input wire write_commit,  // 1-cycle pulse when main path commits a write/update
    input wire [ADDR_WIDTH-1:0] write_addr,  // the slot address that was written
    input wire [TIMER_WIDTH-1:0] time_now,  // current global_timer value (consistent usage)
    input wire frame_end,  // 外部传入：当前帧点云结束脉冲 (1-cycle pulse)
    output wire flush_done,  // 输出：哈希表剩余有效点已全部输出完毕
    output reg [BUCKET_AW-1:0] a_addr_shadow,
    output reg [BUCKET_SLOT_WIDTH-1:0] a_we_shadow,
    input wire [ENTRY_WIDTH-1:0] a_rdata_shadow,
    output reg [ENTRY_WIDTH-1:0] b_wdata,
    output reg [BUCKET_AW-1:0] b_addr,
    output reg [BUCKET_SLOT_WIDTH-1:0] b_we,
    output reg kill_valid,
    output wire kill_expired,
    output wire bram_expire_clk_a,
    // output wire                         bram_expire_rst_a,
    // output wire                         bram_expire_wr_a,
    // output reg  [BRAM_DATA_WIDTH/8-1:0] bram_expire_bwen_a,
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    output wire bram_expire_clk_b,
    output wire bram_expire_rst_b,
    output reg bram_expire_wr_b,
    output reg [BRAM_DATA_WIDTH/8-1:0] bram_expire_bwen_b,
    output reg [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output reg [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    // ---- AXI-stream out (expired events) ----
    output wire m_axis_expire_tvalid,
    input wire m_axis_expire_tready,
    output wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata,
    // fifo almost full -----backpressure
    output wire hash_stall
);
    assign bram_expire_clk_a = clk;
    // assign bram_expire_rst_a = ~rst_n;
    assign bram_expire_clk_b = clk;
    assign bram_expire_rst_b = ~rst_n;
    localparam ST_EMPTY = 2'b00;
    localparam ST_OCCU = 2'b01;
    localparam ST_TOMB = 2'b10;

    localparam integer EW = 2 * COORD_WIDTH + BRAM_ADDR_WIDTH_PFE + VN_WIDTH;
    localparam integer DEPTH = 32;
    wire                       fifo_in_ready;
    wire                       fifo_in_valid;
    wire [             EW-1:0] fifo_in_data;
    wire                       fifo_out_valid;
    wire [             EW-1:0] fifo_out_data;
    wire [$clog2(DEPTH+1)-1:0] fifo_out_level;
    hash_expire_fifo_sync #(  // fifo缓存expired事件等待PFN处理
        .WIDTH(EW),
        .DEPTH(DEPTH)
    ) u_hash_expire_fifo_sync (
        .clk  (clk),
        .rst_n(rst_n),

        .in_valid(fifo_in_valid),
        .in_ready(fifo_in_ready),
        .in_data (fifo_in_data),

        .out_valid(fifo_out_valid),
        .out_ready(fifo_out_ready),
        .out_data (fifo_out_data),

        .level(fifo_out_level)
    );
    assign fifo_out_ready = m_axis_expire_tready;
    assign m_axis_expire_tvalid = fifo_out_valid;
    assign m_axis_expire_tdata = fifo_out_data;
    wire fifo_full = (fifo_out_level == DEPTH);
    // assign hash_stall = (fifo_out_level >= (DEPTH / 2));

    // -------------------------------------------------------------------------
    // Notify FIFO
    //
    // 用途：
    // BRAM_B 最后一行写完之后，再把 expired voxel 信息通知 PFE。
    // 这样可以保证 PFE 读 BRAM_B 时，数据已经写完。
    // -------------------------------------------------------------------------
    localparam integer NOTIFY_DEPTH = 4;
    localparam integer NOTIFY_AW = 2;

    reg [EW-1:0] notify_mem[0:NOTIFY_DEPTH-1];

    reg [NOTIFY_AW-1:0] notify_wr_ptr;
    reg [NOTIFY_AW-1:0] notify_rd_ptr;
    reg [$clog2(NOTIFY_DEPTH+1)-1:0] notify_count;

    wire notify_empty = (notify_count == 0);
    wire notify_full = (notify_count == NOTIFY_DEPTH);

    // 这个信号后面在 BRAM_B 写流水线里产生
    wire notify_push;
    wire [EW-1:0] notify_push_data;

    // notify FIFO -> 原来的 hash_expire_fifo_sync
    assign fifo_in_valid = !notify_empty;
    assign fifo_in_data  = notify_mem[notify_rd_ptr];

    wire notify_pop = fifo_in_valid && fifo_in_ready;

    // 为了防止 notify FIFO 被写满，copy FSM 启动新 task 时要看这个信号
    // 这里保守地要求至少预留 2 个空位
    wire notify_has_room = (notify_count <= NOTIFY_DEPTH - 2);

    always @(posedge clk) begin
        if (notify_push && !notify_full) begin
            notify_mem[notify_wr_ptr] <= notify_push_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            notify_wr_ptr <= {NOTIFY_AW{1'b0}};
            notify_rd_ptr <= {NOTIFY_AW{1'b0}};
            notify_count  <= {($clog2(NOTIFY_DEPTH + 1)) {1'b0}};
        end else begin
            if (notify_push && !notify_full) begin
                notify_wr_ptr <= notify_wr_ptr + 1'b1;
            end

            if (notify_pop) begin
                notify_rd_ptr <= notify_rd_ptr + 1'b1;
            end

            case ({
                notify_push && !notify_full, notify_pop
            })
                2'b10:   notify_count <= notify_count + 1'b1;
                2'b01:   notify_count <= notify_count - 1'b1;
                2'b11:   notify_count <= notify_count;
                default: notify_count <= notify_count;
            endcase
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n) begin
            if (notify_push && notify_full) begin
                $display("[ERROR][hash_expire_manager] notify FIFO overflow at time %0t", $time);
                $stop;
            end
        end
    end
`endif

    // === 新增：FIFO 水位迟滞反压控制 (Hysteresis) ===
    reg hash_stall_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_stall_reg <= 1'b0;
        end else begin
            if (fifo_out_level >= (DEPTH / 2)) begin
                // 高水位：到达一半深度，拉高 stall，通知上游停止喂 write_commit
                hash_stall_reg <= 1'b1;
            end else if (fifo_out_level == 0) begin
                // 低水位：必须等待下游完全抽干 FIFO，才降下 stall
                hash_stall_reg <= 1'b0;
            end
        end
    end
    assign hash_stall = hash_stall_reg;

    localparam integer DQ_DEPTH = LIFE_CYCLE;
    (* ram_style = "distributed" *) reg [ADDR_WIDTH-1:0] dq_mem[0:DQ_DEPTH-1];
    reg [$clog2(DQ_DEPTH)-1:0] dq_wr_ptr, dq_rd_ptr;
    reg [$clog2(DQ_DEPTH+1)-1:0] dq_count;
    reg                          cand_valid;
    reg [        ADDR_WIDTH-1:0] cand_addr;

    // ==============================================================
    // 核心修改 1：纯净的 RAM 写入块 (绝对不能有 negedge rst_n)
    // ==============================================================
    always @(posedge clk) begin
        if (write_commit) begin
            dq_mem[dq_wr_ptr] <= write_addr;
        end
    end

    // ==============================================================
    // 核心修改 2：异步读取端口 (完美契合 LUTRAM 的天生特性)
    // ==============================================================
    wire [ADDR_WIDTH-1:0] dq_rdata = dq_mem[dq_rd_ptr];

    integer k;
    // ==============================================================
    // 指针与状态控制逻辑 (保留异步复位)
    // ==============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dq_wr_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_rd_ptr  <= {($clog2(DQ_DEPTH)) {1'b0}};
            dq_count   <= {($clog2(DQ_DEPTH + 1)) {1'b0}};
            cand_valid <= 1'b0;
            cand_addr  <= {ADDR_WIDTH{1'b0}};
        end else if (frame_end) begin
            // 收到清空指令，直接复位指针
            dq_wr_ptr  <= 0;
            dq_rd_ptr  <= 0;
            dq_count   <= 0;
            cand_valid <= 1'b0;
            // 注意：cand_addr 也可以选择复位，这里保持你原逻辑
        end else if (write_commit) begin

            dq_wr_ptr <= (dq_wr_ptr == DQ_DEPTH - 1) ? {($clog2(DQ_DEPTH)) {1'b0}} : (dq_wr_ptr + 1'b1);

            if (dq_count == DQ_DEPTH - 1) begin
                cand_addr  <= dq_rdata;  // <--- 核心修改 3：使用剥离出来的异步读信号
                cand_valid <= 1'b1;
                dq_rd_ptr  <= (dq_rd_ptr == DQ_DEPTH - 1) ? {($clog2(DQ_DEPTH)) {1'b0}} : (dq_rd_ptr + 1'b1);
            end else begin
                dq_count   <= dq_count + 1'b1;
                cand_valid <= 1'b0;
            end

        end else begin
            cand_valid <= 1'b0;
            cand_addr  <= {ADDR_WIDTH{1'b0}};
        end
    end

    // reg write_commit_delay;
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         write_commit_delay <= 1'b0;
    //     end else if (write_commit) begin
    //         write_commit_delay <= 1'b1;
    //     end
    // end
    // wire write_commit_delayed = write_commit | write_commit_delay;  // 延迟一个周期，确保cand_valid/cand_addr稳定
    reg s0_valid;
    reg [ADDR_WIDTH-1:0] s0_addr;
    reg [TIMER_WIDTH-1:0] s0_time;
    reg s1_valid;
    reg s1_force_expire;  // <--- 新增
    reg [ADDR_WIDTH-1:0] s1_addr;
    reg [TIMER_WIDTH-1:0] s1_time;
    reg [BUCKET_SLOT_WIDTH-1:0] s0_we;
    reg s0_force_expire;  // <--- 新增

    wire [1:0] r_status;
    wire signed [COORD_WIDTH-1:0] r_key_x;
    wire signed [COORD_WIDTH-1:0] r_key_y;
    wire [VN_WIDTH-1:0] r_pn;
    wire [TIMER_WIDTH-1:0] r_ts;
    assign {r_status, r_key_x, r_key_y, r_pn, r_ts} = a_rdata_shadow;


    reg flushing;
    reg [ADDR_WIDTH:0] flush_cnt;  // 多一位用于判断结束 (0~256)
    // 控制流水线步进，只有在未扫完且无反压时才工作
    wire flush_run = flushing && (flush_cnt < TABLE_SIZE) && !hash_stall;

    // 动态选择：正常模式吃 DQ 吐出的 cand_addr，Flush 模式吃扫描计数器 flush_cnt
    wire eff_cand_valid = flush_run ? 1'b1 : cand_valid;
    wire [ADDR_WIDTH-1:0] eff_cand_addr = flush_run ? flush_cnt[ADDR_WIDTH-1:0] : cand_addr;
    // 流水线推进条件：正常的写提交脉冲 OR 正在执行强制扫描
    // wire pipe_advance = write_commit_delayed | flush_run;
    wire pipe_advance = flush_run | cand_valid | s0_valid | s1_valid;
    // fifo pop bram 2-stage pipeline
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_addr_shadow   <= {BUCKET_AW{1'b0}};
            s0_valid        <= 1'b0;
            s0_addr         <= {ADDR_WIDTH{1'b0}};
            s0_time         <= {TIMER_WIDTH{1'b0}};
            s1_valid        <= 1'b0;
            s1_addr         <= {ADDR_WIDTH{1'b0}};
            s1_time         <= {TIMER_WIDTH{1'b0}};
            s0_we           <= {BUCKET_SLOT_WIDTH{1'b0}};
            s0_force_expire <= 1'b0;
            s1_force_expire <= 1'b0;

        end else if (pipe_advance) begin
            s1_force_expire <= s0_force_expire;  // <--- 打拍传递强制标志

            s1_valid        <= s0_valid;
            s1_addr         <= s0_addr;
            s1_time         <= s0_time;
            a_we_shadow     <= s0_we;  // NOTE: 这里we信号要比addr晚一个周期给

            s0_valid        <= eff_cand_valid;
            s0_addr         <= eff_cand_addr;
            s0_time         <= time_now;
            s0_force_expire <= flush_run;  // <--- 如果是 Flush 模式塞进来的点，打上“必死”烙印


            if (eff_cand_valid) begin
                a_addr_shadow <= eff_cand_addr[(ADDR_WIDTH-1)-:BUCKET_AW];
                s0_we <= eff_cand_addr[BUCKET_SLOT_WIDTH-1:0];
            end
        end
    end

    localparam [23:0] MAX_VOXEL_NUM = 24'd20;
    localparam integer PT_WIDTH = 16;
    // 640-bit 一行，每个点 4 * 16 = 64-bit，所以一行 10 点
    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / (4 * PT_WIDTH);  // 640/64 = 10
    // BRAM_A / BRAM_B 中每个 voxel 逻辑上保留 2 行
    localparam integer EXPEND_VOXEL_ROW = 2;
    localparam integer RESERVED_VOXEL_ROW = 2;
    localparam [VN_WIDTH-1:0] POINTS_PER_ROW_V = POINTS_PER_ROW;
    localparam [BRAM_ADDR_WIDTH_PFE-1:0] RESERVED_VOXEL_ROW_V = RESERVED_VOXEL_ROW;


    wire [TIMER_WIDTH-1:0] ts_reg = (s1_valid) ? (s1_time - r_ts) : {TIMER_WIDTH{1'b0}};
    wire expired = (r_status == ST_OCCU) && (s1_force_expire || ts_reg >= LIFE_CYCLE);
    assign kill_expired = expired;
    // wire [BRAM_ADDR_WIDTH-1:0] bram_expire_row_addr = (s1_addr * EXPEND_VOXEL_ROW) + (r_pn / POINTS_PER_ROW);
    // assign bram_expire_addr_a = expired ? bram_expire_row_addr : {BRAM_ADDR_WIDTH{1'b0}};

    // -------------------------------------------------------------------------
    // REVISED: BRAM A to BRAM B 搬运 Pipeline & 环形覆盖缓存设计
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // Task FIFO：缓存 expired voxel 任务
    // -------------------------------------------------------------------------
    localparam integer TASK_DEPTH = 16;
    localparam integer TASK_AW = 4;

    localparam TASK_WIDTH = ADDR_WIDTH + 2 * COORD_WIDTH + VN_WIDTH;
    reg [TASK_WIDTH-1:0] task_fifo_mem[0:TASK_DEPTH-1];  // 16深度通常已足够缓冲峰值
    reg [TASK_AW-1:0] task_wr_ptr;
    reg [TASK_AW-1:0] task_rd_ptr;
    reg [$clog2(TASK_DEPTH+1)-1:0] task_count;

    wire task_empty = (task_count == 0);
    wire task_full = (task_count == TASK_DEPTH);

    wire [TASK_WIDTH-1:0] task_dout = task_fifo_mem[task_rd_ptr];
    // 解析 Task FIFO 输出
    wire [ADDR_WIDTH-1:0] task_addr = task_dout[TASK_WIDTH-1-:ADDR_WIDTH];
    wire signed [COORD_WIDTH-1:0] task_kx = task_dout[2*COORD_WIDTH+VN_WIDTH-1-:COORD_WIDTH];
    wire signed [COORD_WIDTH-1:0] task_ky = task_dout[COORD_WIDTH+VN_WIDTH-1-:COORD_WIDTH];
    wire [VN_WIDTH-1:0] task_pn = task_dout[VN_WIDTH-1-:VN_WIDTH];
    wire [1:0] task_copy_rows = (task_pn <= POINTS_PER_ROW_V) ? 2'd1 : 2'd2;

    // -------------------------------------------------------------------------
    // FSM 状态
    // -------------------------------------------------------------------------
    reg copy_st;
    localparam ST_IDLE = 1'b0;
    localparam ST_READ_R1 = 1'b1;
    // 当前发起 BRAM_A 读请求的地址
    reg  [    BRAM_ADDR_WIDTH-1:0] fsm_read_addr;
    reg                            fsm_read_en;
    // 当前 BRAM_A 读请求对应写入 BRAM_B 的地址
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] fsm_write_addr;
    // BRAM_B 下一个 voxel 的 base row pointer
    // 注意：这个是 voxel 粒度的 base pointer，不是逐行 pointer
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] task_base_ptr;
    // 锁存当前正在处理的 task 信息
    reg  [         ADDR_WIDTH-1:0] fsm_task_addr;
    reg  [BRAM_ADDR_WIDTH_PFE-1:0] fsm_task_base_ptr;

    // 当前读请求如果是最后一行，写完后要通知 PFE 的数据
    // 当前读请求是否是该 voxel 的最后一行
    reg  [                 EW-1:0] fsm_notify_data;
    reg                            fsm_read_last;
    reg  [                 EW-1:0] fsm_task_notify_data;

    // copy FSM 什么时候真正接受一个 task，后面会定义
    // 关键：只有这个握手成立，才允许读取 task_fifo 头部并 pop
    wire                           task_accept = (copy_st == ST_IDLE) && (!task_empty) && notify_has_room;

    // 关键：task_pop 必须是组合信号，不能打一拍
    wire                           task_pop = task_accept;
    wire                           task_push = expired && (!task_full || task_pop);
    // 空 FIFO 时禁止 pop
    wire                           task_pop_safe = task_pop && !task_empty;

    // 独立 RAM 写入逻辑
    always @(posedge clk) begin
        if (task_push) begin
            task_fifo_mem[task_wr_ptr] <= {s1_addr, r_key_x, r_key_y, r_pn};
        end
    end

    // 指针与计数逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            task_wr_ptr <= {TASK_AW{1'b0}};
            task_rd_ptr <= {TASK_AW{1'b0}};
            task_count  <= {($clog2(TASK_DEPTH + 1)) {1'b0}};
        end else begin
            if (task_push) begin
                task_wr_ptr <= task_wr_ptr + 1'b1;
            end

            if (task_pop_safe) begin
                task_rd_ptr <= task_rd_ptr + 1'b1;
            end

            case ({
                task_push, task_pop_safe
            })
                2'b10:   task_count <= task_count + 1'b1;
                2'b01:   task_count <= task_count - 1'b1;
                2'b11:   task_count <= task_count;
                default: task_count <= task_count;
            endcase
        end
    end
    // -------------------------------------------------------------------------
    // 读 BRAM_A
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            copy_st              <= ST_IDLE;

            fsm_read_en          <= 1'b0;
            fsm_read_addr        <= {BRAM_ADDR_WIDTH{1'b0}};
            fsm_write_addr       <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last        <= 1'b0;
            fsm_notify_data      <= {EW{1'b0}};

            fsm_task_addr        <= {ADDR_WIDTH{1'b0}};
            fsm_task_base_ptr    <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_task_notify_data <= {EW{1'b0}};

            task_base_ptr        <= {BRAM_ADDR_WIDTH_PFE{1'b0}};

        end else begin
            // 默认本周期不读
            fsm_read_en     <= 1'b0;
            fsm_read_last   <= 1'b0;
            fsm_notify_data <= {EW{1'b0}};

            case (copy_st)

                // -------------------------------------------------------------
                // IDLE:
                // 如果 task_fifo 有 task，立即读该 voxel 的第 0 行。
                //
                // 如果 pn <= 10：
                //   第 0 行就是最后一行。
                //
                // 如果 pn > 10：
                //   下一拍进入 ST_READ_R1 读第 1 行。
                // -------------------------------------------------------------
                ST_IDLE: begin
                    if (task_accept) begin
                        // 锁存当前 task
                        fsm_task_addr        <= task_addr;
                        fsm_task_base_ptr    <= task_base_ptr;
                        fsm_task_notify_data <= {task_kx, task_ky, task_base_ptr, task_pn};

                        // 发起 BRAM_A 第 0 行读取
                        fsm_read_en          <= 1'b1;
                        fsm_read_addr        <= task_addr * EXPEND_VOXEL_ROW;

                        // 写入 BRAM_B 的 base 行
                        fsm_write_addr       <= task_base_ptr;

                        // 如果只有一行，那么当前读请求就是最后一行
                        if (task_copy_rows == 2'd1) begin
                            fsm_read_last   <= 1'b1;
                            fsm_notify_data <= {task_kx, task_ky, task_base_ptr, task_pn};
                            copy_st         <= ST_IDLE;
                        end else begin
                            fsm_read_last   <= 1'b0;
                            fsm_notify_data <= {EW{1'b0}};
                            copy_st         <= ST_READ_R1;
                        end

                        // BRAM_B 每个 voxel 固定保留 2 行
                        task_base_ptr <= task_base_ptr + RESERVED_VOXEL_ROW_V;
                    end
                end

                // -------------------------------------------------------------
                // ST_READ_R1:
                // 只有 pn = 11~20 的 voxel 会进入这里。
                // 第 1 行一定是该 voxel 的最后一行。
                // -------------------------------------------------------------
                ST_READ_R1: begin
                    if (notify_has_room) begin
                        fsm_read_en <= 1'b1;

                        // 读 BRAM_A 第 1 行
                        fsm_read_addr <= (fsm_task_addr * EXPEND_VOXEL_ROW) + 1'b1;

                        // 写 BRAM_B 的 base + 1 行
                        fsm_write_addr <= fsm_task_base_ptr + {{(BRAM_ADDR_WIDTH_PFE - 1) {1'b0}}, 1'b1};

                        // 第二行是最后一行，写完后通知 PFE
                        fsm_read_last <= 1'b1;
                        fsm_notify_data <= fsm_task_notify_data;

                        // 下一周期可以马上回到 IDLE，接下一个 task
                        copy_st <= ST_IDLE;
                    end else begin
                        // notify FIFO 没空间时，不能发最后一行读请求
                        // 否则最后一行写完后通知无法保存
                        copy_st <= ST_READ_R1;
                    end
                end

                default: begin
                    copy_st <= ST_IDLE;
                end
            endcase
        end
    end



    assign bram_expire_addr_a = fsm_read_addr;
    // -------------------------------------------------------------------------
    // BRAM_B 写控制流水线
    //
    // BRAM_A 读请求发出后一拍，bram_expire_rdata_a 有效。
    // 如果这一拍写的是某 voxel 的最后一行，则写完后通知 PFE。
    // -------------------------------------------------------------------------

    reg                           fsm_read_en_d1;
    reg [BRAM_ADDR_WIDTH_PFE-1:0] fsm_write_addr_d1;
    reg                           fsm_read_last_d1;
    reg [                 EW-1:0] fsm_notify_data_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_read_en_d1       <= 1'b0;
            fsm_write_addr_d1    <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            fsm_read_last_d1     <= 1'b0;
            fsm_notify_data_d1   <= {EW{1'b0}};

            bram_expire_wr_b     <= 1'b0;
            bram_expire_addr_b   <= {BRAM_ADDR_WIDTH_PFE{1'b0}};
            bram_expire_bwen_b   <= {(BRAM_DATA_WIDTH / 8) {1'b0}};
            bram_expire_wrdata_b <= {BRAM_DATA_WIDTH{1'b0}};
        end else begin
            fsm_read_en_d1     <= fsm_read_en;
            fsm_write_addr_d1  <= fsm_write_addr;
            fsm_read_last_d1   <= fsm_read_last;
            fsm_notify_data_d1 <= fsm_notify_data;

            if (fsm_read_en_d1) begin
                bram_expire_wr_b     <= 1'b1;
                bram_expire_addr_b   <= fsm_write_addr_d1;
                bram_expire_wrdata_b <= bram_expire_rdata_a;
                bram_expire_bwen_b   <= {(BRAM_DATA_WIDTH / 8) {1'b1}};
            end else begin
                bram_expire_wr_b   <= 1'b0;
                bram_expire_bwen_b <= {(BRAM_DATA_WIDTH / 8) {1'b0}};
            end
        end
    end

    // 当前拍如果正在把某 voxel 的最后一行写入 BRAM_B，
    // 则把该 voxel 信息推入 notify_fifo。
    // 注意：由于 notify_fifo 是时序写入，真正 fifo_in_valid 会在下一拍拉高，
    // 因此 PFE 不会早于 BRAM_B 写完成。
    assign notify_push      = fsm_read_en_d1 && fsm_read_last_d1;
    assign notify_push_data = fsm_notify_data_d1;


    // -------------------------------------------------------------------------
    // Hash Tombstone (墓碑) 写入逻辑，与 BRAM 搬运完全并行，不受阻塞影响
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kill_valid <= 1'b0;
            b_we       <= {BUCKET_SLOT_WIDTH{1'b0}};
            b_addr     <= {BUCKET_AW{1'b0}};
            b_wdata    <= {ENTRY_WIDTH{1'b0}};
        end else begin
            if (s1_valid && expired) begin  // 发现 expired 即刻击杀，写墓碑
                kill_valid <= 1'b1;
                b_we       <= s1_addr[BUCKET_SLOT_WIDTH-1:0];
                b_addr     <= s1_addr[ADDR_WIDTH-1-:BUCKET_AW];
                b_wdata    <= {ST_TOMB, {COORD_WIDTH{1'b0}}, {COORD_WIDTH{1'b0}}, {VN_WIDTH{1'b0}}, {TIMER_WIDTH{1'b0}}};
            end else begin
                kill_valid <= 1'b0;
                b_we       <= {BUCKET_SLOT_WIDTH{1'b0}};
            end
        end
    end


    // =========================================================================
    // 修改：帧末清空状态机 (EOF Flush FSM) - 精确控制 flush_done
    // =========================================================================

    // 检查流水线和缓存是否全部排空：
    // 1. 扫描结束 (flush_cnt == TABLE_SIZE)
    // 2. S0 和 S1 级流水线中没有因 Flush 而强制过期的任务
    // 3. 中间缓存 fifo 已空 (task_empty)
    // 4. BRAM 搬运状态机处于空闲态 (copy_st == ST_IDLE)
    // 5. 最终过期输出 FIFO 中不仅 level 为 0，且 valid 也拉低了（确保完全被下游吃掉）
    wire all_drained = (flush_cnt == TABLE_SIZE) &&
                        (!s0_force_expire && !s1_force_expire) &&
                        task_empty &&
                        (copy_st == ST_IDLE) &&
                        (!fsm_read_en) &&
                        (!fsm_read_en_d1) &&
                        (!bram_expire_wr_b) &&
                        notify_empty &&
                        (fifo_out_level == 0 && !fifo_out_valid);
    // 只有在所有阶段都彻底排空时，才发出 flush_done（只持续 1 个 cycle）
    assign flush_done = flushing && all_drained;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flushing  <= 1'b0;
            flush_cnt <= {(ADDR_WIDTH + 1) {1'b0}};
        end else begin
            if (frame_end && !flushing) begin
                flushing  <= 1'b1;  // 收到帧末信号，启动清空
                flush_cnt <= 0;
            end else if (flushing) begin
                if (flush_cnt < TABLE_SIZE) begin
                    if (!hash_stall) begin
                        flush_cnt <= flush_cnt + 1'b1;
                    end
                end else if (all_drained) begin
                    // 扫完地址后停留在 flushing 状态，直到数据通路完全排空
                    flushing <= 1'b0;
                end
            end
        end
    end


endmodule
