`timescale 1ns / 1ps

module tb_top ();

    // ==========================================
    // 时钟与复位
    // ==========================================
    reg aclk;
    reg aresetn;

    initial begin
        aclk = 0;
        forever #2.5 aclk = ~aclk;  // 100MHz
    end

    initial begin
        aresetn = 0;
        #100;
        aresetn = 1;
    end

    // ==========================================
    // 互联连线定义
    // ==========================================
    // AXI-Lite
    reg  [ 4:0] s00_axi_awaddr = 0;
    reg         s00_axi_awvalid = 0;
    wire        s00_axi_awready;
    reg  [31:0] s00_axi_wdata = 0;
    reg  [ 3:0] s00_axi_wstrb = 0;
    reg         s00_axi_wvalid = 0;
    wire        s00_axi_wready;
    wire [ 1:0] s00_axi_bresp;
    wire        s00_axi_bvalid;
    reg         s00_axi_bready = 0;

    reg  [ 4:0] s00_axi_araddr = 0;
    reg         s00_axi_arvalid = 0;
    wire        s00_axi_arready;
    wire [31:0] s00_axi_rdata;
    wire [ 1:0] s00_axi_rresp;
    wire        s00_axi_rvalid;
    reg         s00_axi_rready = 0;

    // 清零控制信号
    wire        clear_start_out;
    wire        clear_done_in;
    wire        clear_busy_in;
    wire        clear_error_in;
    wire [31:0] write_base_addr_out;

    // MM2S (Read)
    wire m_axis_cmd_tvalid, m_axis_cmd_tready;
    wire [ 71:0] m_axis_cmd_tdata;
    wire [127:0] s_axis_tdata;
    wire s_axis_tvalid, s_axis_tready, s_axis_tlast;
    wire [15:0] s_axis_tkeep;

    // S2MM (Write) Normal Paths (从 DUT 出来)
    wire [127:0] m_0_axis_tdata, m_1_axis_tdata;
    wire m_0_axis_tvalid, m_1_axis_tvalid;
    wire [15:0] m_0_axis_tkeep, m_1_axis_tkeep;
    wire m_0_axis_tlast, m_1_axis_tlast;
    wire m_0_axis_tready, m_1_axis_tready;
    wire [10:0] m_0_axis_voxel_x, m_1_axis_voxel_x;
    wire [10:0] m_0_axis_voxel_y, m_1_axis_voxel_y;
    wire m_0_axis_voxel_valid, m_1_axis_voxel_valid;
    wire m_0_axis_voxel_ready = 1;
    wire m_1_axis_voxel_ready = 1;


    // S2MM Clear Adapter 最终输出给 DataMover 的接口
    wire dm_s2mm_cmd_tvalid, dm_s2mm_cmd_tready;
    wire [ 71:0] dm_s2mm_cmd_tdata;
    wire [127:0] dm_s2mm_tdata;
    wire dm_s2mm_tvalid, dm_s2mm_tready, dm_s2mm_tlast;
    wire [15:0] dm_s2mm_tkeep;
    wire [ 7:0] dm_s2mm_sts_tdata;
    wire dm_s2mm_sts_tvalid, dm_s2mm_sts_tready;

    // ==========================================
    // 实例化：DUT (顶层 Wrapper)
    // ==========================================
    voxelize_wrapper_kitti_v1_0 u_dut_wrapper (
        .s00_axi_aclk   (aclk),
        .s00_axi_aresetn(aresetn),
        .s00_axi_awaddr (s00_axi_awaddr),
        .s00_axi_awprot (3'b000),
        .s00_axi_awvalid(s00_axi_awvalid),
        .s00_axi_awready(s00_axi_awready),
        .s00_axi_wdata  (s00_axi_wdata),
        .s00_axi_wstrb  (s00_axi_wstrb),
        .s00_axi_wvalid (s00_axi_wvalid),
        .s00_axi_wready (s00_axi_wready),
        .s00_axi_bresp  (s00_axi_bresp),
        .s00_axi_bvalid (s00_axi_bvalid),
        .s00_axi_bready (s00_axi_bready),
        .s00_axi_araddr (s00_axi_araddr),
        .s00_axi_arprot (3'b000),
        .s00_axi_arvalid(s00_axi_arvalid),
        .s00_axi_arready(s00_axi_arready),
        .s00_axi_rdata  (s00_axi_rdata),
        .s00_axi_rresp  (s00_axi_rresp),
        .s00_axi_rvalid (s00_axi_rvalid),
        .s00_axi_rready (s00_axi_rready),

        // MM2S 读入接口
        .s_axis_tdata     (s_axis_tdata),
        .s_axis_tvalid    (s_axis_tvalid),
        .s_axis_tready    (s_axis_tready),
        .s_axis_tkeep     (s_axis_tkeep),
        .s_axis_tlast     (s_axis_tlast),
        .m_axis_cmd_tvalid(m_axis_cmd_tvalid),
        .m_axis_cmd_tready(m_axis_cmd_tready),
        .m_axis_cmd_tdata (m_axis_cmd_tdata),

        // S2MM 正常输出 (Lane 0 接 Adapter, Lane 1 直接抛出或存文件)
        .m_0_axis_tdata      (m_0_axis_tdata),
        .m_0_axis_tvalid     (m_0_axis_tvalid),
        .m_0_axis_tkeep      (m_0_axis_tkeep),
        .m_0_axis_tready     (m_0_axis_tready),
        .m_0_axis_tlast      (m_0_axis_tlast),
        // ... (省略侧通道如 voxel_x, voxel_y 为简略，可按需接出) ...
        .m_1_axis_tdata      (m_1_axis_tdata),
        .m_1_axis_tvalid     (m_1_axis_tvalid),
        .m_1_axis_tkeep      (m_1_axis_tkeep),        // 新增 TKEEP 信号
        .m_1_axis_tready     (m_1_axis_tready),       // 内部暂不支持背压，先预留
        .m_1_axis_tlast      (m_1_axis_tlast),        // 整帧处理彻底结束（由 fl   .志）
        .m_0_axis_voxel_x    (m_0_axis_voxel_x),
        .m_0_axis_voxel_y    (m_0_axis_voxel_y),
        .m_0_axis_voxel_valid(m_0_axis_voxel_valid),  // 侧边通道输出（Voxel 坐标与有效标志）
        .m_1_axis_voxel_x    (m_1_axis_voxel_x),
        .m_1_axis_voxel_y    (m_1_axis_voxel_y),
        .m_1_axis_voxel_valid(m_1_axis_voxel_valid),
        .m_0_axis_voxel_ready(m_0_axis_voxel_ready),
        .m_1_axis_voxel_ready(m_1_axis_voxel_ready),

        // 清零逻辑交互
        .write_base_addr_out(write_base_addr_out),
        .clear_start_out    (clear_start_out),
        .clear_done_in      (clear_done_in),
        .clear_busy_in      (clear_busy_in),
        .clear_error_in     (clear_error_in)
    );

    // ==========================================
    // 实例化：清零适配器 (Clear Adapter)
    // ==========================================
    s2mm_clear_lane0_adapter #(
        .AXIS_DATA_WIDTH(128),
        .CLEAR_CHUNKS   (4)     // 缩短仿真时间，实际你配的是4
    ) u_clear_adapter (
        .aclk   (aclk),
        .aresetn(aresetn),

        .clear_start    (clear_start_out),
        .clear_base_addr(write_base_addr_out),
        .clear_busy     (clear_busy_in),
        .clear_done     (clear_done_in),
        .clear_error    (clear_error_in),

        // 正常的 Scatter 命令流 (这里在仿真中置为0)
        .normal_cmd_tvalid(1'b0),
        .normal_cmd_tready(),
        .normal_cmd_tdata (72'd0),

        // 正常的 数据流 (连到 DUT 的 m_0_axis)
        .normal_axis_tdata (m_0_axis_tdata),
        .normal_axis_tvalid(m_0_axis_tvalid),
        .normal_axis_tready(m_0_axis_),                 // 忽略，前面 m_0_axis_tready 拉高了
        .normal_axis_tkeep (m_0_axis_tkeep),
        .normal_axis_tlast (m_0_axis_tlast),

        // 最终去往 S2MM DataMover 的接口
        .dm_s2mm_cmd_tvalid(dm_s2mm_cmd_tvalid),
        .dm_s2mm_cmd_tready(dm_s2mm_cmd_tready),
        .dm_s2mm_cmd_tdata (dm_s2mm_cmd_tdata),
        .dm_s2mm_tdata     (dm_s2mm_tdata),
        .dm_s2mm_tvalid    (dm_s2mm_tvalid),
        .dm_s2mm_tready    (dm_s2mm_tready),
        .dm_s2mm_tkeep     (dm_s2mm_tkeep),
        .dm_s2mm_tlast     (dm_s2mm_tlast),
        .dm_s2mm_sts_tdata (dm_s2mm_sts_tdata),
        .dm_s2mm_sts_tvalid(dm_s2mm_sts_tvalid),
        .dm_s2mm_sts_tready(dm_s2mm_sts_tready)
    );

    // ==========================================
    // 外围仿真模块
    // ==========================================
    // 1. MM2S 读模拟器
    sim_mm2s_datamover #(
        .DATA_WIDTH   (128),
        .FILE_PATH    ("E:\\mmdetection3d\\data\\kitti\\scripts\\data\\points_sim_bin.txt"),
        .MAX_MEM_DEPTH(126000)
    ) u_mm2s_sim (
        .aclk             (aclk),
        .aresetn          (aresetn),
        .s_axis_cmd_tvalid(m_axis_cmd_tvalid),
        .s_axis_cmd_tready(m_axis_cmd_tready),
        .s_axis_cmd_tdata (m_axis_cmd_tdata),
        .m_axis_tdata     (s_axis_tdata),
        .m_axis_tvalid    (s_axis_tvalid),
        .m_axis_tready    (s_axis_tready),
        .m_axis_tkeep     (s_axis_tkeep),
        .m_axis_tlast     (s_axis_tlast)
    );

    // 2. S2MM 写模拟器 (接收清零数据与最终Voxel数据)
    sim_s2mm_datamover u_s2mm_sim (
        .aclk             (aclk),
        .aresetn          (aresetn),
        .s_axis_cmd_tvalid(dm_s2mm_cmd_tvalid),
        .s_axis_cmd_tready(dm_s2mm_cmd_tready),
        .s_axis_cmd_tdata (dm_s2mm_cmd_tdata),
        .s_axis_tdata     (dm_s2mm_tdata),
        .s_axis_tvalid    (dm_s2mm_tvalid),
        .s_axis_tready    (dm_s2mm_tready),
        .s_axis_tkeep     (dm_s2mm_tkeep),
        .s_axis_tlast     (dm_s2mm_tlast),
        .m_axis_sts_tdata (dm_s2mm_sts_tdata),
        .m_axis_sts_tvalid(dm_s2mm_sts_tvalid),
        .m_axis_sts_tready(dm_s2mm_sts_tready)
    );

    sim_axis_receiver #(
        .DATA_WIDTH(128),
        .PORT_NAME ("PORT_0"),
        .DUMP_FILE ("port0_dump.txt")
    ) u_rx_0 (
        .aclk              (aclk),
        .aresetn           (aresetn),
        .s_axis_tdata      (m_0_axis_tdata),
        .s_axis_tvalid     (m_0_axis_tvalid),
        .s_axis_tready     (m_0_axis_tready),
        .s_axis_tlast      (m_0_axis_tlast),
        .s_axis_voxel_x    (m_0_axis_voxel_x),
        .s_axis_voxel_y    (m_0_axis_voxel_y),
        .s_axis_voxel_valid(m_0_axis_voxel_valid)
    );

    sim_axis_receiver #(
        .DATA_WIDTH(128),
        .PORT_NAME ("PORT_1"),
        .DUMP_FILE ("port1_dump.txt")
    ) u_rx_1 (
        .aclk              (aclk),
        .aresetn           (aresetn),
        .s_axis_tdata      (m_1_axis_tdata),
        .s_axis_tvalid     (m_1_axis_tvalid),
        .s_axis_tready     (m_1_axis_tready),
        .s_axis_tlast      (m_1_axis_tlast),
        .s_axis_voxel_x    (m_1_axis_voxel_x),
        .s_axis_voxel_y    (m_1_axis_voxel_y),
        .s_axis_voxel_valid(m_1_axis_voxel_valid)
    );



    // ==========================================
    // 驱动测试：AXI-Lite 读写任务
    // ==========================================
    task axi_write(input [4:0] addr, input [31:0] data);
        begin
            @(posedge aclk);
            s00_axi_awaddr  <= addr;
            s00_axi_awvalid <= 1;
            s00_axi_wdata   <= data;
            s00_axi_wstrb   <= 4'hf;
            s00_axi_wvalid  <= 1;
            s00_axi_bready  <= 1;

            wait (s00_axi_bvalid);
            @(posedge aclk);
            s00_axi_awvalid <= 0;
            s00_axi_wvalid  <= 0;
            s00_axi_bready  <= 0;
        end
    endtask

    // ==========================================
    // 主测试流程
    // ==========================================
    initial begin
        // 等待复位释放
        @(posedge aresetn);
        #1000;

        $display("[%0t] [CPU] Configuring Core...", $time);

        // slv_reg1: 读基地址
        axi_write(5'h04, 32'h1000_0000);
        // slv_reg2: 帧长字节数 (116000 * 16)
        axi_write(5'h08, 32'd1856000);
        // slv_reg3: 写基地址 (供 Clear Adapter 使用)
        axi_write(5'h0C, 32'h3000_0000);

        $display("[%0t] [CPU] Writing START to reg0[0]...", $time);
        // slv_reg0 bit0 = start
        axi_write(5'h00, 32'h0000_0001);

        // 此时，Wrapper 会拉高 clear_start_out，
        // Clear Adapter 会占据 S2MM 总线，分 4 个 Chunk 下发写 0 的命令和数据。
        // S2MM 模拟器接收完毕后会返回 OKAY。

        $display("[%0t] [CPU] Waiting for Clear Logic to finish...", $time);
        @(posedge clear_done_in);
        $display("[%0t] [CPU] Clear Logic Done! Core will now start automatically.", $time);

        // 清零结束后，Wrapper 内部才会产生 core_start_pulse，Voxelize 才开始跑

        // 等待 Voxelize 计算完毕（Wrapper 内 done_latched 被置位）
        // 在仿真中可以直接等待你的核心 done 信号，或者按真实的读寄存器轮询
        wait (u_dut_wrapper.voxelize_wrapper_v1_0_S00_AXI_inst.done_latched == 1'b1);

        $display("[%0t] [CPU] ==========================================", $time);
        $display("[%0t] [CPU] Voxelize Processing Completed Successfully!", $time);
        $display("[%0t] [CPU] ==========================================", $time);

        #5000;
        $finish;
    end

endmodule
