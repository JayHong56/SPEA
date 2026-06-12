`timescale 1 ns / 1 ps

module voxelize_wrapper_v1_0_S00_AXI #(
    // AXI4LITE 参数
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 5,
    parameter integer DRAM_DATA_WIDTH    = 128,
    parameter integer DRAM_ADDR_WIDTH    = 23,
    // Voxelize 数据位宽参数
    parameter integer AXIS_DATA_WIDTH    = 128
) (
    // -----------------------------------------------------------
    // 新增：引出给外部模块的配置端口
    // -----------------------------------------------------------
    output wire [31:0] write_base_addr_out,  // <--- dynamic output base address

    // -----------------------------------------------------------
    // PL clear control ports
    // -----------------------------------------------------------
    output wire        clear_start_out,
    input  wire        clear_done_in,
    input  wire        clear_busy_in,
    input  wire        clear_error_in,
    input  wire        output_path_idle_in,
    input  wire        output_path_error_in,

    // -----------------------------------------------------------
    // AXI4-Stream Slave (点云数据输入)
    // -----------------------------------------------------------
    input  wire [  DRAM_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire [DRAM_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tlast,

    output wire [  AXIS_DATA_WIDTH-1:0] m_0_axis_tdata,
    output wire                         m_0_axis_tvalid,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_0_axis_tkeep,        // 新增 TKEEP 信号
    input  wire                         m_0_axis_tready,       // 内部暂不支持背压，先预留
    output wire                         m_0_axis_tlast,        // 整帧处理彻底结束（由 flush_done 驱动）
    output wire [  AXIS_DATA_WIDTH-1:0] m_1_axis_tdata,
    output wire                         m_1_axis_tvalid,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_1_axis_tkeep,        // 新增 TKEEP 信号
    input  wire                         m_1_axis_tready,       // 内部暂不支持背压，先预留
    output wire                         m_1_axis_tlast,        // 整帧处理彻底结束（由 flush_done 驱动）
    // 侧边通道输出（Voxel 坐标与有效标志）
    output wire [               11-1:0] m_0_axis_voxel_x,
    output wire [               11-1:0] m_0_axis_voxel_y,
    output wire                         m_0_axis_voxel_valid,  // 侧边通道输出（Voxel 坐标与有效标志）
    output wire [               11-1:0] m_1_axis_voxel_x,
    output wire [               11-1:0] m_1_axis_voxel_y,
    output wire                         m_1_axis_voxel_valid,
    input  wire                         m_0_axis_voxel_ready,
    input  wire                         m_1_axis_voxel_ready,

    // -----------------------------------------------------------
    // AXI-Stream Command Interface (连接到 DataMover)
    // -----------------------------------------------------------
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,
    output wire [71:0] m_axis_cmd_tdata,

    // -----------------------------------------------------------
    // AXI4-Lite Slave 接口
    // -----------------------------------------------------------
    input  wire                                S_AXI_ACLK,
    input  wire                                S_AXI_ARESETN,
    input  wire [    C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input  wire [                       2 : 0] S_AXI_AWPROT,
    input  wire                                S_AXI_AWVALID,
    output wire                                S_AXI_AWREADY,
    input  wire [    C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input  wire                                S_AXI_WVALID,
    output wire                                S_AXI_WREADY,
    output wire [                       1 : 0] S_AXI_BRESP,
    output wire                                S_AXI_BVALID,
    input  wire                                S_AXI_BREADY,
    input  wire [    C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input  wire [                       2 : 0] S_AXI_ARPROT,
    input  wire                                S_AXI_ARVALID,
    output wire                                S_AXI_ARREADY,
    output wire [    C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [                       1 : 0] S_AXI_RRESP,
    output wire                                S_AXI_RVALID,
    input  wire                                S_AXI_RREADY
);




    // AXI4LITE 信号
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg                            axi_awready;
    reg                            axi_wready;
    reg [                   1 : 0] axi_bresp;
    reg                            axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg                            axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
    reg [                   1 : 0] axi_rresp;
    reg                            axi_rvalid;

    localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH / 32) + 1;
    localparam integer OPT_MEM_ADDR_BITS = 2;

    reg     [C_S_AXI_DATA_WIDTH-1:0] slv_reg0;
    reg     [C_S_AXI_DATA_WIDTH-1:0] slv_reg1;
    reg     [C_S_AXI_DATA_WIDTH-1:0] slv_reg2;
    reg     [C_S_AXI_DATA_WIDTH-1:0] slv_reg3;
    reg     [C_S_AXI_DATA_WIDTH-1:0] slv_reg4;
    wire                             slv_reg_rden;
    wire                             slv_reg_wren;
    reg     [C_S_AXI_DATA_WIDTH-1:0] reg_data_out;
    integer                          byte_index;
    reg                              aw_en;

    // 分配连线
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready <= 1'b0;
            aw_en <= 1'b1;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                axi_awready <= 1'b1;
                aw_en <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_en <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awaddr <= 0;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                axi_awaddr <= S_AXI_AWADDR;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_wready <= 1'b0;
        end else begin
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en) begin
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    // === 生成 Start 脉冲 ===
    // 监听：当向 slv_reg0 的 bit0 写入 1 时，产生一个时钟周期的脉冲
    wire start_pulse = (slv_reg_wren && (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 3'h0) && S_AXI_WSTRB[0] && S_AXI_WDATA[0]);
    wire clear_done_by_sw = (slv_reg_wren && (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 3'h0) && S_AXI_WSTRB[0] && S_AXI_WDATA[2]);
    wire voxelize_done;
    reg done_latched;
    reg busy_latched;
    reg error_latched;
    reg core_done_seen;

    // Accept a new frame only when the previous frame is not busy, the clear
    // writer is idle, and the previous normal S2MM output write path has fully
    // drained. This prevents the next frame's clear writer from stealing the
    // DataMover while old normal writes/statuses are still pending.
    wire start_accept = start_pulse && !busy_latched && !clear_busy_in && output_path_idle_in;

    // PS start first launches PL clear. The voxelize core is started only
    // after clear_done_in is returned by s2mm_clear_writer.
    assign clear_start_out = start_accept;

    reg core_start_pulse;
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            core_start_pulse <= 1'b0;
        end else begin
            core_start_pulse <= clear_done_in && !clear_error_in;
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            slv_reg0 <= 0;
            slv_reg1 <= 0;
            slv_reg2 <= 0;
            slv_reg3 <= 0;
            slv_reg4 <= 0;
        end else begin
            // slv_reg0 的 bit0 自动清零，不保持。其他位按需写入。
            if (slv_reg_wren) begin
                case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                    3'h0: begin
                        for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
                        if (S_AXI_WSTRB[byte_index] == 1) slv_reg0[(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
                    end
                    3'h1:
                    for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
                    if (S_AXI_WSTRB[byte_index] == 1) slv_reg1[(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
                    3'h2:
                    for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
                    if (S_AXI_WSTRB[byte_index] == 1) slv_reg2[(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
                    3'h3:
                    for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
                    if (S_AXI_WSTRB[byte_index] == 1) slv_reg3[(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
                    3'h4:
                    for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
                    if (S_AXI_WSTRB[byte_index] == 1) slv_reg4[(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
                    default: ;
                endcase
            end
            // 自动清除 slv_reg0 的 bit0 (触发位)
            slv_reg0[0] <= 1'b0;
        end
    end

    // ------------------------------------------------------------
    // Sticky done/busy status for PS polling
    // m_axis_pfn_flush_done is converted to voxelize_done by core.
    // done_latched stays high until PS writes 1 to reg0[2], so PS will not miss
    // the 1-cycle done pulse.
    // ------------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            done_latched   <= 1'b0;
            busy_latched   <= 1'b0;
            error_latched  <= 1'b0;
            core_done_seen <= 1'b0;
        end else begin
            if (start_accept) begin
                done_latched   <= 1'b0;
                busy_latched   <= 1'b1;
                error_latched  <= 1'b0;
                core_done_seen <= 1'b0;
            end else begin
                if (clear_done_by_sw) begin
                    done_latched  <= 1'b0;
                    error_latched <= 1'b0;
                end

                if (voxelize_done) begin
                    core_done_seen <= 1'b1;
                end

                if (clear_done_in && clear_error_in) begin
                    done_latched   <= 1'b1;
                    busy_latched   <= 1'b0;
                    error_latched  <= 1'b1;
                    core_done_seen <= 1'b0;
                end else if (output_path_error_in && busy_latched) begin
                    done_latched   <= 1'b1;
                    busy_latched   <= 1'b0;
                    error_latched  <= 1'b1;
                    core_done_seen <= 1'b0;
                end else if ((voxelize_done || core_done_seen) && output_path_idle_in) begin
                    done_latched   <= 1'b1;
                    busy_latched   <= 1'b0;
                    core_done_seen <= 1'b0;
                end
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_bvalid <= 0;
            axi_bresp  <= 2'b0;
        end else begin
            if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0;
            end else begin
                if (S_AXI_BREADY && axi_bvalid) begin
                    axi_bvalid <= 1'b0;
                end
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 32'b0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rvalid <= 0;
            axi_rresp  <= 0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b0;
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    wire voxelize_idle_rd_ddr;
    assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;


    assign write_base_addr_out = slv_reg3;
    always @(*) begin
        case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
            // 读取 slv_reg0 时，将内部的 idle 状态映射到 Bit 1
            // bit0=start(read as 0)
            // bit1=voxelize MM2S cmd idle, bit2=done_latched, bit3=busy_latched
            // bit4=clear_busy, bit5=clear_error/status_error
            // bit6=output_path_idle, bit7=core_done_seen
            3'h0   : reg_data_out <= {slv_reg0[31:8], core_done_seen, output_path_idle_in, error_latched, clear_busy_in, busy_latched, done_latched, voxelize_idle_rd_ddr, 1'b0};
            3'h1   : reg_data_out <= slv_reg1;
            3'h2   : reg_data_out <= slv_reg2;
            3'h3   : reg_data_out <= slv_reg3;
            3'h4   : reg_data_out <= slv_reg4;
            default : reg_data_out <= 0;
        endcase
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rdata <= 0;
        end else begin
            if (slv_reg_rden) begin
                axi_rdata <= reg_data_out;
            end
        end
    end

    // =========================================================================
    // 实例化核心的 Voxelize 模块
    // =========================================================================
    voxelize #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .DRAM_DATA_WIDTH(DRAM_DATA_WIDTH),
        .DRAM_ADDR_WIDTH(DRAM_ADDR_WIDTH)
        // 其它参数在 voxelize 内部有默认值
    ) u_voxelize_core (
        .aclk   (S_AXI_ACLK),
        .aresetn(S_AXI_ARESETN),

        // --- 动态配置参数 ---
        // 0x04: 映射为 DDR 读起始地址
        .base_addr_in  (slv_reg1),
        // 0x08: 映射为总字节数 (取低23位)
        .frame_bytes_in(slv_reg2[22:0]),


        // --- AXI-Stream Slave ---
        .s_axis_tdata (s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tkeep (s_axis_tkeep),
        .s_axis_tlast (s_axis_tlast),

        .m_0_axis_tdata      (m_0_axis_tdata),
        .m_0_axis_tvalid     (m_0_axis_tvalid),
        .m_0_axis_tkeep      (m_0_axis_tkeep),        // 新增 TKEEP 信号
        .m_0_axis_tready     (m_0_axis_tready),       // 内部暂不支持背压，先预留
        .m_0_axis_tlast      (m_0_axis_tlast),        // 整帧处理彻底结束（由 flush_done 驱动）
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
        // --- 控制接口 ---
        // start_pulse 是通过写寄存器产生的单周期高电平
        .start_req        (core_start_pulse),
        // voxelize_idle_rd_ddr 接入 AXI 读总线以便软件查询
        .mm2s_cmd_idle    (voxelize_idle_rd_ddr),
        .voxelize_done    (voxelize_done),
        // --- DataMover Command ---
        .m_axis_cmd_tvalid(m_axis_cmd_tvalid),
        .m_axis_cmd_tready(m_axis_cmd_tready),
        .m_axis_cmd_tdata (m_axis_cmd_tdata)
    );

endmodule
