`timescale 1ps / 1ps

module voxelize #(
    parameter integer DRAM_DATA_WIDTH = 128,
    parameter integer DRAM_ADDR_WIDTH = 18,
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer AXIS_ADDR_WIDTH = 16,
    parameter integer BRAM_DATA_WIDTH = 448,  // 64bit * 8points
    parameter integer BRAM_ADDR_WIDTH = 10,  // HASH_VOXEL_NUMBER(256) * 4brams
    parameter integer BRAM_ADDR_WIDTH_PFE = 7,
    parameter MEM_WEIGHT_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_fixed_q88\\pfn_weight.mem",
    parameter MEM_BIAS_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_fixed_q88\\pfn_bias.mem",
    parameter MEM_BIAS_RELU_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_fixed_q88\\bias_relu.mem"


) (
    // 全局时钟与复位
    input  wire                         aclk,
    input  wire                         aresetn,
    // --- 新增：由 AXI-Lite 传入的动态配置端口 ---
    input  wire [                 31:0] base_addr_in,
    input  wire [                 22:0] frame_bytes_in,
    // -----------------------------------------------------------
    // AXI4-Stream Slave（点云数据输入）
    // -----------------------------------------------------------
    input  wire [  DRAM_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire [DRAM_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tlast,          // 外部传入的一帧最后一个点标志
    // -----------------------------------------------------------
    // AXI4-Stream Master（输出 Voxel 特征）
    // -----------------------------------------------------------
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
    // 启动命令接口（由外部控制器触发一次完整的命令发送流程）
    input  wire                         start_req,             // 上升沿触发一次命令发送操作
    output reg                          idle_read_ddr,         // 模块是否处于空闲状态
    output wire                         voxelize_done,
    // -----------------------------------------------------------
    // AXI-Stream Command Interface（连接到 DataMover 的 S_AXIS_MM2S_CMD）
    // -----------------------------------------------------------
    output wire                         m_axis_cmd_tvalid,
    input  wire                         m_axis_cmd_tready,
    output wire [                 71:0] m_axis_cmd_tdata
);


    // 时钟、复位内部映射
    wire        clk_p = aclk;
    wire        rst_n = aresetn;

    // =========================================================================
    // 帧尾信号（Frame End）生成与延迟补偿
    // =========================================================================
    // s_axis_tlast 在最后一个点完成握手时触发
    wire        frame_end_raw;
    wire        hash_stall;
    reg  [63:0] frame_end_shift;
    always @(posedge clk_p or negedge rst_n) begin
        if (!rst_n) begin
            frame_end_shift <= 64'd0;
        end else if (!hash_stall) begin
            frame_end_shift <= {frame_end_shift[62:0], frame_end_raw};
        end
    end


    // 你可以通过查看仿真波形，数一下从 dram 读出到数据进入 hash 表到底差了几拍
    // 然后调整这里的索引用以对齐 (这里暂时设定为 30 拍)
    wire frame_end = frame_end_shift[30];
    wire flush_done;


    localparam COORD_WIDTH = 11;
    localparam PT_WIDTH = 16;
    localparam PT_WIDTH_XY = 16;
    localparam PT_WIDTH_PER = 2 * PT_WIDTH_XY + 2 * PT_WIDTH;
    localparam BYTE_WIDTH = 8;
    localparam VN_WIDTH = 5;
    localparam EXPAND_PT_DIM = 9;  // x,y,z,intensity,(x-mean,y-mean,z-mean,intensity-mean),(x-vmean,y-vmean,z-vmean,intensity-vmean)
    localparam OUT_PT_DIM = 64;
    localparam MAX_VOXEL_NUM = 12'd20;
    localparam HASH_TABLE_SIZE = 256;
    localparam HASH_ADDR_WIDTH = $clog2(HASH_TABLE_SIZE);  // log2(HASH_TABLE_SIZE)
    localparam [15:0] THRESHOLD_CLOSE = 16'h0100;  // 1.00m
    localparam [15:0] THRESHOLD_BOUDARY_X_LOW = 16'h0000;  // 0.00m
    localparam [15:0] THRESHOLD_BOUDARY_X_HIGH = 16'h451f;  // 69.12m
    localparam [15:0] THRESHOLD_BOUDARY_Y = 16'h27ae;  // +/- 39.68m
    localparam [15:0] THRESHOLD_BOUDARY_Z_LOW = 16'h3000;  // abs(-3)m
    localparam [15:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h1000;  // 1m
    localparam integer PRE_LAT = 0;  // 预处理固定延迟（拍数占位）
    localparam [15:0] LIFE_CYCLE = 16'd150;
    wire                                                  bram_voxelpoint_clk;
    wire                                                  bram_voxelpoint_rst;
    wire                                                  bram_voxelpoint_wr;
    wire [                BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen;  // 32-bit Byte Enable
    wire [                           BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata;  // 256-bit Data
    // expired 写入pfe buffer
    wire                                                  bram_expire_clk_a;
    // wire                                              bram_expire_rst_a;
    // wire                                              bram_expire_wr_a;
    // wire [                     BRAM_DATA_WIDTH/8-1:0] bram_expire_bwen_a;  // 32-bit Byte Enable
    wire [                           BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a;  // 256-bit Data
    // expired 读取voxelpoint
    wire                                                  bram_expire_clk_b;
    wire                                                  bram_expire_rst_b;
    wire                                                  bram_expire_wr_b;
    wire [                BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_expire_bwen_b;  // 32-bit Byte Enable
    wire [                       BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b;  // 256-bit Data
    wire                                                  bram_pfe_clk;
    // wire                                              bram_pfe_rst_a;
    // wire                                              bram_pfe_wr_a;
    // wire [                     BRAM_DATA_WIDTH/8-1:0] bram_pfe_bwen_a;  // 32-bit Byte Enable
    wire [                       BRAM_ADDR_WIDTH_PFE-1:0] bram_pfe_addr;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_pfe_rdata;  // 256-bit Data
    wire                                                  m_axis_expire_tvalid;
    wire                                                  m_axis_expire_tready;
    wire [2*COORD_WIDTH+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata;  // NOTE
    voxel_coor_pipe #(
        .DRAM_DATA_WIDTH         (DRAM_DATA_WIDTH),
        .DRAM_ADDR_WIDTH         (DRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH         (BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH         (BRAM_ADDR_WIDTH),
        .BRAM_ADDR_WIDTH_PFE     (BRAM_ADDR_WIDTH_PFE),
        .HASH_ADDR_WIDTH         (HASH_ADDR_WIDTH),
        .THRESHOLD_CLOSE         (THRESHOLD_CLOSE),
        .THRESHOLD_BOUDARY_X_LOW (THRESHOLD_BOUDARY_X_LOW),
        .THRESHOLD_BOUDARY_X_HIGH(THRESHOLD_BOUDARY_X_HIGH),
        .THRESHOLD_BOUDARY_Y     (THRESHOLD_BOUDARY_Y),
        .THRESHOLD_BOUDARY_Z_LOW (THRESHOLD_BOUDARY_Z_LOW),
        .THRESHOLD_BOUDARY_Z_HIGH(THRESHOLD_BOUDARY_Z_HIGH),
        .PRE_LAT                 (PRE_LAT),
        .LIFE_CYCLE              (LIFE_CYCLE),
        .VN_WIDTH                (VN_WIDTH),
        .MAX_VOXEL_NUM           (MAX_VOXEL_NUM),
        .BYTE_WIDTH              (BYTE_WIDTH)
    ) voxel_coordinate_inst (
        .clk                   (clk_p),
        .rst_n                 (rst_n),
        .frame_end             (frame_end),
        .flush_done            (flush_done),
        .hash_stall            (hash_stall),
        // AXI Stream Slave (Input)
        .s_axis_dram_data      (s_axis_tdata),
        .s_axis_dram_keep      (s_axis_tkeep),
        .s_axis_dram_last      (s_axis_tlast),
        .s_axis_dram_valid     (s_axis_tvalid),
        .s_axis_dram_ready     (s_axis_tready),
        .bram_voxelpoint_clk   (bram_voxelpoint_clk),
        .bram_voxelpoint_rst   (),
        .bram_voxelpoint_wr    (bram_voxelpoint_wr),
        .bram_voxelpoint_bwen  (bram_voxelpoint_bwen),
        .bram_voxelpoint_addr  (bram_voxelpoint_addr),
        .bram_voxelpoint_wrdata(bram_voxelpoint_wrdata),
        .bram_expire_clk_a     (bram_expire_clk_a),
        // .bram_expire_rst_a     (),
        // .bram_expire_wr_a      (bram_expire_wr_a),
        // .bram_expire_bwen_a    (bram_expire_bwen_a),
        .bram_expire_addr_a    (bram_expire_addr_a),
        .bram_expire_rdata_a   (bram_expire_rdata_a),
        .bram_expire_clk_b     (bram_expire_clk_b),
        .bram_expire_rst_b     (),
        .bram_expire_wr_b      (bram_expire_wr_b),
        .bram_expire_bwen_b    (bram_expire_bwen_b),
        .bram_expire_addr_b    (bram_expire_addr_b),
        .bram_expire_wrdata_b  (bram_expire_wrdata_b),
        .m_axis_expire_tdata   (m_axis_expire_tdata),
        .m_axis_expire_tvalid  (m_axis_expire_tvalid),
        .m_axis_expire_tready  (m_axis_expire_tready)
    );

    wire                                     m_axis_pfe_valid;
    wire                                     m_axis_pfe_tready;
    wire        [EXPAND_PT_DIM*PT_WIDTH-1:0] m_axis_pfe_data;
    wire                                     m_axis_pfe_last;
    wire                                     m_axis_pfe_voxel_valid;
    wire signed [           COORD_WIDTH-1:0] m_axis_pfe_voxel_x;
    wire signed [           COORD_WIDTH-1:0] m_axis_pfe_voxel_y;
    wire                                     m_axis_pfe_flush_done;

    pfe #(
        .COORD_WIDTH        (COORD_WIDTH),
        .PT_WIDTH           (PT_WIDTH),
        .PT_WIDTH_PER       (PT_WIDTH_PER),
        .MAX_VOXEL_NUM      (MAX_VOXEL_NUM),
        .VN_WIDTH           (VN_WIDTH),
        .EXPAND_PT_DIM      (EXPAND_PT_DIM),
        .BRAM_DATA_WIDTH    (BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH    (BRAM_ADDR_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) u_pfe (
        .clk                   (clk_p),
        .rst_n                 (rst_n),
        .s_axis_expire_tvalid  (m_axis_expire_tvalid),
        .s_axis_expire_tready  (m_axis_expire_tready),
        .s_axis_expire_tdata   (m_axis_expire_tdata),
        .bram_pfe_clk          (bram_pfe_clk),
        // .bram_pfe_rst        (bram_pfe_rst),
        .bram_pfe_addr         (bram_pfe_addr),
        // .bram_pfe_en         (bram_pfe_en),
        .bram_pfe_rdata        (bram_pfe_rdata),
        .m_axis_pfe_valid      (m_axis_pfe_valid),
        .m_axis_pfe_tready     (m_axis_pfe_tready),
        .m_axis_pfe_data       (m_axis_pfe_data),
        .m_axis_pfe_last       (m_axis_pfe_last),
        .m_axis_pfe_voxel_valid(m_axis_pfe_voxel_valid),
        .m_axis_pfe_voxel_x    (m_axis_pfe_voxel_x),
        .m_axis_pfe_voxel_y    (m_axis_pfe_voxel_y),
        .m_axis_pfe_flush_done (m_axis_pfe_flush_done),
        .flush_done            (flush_done)
    );

    localparam WEIGHT_WIDTH = 16;
    localparam ACC_WIDTH = 32;

    wire m_axis_pfn_voxel_valid;
    wire signed [COORD_WIDTH-1:0] m_axis_pfn_voxel_x;
    wire signed [COORD_WIDTH-1:0] m_axis_pfn_voxel_y;
    wire m_axis_pfn_valid;
    wire [OUT_PT_DIM*PT_WIDTH-1:0] m_axis_pfn_data;
    wire m_axis_pfn_tready;
    wire m_axis_pfn_flush_done;
    assign voxelize_done = m_axis_pfn_flush_done || frame_end_raw || m_axis_pfe_flush_done;

    // wire [PT_WIDTH*OUT_PT_DIM-1:0] m_axis_end_data;
    // assign m_axis_end_data_part = m_axis_end_data[16-1:0];
    pfn_layer #(
        .EXPAND_PT_DIM  (EXPAND_PT_DIM),
        .COORD_WIDTH    (COORD_WIDTH),
        .OUT_PT_DIM     (OUT_PT_DIM),
        .PT_WIDTH       (PT_WIDTH),
        .MEM_WEIGHT_FILE(MEM_WEIGHT_FILE),
        .MEM_BIAS_FILE  (MEM_BIAS_FILE),
        // .MEM_BIAS_RELU_FILE(MEM_BIAS_RELU_FILE),
        .WEIGHT_WIDTH   (WEIGHT_WIDTH),
        .ACC_WIDTH      (ACC_WIDTH),
        .MAX_VOXEL_NUM  (MAX_VOXEL_NUM)
    ) u_pfn_layer (
        .clk                       (clk_p),
        .rst_n                     (rst_n),
        // pfe
        .s_axis_pfe_valid          (m_axis_pfe_valid),
        .s_axis_pfe_tready         (m_axis_pfe_tready),
        .s_axis_pfe_data           (m_axis_pfe_data),
        .s_axis_pfe_last           (m_axis_pfe_last),
        .s_axis_pfe_voxel_valid    (m_axis_pfe_voxel_valid),
        .s_axis_pfe_voxel_x        (m_axis_pfe_voxel_x),
        .s_axis_pfe_voxel_y        (m_axis_pfe_voxel_y),
        .s_axis_pfe_flush_done     (m_axis_pfe_flush_done),
        // pfn
        .m_axis_pfn_voxel_valid_out(m_axis_pfn_voxel_valid),
        .m_axis_pfn_voxel_x        (m_axis_pfn_voxel_x),
        .m_axis_pfn_voxel_y        (m_axis_pfn_voxel_y),
        .m_axis_pfn_valid          (m_axis_pfn_valid),
        .m_axis_pfn_tready         (m_axis_pfn_tready),
        .m_axis_pfn_data           (m_axis_pfn_data),
        .m_axis_pfn_flush_done     (m_axis_pfn_flush_done)
    );

    // router
    wire pfn0_in_valid;
    wire pfn0_in_ready;
    wire [OUT_PT_DIM*PT_WIDTH-1:0] pfn0_in_data;

    wire pfn1_in_valid;
    wire pfn1_in_ready;
    wire [OUT_PT_DIM*PT_WIDTH-1:0] pfn1_in_data;

    wire meta0_valid;
    wire meta0_ready;
    wire signed [COORD_WIDTH-1:0] meta0_x;
    wire signed [COORD_WIDTH-1:0] meta0_y;

    wire meta1_valid;
    wire meta1_ready;
    wire signed [COORD_WIDTH-1:0] meta1_x;
    wire signed [COORD_WIDTH-1:0] meta1_y;

    wire route0_fire;
    wire route1_fire;
    wire last_route;

    pfn_dual_widthconv_router #(
        .PFN_DATA_WIDTH(OUT_PT_DIM * PT_WIDTH),
        .COORD_WIDTH   (COORD_WIDTH)
    ) u_pfn_dual_widthconv_router (
        .clk  (clk_p),
        .rst_n(rst_n),

        .s_axis_pfn_tvalid (m_axis_pfn_valid),
        .s_axis_pfn_tready (m_axis_pfn_tready),
        .s_axis_pfn_tdata  (m_axis_pfn_data),
        .s_axis_pfn_voxel_x(m_axis_pfn_voxel_x),
        .s_axis_pfn_voxel_y(m_axis_pfn_voxel_y),

        .m0_axis_pfn_tvalid(pfn0_in_valid),
        .m0_axis_pfn_tready(pfn0_in_ready),
        .m0_axis_pfn_tdata (pfn0_in_data),

        .m0_axis_meta_tvalid (meta0_valid),
        .m0_axis_meta_tready (meta0_ready),
        .m0_axis_meta_voxel_x(meta0_x),
        .m0_axis_meta_voxel_y(meta0_y),

        .m1_axis_pfn_tvalid(pfn1_in_valid),
        .m1_axis_pfn_tready(pfn1_in_ready),
        .m1_axis_pfn_tdata (pfn1_in_data),

        .m1_axis_meta_tvalid (meta1_valid),
        .m1_axis_meta_tready (meta1_ready),
        .m1_axis_meta_voxel_x(meta1_x),
        .m1_axis_meta_voxel_y(meta1_y),

        .route0_fire(route0_fire),
        .route1_fire(route1_fire),
        .last_route (last_route)
    );

    // data width converter
    wire [AXIS_DATA_WIDTH-1:0] m_axis_out_tdata;
    wire conv0_out_tvalid;
    wire conv0_out_tready;
    wire [AXIS_DATA_WIDTH-1:0] conv0_out_tdata;
    wire conv0_out_tlast;

    wire conv1_out_tvalid;
    wire conv1_out_tready;
    wire [AXIS_DATA_WIDTH-1:0] conv1_out_tdata;
    wire conv1_out_tlast;

    pfn_width_converter #(
        .IN_WIDTH (OUT_PT_DIM * PT_WIDTH),
        .OUT_WIDTH(AXIS_DATA_WIDTH)
    ) u_pfn_width_converter_0 (
        .clk  (clk_p),
        .rst_n(rst_n),

        .s_axis_pfn_tvalid(pfn0_in_valid),
        .s_axis_pfn_tready(pfn0_in_ready),
        .s_axis_pfn_tdata (pfn0_in_data),

        .m_axis_out_tvalid(conv0_out_tvalid),
        .m_axis_out_tready(conv0_out_tready),
        .m_axis_out_tdata (conv0_out_tdata),
        .m_axis_out_tlast (conv0_out_tlast)
    );

    pfn_width_converter #(
        .IN_WIDTH (OUT_PT_DIM * PT_WIDTH),
        .OUT_WIDTH(AXIS_DATA_WIDTH)
    ) u_pfn_width_converter_1 (
        .clk  (clk_p),
        .rst_n(rst_n),

        .s_axis_pfn_tvalid(pfn1_in_valid),
        .s_axis_pfn_tready(pfn1_in_ready),
        .s_axis_pfn_tdata (pfn1_in_data),

        .m_axis_out_tvalid(conv1_out_tvalid),
        .m_axis_out_tready(conv1_out_tready),
        .m_axis_out_tdata (conv1_out_tdata),
        .m_axis_out_tlast (conv1_out_tlast)
    );


    dualport_bram #(
        .DATA_WIDTH     (BRAM_DATA_WIDTH),
        .ADDR_WIDTH     (BRAM_ADDR_WIDTH),
        .MEM_FILE       ("NOTHING"),
        .MEM_FILE_LENGTH()
    ) u_voxelpoint_bram (
        // 写
        .a_clk (bram_voxelpoint_clk),
        .a_wr  (bram_voxelpoint_wr),
        .a_en  (1'b1),
        .a_bwen(bram_voxelpoint_bwen),
        .a_addr(bram_voxelpoint_addr),
        .a_din (bram_voxelpoint_wrdata),
        .a_dout(),
        // 读出放到pfe buffer
        .b_clk (bram_expire_clk_a),
        .b_wr  (1'b0),                                  // only read
        .b_en  (1'b1),
        .b_bwen({BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}}),  // only read
        .b_addr(bram_expire_addr_a),
        .b_din (),
        .b_dout(bram_expire_rdata_a)
    );

    // PFE voxelpoint buffer 
    dualport_bram #(
        .DATA_WIDTH     (BRAM_DATA_WIDTH),
        .ADDR_WIDTH     (BRAM_ADDR_WIDTH_PFE),
        .MEM_FILE       ("NOTHING"),
        .MEM_FILE_LENGTH()
    ) u_pfe_bram (
        // voxel_coor_pipe 中的 expired_manager 写入待PFE的voxelpoint
        .a_clk (bram_expire_clk_b),
        .a_wr  (bram_expire_wr_b),
        .a_en  (1'b1),
        .a_bwen(bram_expire_bwen_b),
        .a_addr(bram_expire_addr_b),
        .a_din (bram_expire_wrdata_b),
        .a_dout(),
        // pfe 读取voxelpoint进行特征提取
        .b_clk (bram_pfe_clk),
        .b_wr  (1'b0),                                  // only read
        .b_en  (1'b1),
        .b_bwen({BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}}),  // only read
        .b_addr(bram_pfe_addr),
        .b_din (),
        .b_dout(bram_pfe_rdata)
    );



    // =========================================================================
    // AXI4-Stream 输出信号映射
    // =========================================================================
    // 当前输出数据宽度与 AXIS_DATA_WIDTH 一致，因此直接透传
    // 若后续存在不足位宽的情况，可通过 TKEEP 指示有效字节

    assign m_0_axis_tdata       = conv0_out_tdata;
    assign m_0_axis_tkeep       = {(AXIS_DATA_WIDTH / 8) {1'b1}};
    assign conv0_out_tready     = m_0_axis_tready;
    assign m_0_axis_tvalid      = conv0_out_tvalid;
    assign m_0_axis_tlast       = conv0_out_tlast;

    assign m_1_axis_tdata       = conv1_out_tdata;
    assign m_1_axis_tkeep       = {(AXIS_DATA_WIDTH / 8) {1'b1}};
    assign conv1_out_tready     = m_1_axis_tready;
    assign m_1_axis_tvalid      = conv1_out_tvalid;
    assign m_1_axis_tlast       = conv1_out_tlast;

    assign m_0_axis_voxel_x     = meta0_x;
    assign m_0_axis_voxel_y     = meta0_y;
    assign m_0_axis_voxel_valid = meta0_valid;

    assign m_1_axis_voxel_x     = meta1_x;
    assign m_1_axis_voxel_y     = meta1_y;
    assign m_1_axis_voxel_valid = meta1_valid;

    assign meta0_ready          = conv0_out_tready;
    assign meta1_ready          = conv1_out_tready;

    reg valid_reg;
    reg start_d1;
    // 上升沿检测
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            start_d1 <= 1'b0;
        end else begin
            start_d1 <= start_req;
        end
    end
    wire start_pulse = start_req && !start_d1;

    // =======================================================
    // 拼装 AXI DataMover MM2S Command（72-bit 格式）
    // =======================================================
    // [22:0]  BTT  = 本帧总传输字节数（Bytes To Transfer）
    // [23]    Type = 1，表示 INCR 递增突发
    // [29:24] DSA  = 0，Dynamic Source Address
    // [30]    EOF  = 1，传输完成后在 TLAST 位置结束
    // [31]    DRR  = 0，关闭 Data Realignment
    // [63:32] SADDR= 源地址（DDR 物理地址）
    // [67:64] TAG  = 0
    // [71:68] RSVD = 0
    // 这里将原先固定参数替换为动态输入端口
    assign m_axis_cmd_tdata = {
        4'b0000,
        4'b0000,
        base_addr_in,  // 动态传入的源地址
        1'b0,
        1'b1,
        6'b000000,
        1'b1,
        frame_bytes_in  // 动态传入的传输字节数
    };

    assign m_axis_cmd_tvalid = valid_reg;

    // =======================================================
    // 命令发送握手状态机
    // =======================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_reg     <= 1'b0;
            idle_read_ddr <= 1'b1;
        end else begin
            if (idle_read_ddr) begin
                // 收到启动脉冲后，拉高 valid，准备发送命令
                if (start_pulse) begin
                    valid_reg     <= 1'b1;
                    idle_read_ddr <= 1'b0;
                end
            end else begin
                // DataMover 就绪且 valid 为高时，完成一次命令握手
                if (valid_reg && m_axis_cmd_tready) begin
                    valid_reg     <= 1'b0;
                    idle_read_ddr <= 1'b1;  // 命令发送完成，回到空闲态，等待下一次触发
                end
            end
        end
    end



    reg [22:0] rx_byte_cnt;
    reg        frame_end_raw_by_count;
    assign frame_end_raw = frame_end_raw_by_count;
    wire [22:0] next_rx_byte_cnt;
    localparam integer AXIS_IN_BYTES = DRAM_DATA_WIDTH / 8;
    assign next_rx_byte_cnt = rx_byte_cnt + AXIS_IN_BYTES[22:0];
    wire s_axis_fire;

    assign s_axis_fire = s_axis_tvalid && s_axis_tready;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rx_byte_cnt            <= 23'd0;
            frame_end_raw_by_count <= 1'b0;
        end else begin
            frame_end_raw_by_count <= 1'b0;

            if (start_pulse) begin
                rx_byte_cnt <= 23'd0;
            end else if (s_axis_fire) begin
                if (next_rx_byte_cnt >= frame_bytes_in) begin
                    frame_end_raw_by_count <= 1'b1;
                    rx_byte_cnt            <= 23'd0;
                end else begin
                    rx_byte_cnt <= next_rx_byte_cnt;
                end
            end
        end
    end


    // =========================================================================
    // 仿真检测器: 严格检测 m_axis_cmd_tvalid 与有效 m_axis_out_tlast 的时序关系
    // =========================================================================
    // `ifndef SYNTHESIS
    //     reg valid_tlast_d1;

    //     // 提取并延迟一拍“有效的 tlast”信号
    //     always @(posedge clk_p or negedge rst_n) begin
    //         if (!rst_n) begin
    //             valid_tlast_d1 <= 1'b0;
    //         end else begin
    //             // 前提条件：只有在 tvalid 为高时的 tlast 才是真实有效的数据包尾
    //             valid_tlast_d1 <= m_axis_out_tvalid && m_axis_out_tlast;
    //         end
    //     end

    //     // 实时监测时序合规性
    //     always @(posedge clk_p) begin
    //         if (rst_n) begin
    //             // 规则 1：如果 cmd_tvalid 为高，那么前一拍必须是有效的 tlast
    //             // 无论当前拍 out_tvalid 是高是低，都不影响此判定
    //             if (m_axis_cmd_tvalid && !valid_tlast_d1) begin
    //                 $display(
    //                     "[%0t] [时序违例] 错误: m_axis_cmd_tvalid 拉高，但前一拍并未出现有效的 tlast (tvalid & tlast 均为高)！",
    //                     $time);
    //                 // $stop; 
    //             end

    //             // 规则 2：如果前一拍是有效的 tlast，那么当前拍 cmd_tvalid 必须拉高
    //             // 同样，即使当前拍 out_tvalid 已经拉低，这个条件依然生效
    //             if (!m_axis_cmd_tvalid && valid_tlast_d1) begin
    //                 $display(
    //                     "[%0t] [时序违例] 错误: 前一拍出现了有效的 tlast，但当前拍 m_axis_cmd_tvalid 却没有拉高响应！",
    //                     $time);
    //                 // $stop; 
    //             end
    //         end
    //     end
    // `endif

endmodule
