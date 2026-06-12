`timescale 1ps / 1ps

module voxelize #(
    parameter integer DRAM_DATA_WIDTH = 128,
    parameter integer DRAM_ADDR_WIDTH = 23,
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer AXIS_ADDR_WIDTH = 16,
    parameter integer BRAM_DATA_WIDTH = 576,  // 72bit * 8 points
    parameter integer BRAM_ADDR_WIDTH = 10,  // HASH_VOXEL_NUMBER(256) * 4 rows
    parameter integer BRAM_ADDR_WIDTH_PFE = 12,
    parameter MEM_WEIGHT_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_hardware\\pfn_weight.mem",
    parameter MEM_BIAS_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_hardware\\pfn_bias.mem",
    parameter MEM_BIAS_RELU_FILE = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti_hardware\\bias_relu.mem"

) (
    input wire clk_p,
    input wire clk_n,
    input wire rst_n
);
    localparam integer MEM_FILE_LENGTH = 116000;
    reg                          dram_read_en = 1'b1;
    reg                          dram_write_en = 1'b0;
    wire [DRAM_ADDR_WIDTH - 1:0] dram_write_size = MEM_FILE_LENGTH;
    wire [DRAM_ADDR_WIDTH - 1:0] dram_read_address;
    wire [DRAM_DATA_WIDTH - 1:0] dram_data_out;
    dram #(
        .DATA_WIDTH     (DRAM_DATA_WIDTH),
        .ADDR_WIDTH     (DRAM_ADDR_WIDTH),                                                      // 65536 depth
        .MEM_FILE       ("E:\\mmdetection3d\\data\\kitti\\scripts\\data\\points_sim_bin.txt"),
        .MEM_FILE_LENGTH(MEM_FILE_LENGTH)
    ) u_pointcloud_dram (
        .clk          (clk_p),
        .rst_n        (rst_n),
        .en           (dram_read_en),
        .we           (dram_write_en),
        .write_address(),
        .read_address (dram_read_address),
        .data_in      (),
        .data_out     (dram_data_out)
    );

    // =========================================================================
    // 帧末清空信号 (Frame End) 生成与延迟补偿逻辑
    // =========================================================================
    reg frame_end_raw;
    reg read_done_flag;

    always @(posedge clk_p or negedge rst_n) begin
        if (!rst_n) begin
            frame_end_raw  <= 1'b0;
            read_done_flag <= 1'b0;
        end else begin
            // 核心条件：当读地址等于 MEM_FILE_LENGTH，且之前没触发过
            if ((dram_read_address == MEM_FILE_LENGTH[DRAM_ADDR_WIDTH-1:0]) && !read_done_flag) begin
                frame_end_raw  <= 1'b1;
                read_done_flag <= 1'b1;  // 锁定，保证只产生一个单周期脉冲
            end else begin
                frame_end_raw <= 1'b0;
            end
        end
    end

    reg [63:0] frame_end_shift;
    always @(posedge clk_p or negedge rst_n) begin
        if (!rst_n) begin
            frame_end_shift <= 64'd0;
        end else if (!voxel_coordinate_inst.u_hash_table_tombstone.hash_stall) begin
            frame_end_shift <= {frame_end_shift[62:0], frame_end_raw};
        end
    end


    // 你可以通过查看仿真波形，数一下从 dram 读出到数据进入 hash 表到底差了几拍
    // 然后调整这里的索引用以对齐 (这里暂时设定为 30 拍)
    wire frame_end = frame_end_shift[30];
    wire flush_done;




    wire m_axis_dram_last;
    wire [DRAM_DATA_WIDTH-1:0] m_axis_dram_data;
    wire [DRAM_DATA_WIDTH/8-1:0] m_axis_dram_keep = {DRAM_DATA_WIDTH / 8{1'b1}};
    wire m_axis_dram_valid;
    wire m_axis_dram_ready;
    // axi stream 读fifo
    adapter_ram_2_axi_stream #(
        .AXIS_DATA_WIDTH  (DRAM_DATA_WIDTH),
        .BRAM_DEPTH       (DRAM_ADDR_WIDTH),
        .AXIS_STROBE_WIDTH(DRAM_DATA_WIDTH / 8),
        .USE_KEEP         (0),
        .USER_DEPTH       (1)
    ) u_adapter_ram_2_axi_stream (
        .clk              (clk_p),
        .rst_n            (rst_n),
        //Ping Poing FIFO Read Interface
        .i_axis_user      (),
        .i_bram_en        (dram_read_en),
        .i_bram_size      (dram_write_size),
        .o_bram_addr      (dram_read_address),
        .i_bram_data      (dram_data_out),
        //AXI Stream Output
        .m_axis_dram_user (),
        .m_axis_dram_ready(m_axis_dram_ready),
        .m_axis_dram_data (m_axis_dram_data),
        .m_axis_dram_last (m_axis_dram_last),
        .m_axis_dram_valid(m_axis_dram_valid)
    );


    localparam COORD_WIDTH = 11;
    localparam PT_WIDTH = 16;
    localparam PT_WIDTH_XY = 20;
    localparam PT_WIDTH_PER = 2 * PT_WIDTH_XY + 2 * PT_WIDTH;
    localparam BYTE_WIDTH = 9;
    localparam VN_WIDTH = 7;
    localparam EXPAND_PT_DIM = 9;  // x,y,z,intensity,(x-mean,y-mean,z-mean,intensity-mean),(x-vmean,y-vmean,z-vmean,intensity-vmean)
    localparam OUT_PT_DIM = 64;
    localparam MAX_VOXEL_NUM = 12'd100;
    localparam ENTRY_POINT_CAP = 12'd32;
    localparam HASH_TABLE_SIZE = 256;
    localparam HASH_ADDR_WIDTH = $clog2(HASH_TABLE_SIZE);  // log2(HASH_TABLE_SIZE)
    localparam [PT_WIDTH-1:0] THRESHOLD_CLOSE = 16'h0100;  // 1.00m
    localparam [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_X_LOW = 20'h0000;  // 0.00m
    localparam [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_X_HIGH = 20'h451ec;  // 69.12m
    localparam [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_Y = 20'h27ae1;  // +/- 39.68m
    localparam [PT_WIDTH-1:0] THRESHOLD_BOUDARY_Z_LOW = 16'h3000;  // abs(-3)m
    localparam [PT_WIDTH-1:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h1000;  // 1m
    localparam integer PRE_LAT = 0;  // 预处理固定延迟（拍数占位）
    localparam [15:0] LIFE_CYCLE = 16'd200;
    wire                                                  bram_voxelpoint_clk;
    wire                                                  bram_voxelpoint_rst;
    wire                                                  bram_voxelpoint_wr;
    wire [                BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen;  // 72-bit byte enable
    wire [                           BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata;  // 576-bit data
    // expired 读取voxelpoint 
    wire [                           BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a;  // 10-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a;  // 576-bit data
    // expired 写入pfe buffer
    wire                                                  bram_expire_wr_b;
    wire [                       BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b;  // 8-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b;  // 576-bit data

    wire [                       BRAM_ADDR_WIDTH_PFE-1:0] bram_pfe_addr;  // 8-bit Address
    wire [                           BRAM_DATA_WIDTH-1:0] bram_pfe_rdata;  // 576-bit data
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
        .ENTRY_POINT_CAP         (ENTRY_POINT_CAP),
        .BYTE_WIDTH              (BYTE_WIDTH)
    ) voxel_coordinate_inst (
        .clk                   (clk_p),
        .rst_n                 (rst_n),
        .frame_end             (frame_end),
        .flush_done            (flush_done),
        .hash_stall            (hash_stall),
        // AXI Stream Slave (Input)
        .s_axis_dram_data      (m_axis_dram_data),
        .s_axis_dram_keep      (m_axis_dram_keep),
        .s_axis_dram_last      (m_axis_dram_last),
        .s_axis_dram_valid     (m_axis_dram_valid),
        .s_axis_dram_ready     (m_axis_dram_ready),
        .bram_voxelpoint_wr    (bram_voxelpoint_wr),
        .bram_voxelpoint_bwen  (bram_voxelpoint_bwen),
        .bram_voxelpoint_addr  (bram_voxelpoint_addr),
        .bram_voxelpoint_wrdata(bram_voxelpoint_wrdata),

        .bram_expire_addr_a  (bram_expire_addr_a),
        .bram_expire_rdata_a (bram_expire_rdata_a),
        .bram_expire_wr_b    (bram_expire_wr_b),
        .bram_expire_addr_b  (bram_expire_addr_b),
        .bram_expire_wrdata_b(bram_expire_wrdata_b),
        .m_axis_expire_tdata (m_axis_expire_tdata),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tready(m_axis_expire_tready)
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
        .bram_pfe_addr         (bram_pfe_addr),
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

    // wire [PT_WIDTH*OUT_PT_DIM-1:0] m_axis_end_data;
    // assign m_axis_end_data_part = m_axis_end_data[16-1:0];
    pfn_layer #(
        .EXPAND_PT_DIM     (EXPAND_PT_DIM),
        .COORD_WIDTH       (COORD_WIDTH),
        .OUT_PT_DIM        (OUT_PT_DIM),
        .PT_WIDTH          (PT_WIDTH),
        .MEM_WEIGHT_FILE   (MEM_WEIGHT_FILE),
        .MEM_BIAS_FILE     (MEM_BIAS_FILE),
        .MEM_BIAS_RELU_FILE(MEM_BIAS_RELU_FILE),
        .WEIGHT_WIDTH      (WEIGHT_WIDTH),
        .ACC_WIDTH         (ACC_WIDTH),
        .MAX_VOXEL_NUM     (MAX_VOXEL_NUM)
    ) u_pfn_layer (
        .clk                   (clk_p),
        .rst_n                 (rst_n),
        // pfe
        .s_axis_pfe_valid      (m_axis_pfe_valid),
        .s_axis_pfe_tready     (m_axis_pfe_tready),
        .s_axis_pfe_data       (m_axis_pfe_data),
        .s_axis_pfe_last       (m_axis_pfe_last),
        .s_axis_pfe_voxel_valid(m_axis_pfe_voxel_valid),
        .s_axis_pfe_voxel_x    (m_axis_pfe_voxel_x),
        .s_axis_pfe_voxel_y    (m_axis_pfe_voxel_y),
        .s_axis_pfe_flush_done (m_axis_pfe_flush_done),
        // pfn
        .m_axis_pfn_voxel_x    (m_axis_pfn_voxel_x),
        .m_axis_pfn_voxel_y    (m_axis_pfn_voxel_y),
        .m_axis_pfn_valid      (m_axis_pfn_valid),
        .m_axis_pfn_tready     (m_axis_pfn_tready),
        .m_axis_pfn_data       (m_axis_pfn_data),
        .m_axis_pfn_flush_done (m_axis_pfn_flush_done)
    );

    // router
    wire pfn0_in_valid;
    wire pfn0_in_ready;
    wire [OUT_PT_DIM*PT_WIDTH-1:0] pfn0_in_data;

    wire pfn1_in_valid;
    wire pfn1_in_ready;
    wire [OUT_PT_DIM*PT_WIDTH-1:0] pfn1_in_data;

    wire meta0_valid;
    wire meta0_ready;  // 暂时不对 metadata 路径进行 backpressure
    wire signed [COORD_WIDTH-1:0] meta0_x;
    wire signed [COORD_WIDTH-1:0] meta0_y;

    wire meta1_valid;
    wire meta1_ready;  // 暂时不对 metadata 路径进行 backpressure
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
    wire conv0_out_tready = 1'b1;
    wire [AXIS_DATA_WIDTH-1:0] conv0_out_tdata;
    wire conv0_out_tlast;

    wire conv1_out_tvalid;
    wire conv1_out_tready = 1'b1;
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


    voxel_sdp_bram #(
        .DATA_WIDTH     (BRAM_DATA_WIDTH),
        .ADDR_WIDTH     (BRAM_ADDR_WIDTH),
        .MEM_FILE       ("NOTHING"),
        .MEM_FILE_LENGTH(0),
        .CHUNK_WIDTH    (BYTE_WIDTH)
    ) u_voxelpoint_bram (
        .clk(clk_p),

        // write: voxel_coor_pipe -> voxel point RAM
        .wr_en  (bram_voxelpoint_wr),
        .wr_bwen(bram_voxelpoint_bwen),
        .wr_addr(bram_voxelpoint_addr),
        .wr_data(bram_voxelpoint_wrdata),

        // read: expire_manager -> copy to PFE cache
        .rd_en  (1'b1),
        .rd_addr(bram_expire_addr_a),
        .rd_data(bram_expire_rdata_a)
    );

    pfe_lutram_cache #(
        .DATA_WIDTH(BRAM_DATA_WIDTH),
        .ADDR_WIDTH(BRAM_ADDR_WIDTH_PFE)
    ) u_pfe_bram (
        .clk(clk_p),

        .wr_en  (bram_expire_wr_b),
        .wr_addr(bram_expire_addr_b),
        .wr_data(bram_expire_wrdata_b),

        .rd_en  (1'b1),
        .rd_addr(bram_pfe_addr),
        .rd_data(bram_pfe_rdata)
    );

    wire m_axis_0_cmd_tvalid;
    wire m_axis_0_cmd_tready;
    wire [72-1:0] m_axis_0_cmd_tdata;  // 这里的地址是相对于某个基地址的偏移，单位是“数据宽度”（比如128bit）
    voxel_scatter_addr_gen_kitti u_voxel_scatter_0_addr_gen (
        .aclk             (clk_p),
        .aresetn          (rst_n),
        .base_addr_in     (32'h10000000),
        .s_voxel_valid    (meta0_valid),
        .s_voxel_ready    (meta0_ready),
        .s_voxel_x        (meta0_x),
        .s_voxel_y        (meta0_y),
        .m_axis_cmd_tvalid(m_axis_0_cmd_tvalid),
        .m_axis_cmd_tready(1'b1),
        .m_axis_cmd_tdata (m_axis_0_cmd_tdata)
    );

    wire m_axis_1_cmd_tvalid;
    wire m_axis_1_cmd_tready;
    wire [72-1:0] m_axis_1_cmd_tdata;  // 这里的地址是相对于某个基地址的偏移，单位是“数据宽度”（比如128bit）
    voxel_scatter_addr_gen_kitti u_voxel_scatter_1_addr_gen (
        .aclk             (clk_p),
        .aresetn          (rst_n),
        .base_addr_in     (32'h10000000),
        .s_voxel_valid    (meta1_valid),
        .s_voxel_ready    (meta1_ready),
        .s_voxel_x        (meta1_x),
        .s_voxel_y        (meta1_y),
        .m_axis_cmd_tvalid(m_axis_1_cmd_tvalid),
        .m_axis_cmd_tready(1'b1),
        .m_axis_cmd_tdata (m_axis_1_cmd_tdata)
    );


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
