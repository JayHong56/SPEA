`timescale 1 ns / 1 ps

module voxelize_wrapper_kitti_v1_1 #(
    
    
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer DRAM_DATA_WIDTH = 128,
    parameter integer DRAM_ADDR_WIDTH = 23,
    
    


    
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 5
) (
    
    
    
    
    input  wire [  DRAM_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire [DRAM_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tlast,

    output wire [  AXIS_DATA_WIDTH-1:0] m_0_axis_tdata,
    output wire                         m_0_axis_tvalid,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_0_axis_tkeep,        
    input  wire                         m_0_axis_tready,       
    output wire                         m_0_axis_tlast,        
    output wire [  AXIS_DATA_WIDTH-1:0] m_1_axis_tdata,
    output wire                         m_1_axis_tvalid,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_1_axis_tkeep,        
    input  wire                         m_1_axis_tready,       
    output wire                         m_1_axis_tlast,        
    
    output wire [               11-1:0] m_0_axis_voxel_x,
    output wire [               11-1:0] m_0_axis_voxel_y,
    output wire                         m_0_axis_voxel_valid,  
    output wire [               11-1:0] m_1_axis_voxel_x,
    output wire [               11-1:0] m_1_axis_voxel_y,
    output wire                         m_1_axis_voxel_valid,
    input  wire                         m_0_axis_voxel_ready,
    input  wire                         m_1_axis_voxel_ready,
    
    
    
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,
    output wire [71:0] m_axis_cmd_tdata,
    
    


    
    input  wire                                  s00_axi_aclk,
    input  wire                                  s00_axi_aresetn,
    input  wire [    C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
    input  wire [                         2 : 0] s00_axi_awprot,
    input  wire                                  s00_axi_awvalid,
    output wire                                  s00_axi_awready,
    input  wire [    C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
    input  wire                                  s00_axi_wvalid,
    output wire                                  s00_axi_wready,
    output wire [                         1 : 0] s00_axi_bresp,
    output wire                                  s00_axi_bvalid,
    input  wire                                  s00_axi_bready,
    input  wire [    C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
    input  wire [                         2 : 0] s00_axi_arprot,
    input  wire                                  s00_axi_arvalid,
    output wire                                  s00_axi_arready,
    output wire [    C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
    output wire [                         1 : 0] s00_axi_rresp,
    output wire                                  s00_axi_rvalid,
    input  wire                                  s00_axi_rready,

    
    
    
    output wire [31:0] write_base_addr_out,  

    
    output wire clear_start_out,
    input  wire clear_done_in,
    input  wire clear_busy_in,
    input  wire clear_error_in,

    
    input  wire output_path_idle_in,
    input  wire output_path_error_in
);
    
    voxelize_wrapper_v1_0_S00_AXI #(
        
        .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
        .DRAM_DATA_WIDTH   (DRAM_DATA_WIDTH),
        .DRAM_ADDR_WIDTH   (DRAM_ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) voxelize_wrapper_v1_0_S00_AXI_inst (
        
        .s_axis_tdata (s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tkeep (s_axis_tkeep),
        .s_axis_tlast (s_axis_tlast),

        .m_0_axis_tdata      (m_0_axis_tdata),
        .m_0_axis_tvalid     (m_0_axis_tvalid),
        .m_0_axis_tkeep      (m_0_axis_tkeep),        
        .m_0_axis_tready     (m_0_axis_tready),       
        .m_0_axis_tlast      (m_0_axis_tlast),        
        .m_1_axis_tdata      (m_1_axis_tdata),
        .m_1_axis_tvalid     (m_1_axis_tvalid),
        .m_1_axis_tkeep      (m_1_axis_tkeep),        
        .m_1_axis_tready     (m_1_axis_tready),       
        .m_1_axis_tlast      (m_1_axis_tlast),        
        .m_0_axis_voxel_x    (m_0_axis_voxel_x),
        .m_0_axis_voxel_y    (m_0_axis_voxel_y),
        .m_0_axis_voxel_valid(m_0_axis_voxel_valid),  
        .m_1_axis_voxel_x    (m_1_axis_voxel_x),
        .m_1_axis_voxel_y    (m_1_axis_voxel_y),
        .m_1_axis_voxel_valid(m_1_axis_voxel_valid),
        .m_0_axis_voxel_ready(m_0_axis_voxel_ready),
        .m_1_axis_voxel_ready(m_1_axis_voxel_ready),
        .m_axis_cmd_tvalid   (m_axis_cmd_tvalid),
        .m_axis_cmd_tready   (m_axis_cmd_tready),
        .m_axis_cmd_tdata    (m_axis_cmd_tdata),

        
        .S_AXI_ACLK   (s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR (s00_axi_awaddr),
        .S_AXI_AWPROT (s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA  (s00_axi_wdata),
        .S_AXI_WSTRB  (s00_axi_wstrb),
        .S_AXI_WVALID (s00_axi_wvalid),
        .S_AXI_WREADY (s00_axi_wready),
        .S_AXI_BRESP  (s00_axi_bresp),
        .S_AXI_BVALID (s00_axi_bvalid),
        .S_AXI_BREADY (s00_axi_bready),
        .S_AXI_ARADDR (s00_axi_araddr),
        .S_AXI_ARPROT (s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA  (s00_axi_rdata),
        .S_AXI_RRESP  (s00_axi_rresp),
        .S_AXI_RVALID (s00_axi_rvalid),
        .S_AXI_RREADY (s00_axi_rready),

        
        .write_base_addr_out(write_base_addr_out),
        .clear_start_out    (clear_start_out),
        .clear_done_in      (clear_done_in),
        .clear_busy_in      (clear_busy_in),
        .clear_error_in     (clear_error_in),
        .output_path_idle_in  (output_path_idle_in),
        .output_path_error_in (output_path_error_in)
    );

    

    

endmodule
