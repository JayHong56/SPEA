`timescale 1ps / 1ps

module pfn_dual_widthconv_router #(
    parameter integer PFN_DATA_WIDTH = 1024,
    parameter integer COORD_WIDTH    = 11
) (
    input  wire clk,
    input  wire rst_n,

    // Upstream: PFN whole-voxel output
    input  wire                              s_axis_pfn_tvalid,
    output wire                              s_axis_pfn_tready,
    input  wire [PFN_DATA_WIDTH-1:0]         s_axis_pfn_tdata,
    input  wire signed [COORD_WIDTH-1:0]     s_axis_pfn_voxel_x,
    input  wire signed [COORD_WIDTH-1:0]     s_axis_pfn_voxel_y,

    // Lane 0: to pfn_width_converter_0
    output wire                              m0_axis_pfn_tvalid,
    input  wire                              m0_axis_pfn_tready,
    output wire [PFN_DATA_WIDTH-1:0]         m0_axis_pfn_tdata,

    // Lane 0 metadata: to scatter/cmd path 0
    output wire                              m0_axis_meta_tvalid,
    input  wire                              m0_axis_meta_tready,
    output wire signed [COORD_WIDTH-1:0]     m0_axis_meta_voxel_x,
    output wire signed [COORD_WIDTH-1:0]     m0_axis_meta_voxel_y,

    // Lane 1: to pfn_width_converter_1
    output wire                              m1_axis_pfn_tvalid,
    input  wire                              m1_axis_pfn_tready,
    output wire [PFN_DATA_WIDTH-1:0]         m1_axis_pfn_tdata,

    // Lane 1 metadata: to scatter/cmd path 1
    output wire                              m1_axis_meta_tvalid,
    input  wire                              m1_axis_meta_tready,
    output wire signed [COORD_WIDTH-1:0]     m1_axis_meta_voxel_x,
    output wire signed [COORD_WIDTH-1:0]     m1_axis_meta_voxel_y,

    // Debug
    output wire                              route0_fire,
    output wire                              route1_fire,
    output reg                               last_route
);

    wire avail0 = m0_axis_pfn_tready && m0_axis_meta_tready;
    wire avail1 = m1_axis_pfn_tready && m1_axis_meta_tready;

    reg sel0;
    reg sel1;

    always @(*) begin
        sel0 = 1'b0;
        sel1 = 1'b0;

        // 两路都 ready 时轮询，避免长期偏向 lane0
        if (avail0 && avail1) begin
            if (last_route == 1'b0) begin
                sel1 = 1'b1;
            end else begin
                sel0 = 1'b1;
            end
        end else if (avail0) begin
            sel0 = 1'b1;
        end else if (avail1) begin
            sel1 = 1'b1;
        end
    end

    // PFN 只有在至少一路 converter + metadata path 都可接收时才 ready
    assign s_axis_pfn_tready = sel0 || sel1;

    assign route0_fire = s_axis_pfn_tvalid && sel0;
    assign route1_fire = s_axis_pfn_tvalid && sel1;

    // data 只发给被选中的 converter
    assign m0_axis_pfn_tvalid = route0_fire;
    assign m1_axis_pfn_tvalid = route1_fire;

    assign m0_axis_pfn_tdata  = s_axis_pfn_tdata;
    assign m1_axis_pfn_tdata  = s_axis_pfn_tdata;

    // metadata 和 data 同路由
    assign m0_axis_meta_tvalid  = route0_fire;
    assign m1_axis_meta_tvalid  = route1_fire;

    assign m0_axis_meta_voxel_x = s_axis_pfn_voxel_x;
    assign m0_axis_meta_voxel_y = s_axis_pfn_voxel_y;

    assign m1_axis_meta_voxel_x = s_axis_pfn_voxel_x;
    assign m1_axis_meta_voxel_y = s_axis_pfn_voxel_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_route <= 1'b1;  // reset 后第一包优先走 lane0
        end else begin
            if (route0_fire) begin
                last_route <= 1'b0;
            end else if (route1_fire) begin
                last_route <= 1'b1;
            end
        end
    end

endmodule