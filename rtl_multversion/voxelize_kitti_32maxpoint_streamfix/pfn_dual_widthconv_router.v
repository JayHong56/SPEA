`timescale 1ps / 1ps

module pfn_dual_widthconv_router #(
    parameter integer PFN_DATA_WIDTH = 128,
    parameter integer COORD_WIDTH    = 11
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // Upstream: packetized PFN stream. One voxel is one packet.
    input  wire                              s_axis_pfn_tvalid,
    output wire                              s_axis_pfn_tready,
    input  wire [PFN_DATA_WIDTH-1:0]         s_axis_pfn_tdata,
    input  wire                              s_axis_pfn_tlast,
    input  wire signed [COORD_WIDTH-1:0]     s_axis_pfn_voxel_x,
    input  wire signed [COORD_WIDTH-1:0]     s_axis_pfn_voxel_y,

    // Lane 0: packetized PFN stream.
    output wire                              m0_axis_pfn_tvalid,
    input  wire                              m0_axis_pfn_tready,
    output wire [PFN_DATA_WIDTH-1:0]         m0_axis_pfn_tdata,
    output wire                              m0_axis_pfn_tlast,

    // Lane 0 metadata. Emitted once on the first beat of each packet.
    output wire                              m0_axis_meta_tvalid,
    input  wire                              m0_axis_meta_tready,
    output wire signed [COORD_WIDTH-1:0]     m0_axis_meta_voxel_x,
    output wire signed [COORD_WIDTH-1:0]     m0_axis_meta_voxel_y,

    // Lane 1: packetized PFN stream.
    output wire                              m1_axis_pfn_tvalid,
    input  wire                              m1_axis_pfn_tready,
    output wire [PFN_DATA_WIDTH-1:0]         m1_axis_pfn_tdata,
    output wire                              m1_axis_pfn_tlast,

    // Lane 1 metadata. Emitted once on the first beat of each packet.
    output wire                              m1_axis_meta_tvalid,
    input  wire                              m1_axis_meta_tready,
    output wire signed [COORD_WIDTH-1:0]     m1_axis_meta_voxel_x,
    output wire signed [COORD_WIDTH-1:0]     m1_axis_meta_voxel_y,

    // Debug.
    output wire                              route0_fire,
    output wire                              route1_fire,
    output reg                               last_route
);

    reg in_packet;
    reg packet_lane;  // 0: lane0, 1: lane1

    wire idle = !in_packet;

    wire avail0_first = m0_axis_pfn_tready && m0_axis_meta_tready;
    wire avail1_first = m1_axis_pfn_tready && m1_axis_meta_tready;

    reg first_sel0;
    reg first_sel1;

    always @(*) begin
        first_sel0 = 1'b0;
        first_sel1 = 1'b0;

        if (avail0_first && avail1_first) begin
            if (last_route == 1'b0) begin
                first_sel1 = 1'b1;
            end else begin
                first_sel0 = 1'b1;
            end
        end else if (avail0_first) begin
            first_sel0 = 1'b1;
        end else if (avail1_first) begin
            first_sel1 = 1'b1;
        end
    end

    wire sel0 = idle ? first_sel0 : (packet_lane == 1'b0);
    wire sel1 = idle ? first_sel1 : (packet_lane == 1'b1);

    wire selected_ready =
        idle ? ((first_sel0 && avail0_first) || (first_sel1 && avail1_first)) :
        ((packet_lane == 1'b0) ? m0_axis_pfn_tready : m1_axis_pfn_tready);

    assign s_axis_pfn_tready = selected_ready;

    assign m0_axis_pfn_tvalid = s_axis_pfn_tvalid && sel0;
    assign m1_axis_pfn_tvalid = s_axis_pfn_tvalid && sel1;

    assign m0_axis_pfn_tdata = s_axis_pfn_tdata;
    assign m1_axis_pfn_tdata = s_axis_pfn_tdata;

    assign m0_axis_pfn_tlast = s_axis_pfn_tlast;
    assign m1_axis_pfn_tlast = s_axis_pfn_tlast;

    assign m0_axis_meta_tvalid = s_axis_pfn_tvalid && idle && first_sel0;
    assign m1_axis_meta_tvalid = s_axis_pfn_tvalid && idle && first_sel1;

    assign m0_axis_meta_voxel_x = s_axis_pfn_voxel_x;
    assign m0_axis_meta_voxel_y = s_axis_pfn_voxel_y;

    assign m1_axis_meta_voxel_x = s_axis_pfn_voxel_x;
    assign m1_axis_meta_voxel_y = s_axis_pfn_voxel_y;

    assign route0_fire = s_axis_pfn_tvalid && s_axis_pfn_tready && sel0;
    assign route1_fire = s_axis_pfn_tvalid && s_axis_pfn_tready && sel1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_packet   <= 1'b0;
            packet_lane <= 1'b0;
            last_route  <= 1'b1;  // First packet after reset prefers lane0.
        end else if (s_axis_pfn_tvalid && s_axis_pfn_tready) begin
            if (idle) begin
                packet_lane <= first_sel1;
                if (first_sel0) begin
                    last_route <= 1'b0;
                end else if (first_sel1) begin
                    last_route <= 1'b1;
                end
            end

            in_packet <= !s_axis_pfn_tlast;
        end
    end

endmodule
