`timescale 1ps / 1ps

module pfn_multi_widthconv_router #(
    parameter integer NUM_LANES      = 3,
    parameter integer PFN_DATA_WIDTH = 1024,
    parameter integer COORD_WIDTH    = 11,
    parameter integer SEL_WIDTH      = (NUM_LANES <= 2) ? 1 : $clog2(NUM_LANES)
) (
    input  wire                                      clk,
    input  wire                                      rst_n,

    input  wire                                      s_axis_pfn_tvalid,
    output wire                                      s_axis_pfn_tready,
    input  wire [PFN_DATA_WIDTH-1:0]                 s_axis_pfn_tdata,
    input  wire signed [COORD_WIDTH-1:0]             s_axis_pfn_voxel_x,
    input  wire signed [COORD_WIDTH-1:0]             s_axis_pfn_voxel_y,

    output wire [NUM_LANES-1:0]                      m_axis_pfn_tvalid,
    input  wire [NUM_LANES-1:0]                      m_axis_pfn_tready,
    output wire [NUM_LANES*PFN_DATA_WIDTH-1:0]       m_axis_pfn_tdata,

    output wire [NUM_LANES-1:0]                      m_axis_meta_tvalid,
    input  wire [NUM_LANES-1:0]                      m_axis_meta_tready,
    output wire [NUM_LANES*COORD_WIDTH-1:0]          m_axis_meta_voxel_x,
    output wire [NUM_LANES*COORD_WIDTH-1:0]          m_axis_meta_voxel_y,

    output wire [NUM_LANES-1:0]                      route_fire,
    output reg  [SEL_WIDTH-1:0]                      last_route
);

    wire [NUM_LANES-1:0] lane_ready = m_axis_pfn_tready & m_axis_meta_tready;

    reg [NUM_LANES-1:0] sel;
    reg                 found;
    integer             offset;
    integer             idx;
    integer             lane;

    always @(*) begin
        sel   = {NUM_LANES{1'b0}};
        found = 1'b0;

        for (offset = 1; offset <= NUM_LANES; offset = offset + 1) begin
            idx = last_route + offset;
            if (idx >= NUM_LANES) begin
                idx = idx - NUM_LANES;
            end

            if (!found && lane_ready[idx]) begin
                sel[idx] = 1'b1;
                found    = 1'b1;
            end
        end
    end

    assign s_axis_pfn_tready  = |sel;
    assign route_fire         = {NUM_LANES{s_axis_pfn_tvalid}} & sel;
    assign m_axis_pfn_tvalid  = route_fire;
    assign m_axis_meta_tvalid = route_fire;

    genvar g;
    generate
        for (g = 0; g < NUM_LANES; g = g + 1) begin : gen_lane_outputs
            assign m_axis_pfn_tdata[g*PFN_DATA_WIDTH +: PFN_DATA_WIDTH] = s_axis_pfn_tdata;
            assign m_axis_meta_voxel_x[g*COORD_WIDTH +: COORD_WIDTH]    = s_axis_pfn_voxel_x;
            assign m_axis_meta_voxel_y[g*COORD_WIDTH +: COORD_WIDTH]    = s_axis_pfn_voxel_y;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_route <= NUM_LANES - 1;
        end else begin
            for (lane = 0; lane < NUM_LANES; lane = lane + 1) begin
                if (route_fire[lane]) begin
                    last_route <= lane[SEL_WIDTH-1:0];
                end
            end
        end
    end

endmodule
