`timescale 1 ns / 1 ps









module s2mm_normal_drain_monitor #(
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer OUTSTANDING_WIDTH = 16
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         s_cmd_tvalid,
    output wire                         s_cmd_tready,
    input  wire [71:0]                  s_cmd_tdata,

    input  wire [AXIS_DATA_WIDTH-1:0]   s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire [AXIS_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tlast,

    output wire                         m_cmd_tvalid,
    input  wire                         m_cmd_tready,
    output wire [71:0]                  m_cmd_tdata,

    output wire [AXIS_DATA_WIDTH-1:0]   m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output wire                         m_axis_tlast,

    input  wire [7:0]                   s_sts_tdata,
    input  wire                         s_sts_tvalid,
    output wire                         s_sts_tready,

    output wire                         lane_idle,
    output reg                          lane_error,
    output reg [OUTSTANDING_WIDTH-1:0]  outstanding_count
);

    wire cmd_fire = s_cmd_tvalid && s_cmd_tready;
    wire sts_fire = s_sts_tvalid && s_sts_tready;

    assign m_cmd_tvalid  = s_cmd_tvalid;
    assign s_cmd_tready  = m_cmd_tready;
    assign m_cmd_tdata   = s_cmd_tdata;

    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tlast  = s_axis_tlast;

    assign s_sts_tready = 1'b1;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            outstanding_count <= {OUTSTANDING_WIDTH{1'b0}};
            lane_error        <= 1'b0;
        end else begin
            case ({cmd_fire, sts_fire})
                2'b10: begin
                    if (outstanding_count != {OUTSTANDING_WIDTH{1'b1}})
                        outstanding_count <= outstanding_count + 1'b1;
                    else
                        lane_error <= 1'b1;
                end
                2'b01: begin
                    if (outstanding_count != {OUTSTANDING_WIDTH{1'b0}})
                        outstanding_count <= outstanding_count - 1'b1;
                    else
                        lane_error <= 1'b1;
                end
                default: begin
                    outstanding_count <= outstanding_count;
                end
            endcase

            if (sts_fire) begin
                if ((s_sts_tdata[7] == 1'b0) || (|s_sts_tdata[6:4]))
                    lane_error <= 1'b1;
            end
        end
    end

    assign lane_idle = (outstanding_count == {OUTSTANDING_WIDTH{1'b0}}) &&
                       !s_cmd_tvalid &&
                       !s_axis_tvalid &&
                       !s_sts_tvalid;

endmodule
