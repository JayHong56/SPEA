`timescale 1 ns / 1 ps



















module s2mm_clear_lane0_adapter #(
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer GRID_X          = 432,
    parameter integer GRID_Y          = 496,
    parameter integer BYTES_PER_VOX   = 128,
    parameter integer CLEAR_CHUNKS    = 4
) (
    input wire aclk,
    input wire aresetn,

    input  wire                         clear_start,
    input  wire [                 31:0] clear_base_addr,
    output wire                         clear_busy,
    output wire                         clear_done,
    output wire                         clear_error,
    
    input  wire                         normal_cmd_tvalid,
    output wire                         normal_cmd_tready,
    input  wire [                 71:0] normal_cmd_tdata,
    
    input  wire [  AXIS_DATA_WIDTH-1:0] normal_axis_tdata,
    input  wire                         normal_axis_tvalid,
    output wire                         normal_axis_tready,
    input  wire [AXIS_DATA_WIDTH/8-1:0] normal_axis_tkeep,
    input  wire                         normal_axis_tlast,
    
    output wire                         dm_s2mm_cmd_tvalid,
    input  wire                         dm_s2mm_cmd_tready,
    output wire [                 71:0] dm_s2mm_cmd_tdata,
    
    output wire [  AXIS_DATA_WIDTH-1:0] dm_s2mm_tdata,
    output wire                         dm_s2mm_tvalid,
    input  wire                         dm_s2mm_tready,
    output wire [AXIS_DATA_WIDTH/8-1:0] dm_s2mm_tkeep,
    output wire                         dm_s2mm_tlast,
    
    input  wire [                  7:0] dm_s2mm_sts_tdata,
    input  wire                         dm_s2mm_sts_tvalid,
    output wire                         dm_s2mm_sts_tready,

    
    output wire                         normal_path_idle,
    output reg                          normal_status_error,
    output reg  [                 15:0] normal_outstanding_count
);

    wire                         clear_cmd_tvalid;
    wire                         clear_cmd_tready;
    wire [                 71:0] clear_cmd_tdata;
    wire [  AXIS_DATA_WIDTH-1:0] clear_tdata;
    wire                         clear_tvalid;
    wire                         clear_tready;
    wire [AXIS_DATA_WIDTH/8-1:0] clear_tkeep;
    wire                         clear_tlast;
    wire [                  7:0] clear_sts_tdata;
    wire                         clear_sts_tvalid;
    wire                         clear_sts_tready;

    
    
    wire [                  7:0] normal_sts_tdata;
    wire                         normal_sts_tvalid;
    wire                         normal_sts_tready = 1'b1;

    wire normal_cmd_fire = normal_cmd_tvalid && normal_cmd_tready;
    wire normal_sts_fire = normal_sts_tvalid && normal_sts_tready;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            normal_outstanding_count <= 16'd0;
            normal_status_error      <= 1'b0;
        end else begin
            case ({normal_cmd_fire, normal_sts_fire})
                2'b10: begin
                    if (normal_outstanding_count != 16'hFFFF)
                        normal_outstanding_count <= normal_outstanding_count + 1'b1;
                    else
                        normal_status_error <= 1'b1;
                end
                2'b01: begin
                    if (normal_outstanding_count != 16'd0)
                        normal_outstanding_count <= normal_outstanding_count - 1'b1;
                    else
                        normal_status_error <= 1'b1;
                end
                default: begin
                    normal_outstanding_count <= normal_outstanding_count;
                end
            endcase

            if (normal_sts_fire) begin
                if ((normal_sts_tdata[7] == 1'b0) || (|normal_sts_tdata[6:4]))
                    normal_status_error <= 1'b1;
            end
        end
    end

    assign normal_path_idle = !clear_busy &&
                              (normal_outstanding_count == 16'd0) &&
                              !normal_cmd_tvalid &&
                              !normal_axis_tvalid &&
                              !normal_sts_tvalid;

    s2mm_clear_writer #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .GRID_X         (GRID_X),
        .GRID_Y         (GRID_Y),
        .BYTES_PER_VOX  (BYTES_PER_VOX),
        .CLEAR_CHUNKS   (CLEAR_CHUNKS),
        .WAIT_FOR_STATUS(1)
    ) u_s2mm_clear_writer (
        .clk              (aclk),
        .rst_n            (aresetn),
        .clear_start      (clear_start),
        .base_addr        (clear_base_addr),
        .clear_busy       (clear_busy),
        .clear_done       (clear_done),
        .clear_error      (clear_error),
        .m_axis_cmd_tvalid(clear_cmd_tvalid),
        .m_axis_cmd_tready(clear_cmd_tready),
        .m_axis_cmd_tdata (clear_cmd_tdata),
        .m_axis_tdata     (clear_tdata),
        .m_axis_tvalid    (clear_tvalid),
        .m_axis_tready    (clear_tready),
        .m_axis_tkeep     (clear_tkeep),
        .m_axis_tlast     (clear_tlast),
        .s_axis_sts_tdata (clear_sts_tdata),
        .s_axis_sts_tvalid(clear_sts_tvalid),
        .s_axis_sts_tready(clear_sts_tready)
    );

    s2mm_clear_mux_1dm #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH)
    ) u_s2mm_clear_mux_1dm (
        .clear_busy       (clear_busy),
        .clear_cmd_tvalid (clear_cmd_tvalid),
        .clear_cmd_tready (clear_cmd_tready),
        .clear_cmd_tdata  (clear_cmd_tdata),
        .clear_tdata      (clear_tdata),
        .clear_tvalid     (clear_tvalid),
        .clear_tready     (clear_tready),
        .clear_tkeep      (clear_tkeep),
        .clear_tlast      (clear_tlast),
        .clear_sts_tdata  (clear_sts_tdata),
        .clear_sts_tvalid (clear_sts_tvalid),
        .clear_sts_tready (clear_sts_tready),
        .normal_cmd_tvalid(normal_cmd_tvalid),
        .normal_cmd_tready(normal_cmd_tready),
        .normal_cmd_tdata (normal_cmd_tdata),
        .normal_tdata     (normal_axis_tdata),
        .normal_tvalid    (normal_axis_tvalid),
        .normal_tready    (normal_axis_tready),
        .normal_tkeep     (normal_axis_tkeep),
        .normal_tlast     (normal_axis_tlast),
        .normal_sts_tdata (normal_sts_tdata),
        .normal_sts_tvalid(normal_sts_tvalid),
        .normal_sts_tready(normal_sts_tready),
        .dm_cmd_tvalid    (dm_s2mm_cmd_tvalid),
        .dm_cmd_tready    (dm_s2mm_cmd_tready),
        .dm_cmd_tdata     (dm_s2mm_cmd_tdata),
        .dm_tdata         (dm_s2mm_tdata),
        .dm_tvalid        (dm_s2mm_tvalid),
        .dm_tready        (dm_s2mm_tready),
        .dm_tkeep         (dm_s2mm_tkeep),
        .dm_tlast         (dm_s2mm_tlast),
        .dm_sts_tdata     (dm_s2mm_sts_tdata),
        .dm_sts_tvalid    (dm_s2mm_sts_tvalid),
        .dm_sts_tready    (dm_s2mm_sts_tready)
    );

endmodule
