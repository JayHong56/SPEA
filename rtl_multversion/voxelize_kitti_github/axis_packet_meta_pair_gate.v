`timescale 1ps / 1ps

module axis_packet_meta_pair_gate #(
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer CMD_WIDTH       = 72
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    
    input  wire [CMD_WIDTH-1:0]         s_cmd_tdata,
    input  wire                         s_cmd_tvalid,
    output wire                         s_cmd_tready,

    
    input  wire [AXIS_DATA_WIDTH-1:0]   s_axis_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tlast,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,

    
    output wire [CMD_WIDTH-1:0]         m_cmd_tdata,
    output wire                         m_cmd_tvalid,
    input  wire                         m_cmd_tready,

    
    output wire [AXIS_DATA_WIDTH-1:0]   m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output wire                         m_axis_tlast,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,

    
    output wire                         packet_active,
    output wire                         first_beat_wait_cmd,
    output wire                         first_beat_wait_cmd_fifo,
    output wire                         first_beat_wait_data_fifo,
    output reg  [15:0]                  cmd_packet_count,
    output reg  [15:0]                  data_packet_count,
    output wire                         pair_count_mismatch
);

    reg in_packet;

    wire first_beat = !in_packet;

    







    wire first_beat_can_fire =
        s_axis_tvalid &&
        s_cmd_tvalid &&
        m_cmd_tready &&
        m_axis_tready;

    assign s_axis_tready = first_beat ? 
                           (s_cmd_tvalid && m_cmd_tready && m_axis_tready) :
                           m_axis_tready;

    


    assign s_cmd_tready = first_beat &&
                          s_axis_tvalid &&
                          m_cmd_tready &&
                          m_axis_tready;

    assign m_cmd_tdata  = s_cmd_tdata;
    assign m_cmd_tvalid = first_beat &&
                          s_axis_tvalid &&
                          s_cmd_tvalid &&
                          m_axis_tready;

    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tlast  = s_axis_tlast;

    assign m_axis_tvalid = s_axis_tvalid &&
                           (first_beat ? (s_cmd_tvalid && m_cmd_tready) : 1'b1);

    assign packet_active = in_packet;

    assign first_beat_wait_cmd =
        s_axis_tvalid && first_beat && !s_cmd_tvalid;

    assign first_beat_wait_cmd_fifo =
        s_axis_tvalid && first_beat && s_cmd_tvalid && !m_cmd_tready;

    assign first_beat_wait_data_fifo =
        s_axis_tvalid && first_beat && s_cmd_tvalid && m_cmd_tready && !m_axis_tready;

    assign pair_count_mismatch = (cmd_packet_count != data_packet_count);

    wire data_fire = s_axis_tvalid && s_axis_tready;
    wire cmd_fire  = s_cmd_tvalid && s_cmd_tready;

    wire first_data_fire = data_fire && first_beat;
    wire last_data_fire  = data_fire && s_axis_tlast;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            in_packet         <= 1'b0;
            cmd_packet_count  <= 16'd0;
            data_packet_count <= 16'd0;
        end else begin
            if (data_fire) begin
                if (first_beat && !s_axis_tlast) begin
                    in_packet <= 1'b1;
                end else if (s_axis_tlast) begin
                    in_packet <= 1'b0;
                end
            end

            if (cmd_fire) begin
                cmd_packet_count <= cmd_packet_count + 16'd1;
            end

            if (last_data_fire) begin
                data_packet_count <= data_packet_count + 16'd1;
            end
        end
    end

endmodule