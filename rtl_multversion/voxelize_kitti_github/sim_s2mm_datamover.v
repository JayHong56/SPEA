`timescale 1ps / 1ps

module sim_s2mm_datamover #(
    parameter DATA_WIDTH = 128
)(
    input  wire                    aclk,
    input  wire                    aresetn,
    
    
    input  wire                    s_axis_cmd_tvalid,
    output wire                    s_axis_cmd_tready,
    input  wire [71:0]             s_axis_cmd_tdata,
    
    
    input  wire [DATA_WIDTH-1:0]   s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output wire                    s_axis_tready,
    input  wire [DATA_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                    s_axis_tlast,
    
    
    output reg  [7:0]              m_axis_sts_tdata,
    output reg                     m_axis_sts_tvalid,
    input  wire                    m_axis_sts_tready
);

    localparam IDLE = 0, SINK = 1, STS = 2;
    reg [1:0] state;
    
    assign s_axis_cmd_tready = (state == IDLE);
    assign s_axis_tready     = (state == SINK);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= IDLE;
            m_axis_sts_tvalid <= 1'b0;
            m_axis_sts_tdata  <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    m_axis_sts_tvalid <= 1'b0;
                    if (s_axis_cmd_tvalid && s_axis_cmd_tready) begin
                        state <= SINK;
                        $display("[%0t] [S2MM SIM] Received Write Command. Addr: %h, Bytes: %0d", 
                                 $time, s_axis_cmd_tdata[63:32], s_axis_cmd_tdata[22:0]);
                    end
                end
                
                SINK: begin
                    if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                        state <= STS;
                        m_axis_sts_tdata  <= 8'h80; 
                        m_axis_sts_tvalid <= 1'b1;
                        $display("[%0t] [S2MM SIM] Write Data Sink Complete (TLAST). Sending Status...", $time);
                    end
                end
                
                STS: begin
                    if (m_axis_sts_tvalid && m_axis_sts_tready) begin
                        m_axis_sts_tvalid <= 1'b0;
                        state <= IDLE;
                        $display("[%0t] [S2MM SIM] Status accepted by IP.", $time);
                    end
                end
            endcase
        end
    end
endmodule