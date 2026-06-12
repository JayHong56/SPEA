`timescale 1ps / 1ps

module sim_mm2s_datamover #(
    parameter DATA_WIDTH = 128,
    parameter FILE_PATH  = "E:\\mmdetection3d\\data\\kitti\\scripts\\data\\points_sim_bin.txt",
    
    parameter MAX_MEM_DEPTH = 150000 
)(
    input  wire                  aclk,
    input  wire                  aresetn,
    
    
    input  wire                  s_axis_cmd_tvalid,
    output wire                  s_axis_cmd_tready,
    input  wire [71:0]           s_axis_cmd_tdata,

    
    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [DATA_WIDTH/8-1:0] m_axis_tkeep,
    output wire                  m_axis_tlast
);

    
    reg [DATA_WIDTH-1:0] mem [0:MAX_MEM_DEPTH-1];
    
    initial begin
        
        $readmemb(FILE_PATH, mem);
        $display("[SIM] Point cloud binary data loaded from %s", FILE_PATH);
    end

    
    localparam IDLE = 0, SEND_DATA = 1;
    reg [1:0] state;
    
    reg [31:0] read_ptr;
    reg [31:0] beats_to_send;
    reg [31:0] current_beat;

    
    assign s_axis_cmd_tready = (state == IDLE);
    assign m_axis_tvalid     = (state == SEND_DATA);
    assign m_axis_tdata      = mem[read_ptr];
    assign m_axis_tkeep      = {(DATA_WIDTH/8){1'b1}};
    assign m_axis_tlast      = (state == SEND_DATA) && (current_beat == beats_to_send - 1);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= IDLE;
            read_ptr      <= 0;
            beats_to_send <= 0;
            current_beat  <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (s_axis_cmd_tvalid && s_axis_cmd_tready) begin
                        beats_to_send <= s_axis_cmd_tdata[22:0] / (DATA_WIDTH/8);
                        current_beat  <= 0;
                        read_ptr      <= 0; 
                        state         <= SEND_DATA;
                        $display("[%0t] [DataMover SIM] Received CMD. BTT: %0d bytes, Beats: %0d", 
                                 $time, s_axis_cmd_tdata[22:0], s_axis_cmd_tdata[22:0] / (DATA_WIDTH/8));
                    end
                end

                SEND_DATA: begin
                    
                    if (m_axis_tvalid && m_axis_tready) begin
                        if (current_beat == beats_to_send - 1) begin
                            
                            state <= IDLE;
                            $display("[%0t] [DataMover SIM] Data transmission complete! Sent %0d beats (TLAST triggered).", $time, current_beat + 1);
                        end else begin
                            read_ptr     <= read_ptr + 1;
                            current_beat <= current_beat + 1;
                        end
                    end
                end
            endcase
        end
    end
endmodule