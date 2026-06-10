`timescale 1 ns / 1 ps























module s2mm_clear_writer #(
    parameter integer AXIS_DATA_WIDTH = 128,
    parameter integer GRID_X          = 432,
    parameter integer GRID_Y          = 496,
    parameter integer BYTES_PER_VOX   = 128,
    parameter integer CLEAR_CHUNKS    = 4,
    parameter integer WAIT_FOR_STATUS = 1
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire                         clear_start,
    input  wire [31:0]                  base_addr,
    output reg                          clear_busy,
    output reg                          clear_done,
    output reg                          clear_error,

    
    output wire                         m_axis_cmd_tvalid,
    input  wire                         m_axis_cmd_tready,
    output wire [71:0]                  m_axis_cmd_tdata,

    
    output wire [AXIS_DATA_WIDTH-1:0]   m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire [AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output wire                         m_axis_tlast,

    
    
    input  wire [7:0]                   s_axis_sts_tdata,
    input  wire                         s_axis_sts_tvalid,
    output wire                         s_axis_sts_tready
);

    localparam integer AXIS_BYTES = AXIS_DATA_WIDTH / 8;

    localparam integer CLEAR_TOTAL_BYTES = GRID_X * GRID_Y * BYTES_PER_VOX;
    localparam integer CLEAR_CHUNK_BYTES = CLEAR_TOTAL_BYTES / CLEAR_CHUNKS;
    localparam integer CLEAR_BEATS       = CLEAR_CHUNK_BYTES / AXIS_BYTES;

    localparam [22:0] CLEAR_BTT = CLEAR_CHUNK_BYTES;

    localparam integer CHUNK_W = (CLEAR_CHUNKS <= 1) ? 1 : $clog2(CLEAR_CHUNKS);
    localparam integer BEAT_W  = (CLEAR_BEATS  <= 1) ? 1 : $clog2(CLEAR_BEATS);

    localparam [2:0] ST_IDLE     = 3'd0;
    localparam [2:0] ST_CMD      = 3'd1;
    localparam [2:0] ST_DATA     = 3'd2;
    localparam [2:0] ST_WAIT_STS = 3'd3;

    localparam [CHUNK_W-1:0] LAST_CHUNK = CLEAR_CHUNKS - 1;
    localparam [BEAT_W-1:0]  LAST_BEAT  = CLEAR_BEATS  - 1;

    reg [2:0]           state;
    reg [CHUNK_W-1:0]   chunk_idx;
    reg [BEAT_W-1:0]    beat_cnt;

    wire last_chunk = (chunk_idx == LAST_CHUNK);
    wire last_beat  = (beat_cnt  == LAST_BEAT);

    wire cmd_fire  = m_axis_cmd_tvalid && m_axis_cmd_tready;
    wire data_fire = m_axis_tvalid     && m_axis_tready;
    wire sts_fire  = s_axis_sts_tvalid && s_axis_sts_tready;

    
    
    
    wire [31:0] chunk_offset = chunk_idx * CLEAR_CHUNK_BYTES;
    wire [31:0] clear_addr   = base_addr + chunk_offset;

    assign m_axis_cmd_tvalid = (state == ST_CMD);

    assign m_axis_cmd_tdata = {
        4'b0000,       
        4'b0000,       
        clear_addr,    
        1'b0,          
        1'b1,          
        6'b000000,     
        1'b1,          
        CLEAR_BTT      
    };

    assign m_axis_tvalid = (state == ST_DATA);
    assign m_axis_tdata  = {AXIS_DATA_WIDTH{1'b0}};
    assign m_axis_tkeep  = {(AXIS_DATA_WIDTH/8){1'b1}};
    assign m_axis_tlast  = (state == ST_DATA) && last_beat;

    assign s_axis_sts_tready = (state == ST_WAIT_STS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            chunk_idx   <= {CHUNK_W{1'b0}};
            beat_cnt    <= {BEAT_W{1'b0}};
            clear_busy  <= 1'b0;
            clear_done  <= 1'b0;
            clear_error <= 1'b0;
        end else begin
            clear_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    clear_busy <= 1'b0;
                    beat_cnt   <= {BEAT_W{1'b0}};
                    chunk_idx  <= {CHUNK_W{1'b0}};

                    if (clear_start) begin
                        clear_busy  <= 1'b1;
                        clear_error <= 1'b0;
                        state       <= ST_CMD;
                    end
                end

                ST_CMD: begin
                    clear_busy <= 1'b1;
                    if (cmd_fire) begin
                        beat_cnt <= {BEAT_W{1'b0}};
                        state    <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    clear_busy <= 1'b1;
                    if (data_fire) begin
                        if (last_beat) begin
                            beat_cnt <= {BEAT_W{1'b0}};
                            if (WAIT_FOR_STATUS != 0) begin
                                state <= ST_WAIT_STS;
                            end else begin
                                if (last_chunk) begin
                                    state      <= ST_IDLE;
                                    clear_busy <= 1'b0;
                                    clear_done <= 1'b1;
                                end else begin
                                    chunk_idx <= chunk_idx + 1'b1;
                                    state     <= ST_CMD;
                                end
                            end
                        end else begin
                            beat_cnt <= beat_cnt + 1'b1;
                        end
                    end
                end

                ST_WAIT_STS: begin
                    clear_busy <= 1'b1;
                    if (sts_fire) begin
                        
                        
                        if ((s_axis_sts_tdata[7] == 1'b0) || (|s_axis_sts_tdata[6:4])) begin
                            clear_error <= 1'b1;
                        end

                        if (last_chunk) begin
                            state      <= ST_IDLE;
                            clear_busy <= 1'b0;
                            clear_done <= 1'b1;
                        end else begin
                            chunk_idx <= chunk_idx + 1'b1;
                            state     <= ST_CMD;
                        end
                    end
                end

                default: begin
                    state      <= ST_IDLE;
                    clear_busy <= 1'b0;
                end
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (AXIS_DATA_WIDTH % 8 != 0) begin
            $error("AXIS_DATA_WIDTH must be byte aligned");
        end
        if (CLEAR_TOTAL_BYTES % CLEAR_CHUNKS != 0) begin
            $error("CLEAR_TOTAL_BYTES must be divisible by CLEAR_CHUNKS");
        end
        if (CLEAR_CHUNK_BYTES % AXIS_BYTES != 0) begin
            $error("CLEAR_CHUNK_BYTES must be divisible by AXIS_BYTES");
        end
        if (CLEAR_CHUNK_BYTES > 8388607) begin
            $error("CLEAR_CHUNK_BYTES exceeds AXI DataMover 23-bit BTT maximum");
        end
    end
`endif

endmodule
