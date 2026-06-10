module pfn_width_converter #(
    parameter integer IN_WIDTH  = 1024,
    parameter integer OUT_WIDTH = 128
) (
    input wire clk,
    input wire rst_n,

    
    input  wire                s_axis_pfn_tvalid,
    output wire                s_axis_pfn_tready,
    input  wire [IN_WIDTH-1:0] s_axis_pfn_tdata,

    
    output reg                  m_axis_out_tvalid,
    input  wire                 m_axis_out_tready,
    output reg  [OUT_WIDTH-1:0] m_axis_out_tdata,
    output reg                  m_axis_out_tlast
);

    localparam integer NUM_PARTS = IN_WIDTH / OUT_WIDTH;  
    localparam integer CNT_WIDTH = $clog2(NUM_PARTS);

    
    reg [IN_WIDTH-1:0] data_buf;

    
    reg busy;

    
    reg [CNT_WIDTH-1:0] part_idx;

    
    
    wire last_beat_done = m_axis_out_tvalid && m_axis_out_tready && m_axis_out_tlast;

    
    
    assign s_axis_pfn_tready = (~busy) || last_beat_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_buf          <= {IN_WIDTH{1'b0}};
            busy              <= 1'b0;
            part_idx          <= {CNT_WIDTH{1'b0}};
            m_axis_out_tvalid <= 1'b0;
            m_axis_out_tdata  <= {OUT_WIDTH{1'b0}};
            m_axis_out_tlast  <= 1'b0;
        end else begin
            
            if (!busy || last_beat_done) begin
                if (s_axis_pfn_tvalid) begin
                    
                    data_buf          <= s_axis_pfn_tdata;
                    busy              <= 1'b1;
                    part_idx          <= {CNT_WIDTH{1'b0}};
                    m_axis_out_tvalid <= 1'b1;
                    m_axis_out_tdata  <= s_axis_pfn_tdata[0+:OUT_WIDTH];
                    m_axis_out_tlast  <= (NUM_PARTS == 1);
                end else begin
                    
                    busy              <= 1'b0;
                    m_axis_out_tvalid <= 1'b0;
                    m_axis_out_tlast  <= 1'b0;
                end
            end  
            else if (m_axis_out_tvalid && m_axis_out_tready) begin
                part_idx          <= part_idx + 1'b1;
                m_axis_out_tvalid <= 1'b1;
                m_axis_out_tdata  <= data_buf[(part_idx+1)*OUT_WIDTH+:OUT_WIDTH];
                m_axis_out_tlast  <= ((part_idx + 1) == NUM_PARTS - 1);
            end

            
        end
    end

endmodule
