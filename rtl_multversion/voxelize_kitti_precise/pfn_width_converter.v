module pfn_width_converter #(
    parameter integer IN_WIDTH  = 1024,
    parameter integer OUT_WIDTH = 128
) (
    input wire clk,
    input wire rst_n,

    // 上游输入：来自 PFN 的整包 768bit 数据
    input  wire                s_axis_pfn_tvalid,
    output wire                s_axis_pfn_tready,
    input  wire [IN_WIDTH-1:0] s_axis_pfn_tdata,

    // 下游输出：发给 AXIS / DataMover 的 128bit 数据流
    output reg                  m_axis_out_tvalid,
    input  wire                 m_axis_out_tready,
    output reg  [OUT_WIDTH-1:0] m_axis_out_tdata,
    output reg                  m_axis_out_tlast
);

    localparam integer NUM_PARTS = IN_WIDTH / OUT_WIDTH;  // 768/128 = 6
    localparam integer CNT_WIDTH = $clog2(NUM_PARTS);

    // 缓存整包 768bit
    reg [IN_WIDTH-1:0] data_buf;

    // busy=1 表示正在拆包输出
    reg busy;

    // 当前输出到第几段：0~5
    reg [CNT_WIDTH-1:0] part_idx;

    // 核心修改 1：提取“最后一拍完成”的判定标志
    // 条件：当前输出有效 且 下游准备好 且 是当前包的最后一段
    wire last_beat_done = m_axis_out_tvalid && m_axis_out_tready && m_axis_out_tlast;

    // 核心修改 2：Ready 拉高条件
    // 空闲时，或者上一个包在当前时钟周期刚好发完最后一段时，允许接收新数据
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
            // 场景 A：当前空闲，或者当前周期的最后一拍正在顺利握手
            if (!busy || last_beat_done) begin
                if (s_axis_pfn_tvalid) begin
                    // 立即无缝加载新的 768bit 数据并启动下一次拆分
                    data_buf          <= s_axis_pfn_tdata;
                    busy              <= 1'b1;
                    part_idx          <= {CNT_WIDTH{1'b0}};
                    m_axis_out_tvalid <= 1'b1;
                    m_axis_out_tdata  <= s_axis_pfn_tdata[0+:OUT_WIDTH];
                    m_axis_out_tlast  <= (NUM_PARTS == 1);
                end else begin
                    // 上游没给新数据，排空并进入闲置状态
                    busy              <= 1'b0;
                    m_axis_out_tvalid <= 1'b0;
                    m_axis_out_tlast  <= 1'b0;
                end
            end  // 场景 B：正在输出拆分后的 128bit 数据（且不是最后一段）
            else if (m_axis_out_tvalid && m_axis_out_tready) begin
                part_idx          <= part_idx + 1'b1;
                m_axis_out_tvalid <= 1'b1;
                m_axis_out_tdata  <= data_buf[(part_idx+1)*OUT_WIDTH+:OUT_WIDTH];
                m_axis_out_tlast  <= ((part_idx + 1) == NUM_PARTS - 1);
            end

            // 注：若 m_axis_out_tready 为 0（下游反压），逻辑保持原有状态停滞不动
        end
    end

endmodule
