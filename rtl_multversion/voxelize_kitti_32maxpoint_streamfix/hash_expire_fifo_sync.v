module hash_expire_fifo_sync #(
    parameter integer WIDTH = 32,
    parameter integer DEPTH = 16
) (
    input wire clk,
    input wire rst_n,

    // push side
    input  wire             in_valid,
    output wire             in_ready,  // 反压
    input  wire [WIDTH-1:0] in_data,

    // pop side
    output wire             out_valid,
    input  wire             out_ready,
    output wire [WIDTH-1:0] out_data,

    output wire [$clog2(DEPTH+1)-1:0] level
);
    localparam integer AW = $clog2(DEPTH);

    // 强烈建议加上这个原语，明确告诉综合器使用分布式 RAM (LUTRAM)
    (* ram_style = "distributed" *) reg [WIDTH-1:0] mem[0:DEPTH-1];

    reg [AW-1:0] wptr, rptr;
    reg [$clog2(DEPTH+1)-1:0] count;

    assign level    = count;
    assign in_ready = (count != DEPTH);
    assign out_valid= (count != 0);

    // LUTRAM 特有的异步读出，完美匹配
    assign out_data = mem[rptr];

    wire push = in_valid & in_ready;
    wire pop = out_valid & out_ready;

    // =========================================================
    // 【核心修改 1】：纯净的 RAM 写入块，敏感列表里绝对没有 rst_n
    // =========================================================
    always @(posedge clk) begin
        if (push) begin
            mem[wptr] <= in_data;
        end
    end

    // =========================================================
    // 【核心修改 2】：独立的控制流块，专门管理指针和状态
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {($clog2(DEPTH + 1)) {1'b0}};
        end else begin
            // 写指针更新
            if (push) begin
                wptr <= (wptr == DEPTH - 1) ? {AW{1'b0}} : (wptr + 1'b1);
            end

            // 读指针更新
            if (pop) begin
                rptr <= (rptr == DEPTH - 1) ? {AW{1'b0}} : (rptr + 1'b1);
            end

            // count update
            case ({
                push, pop
            })
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
