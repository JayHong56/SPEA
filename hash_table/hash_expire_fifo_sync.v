module hash_expire_fifo_sync #(
    parameter integer WIDTH = 32,
    parameter integer DEPTH = 16
) (
    input wire clk,
    input wire rst_n,

    
    input  wire             in_valid,
    output wire             in_ready,  
    input  wire [WIDTH-1:0] in_data,

    
    output wire             out_valid,
    input  wire             out_ready,
    output wire [WIDTH-1:0] out_data,

    output wire [$clog2(DEPTH+1)-1:0] level
);
    localparam integer AW = $clog2(DEPTH);

    
    (* ram_style = "distributed" *) reg [WIDTH-1:0] mem[0:DEPTH-1];

    reg [AW-1:0] wptr, rptr;
    reg [$clog2(DEPTH+1)-1:0] count;

    assign level    = count;
    assign in_ready = (count != DEPTH);
    assign out_valid= (count != 0);

    
    assign out_data = mem[rptr];

    wire push = in_valid & in_ready;
    wire pop = out_valid & out_ready;

    
    
    
    always @(posedge clk) begin
        if (push) begin
            mem[wptr] <= in_data;
        end
    end

    
    
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {($clog2(DEPTH + 1)) {1'b0}};
        end else begin
            
            if (push) begin
                wptr <= (wptr == DEPTH - 1) ? {AW{1'b0}} : (wptr + 1'b1);
            end

            
            if (pop) begin
                rptr <= (rptr == DEPTH - 1) ? {AW{1'b0}} : (rptr + 1'b1);
            end

            
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
