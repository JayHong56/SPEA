`timescale 1ns / 1ps








module hash_bucket_lutram_1r1w #(
    parameter integer WIDTH            = 64,
    parameter integer DEPTH            = 64,
    parameter integer AW               = $clog2(DEPTH),

    
    
    parameter integer FORWARD_ON_WRITE = 0
) (
    input  wire             clk,
    input  wire             rst_n,

    
    input  wire [AW-1:0]    r_addr,
    input  wire             r_en,
    output reg  [WIDTH-1:0] r_data,

    
    input  wire [AW-1:0]    w_addr,
    input  wire             w_en,
    input  wire [WIDTH-1:0] w_data
);

    
    (* ram_style = "distributed" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    integer i;

`ifndef SYNTHESIS
    initial begin
        r_data = {WIDTH{1'b0}};
        for (i = 0; i < DEPTH; i = i + 1) begin
            mem[i] = {WIDTH{1'b0}};
        end
    end
`endif

    always @(posedge clk) begin
        if (!rst_n) begin
            r_data <= {WIDTH{1'b0}};
        end else begin
            
            if (r_en) begin
                if (FORWARD_ON_WRITE && w_en && (w_addr == r_addr)) begin
                    r_data <= w_data;
                end else begin
                    r_data <= mem[r_addr];
                end
            end

            
            if (w_en) begin
                mem[w_addr] <= w_data;
            end
        end
    end

endmodule















module hash_bucket_bram #(
    parameter integer WIDTH     = 64,
    parameter integer DEPTH     = 64,
    parameter integer AW        = $clog2(DEPTH),
    parameter         NUM_BYTES = WIDTH / 8
) (
    input wire clk,
    input wire rst_n,

    
    input  wire [AW-1:0]    a_addr,
    input  wire             a_en,
    output wire [WIDTH-1:0] a_rdata,

    
    input  wire [AW-1:0]    b_addr,
    input  wire             b_we,
    input  wire [WIDTH-1:0] b_wdata
);
    hash_bucket_lutram_1r1w #(
        .WIDTH           (WIDTH),
        .DEPTH           (DEPTH),
        .AW              (AW),

        
        
        .FORWARD_ON_WRITE(0)
    ) u_lutram (
        .clk   (clk),
        .rst_n (rst_n),

        .r_addr(a_addr),
        .r_en  (a_en),
        .r_data(a_rdata),

        .w_addr(b_addr),
        .w_en  (b_we),
        .w_data(b_wdata)
    );

endmodule