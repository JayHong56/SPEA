`timescale 1ps / 1ps

module sim_axis_receiver #(
    parameter DATA_WIDTH = 128,
    parameter PORT_NAME  = "PORT_0",
    parameter DUMP_FILE  = "output_dump.txt" 
)(
    input  wire                  aclk,
    input  wire                  aresetn,
    
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,
    
    input  wire [10:0]           s_axis_voxel_x,
    input  wire [10:0]           s_axis_voxel_y,
    input  wire                  s_axis_voxel_valid
);

    
    assign s_axis_tready = 1'b1; 
    
    integer fd;
    initial begin
        fd = $fopen(DUMP_FILE, "w");
    end

    always @(posedge aclk) begin
        if (aresetn && s_axis_tvalid && s_axis_tready) begin
            
            
            
            
            
            
            
        end
    end
endmodule