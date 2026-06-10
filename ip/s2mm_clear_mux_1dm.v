`timescale 1 ns / 1 ps













module s2mm_clear_mux_1dm #(
    parameter integer AXIS_DATA_WIDTH = 128
) (
    input  wire                         clear_busy,

    
    input  wire                         clear_cmd_tvalid,
    output wire                         clear_cmd_tready,
    input  wire [71:0]                  clear_cmd_tdata,

    input  wire [AXIS_DATA_WIDTH-1:0]   clear_tdata,
    input  wire                         clear_tvalid,
    output wire                         clear_tready,
    input  wire [AXIS_DATA_WIDTH/8-1:0] clear_tkeep,
    input  wire                         clear_tlast,

    output wire [7:0]                   clear_sts_tdata,
    output wire                         clear_sts_tvalid,
    input  wire                         clear_sts_tready,

    
    input  wire                         normal_cmd_tvalid,
    output wire                         normal_cmd_tready,
    input  wire [71:0]                  normal_cmd_tdata,

    input  wire [AXIS_DATA_WIDTH-1:0]   normal_tdata,
    input  wire                         normal_tvalid,
    output wire                         normal_tready,
    input  wire [AXIS_DATA_WIDTH/8-1:0] normal_tkeep,
    input  wire                         normal_tlast,

    output wire [7:0]                   normal_sts_tdata,
    output wire                         normal_sts_tvalid,
    input  wire                         normal_sts_tready,

    
    output wire                         dm_cmd_tvalid,
    input  wire                         dm_cmd_tready,
    output wire [71:0]                  dm_cmd_tdata,

    
    output wire [AXIS_DATA_WIDTH-1:0]   dm_tdata,
    output wire                         dm_tvalid,
    input  wire                         dm_tready,
    output wire [AXIS_DATA_WIDTH/8-1:0] dm_tkeep,
    output wire                         dm_tlast,

    
    input  wire [7:0]                   dm_sts_tdata,
    input  wire                         dm_sts_tvalid,
    output wire                         dm_sts_tready
);

    assign dm_cmd_tvalid = clear_busy ? clear_cmd_tvalid : normal_cmd_tvalid;
    assign dm_cmd_tdata  = clear_busy ? clear_cmd_tdata  : normal_cmd_tdata;

    assign clear_cmd_tready  = clear_busy  ? dm_cmd_tready : 1'b0;
    assign normal_cmd_tready = !clear_busy ? dm_cmd_tready : 1'b0;

    assign dm_tdata  = clear_busy ? clear_tdata  : normal_tdata;
    assign dm_tvalid = clear_busy ? clear_tvalid : normal_tvalid;
    assign dm_tkeep  = clear_busy ? clear_tkeep  : normal_tkeep;
    assign dm_tlast  = clear_busy ? clear_tlast  : normal_tlast;

    assign clear_tready  = clear_busy  ? dm_tready : 1'b0;
    assign normal_tready = !clear_busy ? dm_tready : 1'b0;

    assign clear_sts_tdata   = dm_sts_tdata;
    assign normal_sts_tdata  = dm_sts_tdata;
    assign clear_sts_tvalid  = clear_busy  ? dm_sts_tvalid : 1'b0;
    assign normal_sts_tvalid = !clear_busy ? dm_sts_tvalid : 1'b0;
    assign dm_sts_tready     = clear_busy ? clear_sts_tready : normal_sts_tready;

endmodule
