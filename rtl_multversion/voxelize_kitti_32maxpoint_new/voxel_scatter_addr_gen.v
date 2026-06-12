module voxel_scatter_addr_gen_kitti #(
    parameter integer GRID_X        = 432,  // pseudo-image X width
    parameter integer GRID_Y        = 496,  // pseudo-image Y height
    parameter integer BYTES_PER_VOX = 128,  // bytes reserved/written per voxel
    parameter integer DATA_BYTES    = 128,
    parameter integer COORD_WIDTH   = 11
) (
    input wire aclk,
    input wire aresetn,

    // Dynamic output DDR base address
    input wire [31:0] base_addr_in,

    // Voxel coordinate input
    input  wire                   s_voxel_valid,
    output wire                   s_voxel_ready,
    input  wire [COORD_WIDTH-1:0] s_voxel_x,
    input  wire [COORD_WIDTH-1:0] s_voxel_y,

    // AXI DataMover S2MM Command output
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,
    output wire [71:0] m_axis_cmd_tdata
);

    wire clk   = aclk;
    wire rst_n = aresetn;

    localparam [31:0] GRID_X_U = GRID_X;
    localparam [31:0] GRID_Y_U = GRID_Y;

    localparam [31:0] ROW_BYTES = GRID_X * BYTES_PER_VOX;
    localparam [22:0] BTT_BYTES = DATA_BYTES[22:0];

    wire [31:0] ux_raw = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_x};
    wire [31:0] uy_raw = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_y};

    wire voxel_in_range = (ux_raw < GRID_X_U) && (uy_raw < GRID_Y_U);

    // -------------------------------------------------------------------------
    // Important deadlock fix
    // -------------------------------------------------------------------------
    // Do NOT make s_voxel_ready depend on the current s_voxel_x/y value.
    // In this design the upstream router uses *_voxel_ready to decide whether a
    // lane can be selected, and the coordinate bus may hold a stale/out-of-range
    // value when *_voxel_valid is 0.  If ready is deasserted only because that
    // stale coordinate is out-of-range, the router can stop selecting the lane,
    // PFN/core_done will never complete, and PS will timeout before any response
    // header is sent.
    //
    // Therefore, ready only means this module has space for one command.
    // If a truly out-of-range valid coordinate ever arrives, generate a safe
    // clipped command instead of silently dropping the command.  Silent dropping
    // would break the data/command packet pairing and deadlock the pair gate.
    // -------------------------------------------------------------------------
    reg         cmd_valid;
    reg  [31:0] target_addr;

    wire cmd_slot_ready = m_axis_cmd_tready || !cmd_valid;
    assign s_voxel_ready = cmd_slot_ready;

    wire voxel_fire = s_voxel_valid && cmd_slot_ready;

    // Clamp only for the exceptional OOB case.  Normal in-range coordinates are
    // unchanged.  This prevents deadlock if an OOB coordinate leaks through while
    // preserving one command per data packet.
    wire [31:0] ux_clamped = (ux_raw < GRID_X_U) ? ux_raw : (GRID_X_U - 32'd1);
    wire [31:0] uy_clamped = (uy_raw < GRID_Y_U) ? uy_raw : (GRID_Y_U - 32'd1);

    wire [31:0] x_offset = ux_clamped * BYTES_PER_VOX;
    wire [31:0] y_offset = uy_clamped * ROW_BYTES;
    wire [31:0] calc_addr = base_addr_in + y_offset + x_offset;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_addr <= 32'd0;
            cmd_valid   <= 1'b0;
        end else begin
            if (cmd_slot_ready) begin
                cmd_valid <= voxel_fire;

                if (voxel_fire) begin
                    target_addr <= calc_addr;
                end
            end
        end
    end

    assign m_axis_cmd_tdata = {
        4'b0000,      // [71:68] RSVD
        4'b0000,      // [67:64] TAG
        target_addr,  // [63:32] SADDR/DADDR
        1'b0,         // [31]    DRR
        1'b1,         // [30]    EOF
        6'b000000,    // [29:24] DSA
        1'b1,         // [23]    Type = INCR
        BTT_BYTES     // [22:0]  BTT
    };

    assign m_axis_cmd_tvalid = cmd_valid;

endmodule
