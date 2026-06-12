module voxel_scatter_addr_gen #(
    parameter integer GRID_X        = 496,  // 伪图像 X 方向宽度
    parameter integer GRID_Y        = 432,  // 伪图像 Y 方向高度
    parameter integer BYTES_PER_VOX = 128,  // 每个 voxel 实际占 128 bytes
    parameter integer DATA_BYTES    = 128,  // 实际写入 128 bytes，不再只写 96 bytes
    parameter integer COORD_WIDTH   = 11
) (
    input wire aclk,
    input wire aresetn,

    // 外部输入的动态基地址
    input wire [31:0] base_addr_in,

    // 输入：坐标信息
    input wire                   s_voxel_valid,
    input wire [COORD_WIDTH-1:0] s_voxel_x,
    input wire [COORD_WIDTH-1:0] s_voxel_y,

    // 输出：AXI DataMover S2MM Command
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,
    output wire [71:0] m_axis_cmd_tdata
);

    wire clk = aclk;
    wire rst_n = aresetn;

    // 坐标零扩展到 32 bit
    wire [31:0] ux = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_x};
    wire [31:0] uy = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_y};

    localparam [31:0] GRID_X_U = GRID_X;
    localparam [31:0] GRID_Y_U = GRID_Y;

    // 每一行的字节跨度：
    // GRID_X * BYTES_PER_VOX = 496 * 128 = 63488 bytes
    localparam [31:0] ROW_BYTES = GRID_X * BYTES_PER_VOX;

    // BTT 字段宽度是 23 bit
    localparam [22:0] BTT_BYTES = DATA_BYTES[22:0];

    // 坐标范围检查，防止越界写 DDR
    wire        voxel_in_range = (ux < GRID_X_U) && (uy < GRID_Y_U);

    wire        voxel_fire = s_voxel_valid && voxel_in_range;

    // =======================================================
    // 地址偏移计算
    //
    // target_addr = base + (y * GRID_X + x) * 128
    //
    // x_offset = x * 128
    // y_offset = y * GRID_X * 128
    //
    // GRID_X = 496 时：
    // y_offset = y * 63488
    //          = y * 0xF800
    //          = (y << 16) - (y << 11)
    //
    // 这里用通用写法，综合器会把常数乘法优化成移位加减。
    // =======================================================
    wire [31:0] x_offset = ux * BYTES_PER_VOX;
    wire [31:0] y_offset = uy * ROW_BYTES;

    wire [31:0] calc_addr = base_addr_in + y_offset + x_offset;

    // 内部寄存器
    reg  [31:0] target_addr;
    reg         cmd_valid;

    // =======================================================
    // 命令寄存
    //
    // 当当前命令被 DataMover 接收，或者当前没有待发送命令时，
    // 可以装载新的 voxel 命令。
    // =======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_addr <= 32'd0;
            cmd_valid   <= 1'b0;
        end else begin
            if (m_axis_cmd_tready || !cmd_valid) begin
                cmd_valid <= voxel_fire;

                if (voxel_fire) begin
                    target_addr <= calc_addr;
                end
            end
        end
    end

    // =======================================================
    // AXI DataMover S2MM Command, 72-bit
    //
    // [71:68] RSVD
    // [67:64] TAG
    // [63:32] SADDR/DADDR
    // [31]    DRR
    // [30]    EOF
    // [29:24] DSA
    // [23]    Type = INCR
    // [22:0]  BTT
    //
    // 现在每个 voxel 写满 128 bytes：
    // BTT = 128
    // =======================================================
    assign m_axis_cmd_tdata = {
        4'b0000,  // [71:68] RSVD
        4'b0000,  // [67:64] TAG
        target_addr,  // [63:32] SADDR/DADDR
        1'b0,  // [31]    DRR
        1'b1,  // [30]    EOF
        6'b000000,  // [29:24] DSA
        1'b1,  // [23]    Type = INCR
        BTT_BYTES  // [22:0]  BTT = 128 bytes
    };

    assign m_axis_cmd_tvalid = cmd_valid;

endmodule
