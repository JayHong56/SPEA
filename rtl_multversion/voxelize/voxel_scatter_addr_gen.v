module voxel_scatter_addr_gen #(
    parameter integer GRID_WIDTH    = 720,  // 伪图像的 X 轴网格数
    parameter integer BYTES_PER_VOX = 128,  // 每个 Voxel 在 DDR 中预留 128 字节空间
    parameter integer DATA_BYTES    = 96,   // 实际写入的数据仍然是 96 字节
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

    // 输出：AXI DataMover 命令总线
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,
    output wire [71:0] m_axis_cmd_tdata
);

    wire        clk = aclk;
    wire        rst_n = aresetn;

    // 为了避免位宽溢出警告，先将坐标零扩展到 32 位
    wire [31:0] ux = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_x};
    wire [31:0] uy = {{(32 - COORD_WIDTH) {1'b0}}, s_voxel_y};

    // 内部寄存器：锁存当前 voxel 对应的目标地址和命令有效信号
    reg  [31:0] target_addr;
    reg         cmd_valid;

    // =======================================================
    // 地址偏移计算
    // 现在每个 voxel 在 DDR 中占 128B 空间，但只写前 96B 数据
    //
    // x 方向步长：
    //   x_offset = ux * 128 = ux << 7
    //
    // y 方向一整行跨度：
    //   GRID_WIDTH * BYTES_PER_VOX = 720 * 128 = 92160
    //   92160 = 65536 + 16384 + 8192 + 2048
    //         = 2^16 + 2^14 + 2^13 + 2^11
    // =======================================================
    wire [31:0] x_offset = (ux << 7);

    wire [31:0] y_offset = (uy << 16) + (uy << 14) + (uy << 13) + (uy << 11);

    // =======================================================
    // 单周期锁存目标地址
    // 当当前命令已被接收，或者当前没有待发送命令时，允许装载新地址
    // =======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_addr <= 32'd0;
            cmd_valid   <= 1'b0;
        end else begin
            if (m_axis_cmd_tready || !cmd_valid) begin
                cmd_valid <= s_voxel_valid;

                if (s_voxel_valid) begin
                    target_addr <= base_addr_in + y_offset + x_offset;
                end
            end
        end
    end

    // =======================================================
    // 拼装 AXI DataMover S2MM Command（72-bit）
    //
    // 注意：
    // 1. 地址间隔已经改为 128B
    // 2. 但实际写入的数据长度仍然是 96B，所以 BTT 仍然是 96
    // =======================================================
    assign m_axis_cmd_tdata = {
        4'b0000,  // [71:68] RSVD
        4'b0000,  // [67:64] TAG
        target_addr,  // [63:32] SADDR/DADDR
        1'b0,  // [31]    DRR
        1'b1,  // [30]    EOF
        6'b000000,  // [29:24] DSA
        1'b1,  // [23]    Type = INCR
        23'd96  // [22:0]  BTT = 实际传输 96 bytes
    };

    assign m_axis_cmd_tvalid = cmd_valid;

endmodule
