module voxel_coor_pipe_2ic #(
    parameter integer BRAM_DATA_WIDTH = 576,  // 72 * 8
    parameter integer BRAM_ADDR_WIDTH = 10,  // 256 pillar * 4 rows
    parameter integer BRAM_ADDR_WIDTH_PFE = 8,  // PFE cache row address width
    parameter integer PREFILTER_LANES = 2,
    parameter integer DRAM_DATA_WIDTH = 128 * PREFILTER_LANES,  // ?????2 points/beat, {x,y,z,intensity} float32 per lane
    parameter integer DRAM_ADDR_WIDTH = 18,
    parameter integer HASH_ADDR_WIDTH = 8,  // log2(HASH_TABLE_SIZE)
    parameter [15:0] THRESHOLD_CLOSE = 16'h0100,  // 1.00m
    parameter [20-1:0] THRESHOLD_BOUDARY_X_LOW = 20'h0000,  // 0.00m
    parameter [20-1:0] THRESHOLD_BOUDARY_X_HIGH = 20'h451ec,  // 69.12m
    parameter [20-1:0] THRESHOLD_BOUDARY_Y = 20'h27ae1,  // +/- 39.68m
    parameter [15:0] THRESHOLD_BOUDARY_Z_LOW = 16'h3000,  // abs(-3)m
    parameter [15:0] THRESHOLD_BOUDARY_Z_HIGH = 16'h1000,  // 1m
    parameter integer PRE_LAT = 0,  // ?????????????
    parameter [15:0] LIFE_CYCLE = 16'd100,
    parameter integer VN_WIDTH = 6,  // ??????hash ??? 1..32?
    parameter [23:0] MAX_VOXEL_NUM = 24'd32,  // ????? 32 ?
    parameter integer BYTE_WIDTH = 9  // 1 Byte = 8 bit, ?? BRAM ???? 9-bit chunk
) (
    input wire clk,
    input wire rst_n,

    input wire frame_end,  // ?????????????? (1-cycle pulse)
    output wire flush_done,  // ??????????????????
    output wire hash_stall,
    // AXI-Stream in (from DRAM FIFO), 256-bit when PREFILTER_LANES=2: lane0 [127:0], lane1 [255:128]
    input wire [DRAM_DATA_WIDTH-1:0] s_axis_dram_data,
    input wire [DRAM_DATA_WIDTH/8-1:0] s_axis_dram_keep,
    input wire s_axis_dram_last,
    input wire s_axis_dram_valid,
    output wire s_axis_dram_ready,
    // BRAM Native Interface
    output reg bram_voxelpoint_wr,
    output reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] bram_voxelpoint_bwen,
    output reg [BRAM_ADDR_WIDTH-1:0] bram_voxelpoint_addr,
    output reg [BRAM_DATA_WIDTH-1:0] bram_voxelpoint_wrdata,
    // BRAM_VOXELPOINT_A
    output wire [BRAM_ADDR_WIDTH-1:0] bram_expire_addr_a,
    input wire [BRAM_DATA_WIDTH-1:0] bram_expire_rdata_a,
    // BRAM_VOXELPOINT_B
    output wire bram_expire_wr_b,
    output wire [BRAM_ADDR_WIDTH_PFE-1:0] bram_expire_addr_b,
    output wire [BRAM_DATA_WIDTH-1:0] bram_expire_wrdata_b,
    // AXI-Stream out (expire voxel coordinates)
    output wire m_axis_expire_tvalid,
    input wire m_axis_expire_tready,
    output wire [22+BRAM_ADDR_WIDTH_PFE+VN_WIDTH-1:0] m_axis_expire_tdata
);

    localparam integer PT_WIDTH_I_XY = 8;
    localparam integer PT_WIDTH_F_XY = 12;
    localparam integer PT_WIDTH_I_Z = 4;
    localparam integer PT_WIDTH_F_Z = 12;
    localparam integer PT_WIDTH_I_IS = 1;
    localparam integer PT_WIDTH_F_IS = 15;

    localparam integer PT_WIDTH_XY = PT_WIDTH_I_XY + PT_WIDTH_F_XY;  // 20
    localparam integer PT_WIDTH = 16;  // 16
    localparam integer PT_WIDTH_PER = 2 * PT_WIDTH_XY + 2 * PT_WIDTH;  // 72 bit ??
    localparam integer PT_WIDTH_FLOAT32 = 32;
    localparam integer VOXEL_WIDTH = 11;  // NOTE ????????

    localparam integer POINTS_PER_ROW = BRAM_DATA_WIDTH / PT_WIDTH_PER;  // 576/72 = 8
    localparam integer EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) / POINTS_PER_ROW;  // ceil(32/8)=4
    localparam [VN_WIDTH-1:0] MAX_VOXEL_POINTS = MAX_VOXEL_NUM[VN_WIDTH-1:0];
    // ============================================================
    // ?? prefilter + ?? voxel/hash ??
    // ============================================================
    localparam integer RAW_PT_WIDTH = 4 * PT_WIDTH_FLOAT32;  // 128 bit

    // ?? FIFO ???
    // {x_fix16, y_fix16, z_fix16, intensity_float32}
    // intensity ??? fixed???????????
    localparam integer CAND_WIDTH = 4 * PT_WIDTH_XY + PT_WIDTH + PT_WIDTH_FLOAT32;
    // ??? 64 ? 128???? 2 ??
    localparam integer CAND_FIFO_DEPTH = 32;
    localparam integer CAND_FIFO_PTR_W = $clog2(CAND_FIFO_DEPTH);
    localparam integer CAND_FIFO_CNT_W = $clog2(CAND_FIFO_DEPTH + 1);

`ifndef SYNTHESIS
    initial begin
        if (DRAM_DATA_WIDTH != RAW_PT_WIDTH * PREFILTER_LANES) begin
            $display("[ERROR][voxel_coor_pipe] DRAM_DATA_WIDTH must equal 128 * PREFILTER_LANES.");
            $stop;
        end

        if ((CAND_FIFO_DEPTH & (CAND_FIFO_DEPTH - 1)) != 0) begin
            $display("[ERROR][voxel_coor_pipe] CAND_FIFO_DEPTH must be power of 2.");
            $stop;
        end
    end
`endif

    // ------------------------------------------------------------
    // AXIS ????
    // ------------------------------------------------------------
    wire s_fire;
    assign s_fire = s_axis_dram_valid && s_axis_dram_ready;

    // ------------------------------------------------------------
    // ? lane ? raw point
    // ?? lane0 ? s_axis_dram_data[127:0]
    // lane1 ? [255:128]
    // lane2 ? [383:256]
    // ------------------------------------------------------------
    wire [RAW_PT_WIDTH-1:0] lane_raw[0:PREFILTER_LANES-1];

    wire [PT_WIDTH_FLOAT32-1:0] lane_x_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_y_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_z_float32[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_FLOAT32-1:0] lane_i_float32[0:PREFILTER_LANES-1];

    wire signed [PT_WIDTH_XY-1:0] lane_x_fix20_data[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH_XY-1:0] lane_y_fix20_data[0:PREFILTER_LANES-1];
    // voxel: ????? voxel_x / voxel_y??? ceil(+inf)
    wire signed [PT_WIDTH_XY-1:0] lane_x_fix20_voxel[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH_XY-1:0] lane_y_fix20_voxel[0:PREFILTER_LANES-1];
    wire signed [PT_WIDTH-1:0] lane_z_fix16[0:PREFILTER_LANES-1];

    wire [PT_WIDTH_XY-1:0] lane_abs_x[0:PREFILTER_LANES-1];
    wire [PT_WIDTH_XY-1:0] lane_abs_y[0:PREFILTER_LANES-1];
    wire [PT_WIDTH-1:0] lane_abs_z[0:PREFILTER_LANES-1];

    wire lane_keep[0:PREFILTER_LANES-1];
    wire lane_present[0:PREFILTER_LANES-1];

    wire [CAND_WIDTH-1:0] lane_cand_data[0:PREFILTER_LANES-1];

    genvar gi;
    generate
        for (gi = 0; gi < PREFILTER_LANES; gi = gi + 1) begin : GEN_PREFILTER_LANE
            assign lane_raw[gi] = s_axis_dram_data[gi*RAW_PT_WIDTH+:RAW_PT_WIDTH];

            assign lane_x_float32[gi] = lane_raw[gi][127:96];
            assign lane_y_float32[gi] = lane_raw[gi][95:64];
            assign lane_z_float32[gi] = lane_raw[gi][63:32];
            assign lane_i_float32[gi] = lane_raw[gi][31:0];

            // ?? lane ????? 16-byte point
            assign lane_present[gi] = &s_axis_dram_keep[gi*(RAW_PT_WIDTH/8)+:(RAW_PT_WIDTH/8)];

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_x_data (
                .float_in        (lane_x_float32[gi]),
                .true_fixed_value(lane_x_fix20_data[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_x_voxel (
                .float_in        (lane_x_float32[gi]),
                .true_fixed_value(lane_x_fix20_voxel[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_y_data (
                .float_in        (lane_y_float32[gi]),
                .true_fixed_value(lane_y_fix20_data[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_XY),
                .FIXED_FRACTIONAL(PT_WIDTH_F_XY)
            ) u_float2fxp_y_voxel (
                .float_in        (lane_y_float32[gi]),
                .true_fixed_value(lane_y_fix20_voxel[gi]),
                .fixed_sign      ()
            );

            float_to_fixed_data #(
                .FIXED_WIDTH     (PT_WIDTH_F_Z + PT_WIDTH_I_Z),
                .FIXED_FRACTIONAL(PT_WIDTH_F_Z)
            ) u_float2fxp_z (
                .float_in        (lane_z_float32[gi]),
                .true_fixed_value(lane_z_fix16[gi]),
                .fixed_sign      ()
            );

            assign lane_abs_x[gi] = lane_x_fix20_data[gi][PT_WIDTH_XY-1] ? (~lane_x_fix20_data[gi] + 1'b1) : lane_x_fix20_data[gi];
            assign lane_abs_y[gi] = lane_y_fix20_data[gi][PT_WIDTH_XY-1] ? (~lane_y_fix20_data[gi] + 1'b1) : lane_y_fix20_data[gi];
            assign lane_abs_z[gi] = lane_z_fix16[gi][PT_WIDTH-1] ? (~lane_z_fix16[gi] + 1'b1) : lane_z_fix16[gi];

            // keep_point ??????? x/y/z ????? voxel ????
            assign lane_keep[gi] =
                lane_present[gi] &&
                (lane_raw[gi] != {RAW_PT_WIDTH{1'b0}}) &&
                !(^lane_x_fix20_data[gi] === 1'bx) &&
                !lane_x_fix20_data[gi][PT_WIDTH_XY-1] &&
                !(lane_abs_x[gi] > THRESHOLD_BOUDARY_X_HIGH) &&
                !(lane_abs_y[gi] > THRESHOLD_BOUDARY_Y) &&
                !(lane_z_fix16[gi][PT_WIDTH-1] ? 
                    (lane_abs_z[gi] > THRESHOLD_BOUDARY_Z_LOW) :
                    (lane_abs_z[gi] > THRESHOLD_BOUDARY_Z_HIGH));

            assign lane_cand_data[gi] = {
                lane_x_fix20_data[gi],
                lane_y_fix20_data[gi],
                lane_x_fix20_voxel[gi],
                lane_y_fix20_voxel[gi],
                lane_z_fix16[gi],
                lane_i_float32[gi]
            };
        end
    endgenerate


    // ============================================================
    // Candidate FIFO??????? PREFILTER_LANES ????
    // ?????? 1 ??????? voxel ??
    // ============================================================
    (* ram_style = "distributed" *)
    reg [CAND_WIDTH-1:0] cand_fifo_mem[0:CAND_FIFO_DEPTH-1];

    reg [CAND_FIFO_PTR_W-1:0] cand_fifo_wr_ptr;
    reg [CAND_FIFO_PTR_W-1:0] cand_fifo_rd_ptr;
    reg [CAND_FIFO_CNT_W-1:0] cand_fifo_level;

    wire cand_fifo_empty;
    wire cand_fifo_full;

    assign cand_fifo_empty = (cand_fifo_level == {CAND_FIFO_CNT_W{1'b0}});
    assign cand_fifo_full  = (cand_fifo_level == CAND_FIFO_DEPTH[CAND_FIFO_CNT_W-1:0]);

    wire [CAND_WIDTH-1:0] cand_fifo_dout;
    assign cand_fifo_dout = cand_fifo_mem[cand_fifo_rd_ptr];

    function [CAND_FIFO_PTR_W-1:0] cand_ptr_add;
        input [CAND_FIFO_PTR_W-1:0] ptr;
        input [CAND_FIFO_CNT_W-1:0] inc;
        reg [CAND_FIFO_CNT_W:0] tmp;
        begin
            tmp = ptr + inc;
            cand_ptr_add = tmp[CAND_FIFO_PTR_W-1:0];
        end
    endfunction

    // ?? beat ??? lane ?? push
    wire lane_push[0:PREFILTER_LANES-1];

    genvar gp;
    generate
        for (gp = 0; gp < PREFILTER_LANES; gp = gp + 1) begin : GEN_LANE_PUSH
            assign lane_push[gp] = s_fire && lane_keep[gp];
        end
    endgenerate

    // ???? lane ????????? beat ? push ?
    reg [CAND_FIFO_CNT_W-1:0] lane_prefix[0:PREFILTER_LANES-1];
    reg [CAND_FIFO_CNT_W-1:0] cand_push_count;

    integer pi;
    integer pj;
    always @(*) begin
        cand_push_count = {CAND_FIFO_CNT_W{1'b0}};

        // ????????????? lane
        for (pi = 0; pi < PREFILTER_LANES; pi = pi + 1) begin
            if (lane_push[pi]) begin
                cand_push_count = cand_push_count + 1'b1;
            end
        end

        // ???? lane ? FIFO ??????
        // ??? lane ???? FIFO?
        // PREFILTER_LANES=3 ????? lane2 -> lane1 -> lane0
        for (pi = 0; pi < PREFILTER_LANES; pi = pi + 1) begin
            lane_prefix[pi] = {CAND_FIFO_CNT_W{1'b0}};

            // ??? lane ????????? lane???????
            for (pj = pi + 1; pj < PREFILTER_LANES; pj = pj + 1) begin
                if (lane_push[pj]) begin
                    lane_prefix[pi] = lane_prefix[pi] + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // ?? voxel ?????? candidate FIFO ??????
    // ============================================================
    wire signed [PT_WIDTH_XY-1:0] cand_x_fix20_data;
    wire signed [PT_WIDTH_XY-1:0] cand_y_fix20_data;
    wire signed [PT_WIDTH_XY-1:0] cand_x_fix20_voxel;
    wire signed [PT_WIDTH_XY-1:0] cand_y_fix20_voxel;
    wire signed [PT_WIDTH-1:0]    cand_z_fix16;
    wire [PT_WIDTH_FLOAT32-1:0]   cand_i_float32;

    assign cand_x_fix20_data  = cand_fifo_dout[CAND_WIDTH-1-:PT_WIDTH_XY];
    assign cand_y_fix20_data  = cand_fifo_dout[CAND_WIDTH-1-PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_x_fix20_voxel = cand_fifo_dout[CAND_WIDTH-1-2*PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_y_fix20_voxel = cand_fifo_dout[CAND_WIDTH-1-3*PT_WIDTH_XY-:PT_WIDTH_XY];
    assign cand_z_fix16       = cand_fifo_dout[CAND_WIDTH-1-4*PT_WIDTH_XY-:PT_WIDTH];
    assign cand_i_float32     = cand_fifo_dout[PT_WIDTH_FLOAT32-1:0];

    // ?? intensity float_to_fixed????????????
    wire signed [PT_WIDTH-1:0] cand_intensity_fix16;

    float_to_fixed_data #(
        .FIXED_WIDTH     (PT_WIDTH_F_IS + PT_WIDTH_I_IS),
        .FIXED_FRACTIONAL(PT_WIDTH_F_IS)
    ) u_float2fxp_intensity_single (
        .float_in        (cand_i_float32),
        .true_fixed_value(cand_intensity_fix16),
        .fixed_sign      ()
    );

    // ------------------------------------------------------------
    // ?? voxel_x / voxel_y ??
    // ------------------------------------------------------------
    localparam integer SCALE_VOXEL_FACTOR = 20;
    localparam integer SCALE_VOXEL = 16;
    localparam signed [SCALE_VOXEL_FACTOR-1:0] MULT_FACTOR = 20'd409600;  // 6.666666 (scale = 2^16)
    localparam integer ROUND = 1;

    wire signed [PT_WIDTH_XY-1:0] bourdary_fix20_Y;
    localparam signed [PT_WIDTH_XY-1:0] THRESHOLD_BOUDARY_Y_CEIL_FIX = 20'sh27ae2;  // 39.68m ?????LSB
    assign bourdary_fix20_Y = THRESHOLD_BOUDARY_Y_CEIL_FIX;

    wire [PT_WIDTH_XY+SCALE_VOXEL_FACTOR-1:0] single_fxp_mul_x_out;
    wire [PT_WIDTH_XY-1:0]    single_fxp_add_y_out;
    wire [PT_WIDTH_XY+SCALE_VOXEL_FACTOR-1:0] single_fxp_mul_y_out;

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (SCALE_VOXEL_FACTOR),
        .WIFB (0),
        .WOI  (SCALE_VOXEL_FACTOR + PT_WIDTH_XY),
        .WOF  (0),
        .ROUND(ROUND)
    ) u_single_fxp_mul_voxel_x (
        .ina     (cand_x_fix20_voxel),
        .inb     (MULT_FACTOR),
        .out     (single_fxp_mul_x_out),
        .overflow()
    );

    fxp_add #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (PT_WIDTH_I_XY),
        .WIFB (PT_WIDTH_F_XY),
        .WOI  (PT_WIDTH_I_XY),
        .WOF  (PT_WIDTH_F_XY),
        .ROUND(ROUND)
    ) u_single_fxp_add_voxel_y (
        .ina     (cand_y_fix20_voxel),
        .inb     (bourdary_fix20_Y),
        .out     (single_fxp_add_y_out),
        .overflow()
    );

    fxp_mul #(
        .WIIA (PT_WIDTH_I_XY),
        .WIFA (PT_WIDTH_F_XY),
        .WIIB (SCALE_VOXEL_FACTOR),
        .WIFB (0),
        .WOI  (SCALE_VOXEL_FACTOR + PT_WIDTH_XY),
        .WOF  (0),
        .ROUND(ROUND)
    ) u_single_fxp_mul_voxel_y (
        .ina     (single_fxp_add_y_out),
        .inb     (MULT_FACTOR),
        .out     (single_fxp_mul_y_out),
        .overflow()
    );

    wire [VOXEL_WIDTH-1:0] single_voxel_x;
    wire [VOXEL_WIDTH-1:0] single_voxel_y;

    assign single_voxel_x = single_fxp_mul_x_out[SCALE_VOXEL+VOXEL_WIDTH-1:SCALE_VOXEL];
    assign single_voxel_y = single_fxp_mul_y_out[SCALE_VOXEL+VOXEL_WIDTH-1:SCALE_VOXEL];

    wire [PT_WIDTH_PER-1:0] single_point_proc;
    assign single_point_proc = {cand_x_fix20_data, cand_y_fix20_data, cand_z_fix16, cand_intensity_fix16};


    // ============================================================
    // hash ?? pipe1?????? 1 ????? hash_table
    // ============================================================
    reg                    pipe1_valid;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_x;
    reg [ VOXEL_WIDTH-1:0] pipe1_voxel_y;
    reg [PT_WIDTH_PER-1:0] pipe1_point_proc;

    assign hash_req_valid = pipe1_valid;

    wire [VOXEL_WIDTH-1:0] hash_key_x;
    wire [VOXEL_WIDTH-1:0] hash_key_y;
    wire hash_req_ready;
    wire hash_fire = hash_req_valid && hash_req_ready;

    assign hash_key_x = pipe1_voxel_x;
    assign hash_key_y = pipe1_voxel_y;

    // pipe1 ?????????? hash ??????????????
    wire pipe1_can_accept;
    assign pipe1_can_accept = !pipe1_valid || hash_fire;

    wire cand_fifo_pop;
    assign cand_fifo_pop = pipe1_can_accept && !cand_fifo_empty;

    // ?? ready????? candidate FIFO ???? PREFILTER_LANES ???
    // ???????? pop ? 1 ?
    wire [CAND_FIFO_CNT_W:0] cand_fifo_space_after_pop;
    assign cand_fifo_space_after_pop = (CAND_FIFO_DEPTH - cand_fifo_level) + (cand_fifo_pop ? 1'b1 : 1'b0);

    localparam [CAND_FIFO_CNT_W:0] CAND_FIFO_DEPTH_W = CAND_FIFO_DEPTH;
    localparam [CAND_FIFO_CNT_W:0] PREFILTER_LANES_W = PREFILTER_LANES;

    wire [CAND_FIFO_CNT_W:0] cand_fifo_level_ext;
    wire [CAND_FIFO_CNT_W:0] cand_fifo_space_now;

    assign cand_fifo_level_ext = {1'b0, cand_fifo_level};
    assign cand_fifo_space_now = CAND_FIFO_DEPTH_W - cand_fifo_level_ext;

    assign s_axis_dram_ready   = (cand_fifo_space_now >= PREFILTER_LANES_W);
    // candidate FIFO ????? level ??
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_fifo_wr_ptr <= {CAND_FIFO_PTR_W{1'b0}};
            cand_fifo_rd_ptr <= {CAND_FIFO_PTR_W{1'b0}};
            cand_fifo_level  <= {CAND_FIFO_CNT_W{1'b0}};
        end else begin
            // ? lane ??
            for (wi = 0; wi < PREFILTER_LANES; wi = wi + 1) begin
                if (lane_push[wi]) begin
                    cand_fifo_mem[cand_ptr_add(cand_fifo_wr_ptr, lane_prefix[wi])] <= lane_cand_data[wi];
                end
            end

            if (cand_push_count != {CAND_FIFO_CNT_W{1'b0}}) begin
                cand_fifo_wr_ptr <= cand_ptr_add(cand_fifo_wr_ptr, cand_push_count);
            end

            if (cand_fifo_pop) begin
                cand_fifo_rd_ptr <= cand_ptr_add(cand_fifo_rd_ptr, {{(CAND_FIFO_CNT_W - 1) {1'b0}}, 1'b1});
            end

            cand_fifo_level <= cand_fifo_level
                             + cand_push_count
                             - (cand_fifo_pop ? {{(CAND_FIFO_CNT_W-1){1'b0}}, 1'b1}
                                              : {CAND_FIFO_CNT_W{1'b0}});
        end
    end

    // pipe1 ???? voxel ??
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_valid      <= 1'b0;
            pipe1_voxel_x    <= {VOXEL_WIDTH{1'b0}};
            pipe1_voxel_y    <= {VOXEL_WIDTH{1'b0}};
            pipe1_point_proc <= {PT_WIDTH_PER{1'b0}};
        end else begin
            if (pipe1_can_accept) begin
                pipe1_valid <= cand_fifo_pop;

                if (cand_fifo_pop) begin
                    pipe1_voxel_x    <= single_voxel_x;
                    pipe1_voxel_y    <= single_voxel_y;
                    pipe1_point_proc <= single_point_proc;
                end
            end
        end
    end


    // ============================================================
    // frame_end ????? candidate FIFO?pipe1?hash ??????
    // ============================================================
    reg  frame_end_pending;

    wire frontend_empty;

    reg  pproc_vld_d1;
    reg  pproc_vld_d2;
    reg  pproc_vld_d3;
    // ??? pproc_vld_d1/d2/d3 ??????
    assign frontend_empty = cand_fifo_empty && !pipe1_valid && !pproc_vld_d1 && !pproc_vld_d2 && !pproc_vld_d3;

    wire hash_frame_end;
    assign hash_frame_end = frame_end_pending && frontend_empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_end_pending <= 1'b0;
        end else begin
            if (frame_end) begin
                frame_end_pending <= 1'b1;
            end else if (hash_frame_end) begin
                frame_end_pending <= 1'b0;
            end
        end
    end


    wire                       hash_found;
    wire                       hash_busy;
    wire                       hash_table_full;
    wire [HASH_ADDR_WIDTH-1:0] hash_out_idx;
    wire [       VN_WIDTH-1:0] hash_out_point_number;
    hash_bucket_table #(
        .COORD_WIDTH        (VOXEL_WIDTH),
        .VN_WIDTH           (VN_WIDTH),
        .LIFE_CYCLE         (LIFE_CYCLE),
        .TIMER_WIDTH        (DRAM_ADDR_WIDTH),     // NOTE max 37650
        .ADDR_WIDTH         (HASH_ADDR_WIDTH),
        .BUCKETS            (64),
        .BUCKET_SLOT_WIDTH  (2),
        .BRAM_ADDR_WIDTH    (BRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH    (BRAM_DATA_WIDTH),
        .BRAM_ADDR_WIDTH_PFE(BRAM_ADDR_WIDTH_PFE)
    ) u_hash_table_tombstone (
        .clk                 (clk),
        .rst_n               (rst_n),
        .req_ready           (hash_req_ready),
        .req_valid           (hash_req_valid),
        .frame_end           (hash_frame_end),
        .flush_done          (flush_done),
        .hash_stall          (hash_stall),
        .key_x               (hash_key_x),
        .key_y               (hash_key_y),
        .out_idx             (hash_out_idx),           // 0-255
        .out_point_number    (hash_out_point_number),  // 1-32
        .table_full          (hash_table_full),
        .bram_expire_addr_a  (bram_expire_addr_a),     // 10-bit Address
        .bram_expire_rdata_a (bram_expire_rdata_a),    // 576-bit Data
        .bram_expire_wr_b    (bram_expire_wr_b),
        .bram_expire_addr_b  (bram_expire_addr_b),     // 10-bit Address
        .bram_expire_wrdata_b(bram_expire_wrdata_b),   // 576-bit Data
        .m_axis_expire_tready(m_axis_expire_tready),
        .m_axis_expire_tvalid(m_axis_expire_tvalid),
        .m_axis_expire_tdata (m_axis_expire_tdata)
    );

    // ---------------------------? hash ??-------------------------
    reg [PT_WIDTH_PER-1:0] pproc_d1, pproc_d2, pproc_d3;
    reg hash_req_ready_1delay;
    reg pproc_drop_d1, pproc_drop_d2, pproc_drop_d3;
    reg [5:0] full_voxel_lfsr;
    reg [BRAM_ADDR_WIDTH-1:0] full_rand_row_d1, full_rand_row_d2, full_rand_row_d3;
    reg [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_d1, full_rand_slot_d2, full_rand_slot_d3;
    reg [5:0] full_rand_value_d1, full_rand_value_d2, full_rand_value_d3;
    reg [7:0] full_point_count[0:(1 << HASH_ADDR_WIDTH)-1];
    integer full_cnt_i;

    wire full_voxel_lfsr_feedback = full_voxel_lfsr[5] ^ full_voxel_lfsr[4];

    function [4:0] full_rand_choice_32;
        input [5:0] lfsr_state;
        reg [5:0] nonzero_state;
        reg [5:0] choice6;
        begin
            nonzero_state = (lfsr_state == 6'd0) ? 6'd1 : lfsr_state;
            if (nonzero_state <= 6'd32) begin
                choice6 = nonzero_state - 6'd1;
            end else begin
                choice6 = nonzero_state - 6'd33;
            end
            full_rand_choice_32 = choice6[4:0];
        end
    endfunction

    function [BRAM_ADDR_WIDTH-1:0] full_rand_row_offset;
        input [4:0] choice;
        begin
            if (choice < 5'd8) begin
                full_rand_row_offset = {BRAM_ADDR_WIDTH{1'b0}};
            end else if (choice < 5'd16) begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 1) {1'b0}}, 1'b1};
            end else if (choice < 5'd24) begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 2) {1'b0}}, 2'd2};
            end else begin
                full_rand_row_offset = {{(BRAM_ADDR_WIDTH - 2) {1'b0}}, 2'd3};
            end
        end
    endfunction

    function [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_idx;
        input [4:0] choice;
        begin
            case (choice)
                5'd0, 5'd8, 5'd16, 5'd24: full_rand_slot_idx = 3'd0;
                5'd1, 5'd9, 5'd17, 5'd25: full_rand_slot_idx = 3'd1;
                5'd2, 5'd10, 5'd18, 5'd26: full_rand_slot_idx = 3'd2;
                5'd3, 5'd11, 5'd19, 5'd27: full_rand_slot_idx = 3'd3;
                5'd4, 5'd12, 5'd20, 5'd28: full_rand_slot_idx = 3'd4;
                5'd5, 5'd13, 5'd21, 5'd29: full_rand_slot_idx = 3'd5;
                5'd6, 5'd14, 5'd22, 5'd30: full_rand_slot_idx = 3'd6;
                5'd7, 5'd15, 5'd23, 5'd31: full_rand_slot_idx = 3'd7;
                default: full_rand_slot_idx = 3'd0;
            endcase
        end
    endfunction

    wire [4:0] full_rand_choice = full_rand_choice_32(full_voxel_lfsr);
    wire [BRAM_ADDR_WIDTH-1:0] full_rand_row_next = full_rand_row_offset(full_rand_choice);
    wire [$clog2(POINTS_PER_ROW)-1:0] full_rand_slot_next = full_rand_slot_idx(full_rand_choice);

    function [5:0] full_keep_threshold;
        input [7:0] total_count;
        begin
            case (total_count)
                8'd0, 8'd1, 8'd2, 8'd3, 8'd4, 8'd5, 8'd6, 8'd7,
                8'd8, 8'd9, 8'd10, 8'd11, 8'd12, 8'd13, 8'd14, 8'd15,
                8'd16, 8'd17, 8'd18, 8'd19, 8'd20, 8'd21, 8'd22, 8'd23,
                8'd24, 8'd25, 8'd26, 8'd27, 8'd28, 8'd29, 8'd30, 8'd31,
                8'd32:
                full_keep_threshold = 6'd63;
                8'd33: full_keep_threshold = 6'd62;
                8'd34: full_keep_threshold = 6'd60;
                8'd35: full_keep_threshold = 6'd58;
                8'd36: full_keep_threshold = 6'd57;
                8'd37: full_keep_threshold = 6'd55;
                8'd38: full_keep_threshold = 6'd54;
                8'd39: full_keep_threshold = 6'd52;
                8'd40: full_keep_threshold = 6'd51;
                8'd41: full_keep_threshold = 6'd50;
                8'd42: full_keep_threshold = 6'd48;
                8'd43: full_keep_threshold = 6'd47;
                8'd44: full_keep_threshold = 6'd46;
                8'd45: full_keep_threshold = 6'd45;
                8'd46: full_keep_threshold = 6'd44;
                8'd47: full_keep_threshold = 6'd43;
                8'd48: full_keep_threshold = 6'd42;
                8'd49: full_keep_threshold = 6'd41;
                8'd50: full_keep_threshold = 6'd40;
                8'd51: full_keep_threshold = 6'd40;
                8'd52: full_keep_threshold = 6'd39;
                8'd53: full_keep_threshold = 6'd38;
                8'd54: full_keep_threshold = 6'd37;
                8'd55: full_keep_threshold = 6'd37;
                8'd56: full_keep_threshold = 6'd36;
                8'd57: full_keep_threshold = 6'd35;
                8'd58: full_keep_threshold = 6'd35;
                8'd59: full_keep_threshold = 6'd34;
                8'd60: full_keep_threshold = 6'd34;
                8'd61: full_keep_threshold = 6'd33;
                8'd62: full_keep_threshold = 6'd33;
                8'd63, 8'd64: full_keep_threshold = 6'd32;
                default: begin
                    if (total_count <= 8'd84) begin
                        full_keep_threshold = 6'd24;
                    end else if (total_count <= 8'd126) begin
                        full_keep_threshold = 6'd16;
                    end else if (total_count <= 8'd180) begin
                        full_keep_threshold = 6'd11;
                    end else begin
                        full_keep_threshold = 6'd8;
                    end
                end
            endcase
        end
    endfunction



    wire pipe_pproc_stall = hash_req_ready;
    wire pipe_bram_stall = hash_req_ready_1delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pproc_vld_d1 <= 1'b0;
            pproc_vld_d2 <= 1'b0;
            pproc_vld_d3 <= 1'b0;
            pproc_drop_d1 <= 1'b0;
            pproc_drop_d2 <= 1'b0;
            pproc_drop_d3 <= 1'b0;
            full_rand_row_d1 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_row_d2 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_row_d3 <= {BRAM_ADDR_WIDTH{1'b0}};
            full_rand_slot_d1 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_slot_d2 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_slot_d3 <= {$clog2(POINTS_PER_ROW) {1'b0}};
            full_rand_value_d1 <= 6'd1;
            full_rand_value_d2 <= 6'd1;
            full_rand_value_d3 <= 6'd1;
        end else if (pipe_pproc_stall) begin  // pproc_d3??? hash_out_idx??
            pproc_drop_d1      <= 1'b0;
            pproc_drop_d2      <= pproc_drop_d1;
            pproc_drop_d3      <= pproc_drop_d2;

            full_rand_row_d1   <= full_rand_row_next;
            full_rand_row_d2   <= full_rand_row_d1;
            full_rand_row_d3   <= full_rand_row_d2;
            full_rand_slot_d1  <= full_rand_slot_next;
            full_rand_slot_d2  <= full_rand_slot_d1;
            full_rand_slot_d3  <= full_rand_slot_d2;
            full_rand_value_d1 <= full_voxel_lfsr;
            full_rand_value_d2 <= full_rand_value_d1;
            full_rand_value_d3 <= full_rand_value_d2;

            pproc_vld_d1       <= hash_fire;
            pproc_vld_d2       <= pproc_vld_d1;
            pproc_vld_d3       <= pproc_vld_d2;

            pproc_d1           <= pipe1_point_proc;
            pproc_d2           <= pproc_d1;
            pproc_d3           <= pproc_d2;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            full_voxel_lfsr <= 6'h2d;
        end else if (hash_fire) begin
            full_voxel_lfsr <= {full_voxel_lfsr[4:0], full_voxel_lfsr_feedback};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hash_req_ready_1delay <= 1'b0;
        end else begin
            hash_req_ready_1delay <= hash_req_ready;
        end
    end
    // ------------------------------------------------------------
    // 7) BRAM ??/?????????? (point_number-1)>>2 ? slot 0-based?
    // ------------------------------------------------------------
    wire [VN_WIDTH-1:0] pt_idx_minus_1 = hash_out_point_number - 1'b1;
    wire [BRAM_ADDR_WIDTH-1:0] row_offset = pt_idx_minus_1 / POINTS_PER_ROW;
    wire [$clog2(POINTS_PER_ROW)-1:0] bram_slot_idx = pt_idx_minus_1 % POINTS_PER_ROW;
    wire [BRAM_ADDR_WIDTH-1:0] bram_row_addr = (hash_out_idx * EXPEND_VOXEL_ROW) + row_offset;
    wire hash_point_number_valid = (hash_out_point_number <= MAX_VOXEL_POINTS) && (hash_out_point_number != {VN_WIDTH{1'b0}});
    wire [7:0] max_voxel_points_u8 = {{(8 - VN_WIDTH) {1'b0}}, MAX_VOXEL_POINTS};
    wire [7:0] hash_point_number_u8 = {{(8 - VN_WIDTH) {1'b0}}, hash_out_point_number};
    wire [7:0] full_point_count_cur = full_point_count[hash_out_idx];
    wire hash_point_number_full = (hash_out_point_number == MAX_VOXEL_POINTS) && (full_point_count_cur >= max_voxel_points_u8);
    wire [7:0] full_point_count_inc = (full_point_count_cur == 8'hff) ? 8'hff : (full_point_count_cur + 8'd1);
    wire [7:0] full_point_count_next = hash_point_number_full ? full_point_count_inc : hash_point_number_u8;
    wire [5:0] full_keep_threshold_sel = full_keep_threshold(full_point_count_next);
    wire full_voxel_random_write = !hash_point_number_full || (full_rand_value_d3 <= full_keep_threshold_sel);
    wire [BRAM_ADDR_WIDTH-1:0] bram_write_addr_sel =
        hash_point_number_full ? ((hash_out_idx * EXPEND_VOXEL_ROW) + full_rand_row_d3) : bram_row_addr;
    wire [$clog2(POINTS_PER_ROW)-1:0] bram_slot_idx_sel = hash_point_number_full ? full_rand_slot_d3 : bram_slot_idx;

    localparam integer CHUNKS_PER_POINT = PT_WIDTH_PER / BYTE_WIDTH;  // 72/9 = 8
    function [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] slot_bwen_64;
        input [3:0] slot;
        reg [BRAM_DATA_WIDTH/BYTE_WIDTH-1:0] mask;
        begin
            mask                                          = {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            mask[slot*CHUNKS_PER_POINT+:CHUNKS_PER_POINT] = {{CHUNKS_PER_POINT{1'b1}}};
            slot_bwen_64                                  = mask;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            bram_voxelpoint_addr <= {BRAM_ADDR_WIDTH{1'b0}};
            for (full_cnt_i = 0; full_cnt_i < (1 << HASH_ADDR_WIDTH); full_cnt_i = full_cnt_i + 1) begin
                full_point_count[full_cnt_i] <= 8'd0;
            end
        end else begin
            bram_voxelpoint_wr   <= 1'b0;
            bram_voxelpoint_bwen <= {BRAM_DATA_WIDTH / BYTE_WIDTH{1'b0}};
            // ??? hash_busy ????????? pop ???????
            if (pipe_bram_stall) begin
                // hash_found ?? hit ????????table_full ? found=0
                // ????????????? 32 ?
                if (!pproc_drop_d3 && hash_point_number_valid) begin
                    full_point_count[hash_out_idx] <= full_point_count_next;
                    if (full_voxel_random_write) begin
                        bram_voxelpoint_wr     <= 1'b1;
                        bram_voxelpoint_addr   <= bram_write_addr_sel;
                        bram_voxelpoint_wrdata <= {POINTS_PER_ROW{pproc_d3}};
                        bram_voxelpoint_bwen   <= slot_bwen_64(bram_slot_idx_sel);
                    end
                end
            end
        end
    end




`ifndef SYNTHESIS
    // ===================================================================
    // ?????????????? Intensity ??????
    // ===================================================================
    real dbg_pt_x, dbg_pt_y, dbg_pt_z, dbg_pt_i;
    real orig_float_x, orig_float_y, orig_float_z, orig_float_i;

    // ? Verilog-2001 ??? IEEE-754 ???????
    function real decode_float32(input [31:0] bits);
        reg        sign;
        reg [ 7:0] exp;
        reg [22:0] frac;
        begin
            sign = bits[31];
            exp  = bits[30:23];
            frac = bits[22:0];

            if (exp == 8'h00) begin
                decode_float32 = 0.0;
            end else begin
                // ??: (-1)^sign * 2^(exp-127) * (1 + frac / 2^23)
                // 2^23 = 8388608.0
                decode_float32 = (sign ? -1.0 : 1.0) * (2.0 ** (exp - 127)) * (1.0 + ($itor(frac) / 8388608.0));
            end
        end
    endfunction

    // always @(posedge clk) begin
    //     // ?????????????????
    //     if (rst_n && point_raw_vld) begin
    //         // 1. ???????????
    //         dbg_pt_x = $itor($signed(pt_x_fix16)) / 256.0;
    //         dbg_pt_y = $itor($signed(pt_y_fix16)) / 256.0;
    //         dbg_pt_z = $itor($signed(pt_z_fix16)) / 256.0;
    //         dbg_pt_i = $itor($signed(pt_intensity_fix16)) / 256.0;

    //         // 2. ????????????? $bitstoshortreal ??
    //         orig_float_x = decode_float32(pt_x_float32);
    //         orig_float_y = decode_float32(pt_y_float32);
    //         orig_float_z = decode_float32(pt_z_float32);
    //         orig_float_i = decode_float32(pt_intensity_float32);

    //         // 1. ?? X ??: [-54.0, 54.0]
    //         if (dbg_pt_x < -54.0 || dbg_pt_x > 54.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_x OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_x, pt_x_fix16 & 16'hFFFF, orig_float_x);
    //         end

    //         // 2. ?? Y ??: [-54.0, 54.0]
    //         if (dbg_pt_y < -54.0 || dbg_pt_y > 54.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_y OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_y, pt_y_fix16 & 16'hFFFF, orig_float_y);
    //         end

    //         // 3. ?? Z ??: [-5.0, 3.0]
    //         if (dbg_pt_z < -5.0 || dbg_pt_z > 3.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_z OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_z, pt_z_fix16 & 16'hFFFF, orig_float_z);
    //         end

    //         // 4. ?? Intensity: [0, 256.0]
    //         if (dbg_pt_i < 0.0 || dbg_pt_i > 256.0) begin
    //             $display(
    //                 "[RANGE WARNING][%0t] pt_intensity OUT OF BOUNDS! Fixed Val: %0.3f (Q8.8: 0x%04X) | Orig Float32: %0.3f",
    //                 $time, dbg_pt_i, pt_intensity_fix16 & 16'hFFFF, orig_float_i);
    //         end
    //     end
    // end


    // reg  [31:0] stall_cnt;  // 32?????????????
    // ?????????? (point_raw_vld && keep_point) ? Hash ?? Ready
    // wire        is_stalled = (point_raw_vld && keep_point) && (!hash_req_ready);
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         stall_cnt <= 32'd0;
    //     end else begin
    //         if (is_stalled) begin
    //             stall_cnt <= stall_cnt + 1'b1;
    //         end
    //     end
    // end

    wire test_coor = (cand_y_fix20_data == 20'h15c28);
    reg [VOXEL_WIDTH-1:0] voxel_x_test = 11'd1;
    reg [VOXEL_WIDTH-1:0] voxel_y_test = 11'd433;

    wire test_reg = (single_voxel_x == voxel_x_test) && (single_voxel_y == voxel_y_test);
    wire test_vector = (bram_voxelpoint_addr == 9'd80) && ((bram_voxelpoint_bwen == 80'h000000000000000000ff));
    // wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_x = pproc_d3_voxel[VOXEL_WIDTH*2-1:VOXEL_WIDTH];
    // wire [VOXEL_WIDTH-1:0] pproc_d3_voxel_y = pproc_d3_voxel[VOXEL_WIDTH-1:0];
    // reg [1:0] test_reg_rise_cnt;
    // reg test_reg_d;

    // // test_reg ??????????? BRAM ??????
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         test_reg_rise_cnt <= 2'd0;
    //         test_reg_d        <= 1'b0;
    //     end else begin
    //         test_reg_d <= test_reg;
    //         if (test_reg && !test_reg_d) begin
    //             if (test_reg_rise_cnt == 2'd2) begin
    //                 $display("[voxel_coor_pipe][%0d] TEST_REG_3RD: mem[60]=0x%0h", u_hash_table_tombstone.global_timer,
    //                          tb_voxelize.dut.u_voxelpoint_bram.mem[60]);
    //             end
    //             if (test_reg_rise_cnt != 2'd3) begin
    //                 test_reg_rise_cnt <= test_reg_rise_cnt + 1'b1;
    //             end
    //         end
    //     end
    // end

    // // ????????? BRAM ???/?????? pproc_d3_voxel ??????
    // always @(posedge clk) begin
    //     if (rst_n && test_vector) begin
    //         $display("[voxel_coor_pipe][%0d] BRAM_HIT: addr=%0d bwen=0x%020h pproc_d3_voxel=0x%0h voxel_x=%0d voxel_y=%0d",
    //                  u_hash_table_tombstone.global_timer, bram_voxelpoint_addr, bram_voxelpoint_bwen, pproc_d3_voxel,
    //                  pproc_d3_voxel_x, pproc_d3_voxel_y);
    //     end
    // end

    // // ????????????????????a????
    // always @(posedge clk) begin
    //     if (rst_n && point_raw_vld && keep_point && test_reg) begin
    //         $display(
    //             "[voxel_coor_pipe][%0t] BREAK: voxel_x=%0d voxel_y=%0d global_timer=%0d pt_x=%0f pt_y=%0f pt_z=%0f (Q8.8)",
    //             $time, voxel_x, voxel_y, u_hash_table_tombstone.global_timer, $itor($signed(pt_x_fix16)) / 256.0, $itor
    //             ($signed(pt_y_fix16)) / 256.0, $itor($signed(pt_z_fix16)) / 256.0);
    //         // $stop;
    //     end
    // end


    // wire test_reg_voxel = (pproc_d3_voxel == {voxel_x_test, voxel_y_test});

`endif

endmodule
