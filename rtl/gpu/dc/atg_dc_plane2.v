/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_dc_plane2: Display Controller Multi-Plane Hardware Overlay Engine.
 *
 * Implements:
 * 1. Independent video/UI overlay plane (Plane 2).
 * 2. Hardware NV12 YUV 4:2:0 -> RGB color space converter:
 *    R = Y + 1.402 * (V - 128)
 *    G = Y - 0.344 * (U - 128) - 0.714 * (V - 128)
 *    B = Y + 1.772 * (U - 128)
 *    All multiplications performed using am_mul, additions/subtractions
 *    via am_add / am_sub.
 * 3. Color Key matching and transparency pass-through.
 * 4. Per-pixel / global alpha blending over primary plane:
 *    C_out = (alpha * C_overlay + (255 - alpha) * C_primary + 128) >> 8
 * 5. Optional AMB client DMA interface and direct video stream feed.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_dc_plane2 #(
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5
) (
	input  wire        clk,
	input  wire        rst_n,
	input  wire        enable,

	/* Configuration */
	input  wire        plane_en,
	input  wire [11:0] pos_x,
	input  wire [11:0] pos_y,
	input  wire [11:0] width,
	input  wire [11:0] height,
	input  wire [7:0]  alpha,
	input  wire        colorkey_en,
	input  wire [23:0] colorkey,
	input  wire        yuv_mode, /* 1: NV12 YUV, 0: RGB */

	/* Primary plane pixel stream */
	input  wire [11:0] cur_x,
	input  wire [11:0] cur_y,
	input  wire [23:0] prim_rgb,
	input  wire        prim_valid,
	output reg  [23:0] out_rgb,
	output reg         out_valid,

	/* Video overlay feed (direct stream or decoded from DMA) */
	input  wire        ov_valid,
	output wire        ov_ready,
	input  wire [7:0]  ov_y,
	input  wire [7:0]  ov_u,
	input  wire [7:0]  ov_v,
	input  wire [23:0] ov_rgb,

	/* AMB client port (for autonomous video DMA fetch) */
	output wire        req_valid,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire        req_ready,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        req_write,
	output wire [ADDR_W-LINE_SHIFT-1:0] req_line,
	output wire [LINE_BYTES-1:0] req_be,
	output wire [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0]  req_tag,
	input  wire        rsp_valid,
	input  wire        rsp_write,
	input  wire [7:0]  rsp_tag,
	input  wire [LINE_BYTES*8-1:0] rsp_rdata,
	input  wire        rsp_error
);
	localparam LW = LINE_BYTES * 8;
	localparam AW = ADDR_W - LINE_SHIFT;

	assign req_valid = 1'b0;
	assign req_write = 1'b0;
	assign req_line  = {AW{1'b0}};
	assign req_be    = {LINE_BYTES{1'b0}};
	assign req_wdata = {LW{1'b0}};
	assign req_tag   = 8'd0;
	assign ov_ready  = prim_valid;

	/* Bounding box end coordinates */
	wire [11:0] end_x, end_y;
	wire [1:0] box_co_unused;
	am_add #(.W(12)) a_end_x (.a(pos_x), .b(width),  .ci(1'b0), .s(end_x), .co(box_co_unused[0]));
	am_add #(.W(12)) a_end_y (.a(pos_y), .b(height), .ci(1'b0), .s(end_y), .co(box_co_unused[1]));

`ifdef SYNTH_KINTEX7
	reg [11:0] end_x_r, end_y_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			end_x_r <= 12'd0;
			end_y_r <= 12'd0;
		end else begin
			end_x_r <= end_x;
			end_y_r <= end_y;
		end
	end
	wire in_box = (cur_x >= pos_x) && (cur_x < end_x_r) && (cur_y >= pos_y) && (cur_y < end_y_r);
`elsif SYNTH_KINTEX_ULTRASCALE
	reg [11:0] end_x_r, end_y_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			end_x_r <= 12'd0;
			end_y_r <= 12'd0;
		end else begin
			end_x_r <= end_x;
			end_y_r <= end_y;
		end
	end
	wire in_box = (cur_x >= pos_x) && (cur_x < end_x_r) && (cur_y >= pos_y) && (cur_y < end_y_r);
`elsif SYNTH_ASIC
	reg [11:0] end_x_r, end_y_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			end_x_r <= 12'd0;
			end_y_r <= 12'd0;
		end else begin
			end_x_r <= end_x;
			end_y_r <= end_y;
		end
	end
	wire gx_hi_gt = (cur_x[11:6] > pos_x[11:6]);
	wire gx_hi_eq = (cur_x[11:6] == pos_x[11:6]);
	wire gx_lo_ge = (cur_x[5:0] >= pos_x[5:0]);
	wire lx_hi_lt = (cur_x[11:6] < end_x_r[11:6]);
	wire lx_hi_eq = (cur_x[11:6] == end_x_r[11:6]);
	wire lx_lo_lt = (cur_x[5:0] < end_x_r[5:0]);
	wire gy_hi_gt = (cur_y[11:6] > pos_y[11:6]);
	wire gy_hi_eq = (cur_y[11:6] == pos_y[11:6]);
	wire gy_lo_ge = (cur_y[5:0] >= pos_y[5:0]);
	wire ly_hi_lt = (cur_y[11:6] < end_y_r[11:6]);
	wire ly_hi_eq = (cur_y[11:6] == end_y_r[11:6]);
	wire ly_lo_lt = (cur_y[5:0] < end_y_r[5:0]);
	reg in_x_q, in_y_q;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			in_x_q <= 1'b0;
			in_y_q <= 1'b0;
		end else begin
			in_x_q <= (gx_hi_gt || (gx_hi_eq && gx_lo_ge)) && (lx_hi_lt || (lx_hi_eq && lx_lo_lt));
			in_y_q <= (gy_hi_gt || (gy_hi_eq && gy_lo_ge)) && (ly_hi_lt || (ly_hi_eq && ly_lo_lt));
		end
	end
	wire in_box = in_x_q && in_y_q;
`else
	wire in_box = (cur_x >= pos_x) && (cur_x < end_x) && (cur_y >= pos_y) && (cur_y < end_y);
`endif

	/*
	 * YUV to RGB Conversion:
	 * diff_u = U - 128, diff_v = V - 128
	 * R_raw = (Y << 8) + 359 * diff_v
	 * G_raw = (Y << 8) - 88 * diff_u - 183 * diff_v
	 * B_raw = (Y << 8) + 454 * diff_u
	 */
	wire [8:0] diff_u, diff_v;
	wire [1:0] diff_bo_unused;
	am_sub #(.W(9)) s_diff_u (.a({1'b0, ov_u}), .b(9'd128), .bi(1'b0), .d(diff_u), .bo(diff_bo_unused[0]));
	am_sub #(.W(9)) s_diff_v (.a({1'b0, ov_v}), .b(9'd128), .bi(1'b0), .d(diff_v), .bo(diff_bo_unused[1]));

`ifdef SYNTH_KINTEX7
	localparam P2_MUL_LAT = 2;
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam P2_MUL_LAT = 2;
`elsif SYNTH_ASIC
	localparam P2_MUL_LAT = 3;
	reg [8:0] diff_u_q, diff_v_q;
	reg [7:0] ov_y_q, ov_y_s1, ov_y_s2, ov_y_s3;
	reg [23:0] ov_rgb_s0, ov_rgb_s1, ov_rgb_s2, ov_rgb_s3, ov_rgb_s4, ov_rgb_s5, ov_rgb_s6;
	reg [23:0] prim_rgb_s0, prim_rgb_s1, prim_rgb_s2, prim_rgb_s3, prim_rgb_s4, prim_rgb_s5, prim_rgb_s6;
	reg prim_v_s0, prim_v_s1, prim_v_s2, prim_v_s3, prim_v_s4, prim_v_s5, prim_v_s6;
	reg in_box_s1, in_box_s2, in_box_s3, in_box_s4, in_box_s5, in_box_s6;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			diff_u_q <= 9'd0;
			diff_v_q <= 9'd0;
			ov_y_q   <= 8'd0;
			ov_y_s1  <= 8'd0;
			ov_y_s2  <= 8'd0;
			ov_y_s3  <= 8'd0;
			prim_v_s0 <= 1'b0; prim_v_s1 <= 1'b0; prim_v_s2 <= 1'b0; prim_v_s3 <= 1'b0;
			prim_v_s4 <= 1'b0; prim_v_s5 <= 1'b0; prim_v_s6 <= 1'b0;
			in_box_s1 <= 1'b0; in_box_s2 <= 1'b0; in_box_s3 <= 1'b0;
			in_box_s4 <= 1'b0; in_box_s5 <= 1'b0; in_box_s6 <= 1'b0;
		end else begin
			diff_u_q <= diff_u;
			diff_v_q <= diff_v;
			ov_y_q   <= ov_y;
			ov_y_s1  <= ov_y_q;
			ov_y_s2  <= ov_y_s1;
			ov_y_s3  <= ov_y_s2;
			prim_v_s0 <= prim_valid; prim_v_s1 <= prim_v_s0; prim_v_s2 <= prim_v_s1; prim_v_s3 <= prim_v_s2;
			prim_v_s4 <= prim_v_s3;  prim_v_s5 <= prim_v_s4; prim_v_s6 <= prim_v_s5;
			in_box_s1 <= in_box;    in_box_s2 <= in_box_s1; in_box_s3 <= in_box_s2;
			in_box_s4 <= in_box_s3; in_box_s5 <= in_box_s4; in_box_s6 <= in_box_s5;
		end
	end
	always @(posedge clk) begin
		ov_rgb_s0   <= ov_rgb;   ov_rgb_s1   <= ov_rgb_s0;   ov_rgb_s2   <= ov_rgb_s1;
		ov_rgb_s3   <= ov_rgb_s2; ov_rgb_s4   <= ov_rgb_s3;  ov_rgb_s5   <= ov_rgb_s4;  ov_rgb_s6   <= ov_rgb_s5;
		prim_rgb_s0 <= prim_rgb; prim_rgb_s1 <= prim_rgb_s0; prim_rgb_s2 <= prim_rgb_s1;
		prim_rgb_s3 <= prim_rgb_s2; prim_rgb_s4 <= prim_rgb_s3; prim_rgb_s5 <= prim_rgb_s4; prim_rgb_s6 <= prim_rgb_s5;
	end
`else
	localparam P2_MUL_LAT = 0;
`endif

`ifdef SYNTH_ASIC
	wire [8:0] m_diff_u = diff_u_q;
	wire [8:0] m_diff_v = diff_v_q;
	wire [7:0] m_ov_y   = ov_y_s3;
`else
	wire [8:0] m_diff_u = diff_u;
	wire [8:0] m_diff_v = diff_v;
	wire [7:0] m_ov_y   = ov_y;
`endif

	wire [18:0] p_rv, p_gu, p_gv, p_bu;
	/* Signed multipliers */
	am_mul #(.AW(9), .BW(10), .SIGNED(1), .LAT(P2_MUL_LAT)) u_mul_rv (.clk(clk), .en(1'b1), .a(m_diff_v), .b(10'sd359),  .p(p_rv));
	am_mul #(.AW(9), .BW(10), .SIGNED(1), .LAT(P2_MUL_LAT)) u_mul_gu (.clk(clk), .en(1'b1), .a(m_diff_u), .b(-10'sd88),  .p(p_gu));
	am_mul #(.AW(9), .BW(10), .SIGNED(1), .LAT(P2_MUL_LAT)) u_mul_gv (.clk(clk), .en(1'b1), .a(m_diff_v), .b(-10'sd183), .p(p_gv));
	am_mul #(.AW(9), .BW(10), .SIGNED(1), .LAT(P2_MUL_LAT)) u_mul_bu (.clk(clk), .en(1'b1), .a(m_diff_u), .b(10'sd454),  .p(p_bu));

	wire [18:0] y_scaled = {3'd0, m_ov_y, 8'd0};

`ifdef SYNTH_ASIC
	/* Stage 4: YUV CSA & register operands; Stages 5-6: split 8+8+3 addition (<= 10 cmos3/stage) */
	/* verilator lint_off UNUSEDSIGNAL */
	wire [18:0] g_csa_c_w = (y_scaled & p_gu) | (y_scaled & p_gv) | (p_gu & p_gv);
	wire [18:0] g_csa_s_w = y_scaled ^ p_gu ^ p_gv;
	reg [18:0] r_a_s4, r_b_s4, g_s_s4, g_c_s4, b_a_s4, b_b_s4;
	always @(posedge clk) begin
		r_a_s4 <= y_scaled;
		r_b_s4 <= p_rv;
		g_s_s4 <= g_csa_s_w;
		g_c_s4 <= {g_csa_c_w[17:0], 1'b0};
		b_a_s4 <= y_scaled;
		b_b_s4 <= p_bu;
	end

	wire [7:0] r_lo_unused, g_lo_unused, b_lo_unused;
	wire r_co8_w, g_co8_w, b_co8_w;
	am_add #(.W(8)) a_r_lo (.a(r_a_s4[7:0]), .b(r_b_s4[7:0]), .ci(1'b0), .s(r_lo_unused), .co(r_co8_w));
	am_add #(.W(8)) a_g_lo (.a(g_s_s4[7:0]), .b(g_c_s4[7:0]), .ci(1'b0), .s(g_lo_unused), .co(g_co8_w));
	am_add #(.W(8)) a_b_lo (.a(b_a_s4[7:0]), .b(b_b_s4[7:0]), .ci(1'b0), .s(b_lo_unused), .co(b_co8_w));

	reg r_co8_q, g_co8_q, b_co8_q;
	reg [18:8] r_a_s5, r_b_s5, g_s_s5, g_c_s5, b_a_s5, b_b_s5;
	always @(posedge clk) begin
		r_co8_q <= r_co8_w; r_a_s5 <= r_a_s4[18:8]; r_b_s5 <= r_b_s4[18:8];
		g_co8_q <= g_co8_w; g_s_s5 <= g_s_s4[18:8]; g_c_s5 <= g_c_s4[18:8];
		b_co8_q <= b_co8_w; b_a_s5 <= b_a_s4[18:8]; b_b_s5 <= b_b_s4[18:8];
	end

	wire [7:0] r_mid_w, g_mid_w, b_mid_w;
	wire r_co16, g_co16, b_co16;
	wire [2:0] r_top0, r_top1, g_top0, g_top1, b_top0, b_top1;
	wire [5:0] yuv_co_hi_unused;
	am_add #(.W(8)) a_r_mid  (.a(r_a_s5[15:8]),  .b(r_b_s5[15:8]),  .ci(r_co8_q), .s(r_mid_w), .co(r_co16));
	am_add #(.W(3)) a_r_top0 (.a(r_a_s5[18:16]), .b(r_b_s5[18:16]), .ci(1'b0),    .s(r_top0),  .co(yuv_co_hi_unused[0]));
	am_add #(.W(3)) a_r_top1 (.a(r_a_s5[18:16]), .b(r_b_s5[18:16]), .ci(1'b1),    .s(r_top1),  .co(yuv_co_hi_unused[1]));
	am_add #(.W(8)) a_g_mid  (.a(g_s_s5[15:8]),  .b(g_c_s5[15:8]),  .ci(g_co8_q), .s(g_mid_w), .co(g_co16));
	am_add #(.W(3)) a_g_top0 (.a(g_s_s5[18:16]), .b(g_c_s5[18:16]), .ci(1'b0),    .s(g_top0),  .co(yuv_co_hi_unused[2]));
	am_add #(.W(3)) a_g_top1 (.a(g_s_s5[18:16]), .b(g_c_s5[18:16]), .ci(1'b1),    .s(g_top1),  .co(yuv_co_hi_unused[3]));
	am_add #(.W(8)) a_b_mid  (.a(b_a_s5[15:8]),  .b(b_b_s5[15:8]),  .ci(b_co8_q), .s(b_mid_w), .co(b_co16));
	am_add #(.W(3)) a_b_top0 (.a(b_a_s5[18:16]), .b(b_b_s5[18:16]), .ci(1'b0),    .s(b_top0),  .co(yuv_co_hi_unused[4]));
	am_add #(.W(3)) a_b_top1 (.a(b_a_s5[18:16]), .b(b_b_s5[18:16]), .ci(1'b1),    .s(b_top1),  .co(yuv_co_hi_unused[5]));
	/* verilator lint_on UNUSEDSIGNAL */

	reg [10:0] r_raw_hi_q, g_raw_hi_q, b_raw_hi_q;
	always @(posedge clk) begin
		r_raw_hi_q <= {r_co16 ? r_top1 : r_top0, r_mid_w};
		g_raw_hi_q <= {g_co16 ? g_top1 : g_top0, g_mid_w};
		b_raw_hi_q <= {b_co16 ? b_top1 : b_top0, b_mid_w};
	end

	wire [7:0] r_yuv = r_raw_hi_q[10] ? 8'd0 : (r_raw_hi_q[9:8] != 2'd0 ? 8'hFF : r_raw_hi_q[7:0]);
	wire [7:0] g_yuv = g_raw_hi_q[10] ? 8'd0 : (g_raw_hi_q[9:8] != 2'd0 ? 8'hFF : g_raw_hi_q[7:0]);
	wire [7:0] b_yuv = b_raw_hi_q[10] ? 8'd0 : (b_raw_hi_q[9:8] != 2'd0 ? 8'hFF : b_raw_hi_q[7:0]);
	wire [23:0] sel_ov_rgb = yuv_mode ? {r_yuv, g_yuv, b_yuv} : ov_rgb_s6;
`else
	/* verilator lint_off UNUSEDSIGNAL */
	wire [18:0] r_raw, g_raw, b_raw;
	wire [18:0] g_csa_c = (y_scaled & p_gu) | (y_scaled & p_gv) | (p_gu & p_gv);
	/* verilator lint_on UNUSEDSIGNAL */
	wire [18:0] g_csa_s = y_scaled ^ p_gu ^ p_gv;
	wire [3:0] yuv_co_unused;
	assign yuv_co_unused[1] = 1'b0;
	am_add #(.W(19)) a_r_raw   (.a(y_scaled), .b(p_rv),                 .ci(1'b0), .s(r_raw), .co(yuv_co_unused[0]));
	am_add #(.W(19)) a_g_raw   (.a(g_csa_s),  .b({g_csa_c[17:0], 1'b0}), .ci(1'b0), .s(g_raw), .co(yuv_co_unused[2]));
	am_add #(.W(19)) a_b_raw   (.a(y_scaled), .b(p_bu),                 .ci(1'b0), .s(b_raw), .co(yuv_co_unused[3]));

	/* Clamping to [0, 255] */
	wire [7:0] r_yuv = r_raw[18] ? 8'd0 : (r_raw[17:16] != 2'd0 ? 8'hFF : r_raw[15:8]);
	wire [7:0] g_yuv = g_raw[18] ? 8'd0 : (g_raw[17:16] != 2'd0 ? 8'hFF : g_raw[15:8]);
	wire [7:0] b_yuv = b_raw[18] ? 8'd0 : (b_raw[17:16] != 2'd0 ? 8'hFF : b_raw[15:8]);

	/* Selected overlay RGB */
`ifdef SYNTH_KINTEX7
	wire yuv_mode_l, yuv_mode_h;
	LUT1 #(.INIT(2'b10)) u_buf_yl (.I0(yuv_mode), .O(yuv_mode_l));
	LUT1 #(.INIT(2'b10)) u_buf_yh (.I0(yuv_mode), .O(yuv_mode_h));
	wire [23:0] sel_ov_rgb = {
		yuv_mode_h ? {r_yuv, g_yuv[7:4]} : ov_rgb[23:12],
		yuv_mode_l ? {g_yuv[3:0], b_yuv} : ov_rgb[11:0]
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire yuv_mode_l, yuv_mode_h;
	assign yuv_mode_l = yuv_mode;
	assign yuv_mode_h = yuv_mode;
	wire [23:0] sel_ov_rgb = {
		yuv_mode_h ? {r_yuv, g_yuv[7:4]} : ov_rgb[23:12],
		yuv_mode_l ? {g_yuv[3:0], b_yuv} : ov_rgb[11:0]
	};
`else
	wire [23:0] sel_ov_rgb = yuv_mode ? {r_yuv, g_yuv, b_yuv} : ov_rgb;
`endif
`endif

`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [23:0] sel_ov_rgb_q;
	(* max_fanout = 16 *) reg [23:0] prim_rgb_q;
	(* max_fanout = 16 *) reg [7:0]  alpha_q;
	always @(posedge clk) begin
		sel_ov_rgb_q <= sel_ov_rgb;
		prim_rgb_q   <= prim_rgb;
		alpha_q      <= alpha;
	end
	wire [23:0] p2_sel_ov_rgb = sel_ov_rgb_q;
	wire [23:0] p2_prim_rgb   = prim_rgb_q;
	wire [7:0]  p2_alpha      = alpha_q;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [23:0] sel_ov_rgb_q;
	(* max_fanout = 16 *) reg [23:0] prim_rgb_q;
	(* max_fanout = 16 *) reg [7:0]  alpha_q;
	always @(posedge clk) begin
		sel_ov_rgb_q <= sel_ov_rgb;
		prim_rgb_q   <= prim_rgb;
		alpha_q      <= alpha;
	end
	wire [23:0] p2_sel_ov_rgb = sel_ov_rgb_q;
	wire [23:0] p2_prim_rgb   = prim_rgb_q;
	wire [7:0]  p2_alpha      = alpha_q;
`elsif SYNTH_ASIC
	(* keep = "true", dont_touch = "true" *) reg [23:0] sel_ov_rgb_q;
	(* keep = "true", dont_touch = "true" *) reg [23:0] prim_rgb_q;
	(* keep = "true", dont_touch = "true" *) reg [7:0]  alpha_q;
	reg prim_v_s7, in_box_s7;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			prim_v_s7 <= 1'b0;
			in_box_s7 <= 1'b0;
		end else begin
			prim_v_s7 <= prim_v_s6;
			in_box_s7 <= in_box_s6;
		end
	end
	always @(posedge clk) begin
		sel_ov_rgb_q <= sel_ov_rgb;
		prim_rgb_q   <= prim_rgb_s6;
		alpha_q      <= alpha;
	end
	wire [23:0] p2_sel_ov_rgb = sel_ov_rgb_q;
	wire [23:0] p2_prim_rgb   = prim_rgb_q;
	wire [7:0]  p2_alpha      = alpha_q;
`else
	wire [23:0] p2_sel_ov_rgb = sel_ov_rgb;
	wire [23:0] p2_prim_rgb   = prim_rgb;
	wire [7:0]  p2_alpha      = alpha;
`endif

	/* Color key matching */
	wire colorkey_match = colorkey_en && (p2_sel_ov_rgb == colorkey);
`ifdef SYNTH_ASIC
	wire active_overlay = plane_en && in_box_s7 && !colorkey_match;
`else
	wire active_overlay = plane_en && in_box && !colorkey_match;
`endif

	/* Alpha Blending */
	wire [7:0] inv_alpha;
	wire alpha_bo_unused;
	am_sub #(.W(8)) s_inv_alpha (.a(8'hFF), .b(p2_alpha), .bi(1'b0), .d(inv_alpha), .bo(alpha_bo_unused));
`ifdef SYNTH_ASIC
	reg [7:0] inv_alpha_q;
	reg [7:0] p2_alpha_q;
	reg [23:0] p2_sel_ov_rgb_q, p2_prim_rgb_q;
	reg active_overlay_q, b_prim_v_q;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			inv_alpha_q <= 8'd0;
			p2_alpha_q <= 8'd0;
			p2_sel_ov_rgb_q <= 24'd0;
			p2_prim_rgb_q <= 24'd0;
			active_overlay_q <= 1'b0;
			b_prim_v_q <= 1'b0;
		end else begin
			inv_alpha_q <= inv_alpha;
			p2_alpha_q <= p2_alpha;
			p2_sel_ov_rgb_q <= p2_sel_ov_rgb;
			p2_prim_rgb_q <= p2_prim_rgb;
			active_overlay_q <= active_overlay;
			b_prim_v_q <= prim_v_s7;
		end
	end
	wire [7:0] b_alpha = p2_alpha_q;
	wire [7:0] b_inv_alpha = inv_alpha_q;
	wire [23:0] b_sel_ov_rgb = p2_sel_ov_rgb_q;
	wire [23:0] b_prim_rgb = p2_prim_rgb_q;
	wire b_active_overlay = active_overlay_q;
`else
	wire [7:0] b_alpha = p2_alpha;
	wire [7:0] b_inv_alpha = inv_alpha;
	wire [23:0] b_sel_ov_rgb = p2_sel_ov_rgb;
	wire [23:0] b_prim_rgb = p2_prim_rgb;
	wire b_active_overlay = active_overlay;
`endif

	/* Red channel blending */
	wire [15:0] p_r_ov, p_r_prim;
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_r_ov   (.clk(clk), .en(1'b1), .a(b_alpha),     .b(b_sel_ov_rgb[23:16]), .p(p_r_ov));
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_r_prim (.clk(clk), .en(1'b1), .a(b_inv_alpha), .b(b_prim_rgb[23:16]),   .p(p_r_prim));

	/* Green channel blending */
	wire [15:0] p_g_ov, p_g_prim;
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_g_ov   (.clk(clk), .en(1'b1), .a(b_alpha),     .b(b_sel_ov_rgb[15:8]),  .p(p_g_ov));
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_g_prim (.clk(clk), .en(1'b1), .a(b_inv_alpha), .b(b_prim_rgb[15:8]),    .p(p_g_prim));

	/* Blue channel blending */
	wire [15:0] p_b_ov, p_b_prim;
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_b_ov   (.clk(clk), .en(1'b1), .a(b_alpha),     .b(b_sel_ov_rgb[7:0]),   .p(p_b_ov));
	am_mul #(.AW(8), .BW(8), .SIGNED(0), .LAT(P2_MUL_LAT)) u_mul_b_prim (.clk(clk), .en(1'b1), .a(b_inv_alpha), .b(b_prim_rgb[7:0]),     .p(p_b_prim));

`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [15:0] p_r_ov_q, p_r_prim_q;
	(* max_fanout = 16 *) reg [15:0] p_g_ov_q, p_g_prim_q;
	(* max_fanout = 16 *) reg [15:0] p_b_ov_q, p_b_prim_q;
	(* max_fanout = 16 *) reg [23:0] sel_ov_rgb_q2, prim_rgb_q2;
	(* max_fanout = 16 *) reg [7:0] alpha_q2;
	(* max_fanout = 16 *) reg active_overlay_q2;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			p_r_ov_q <= 16'd0; p_r_prim_q <= 16'd0;
			p_g_ov_q <= 16'd0; p_g_prim_q <= 16'd0;
			p_b_ov_q <= 16'd0; p_b_prim_q <= 16'd0;
			active_overlay_q2 <= 1'b0;
		end else begin
			p_r_ov_q <= p_r_ov; p_r_prim_q <= p_r_prim;
			p_g_ov_q <= p_g_ov; p_g_prim_q <= p_g_prim;
			p_b_ov_q <= p_b_ov; p_b_prim_q <= p_b_prim;
			active_overlay_q2 <= active_overlay;
		end
	end
	always @(posedge clk) begin
		sel_ov_rgb_q2 <= p2_sel_ov_rgb;
		prim_rgb_q2 <= p2_prim_rgb;
		alpha_q2 <= p2_alpha;
	end
	wire [15:0] d_r_ov = p_r_ov_q, d_r_prim = p_r_prim_q;
	wire [15:0] d_g_ov = p_g_ov_q, d_g_prim = p_g_prim_q;
	wire [15:0] d_b_ov = p_b_ov_q, d_b_prim = p_b_prim_q;
	wire [23:0] d_sel_ov_rgb = sel_ov_rgb_q2, d_prim_rgb = prim_rgb_q2;
	wire [7:0]  d_alpha = alpha_q2;
	wire d_active_overlay = active_overlay_q2;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [15:0] p_r_ov_q, p_r_prim_q;
	(* max_fanout = 16 *) reg [15:0] p_g_ov_q, p_g_prim_q;
	(* max_fanout = 16 *) reg [15:0] p_b_ov_q, p_b_prim_q;
	(* max_fanout = 16 *) reg [23:0] sel_ov_rgb_q2, prim_rgb_q2;
	(* max_fanout = 16 *) reg [7:0] alpha_q2;
	(* max_fanout = 16 *) reg active_overlay_q2;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			p_r_ov_q <= 16'd0; p_r_prim_q <= 16'd0;
			p_g_ov_q <= 16'd0; p_g_prim_q <= 16'd0;
			p_b_ov_q <= 16'd0; p_b_prim_q <= 16'd0;
			active_overlay_q2 <= 1'b0;
		end else begin
			p_r_ov_q <= p_r_ov; p_r_prim_q <= p_r_prim;
			p_g_ov_q <= p_g_ov; p_g_prim_q <= p_g_prim;
			p_b_ov_q <= p_b_ov; p_b_prim_q <= p_b_prim;
			active_overlay_q2 <= active_overlay;
		end
	end
	always @(posedge clk) begin
		sel_ov_rgb_q2 <= p2_sel_ov_rgb;
		prim_rgb_q2 <= p2_prim_rgb;
		alpha_q2 <= p2_alpha;
	end
	wire [15:0] d_r_ov = p_r_ov_q, d_r_prim = p_r_prim_q;
	wire [15:0] d_g_ov = p_g_ov_q, d_g_prim = p_g_prim_q;
	wire [15:0] d_b_ov = p_b_ov_q, d_b_prim = p_b_prim_q;
	wire [23:0] d_sel_ov_rgb = sel_ov_rgb_q2, d_prim_rgb = prim_rgb_q2;
	wire [7:0]  d_alpha = alpha_q2;
	wire d_active_overlay = active_overlay_q2;
`elsif SYNTH_ASIC
	reg [15:0] p_r_ov_q, p_r_prim_q;
	reg [15:0] p_g_ov_q, p_g_prim_q;
	reg [15:0] p_b_ov_q, p_b_prim_q;
	reg [23:0] sel_ov_rgb_q1, prim_rgb_q1, sel_ov_rgb_q2, prim_rgb_q2, sel_ov_rgb_q3, prim_rgb_q3;
	reg [7:0] alpha_q1, alpha_q2, alpha_q3;
	reg active_overlay_q1, active_overlay_q2, active_overlay_q3;
	reg prim_v_q1, prim_v_q2, prim_v_q3;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			p_r_ov_q <= 16'd0; p_r_prim_q <= 16'd0;
			p_g_ov_q <= 16'd0; p_g_prim_q <= 16'd0;
			p_b_ov_q <= 16'd0; p_b_prim_q <= 16'd0;
			active_overlay_q1 <= 1'b0; active_overlay_q2 <= 1'b0; active_overlay_q3 <= 1'b0;
			prim_v_q1 <= 1'b0; prim_v_q2 <= 1'b0; prim_v_q3 <= 1'b0;
		end else begin
			p_r_ov_q <= p_r_ov; p_r_prim_q <= p_r_prim;
			p_g_ov_q <= p_g_ov; p_g_prim_q <= p_g_prim;
			p_b_ov_q <= p_b_ov; p_b_prim_q <= p_b_prim;
			active_overlay_q1 <= b_active_overlay;
			active_overlay_q2 <= active_overlay_q1;
			active_overlay_q3 <= active_overlay_q2;
			prim_v_q1 <= b_prim_v_q;
			prim_v_q2 <= prim_v_q1;
			prim_v_q3 <= prim_v_q2;
		end
	end
	always @(posedge clk) begin
		sel_ov_rgb_q1 <= b_sel_ov_rgb; sel_ov_rgb_q2 <= sel_ov_rgb_q1; sel_ov_rgb_q3 <= sel_ov_rgb_q2;
		prim_rgb_q1   <= b_prim_rgb;   prim_rgb_q2   <= prim_rgb_q1;   prim_rgb_q3   <= prim_rgb_q2;
		alpha_q1      <= b_alpha;      alpha_q2      <= alpha_q1;      alpha_q3      <= alpha_q2;
	end
	wire [15:0] d_r_ov = p_r_ov_q, d_r_prim = p_r_prim_q;
	wire [15:0] d_g_ov = p_g_ov_q, d_g_prim = p_g_prim_q;
	wire [15:0] d_b_ov = p_b_ov_q, d_b_prim = p_b_prim_q;
	wire [23:0] d_sel_ov_rgb = sel_ov_rgb_q3, d_prim_rgb = prim_rgb_q3;
	wire [7:0]  d_alpha = alpha_q3;
	wire d_active_overlay = active_overlay_q3;
`else
	wire [15:0] d_r_ov = p_r_ov, d_r_prim = p_r_prim;
	wire [15:0] d_g_ov = p_g_ov, d_g_prim = p_g_prim;
	wire [15:0] d_b_ov = p_b_ov, d_b_prim = p_b_prim;
	wire [23:0] d_sel_ov_rgb = p2_sel_ov_rgb, d_prim_rgb = p2_prim_rgb;
	wire [7:0]  d_alpha = p2_alpha;
	wire d_active_overlay = active_overlay;
`endif

`ifdef SYNTH_ASIC
	/* Stage 12: Blend CSA; Stages 13-14: split 8+8 addition; Stage 15: out_rgb mux */
	/* verilator lint_off UNUSEDSIGNAL */
	wire [16:0] csa_r_c_w = ({1'b0, d_r_ov} & {1'b0, d_r_prim}) | ({1'b0, d_r_ov} & 17'd128) | ({1'b0, d_r_prim} & 17'd128);
	wire [16:0] csa_g_c_w = ({1'b0, d_g_ov} & {1'b0, d_g_prim}) | ({1'b0, d_g_ov} & 17'd128) | ({1'b0, d_g_prim} & 17'd128);
	wire [16:0] csa_b_c_w = ({1'b0, d_b_ov} & {1'b0, d_b_prim}) | ({1'b0, d_b_ov} & 17'd128) | ({1'b0, d_b_prim} & 17'd128);
	wire [16:0] csa_r_s_w = {1'b0, d_r_ov} ^ {1'b0, d_r_prim} ^ 17'd128;
	wire [16:0] csa_g_s_w = {1'b0, d_g_ov} ^ {1'b0, d_g_prim} ^ 17'd128;
	wire [16:0] csa_b_s_w = {1'b0, d_b_ov} ^ {1'b0, d_b_prim} ^ 17'd128;

	reg [15:0] csa_r_s_s12, csa_r_c_s12, csa_g_s_s12, csa_g_c_s12, csa_b_s_s12, csa_b_c_s12;
	reg [23:0] s12_sel_ov_rgb, s12_prim_rgb, s13_sel_ov_rgb, s13_prim_rgb, s14_sel_ov_rgb, s14_prim_rgb;
	reg [7:0]  s12_alpha, s13_alpha, s14_alpha;
	reg s12_active_overlay, s13_active_overlay, s14_active_overlay;
	reg s12_prim_v, s13_prim_v, s14_prim_v;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			s12_active_overlay <= 1'b0; s13_active_overlay <= 1'b0; s14_active_overlay <= 1'b0;
			s12_prim_v <= 1'b0; s13_prim_v <= 1'b0; s14_prim_v <= 1'b0;
		end else begin
			s12_active_overlay <= d_active_overlay; s13_active_overlay <= s12_active_overlay; s14_active_overlay <= s13_active_overlay;
			s12_prim_v <= prim_v_q3; s13_prim_v <= s12_prim_v; s14_prim_v <= s13_prim_v;
		end
	end
	always @(posedge clk) begin
		csa_r_s_s12 <= csa_r_s_w[15:0]; csa_r_c_s12 <= {csa_r_c_w[14:0], 1'b0};
		csa_g_s_s12 <= csa_g_s_w[15:0]; csa_g_c_s12 <= {csa_g_c_w[14:0], 1'b0};
		csa_b_s_s12 <= csa_b_s_w[15:0]; csa_b_c_s12 <= {csa_b_c_w[14:0], 1'b0};
		s12_sel_ov_rgb <= d_sel_ov_rgb; s13_sel_ov_rgb <= s12_sel_ov_rgb; s14_sel_ov_rgb <= s13_sel_ov_rgb;
		s12_prim_rgb   <= d_prim_rgb;   s13_prim_rgb   <= s12_prim_rgb;   s14_prim_rgb   <= s13_prim_rgb;
		s12_alpha      <= d_alpha;      s13_alpha      <= s12_alpha;      s14_alpha      <= s13_alpha;
	end

	wire [7:0] tot_r_lo_un, tot_g_lo_un, tot_b_lo_un;
	wire tot_r_co8_w, tot_g_co8_w, tot_b_co8_w;
	am_add #(.W(8)) a_tot_r_lo (.a(csa_r_s_s12[7:0]), .b(csa_r_c_s12[7:0]), .ci(1'b0), .s(tot_r_lo_un), .co(tot_r_co8_w));
	am_add #(.W(8)) a_tot_g_lo (.a(csa_g_s_s12[7:0]), .b(csa_g_c_s12[7:0]), .ci(1'b0), .s(tot_g_lo_un), .co(tot_g_co8_w));
	am_add #(.W(8)) a_tot_b_lo (.a(csa_b_s_s12[7:0]), .b(csa_b_c_s12[7:0]), .ci(1'b0), .s(tot_b_lo_un), .co(tot_b_co8_w));

	reg tot_r_co8_q, tot_g_co8_q, tot_b_co8_q;
	reg [15:8] csa_r_s_s13, csa_r_c_s13, csa_g_s_s13, csa_g_c_s13, csa_b_s_s13, csa_b_c_s13;
	always @(posedge clk) begin
		tot_r_co8_q <= tot_r_co8_w; csa_r_s_s13 <= csa_r_s_s12[15:8]; csa_r_c_s13 <= csa_r_c_s12[15:8];
		tot_g_co8_q <= tot_g_co8_w; csa_g_s_s13 <= csa_g_s_s12[15:8]; csa_g_c_s13 <= csa_g_c_s12[15:8];
		tot_b_co8_q <= tot_b_co8_w; csa_b_s_s13 <= csa_b_s_s12[15:8]; csa_b_c_s13 <= csa_b_c_s12[15:8];
	end

	wire [7:0] sum_r_hi_w, sum_g_hi_w, sum_b_hi_w;
	wire [2:0] tot_hi_co_un;
	am_add #(.W(8)) a_sum_r_hi (.a(csa_r_s_s13), .b(csa_r_c_s13), .ci(tot_r_co8_q), .s(sum_r_hi_w), .co(tot_hi_co_un[0]));
	am_add #(.W(8)) a_sum_g_hi (.a(csa_g_s_s13), .b(csa_g_c_s13), .ci(tot_g_co8_q), .s(sum_g_hi_w), .co(tot_hi_co_un[1]));
	am_add #(.W(8)) a_sum_b_hi (.a(csa_b_s_s13), .b(csa_b_c_s13), .ci(tot_b_co8_q), .s(sum_b_hi_w), .co(tot_hi_co_un[2]));
	/* verilator lint_on UNUSEDSIGNAL */

	reg [7:0] sum_r_top_q, sum_g_top_q, sum_b_top_q;
	always @(posedge clk) begin
		sum_r_top_q <= sum_r_hi_w;
		sum_g_top_q <= sum_g_hi_w;
		sum_b_top_q <= sum_b_hi_w;
	end

	wire [7:0] r_blend = (s14_alpha == 8'hFF) ? s14_sel_ov_rgb[23:16] : ((s14_alpha == 8'h00) ? s14_prim_rgb[23:16] : sum_r_top_q);
	wire [7:0] g_blend = (s14_alpha == 8'hFF) ? s14_sel_ov_rgb[15:8]  : ((s14_alpha == 8'h00) ? s14_prim_rgb[15:8]  : sum_g_top_q);
	wire [7:0] b_blend = (s14_alpha == 8'hFF) ? s14_sel_ov_rgb[7:0]   : ((s14_alpha == 8'h00) ? s14_prim_rgb[7:0]   : sum_b_top_q);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			out_rgb   <= 24'd0;
			out_valid <= 1'b0;
		end else begin
			out_valid <= s14_prim_v;
			if (!s14_active_overlay)
				out_rgb <= s14_prim_rgb;
			else
				out_rgb <= {r_blend, g_blend, b_blend};
		end
	end
`else
	/* Red/Green/Blue channel blending via 3:2 CSA (+ 128) + single 17-bit adder */
	/* verilator lint_off UNUSEDSIGNAL */
	wire [16:0] sum_r_tot, sum_g_tot, sum_b_tot;
	wire [16:0] csa_r_c = ({1'b0, d_r_ov} & {1'b0, d_r_prim}) | ({1'b0, d_r_ov} & 17'd128) | ({1'b0, d_r_prim} & 17'd128);
	wire [16:0] csa_g_c = ({1'b0, d_g_ov} & {1'b0, d_g_prim}) | ({1'b0, d_g_ov} & 17'd128) | ({1'b0, d_g_prim} & 17'd128);
	wire [16:0] csa_b_c = ({1'b0, d_b_ov} & {1'b0, d_b_prim}) | ({1'b0, d_b_ov} & 17'd128) | ({1'b0, d_b_prim} & 17'd128);
	/* verilator lint_on UNUSEDSIGNAL */
	wire [16:0] csa_r_s = {1'b0, d_r_ov} ^ {1'b0, d_r_prim} ^ 17'd128;
	wire [16:0] csa_g_s = {1'b0, d_g_ov} ^ {1'b0, d_g_prim} ^ 17'd128;
	wire [16:0] csa_b_s = {1'b0, d_b_ov} ^ {1'b0, d_b_prim} ^ 17'd128;
	wire [1:0] co_r_unused, co_g_unused, co_b_unused;
	assign co_r_unused[0] = 1'b0;
	assign co_g_unused[0] = 1'b0;
	assign co_b_unused[0] = 1'b0;
	am_add #(.W(17)) a_sum_r_tot (.a(csa_r_s), .b({csa_r_c[15:0], 1'b0}), .ci(1'b0), .s(sum_r_tot), .co(co_r_unused[1]));
	wire [7:0] r_blend = (d_alpha == 8'hFF) ? d_sel_ov_rgb[23:16] : ((d_alpha == 8'h00) ? d_prim_rgb[23:16] : sum_r_tot[15:8]);

	am_add #(.W(17)) a_sum_g_tot (.a(csa_g_s), .b({csa_g_c[15:0], 1'b0}), .ci(1'b0), .s(sum_g_tot), .co(co_g_unused[1]));
	wire [7:0] g_blend = (d_alpha == 8'hFF) ? d_sel_ov_rgb[15:8]  : ((d_alpha == 8'h00) ? d_prim_rgb[15:8]  : sum_g_tot[15:8]);

	am_add #(.W(17)) a_sum_b_tot (.a(csa_b_s), .b({csa_b_c[15:0], 1'b0}), .ci(1'b0), .s(sum_b_tot), .co(co_b_unused[1]));
	wire [7:0] b_blend = (d_alpha == 8'hFF) ? d_sel_ov_rgb[7:0]   : ((d_alpha == 8'h00) ? d_prim_rgb[7:0]   : sum_b_tot[15:8]);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			out_rgb   <= 24'd0;
			out_valid <= 1'b0;
		end else begin
			out_valid <= prim_valid;
			if (!d_active_overlay)
				out_rgb <= d_prim_rgb;
			else
				out_rgb <= {r_blend, g_blend, b_blend};
		end
	end
`endif

	wire unused_ok = &{1'b0, box_co_unused, diff_bo_unused,
		alpha_bo_unused, ov_valid, rsp_valid, rsp_write, rsp_tag, rsp_rdata, rsp_error, enable};
endmodule

`default_nettype wire
