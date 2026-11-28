/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_ms: the render output's multisample coverage (docs/ghci.md
 * section 12.11), between atg_rop_order and atg_rop_ds: per quad the
 * samples that go on (o_smask, bit 4 l + s: sample s of lane l), the
 * depth of every sample (o_sdepth, at bits 32 (4 l + s)) and the colour
 * (alpha to one).
 *
 * MS_CTRL (0x05E): [3:0] = 2 four samples, else one; [8] alpha to
 * coverage; [9] alpha to one.  i_aux[147:0] = {d_s, zexp, smask}
 * from atg_su_exp (the direct path: every sample of a lane, its depth).
 *   Four samples: smask = i_aux's & the lanes of i_mask & (alpha to
 *   coverage: a mask of COLOR0's alpha: n = the number of 0.125, 0.375,
 *   0.625, 0.875 it reaches; n = 0 none, 1 {0}, 2 {0, 3}, 3 {0, 1, 3},
 *   4 all; NaN and negative alphas none); the depth of sample s of lane l
 *   is the exported depth (zexp) or clamp01(z + d_s) (fadd, RNE, then NaN
 *   and negatives 0, above 1 one), 16 adds one per cycle (24 cycles).
 *   One sample: bit 4 l = i_mask[l] & i_aux[4 l] (& alpha >= 0.5 with alpha
 *   to coverage), the depth i_depth's (no add).
 * Alpha to one sets COLOR0's alpha to 1.0 after alpha to coverage.  Neither
 * applies to the direct path (i_dbg: DEBUG_COLOR later).  A quad whose
 * smask is 0 is dropped.  States: IDLE → (four samples: Z →) OUT → IDLE.
 * CAPS[30]: with DEPTH_CLAMP[1] (0x059, Vulkan's depth clipping in the
 * RX) z + d_s is not clamped to [0, 1]; with DEPTH_CLAMP[0] every
 * sample's depth (exported or not) is clamped to [min(n, f), max(n, f)],
 * n = VIEWPORT z offset (0x055), f = n + z scale (0x054; one fadd when
 * either is written, while the ROP is idle), a NaN to the minimum.  i_aux
 * [148] is the facing, passed on as o_front.
 * Beats (CAPS[30]): a quad comes as one beat per colour attachment; the
 * first (i_aux[150]) gives the coverage, the depths and, for alpha to
 * coverage, the alpha (the lowest attachment exported: attachment 0's
 * when it is); the later beats of the quad keep its o_smask and o_sdepth
 * (dropped with it), and alpha to one applies to every beat.  o_beat =
 * {rt, nocol, first} of i_aux[153:152], [149], [150].
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_ms #(
	parameter ENABLE_MSAA = 1
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* From atg_rop_order */
	input wire i_valid,
	output wire i_ready,
	input wire [31:0] i_xy,
	input wire [3:0] i_mask,
	input wire [511:0] i_color,
	input wire [511:0] i_color1,
	input wire [127:0] i_depth,
	input wire i_dbg,
	input wire [154:0] i_aux,	/* {rt[2:0], qlast, qfirst, nocol, front, d_s, zexp, smask} */
	/* To atg_rop_ds */
	output reg o_valid,
	input wire o_ready,
	output reg [31:0] o_xy,
	output reg [15:0] o_smask,
	output reg [511:0] o_color,
	output reg [511:0] o_color1,
	output wire [511:0] o_sdepth,
	output reg o_dbg,
	output reg o_front,
	output reg [4:0] o_beat,	/* {rt[2:0], nocol, first} */
	output wire busy
);
	`include "ghci_regs.vh"
	localparam [31:0] ONE = 32'h3F800000;
	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	(* keep = "true", dont_touch = "true" *) reg [2:0] ms4_rep;
	reg a2c, a2one;
	reg [1:0] dc;	/* DEPTH_CLAMP */
	reg [31:0] vzs, vzo, vzf;	/* VIEWPORT z scale, z offset; f = z offset + z scale */
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] zmin, zmax;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] zmin_d, zmax_d;
	reg [4:0] zt;	/* the fadd's issues of f, by age */
	reg zf_le_r;
	wire vz_we = st_we && (st_idx == GHCI_S_VIEWPORT + 9'd4 || st_idx == GHCI_S_VIEWPORT + 9'd5);
	wire zf_le;
	atg_rop_ms_fle u_fle_vzo (.a(vzo), .b(fa_r), .le(zf_le));

`ifdef SYNTH_KINTEX7
	wire [1:0] zt3_buf;
	wire [7:0] zt4_buf;
	wire [7:0] zf_le_buf;
	wire vzs_we_raw = st_we && (st_idx == GHCI_S_VIEWPORT + 9'd4);
	wire vzo_we_raw = st_we && (st_idx == GHCI_S_VIEWPORT + 9'd5);
	wire [1:0] vzs_we_buf, vzo_we_buf;
	genvar g_zt;
	generate
	for (g_zt = 0; g_zt < 2; g_zt = g_zt + 1) begin : g_buf_zt3
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bzt3 (.I0(zt[3]), .O(zt3_buf[g_zt]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bvzs (.I0(vzs_we_raw), .O(vzs_we_buf[g_zt]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bvzo (.I0(vzo_we_raw), .O(vzo_we_buf[g_zt]));
	end
	for (g_zt = 0; g_zt < 8; g_zt = g_zt + 1) begin : g_buf_zt4
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bzt4 (.I0(zt[4]), .O(zt4_buf[g_zt]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bzl (.I0(zf_le_r), .O(zf_le_buf[g_zt]));
	end
	for (g_zt = 0; g_zt < 2; g_zt = g_zt + 1) begin : g_reg_zbounds
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				vzs[16 * g_zt +: 16] <= 16'd0;
				vzo[16 * g_zt +: 16] <= 16'd0;
				vzf[16 * g_zt +: 16] <= 16'd0;
				zmin[16 * g_zt +: 16] <= 16'd0;
				zmax[16 * g_zt +: 16] <= ONE[16 * g_zt +: 16];
				zmin_d[16 * g_zt +: 16] <= 16'd0;
				zmax_d[16 * g_zt +: 16] <= ONE[16 * g_zt +: 16];
			end else begin
				if (vzs_we_buf[g_zt])
					vzs[16 * g_zt +: 16] <= st_wdata[16 * g_zt +: 16];
				if (vzo_we_buf[g_zt])
					vzo[16 * g_zt +: 16] <= st_wdata[16 * g_zt +: 16];
				if (zt3_buf[g_zt])
					vzf[16 * g_zt +: 16] <= fa_r[16 * g_zt +: 16];
				if (zt4_buf[g_zt])
					zmin[16 * g_zt +: 16] <= zf_le_buf[g_zt] ? vzo[16 * g_zt +: 16] : vzf[16 * g_zt +: 16];
				if (zt4_buf[2 + g_zt])
					zmax[16 * g_zt +: 16] <= zf_le_buf[2 + g_zt] ? vzf[16 * g_zt +: 16] : vzo[16 * g_zt +: 16];
				if (zt4_buf[4 + g_zt])
					zmin_d[16 * g_zt +: 16] <= zf_le_buf[4 + g_zt] ? vzo[16 * g_zt +: 16] : vzf[16 * g_zt +: 16];
				if (zt4_buf[6 + g_zt])
					zmax_d[16 * g_zt +: 16] <= zf_le_buf[6 + g_zt] ? vzf[16 * g_zt +: 16] : vzo[16 * g_zt +: 16];
			end
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] zt3_buf;
	wire [7:0] zt4_buf;
	wire [7:0] zf_le_buf;
	wire vzs_we_raw = st_we && (st_idx == GHCI_S_VIEWPORT + 9'd4);
	wire vzo_we_raw = st_we && (st_idx == GHCI_S_VIEWPORT + 9'd5);
	wire [1:0] vzs_we_buf, vzo_we_buf;
	genvar g_zt;
	generate
	for (g_zt = 0; g_zt < 2; g_zt = g_zt + 1) begin : g_buf_zt3
		assign zt3_buf[g_zt] = zt[3];
		assign vzs_we_buf[g_zt] = vzs_we_raw;
		assign vzo_we_buf[g_zt] = vzo_we_raw;
	end
	for (g_zt = 0; g_zt < 8; g_zt = g_zt + 1) begin : g_buf_zt4
		assign zt4_buf[g_zt] = zt[4];
		assign zf_le_buf[g_zt] = zf_le_r;
	end
	for (g_zt = 0; g_zt < 2; g_zt = g_zt + 1) begin : g_reg_zbounds
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				vzs[16 * g_zt +: 16] <= 16'd0;
				vzo[16 * g_zt +: 16] <= 16'd0;
				vzf[16 * g_zt +: 16] <= 16'd0;
				zmin[16 * g_zt +: 16] <= 16'd0;
				zmax[16 * g_zt +: 16] <= ONE[16 * g_zt +: 16];
				zmin_d[16 * g_zt +: 16] <= 16'd0;
				zmax_d[16 * g_zt +: 16] <= ONE[16 * g_zt +: 16];
			end else begin
				if (vzs_we_buf[g_zt])
					vzs[16 * g_zt +: 16] <= st_wdata[16 * g_zt +: 16];
				if (vzo_we_buf[g_zt])
					vzo[16 * g_zt +: 16] <= st_wdata[16 * g_zt +: 16];
				if (zt3_buf[g_zt])
					vzf[16 * g_zt +: 16] <= fa_r[16 * g_zt +: 16];
				if (zt4_buf[g_zt])
					zmin[16 * g_zt +: 16] <= zf_le_buf[g_zt] ? vzo[16 * g_zt +: 16] : vzf[16 * g_zt +: 16];
				if (zt4_buf[2 + g_zt])
					zmax[16 * g_zt +: 16] <= zf_le_buf[2 + g_zt] ? vzf[16 * g_zt +: 16] : vzo[16 * g_zt +: 16];
				if (zt4_buf[4 + g_zt])
					zmin_d[16 * g_zt +: 16] <= zf_le_buf[4 + g_zt] ? vzo[16 * g_zt +: 16] : vzf[16 * g_zt +: 16];
				if (zt4_buf[6 + g_zt])
					zmax_d[16 * g_zt +: 16] <= zf_le_buf[6 + g_zt] ? vzf[16 * g_zt +: 16] : vzo[16 * g_zt +: 16];
			end
		end
	end
	endgenerate
`endif

	always @(posedge clk or posedge rst_sync)
		if (rst_sync) begin
			ms4_rep <= 3'd0;
			a2c <= 1'b0;
			a2one <= 1'b0;
			dc <= 2'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			vzs <= 32'd0;
			vzo <= 32'd0;
			vzf <= 32'd0;
			zmin <= 32'd0;
			zmax <= ONE;
			zmin_d <= 32'd0;
			zmax_d <= ONE;
`endif
			zt <= 5'd0;
			zf_le_r <= 1'b0;
		end else begin
			zt <= {zt[3:0], zf_go};
			if (zt[3])
				zf_le_r <= zf_le;
			if (st_we && st_idx == GHCI_S_MS_CTRL) begin
				ms4_rep <= {3{(ENABLE_MSAA != 0) && (st_wdata[3:0] == 4'd2)}};
				a2c <= st_wdata[8];
				a2one <= st_wdata[9];
			end
			if (st_we && st_idx == GHCI_S_DEPTH_CLAMP)
				dc <= st_wdata[1:0];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			if (st_we && st_idx == GHCI_S_VIEWPORT + 9'd4)
				vzs <= st_wdata;
			if (st_we && st_idx == GHCI_S_VIEWPORT + 9'd5)
				vzo <= st_wdata;
			if (zt[3])
				vzf <= fa_r;
			if (zt[4]) begin
				zmin <= zf_le_r ? vzo : vzf;
				zmax <= zf_le_r ? vzf : vzo;
				zmin_d <= zf_le_r ? vzo : vzf;
				zmax_d <= zf_le_r ? vzf : vzo;
			end
`endif
		end
	/* f is added the cycle after a write of either word */
	reg zf_prep;
	reg zf_go;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			zf_prep <= 1'b0;
			zf_go <= 1'b0;
		end else begin
			zf_prep <= vz_we;
			zf_go <= zf_prep;
		end
	end

	/* Alpha to coverage of one lane (fp32 alpha) - pure LUT, 0 CARRY4s. */
	function [3:0] a2c_mask;
		input [31:0] a;
		input four;
		reg is_nan, pos_ok, hi_ge1, in_band;
		reg [3:0] m4;
		begin
			is_nan = (a[30:23] == 8'hFF) && (a[22:0] != 23'd0);
			pos_ok = !a[31] && !is_nan;
			hi_ge1 = a[30];
			in_band = (a[30:25] == 6'b011111);
			if (hi_ge1)
				m4 = 4'b1111;
			else if (in_band) begin
				if (a[24] && (a[23] || (a[22] && a[21])))
					m4 = 4'b1111;	/* >= 0.875 (32'h3F600000, a[24:21] >= 4'd11) */
				else if (a[24] && (a[23] || a[22] || a[21]))
					m4 = 4'b1011;	/* >= 0.625 (32'h3F200000, a[24:21] >= 4'd9) */
				else if (a[24] || (a[23] && a[22]))
					m4 = 4'b1001;	/* >= 0.375 (32'h3EC00000, a[24:21] >= 4'd6) */
`ifdef MUT_A2C_STEP
				else if (a[24] || a[23])
					m4 = 4'b0001;	/* >= 0.25 (32'h3E800000, a[24:21] >= 4'd4) */
				else
					m4 = 4'b0000;
`else
				else
					m4 = 4'b0001;	/* >= 0.125 (32'h3E000000, a[24:21] >= 4'd0) */
`endif
			end else begin
				m4 = 4'b0000;
			end
			if (!pos_ok)
				a2c_mask = 4'b0000;
			else if (!four)
				a2c_mask = {3'd0, hi_ge1 || (in_band && a[24])};
			else
				a2c_mask = m4;
		end
	endfunction

	/* Registered inputs for S_PREP so S_IDLE has <= 3 LUTs from atg_rop_order */
	reg [15:0] r_smask_in;
	reg [3:0] r_mask;
	reg [15:0] r_a2c_4x;
	reg [3:0] r_a2c_1x;
	wire [15:0] i_a2c_4x_w;
	wire [3:0] i_a2c_1x_w;
	genvar ga;
	generate
	for (ga = 0; ga < 4; ga = ga + 1) begin : g_a2c_in
		wire [3:0] a2c_4 = a2c_mask(i_color[128 * ga + 96 +: 32], 1'b1);
		wire [3:0] a2c_1 = a2c_mask(i_color[128 * ga + 96 +: 32], 1'b0);
		assign i_a2c_4x_w[4 * ga +: 4] = a2c_4;
		assign i_a2c_1x_w[ga] = a2c_1[0];
	end
	endgenerate

	/* The quad's sample mask (evaluated in S_PREP from registered inputs). */
	reg [15:0] sm;
	reg [3:0] am;
	integer l;
	always @(*) begin
		for (l = 0; l < 4; l = l + 1) begin
			am = a2c && !o_dbg ? (ms4_rep[l / 2] ? r_a2c_4x[4 * l +: 4] : {3'b111, r_a2c_1x[l]}) : 4'hF;
			sm[4 * l +: 4] = ms4_rep[l / 2] ? r_smask_in[4 * l +: 4] & {4{r_mask[l]}} & am :
				{3'd0, r_smask_in[4 * l] & r_mask[l] & am[0]};
		end
	end

	localparam [2:0] S_IDLE = 3'd0, S_Z = 3'd1, S_OUT = 3'd2, S_PREP = 3'd3, S_D1 = 3'd4, S_D1_PRE = 3'd5;
	reg [2:0] st;
	reg [127:0] z;
	wire [31:0] fa_in_a_w;
	wire [31:0] fa_in_b_w;
	wire fa_sz_ival;
	wire sz_done;

	wire fa_o;
	wire [31:0] fa_r;
	atg_rop_fadd #(.LAT(4)) fadd (.clk(clk), .rst_n(rst_n), .en(1'b1), .i_valid(fa_sz_ival || zf_go),
		.i_a(fa_in_a_w), .i_b(fa_in_b_w),
		.i_sub(1'b0), .o_valid(fa_o), .o_r(fa_r));

	wire [3:0] d1_cmp_ulo, d1_cmp_uhi;
	wire [4*17-1:0] d1_lo_s1_w, d1_hi_s1_w;
	reg [4*17-1:0] r_d1_lo_s1, r_d1_hi_s1;
	(* equivalent_register_removal = "no" *) reg [7:0] r_d1_ulo;
	(* equivalent_register_removal = "no" *) reg [7:0] r_d1_uhi;
	wire [127:0] d1c;
	genvar g;
	generate
	for (g = 0; g < 4; g = g + 1) begin : g_d1
		wire [31:0] lo_g = (g < 2) ? zmin : zmin_d;
		wire [31:0] hi_g = (g < 2) ? zmax : zmax_d;
		atg_rop_ms_fle_s1 u_d1_lo_s1 (.a(z[32 * g +: 32]), .b(lo_g), .s1(d1_lo_s1_w[17 * g +: 17]));
		atg_rop_ms_fle_s1 u_d1_hi_s1 (.a(hi_g), .b(z[32 * g +: 32]), .s1(d1_hi_s1_w[17 * g +: 17]));
		atg_rop_ms_fle_s2 #(.NAN_TRUE(1)) u_d1_lo_s2 (.s1(r_d1_lo_s1[17 * g +: 17]), .le(d1_cmp_ulo[g]));
		atg_rop_ms_fle_s2 #(.NAN_TRUE(0)) u_d1_hi_s2 (.s1(r_d1_hi_s1[17 * g +: 17]), .le(d1_cmp_uhi[g]));
		assign d1c[32 * g +: 16] = r_d1_ulo[2 * g] ? lo_g[15:0] :
			(r_d1_uhi[2 * g] ? hi_g[15:0] : z[32 * g +: 16]);
		assign d1c[32 * g + 16 +: 16] = r_d1_ulo[2 * g + 1] ? lo_g[31:16] :
			(r_d1_uhi[2 * g + 1] ? hi_g[31:16] : z[32 * g + 16 +: 16]);
	end
	endgenerate

	wire first = i_aux[150];
	reg drop;	/* the quad's first beat had no sample */

	assign i_ready = st == S_IDLE;
	assign busy = st != S_IDLE;

`ifdef SYNTH_KINTEX7
	wire en_idle_only = (st == S_IDLE);
	wire en_prep_clear = (st == S_PREP);
	wire en_d1_load = (st == S_D1);
	wire [2:0] en_if_mid;
	wire [3:0] en_prep_mid;
	wire [1:0] en_d1_mid;
	wire [17:0] en_zr_buf;
	wire [3:0] en_col_mid;
	wire [65:0] en_col_buf;
	wire [7:0] a2one_sel_buf;
	genvar g_zds, g_cb;
	generate
	for (g_zds = 0; g_zds < 3; g_zds = g_zds + 1) begin : g_if_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ifm (
			.I0(en_idle_only), .O(en_if_mid[g_zds])
		);
	end
	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_prep_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pm (.I0(en_prep_clear), .O(en_prep_mid[g_zds]));
	end
	for (g_zds = 0; g_zds < 2; g_zds = g_zds + 1) begin : g_pd_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dm (.I0(en_d1_load), .O(en_d1_mid[g_zds]));
	end
	for (g_zds = 0; g_zds < 11; g_zds = g_zds + 1) begin : g_buf_zr
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bzr (
			.I0(en_if_mid[g_zds / 6]), .O(en_zr_buf[g_zds])
		);
	end
	for (g_zds = 0; g_zds < 8; g_zds = g_zds + 1) begin : g_reg_z
		always @(posedge clk) begin
			if (en_zr_buf[g_zds])
				z[16 * g_zds +: 16] <= i_depth[16 * g_zds +: 16];
		end
	end
	always @(posedge clk) begin
		if (en_zr_buf[8]) begin
			r_a2c_4x <= i_a2c_4x_w;
			r_a2c_1x <= i_a2c_1x_w;
		end
		if (en_zr_buf[9])
			r_smask_in <= i_aux[15:0];
		if (en_zr_buf[10])
			r_mask <= i_mask;
	end

	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_col_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cm (
			.I0(en_idle_only), .O(en_col_mid[g_zds])
		);
	end
	for (g_zds = 0; g_zds < 66; g_zds = g_zds + 1) begin : g_col_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cl (
			.I0(en_col_mid[g_zds / 17]), .O(en_col_buf[g_zds])
		);
	end
	for (g_zds = 0; g_zds < 8; g_zds = g_zds + 1) begin : g_a2one_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_al (
			.I0(a2one && !i_dbg), .O(a2one_sel_buf[g_zds])
		);
	end
	for (g_zds = 0; g_zds < 32; g_zds = g_zds + 1) begin : g_reg_col
		localparam IS_ALPHA = ((g_zds % 8) == 6) || ((g_zds % 8) == 7);
		localparam ALPHA_IDX = (g_zds / 8) * 2 + ((g_zds % 8) - 6);
		wire [15:0] col_d_in;
		if (IS_ALPHA) begin : g_alpha_d
			for (g_cb = 0; g_cb < 16; g_cb = g_cb + 1) begin : g_ab
				if (ONE[16 * (g_zds % 2) + g_cb]) begin : g_set1
					(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hE)) u_cd (
						.I0(i_color[16 * g_zds + g_cb]),
						.I1(a2one_sel_buf[ALPHA_IDX]),
						.O(col_d_in[g_cb])
					);
				end else begin : g_clr0
					(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h2)) u_cd (
						.I0(i_color[16 * g_zds + g_cb]),
						.I1(a2one_sel_buf[ALPHA_IDX]),
						.O(col_d_in[g_cb])
					);
				end
			end
		end else begin : g_non_alpha_d
			assign col_d_in = i_color[16 * g_zds +: 16];
		end
		always @(posedge clk) begin
			if (en_col_buf[g_zds])
				o_color[16 * g_zds +: 16] <= col_d_in;
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				o_color1[16 * g_zds +: 16] <= 16'd0;
			else if (en_col_buf[32 + g_zds])
				o_color1[16 * g_zds +: 16] <= i_color1[16 * g_zds +: 16];
		end
	end
	for (g_zds = 0; g_zds < 2; g_zds = g_zds + 1) begin : g_reg_xy
		always @(posedge clk) begin
			if (en_col_buf[64 + g_zds])
				o_xy[16 * g_zds +: 16] <= i_xy[16 * g_zds +: 16];
		end
	end
	endgenerate
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_d1_ulo <= 8'd0;
			r_d1_uhi <= 8'd0;
			r_d1_lo_s1 <= {4*17{1'b0}};
			r_d1_hi_s1 <= {4*17{1'b0}};
		end else begin
			if (en_prep_mid[2])
				o_smask <= sm;
			if (st == S_D1_PRE) begin
				for (l = 0; l < 4; l = l + 1) begin
					r_d1_ulo[2 * l +: 2] <= {2{dc[0] && d1_cmp_ulo[l]}};
					r_d1_uhi[2 * l +: 2] <= {2{dc[0] && !d1_cmp_ulo[l] && d1_cmp_uhi[l]}};
				end
			end
		end
	end
	generate
	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_reg_d1s1
		wire en_pm3_lo, en_pm3_hi;
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pm3l (.I0(en_prep_mid[3]), .O(en_pm3_lo));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pm3h (.I0(en_prep_mid[3]), .O(en_pm3_hi));
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				r_d1_lo_s1[17 * g_zds +: 17] <= 17'd0;
				r_d1_hi_s1[17 * g_zds +: 17] <= 17'd0;
			end else begin
				if (en_pm3_lo) r_d1_lo_s1[17 * g_zds +: 17] <= d1_lo_s1_w[17 * g_zds +: 17];
				if (en_pm3_hi) r_d1_hi_s1[17 * g_zds +: 17] <= d1_hi_s1_w[17 * g_zds +: 17];
			end
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire en_idle_only = (st == S_IDLE);
	wire en_prep_clear = (st == S_PREP);
	wire en_d1_load = (st == S_D1);
	wire [2:0] en_if_mid;
	wire [3:0] en_prep_mid;
	wire [1:0] en_d1_mid;
	wire [17:0] en_zr_buf;
	wire [3:0] en_col_mid;
	wire [65:0] en_col_buf;
	wire [7:0] a2one_sel_buf;
	genvar g_zds, g_cb;
	generate
	for (g_zds = 0; g_zds < 3; g_zds = g_zds + 1) begin : g_if_mid
		assign en_if_mid[g_zds] = en_idle_only;
	end
	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_prep_mid
		assign en_prep_mid[g_zds] = en_prep_clear;
	end
	for (g_zds = 0; g_zds < 2; g_zds = g_zds + 1) begin : g_pd_mid
		assign en_d1_mid[g_zds] = en_d1_load;
	end
	for (g_zds = 0; g_zds < 11; g_zds = g_zds + 1) begin : g_buf_zr
		assign en_zr_buf[g_zds] = en_if_mid[g_zds / 6];
	end
	for (g_zds = 0; g_zds < 8; g_zds = g_zds + 1) begin : g_reg_z
		always @(posedge clk) begin
			if (en_zr_buf[g_zds])
				z[16 * g_zds +: 16] <= i_depth[16 * g_zds +: 16];
		end
	end
	always @(posedge clk) begin
		if (en_zr_buf[8]) begin
			r_a2c_4x <= i_a2c_4x_w;
			r_a2c_1x <= i_a2c_1x_w;
		end
		if (en_zr_buf[9])
			r_smask_in <= i_aux[15:0];
		if (en_zr_buf[10])
			r_mask <= i_mask;
	end

	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_col_mid
		assign en_col_mid[g_zds] = en_idle_only;
	end
	for (g_zds = 0; g_zds < 66; g_zds = g_zds + 1) begin : g_col_leaf
		assign en_col_buf[g_zds] = en_col_mid[g_zds / 17];
	end
	for (g_zds = 0; g_zds < 8; g_zds = g_zds + 1) begin : g_a2one_leaf
		assign a2one_sel_buf[g_zds] = a2one && !i_dbg;
	end
	for (g_zds = 0; g_zds < 32; g_zds = g_zds + 1) begin : g_reg_col
		localparam IS_ALPHA = ((g_zds % 8) == 6) || ((g_zds % 8) == 7);
		localparam ALPHA_IDX = (g_zds / 8) * 2 + ((g_zds % 8) - 6);
		wire [15:0] col_d_in;
		if (IS_ALPHA) begin : g_alpha_d
			for (g_cb = 0; g_cb < 16; g_cb = g_cb + 1) begin : g_ab
				if (ONE[16 * (g_zds % 2) + g_cb]) begin : g_set1
					(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hE)) u_cd (
						.I0(i_color[16 * g_zds + g_cb]),
						.I1(a2one_sel_buf[ALPHA_IDX]),
						.O(col_d_in[g_cb])
					);
				end else begin : g_clr0
					(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h2)) u_cd (
						.I0(i_color[16 * g_zds + g_cb]),
						.I1(a2one_sel_buf[ALPHA_IDX]),
						.O(col_d_in[g_cb])
					);
				end
			end
		end else begin : g_non_alpha_d
			assign col_d_in = i_color[16 * g_zds +: 16];
		end
		always @(posedge clk) begin
			if (en_col_buf[g_zds])
				o_color[16 * g_zds +: 16] <= col_d_in;
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				o_color1[16 * g_zds +: 16] <= 16'd0;
			else if (en_col_buf[32 + g_zds])
				o_color1[16 * g_zds +: 16] <= i_color1[16 * g_zds +: 16];
		end
	end
	for (g_zds = 0; g_zds < 2; g_zds = g_zds + 1) begin : g_reg_xy
		always @(posedge clk) begin
			if (en_col_buf[64 + g_zds])
				o_xy[16 * g_zds +: 16] <= i_xy[16 * g_zds +: 16];
		end
	end
	endgenerate
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_d1_ulo <= 8'd0;
			r_d1_uhi <= 8'd0;
		end else begin
			if (en_prep_mid[2])
				o_smask <= sm;
			if (st == S_D1_PRE) begin
				for (l = 0; l < 4; l = l + 1) begin
					r_d1_ulo[2 * l +: 2] <= {2{dc[0] && d1_cmp_ulo[l]}};
					r_d1_uhi[2 * l +: 2] <= {2{dc[0] && !d1_cmp_ulo[l] && d1_cmp_uhi[l]}};
				end
			end
		end
	end
	generate
	for (g_zds = 0; g_zds < 4; g_zds = g_zds + 1) begin : g_reg_d1s1
		wire en_pm3_lo, en_pm3_hi;
		assign en_pm3_lo = en_prep_mid[3];
		assign en_pm3_hi = en_prep_mid[3];
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				r_d1_lo_s1[17 * g_zds +: 17] <= 17'd0;
				r_d1_hi_s1[17 * g_zds +: 17] <= 17'd0;
			end else begin
				if (en_pm3_lo) r_d1_lo_s1[17 * g_zds +: 17] <= d1_lo_s1_w[17 * g_zds +: 17];
				if (en_pm3_hi) r_d1_hi_s1[17 * g_zds +: 17] <= d1_hi_s1_w[17 * g_zds +: 17];
			end
		end
	end
	endgenerate
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			st <= S_IDLE;
			o_valid <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			o_color1 <= 512'd0;
			r_d1_ulo <= 8'd0;
			r_d1_uhi <= 8'd0;
			r_d1_lo_s1 <= {4*17{1'b0}};
			r_d1_hi_s1 <= {4*17{1'b0}};
`endif
			o_beat <= 5'd0;
			drop <= 1'b0;
			o_dbg <= 1'b0;
			o_front <= 1'b0;
		end else begin
			case (st)
			S_IDLE:
				if (i_valid && !first) begin
					/* a later beat: the quad's coverage and depths stay */
					o_dbg <= i_dbg;
					o_beat <= {i_aux[154:152], i_aux[149], 1'b0};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					o_xy <= i_xy;
					o_color <= i_color;
					o_color1 <= i_color1;
					if (a2one && !i_dbg)
						for (l = 0; l < 4; l = l + 1)
							o_color[128 * l + 96 +: 32] <= ONE;
`endif
					if (!drop) begin
						o_valid <= 1'b1;
						st <= S_OUT;
					end
				end else if (i_valid) begin
					o_dbg <= i_dbg;
					o_front <= i_aux[148];
					o_beat <= {i_aux[154:152], i_aux[149], 1'b1};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					o_xy <= i_xy;
					o_color <= i_color;
					o_color1 <= i_color1;
					if (a2one && !i_dbg)
						for (l = 0; l < 4; l = l + 1)
							o_color[128 * l + 96 +: 32] <= ONE;
					z <= i_depth;
					r_smask_in <= i_aux[15:0];
					r_mask <= i_mask;
					r_a2c_4x <= i_a2c_4x_w;
					r_a2c_1x <= i_a2c_1x_w;
`endif
					st <= S_PREP;
				end
			S_PREP: begin
				drop <= (sm == 16'd0);
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				o_smask <= sm;
				r_d1_lo_s1 <= d1_lo_s1_w;
				r_d1_hi_s1 <= d1_hi_s1_w;
`endif
				if (sm == 16'd0)
					st <= S_IDLE;
				else if (ENABLE_MSAA != 0 && ms4_rep[2])
					st <= S_Z;
				else
					st <= S_D1_PRE;
			end
			S_D1_PRE: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				for (l = 0; l < 4; l = l + 1) begin
					r_d1_ulo[2 * l +: 2] <= {2{dc[0] && d1_cmp_ulo[l]}};
					r_d1_uhi[2 * l +: 2] <= {2{dc[0] && !d1_cmp_ulo[l] && d1_cmp_uhi[l]}};
				end
`endif
				st <= S_D1;
			end
			S_D1: begin
				o_valid <= 1'b1;
				st <= S_OUT;
			end
			S_Z: begin
				if (sz_done) begin
					o_valid <= 1'b1;
					st <= S_OUT;
				end
			end
			default:	/* S_OUT */
				if (o_ready) begin
					o_valid <= 1'b0;
					st <= S_IDLE;
				end
			endcase
		end
	end

	generate
		if (ENABLE_MSAA != 0) begin : g_ms_dp
			reg [4:0] c, c_next;
			reg [3:0] r_co;
			reg [127:0] ds;
			reg [3:0] zexp;
			reg [31:0] zs_pipe;
			reg [3:0] co_pipe;
			reg zs_pipe_en;
			reg [31:0] zs_pipe1b;
			reg [3:0] co_pipe1b;
			reg zs_pipe1b_en;
			reg [16:0] r_zs_lo_s1, r_zs_hi_s1;
			reg [31:0] zs_pipe2;
			(* keep = "true", dont_touch = "true" *) reg [3:0] co_pipe2_rep [0:3];
			reg zs_pipe2_en;
			(* equivalent_register_removal = "no" *) reg [1:0] zs_pipe2_ulo;
			(* equivalent_register_removal = "no" *) reg [1:0] zs_pipe2_uhi;
			reg [31:0] r_zs_final;
			reg [3:0] r_co_final;
			reg r_zs_en_final;
			reg [31:0] fa_in_a;
			reg [31:0] fa_in_b;
			reg [511:0] r_sdepth_ms;
			assign o_sdepth = r_sdepth_ms;
			assign fa_in_a_w = fa_in_a;
			assign fa_in_b_w = fa_in_b;
			assign fa_sz_ival = (st == S_Z && c < 5'd16);
			assign sz_done = (c == 5'd23);

			wire [3:0] r_co_p1;
			wire r_co_p1_co_unused;
			am_add #(.W(4)) u_add_co (.a(r_co), .b(4'd1), .ci(1'b0), .s(r_co_p1), .co(r_co_p1_co_unused));
			wire [3:0] co = r_co;

			wire [4:0] c1_inc;
			wire c1_co_unused;
			am_add #(.W(5)) u_add_c1 (.a(c_next), .b(5'd1), .ci(1'b0), .s(c1_inc), .co(c1_co_unused));
			wire [4:0] c1 = c_next;
`ifdef SYNTH_KINTEX7
			wire [1:0] co_hi_a, co_hi_b;
			wire [1:0] c1_hi_a, c1_hi_b, c1_lo_a, c1_lo_b;
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_coh0a (.I0(co[2]), .O(co_hi_a[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_coh1a (.I0(co[3]), .O(co_hi_a[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_coh0b (.I0(co[2]), .O(co_hi_b[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_coh1b (.I0(co[3]), .O(co_hi_b[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1h0a (.I0(c1[2]), .O(c1_hi_a[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1h1a (.I0(c1[3]), .O(c1_hi_a[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1h0b (.I0(c1[2]), .O(c1_hi_b[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1h1b (.I0(c1[3]), .O(c1_hi_b[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1l0a (.I0(c1[0]), .O(c1_lo_a[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1l1a (.I0(c1[1]), .O(c1_lo_a[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1l0b (.I0(c1[0]), .O(c1_lo_b[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c1l1b (.I0(c1[1]), .O(c1_lo_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			wire [1:0] co_hi_a, co_hi_b;
			wire [1:0] c1_hi_a, c1_hi_b, c1_lo_a, c1_lo_b;
			assign co_hi_a[0] = co[2];
			assign co_hi_a[1] = co[3];
			assign co_hi_b[0] = co[2];
			assign co_hi_b[1] = co[3];
			assign c1_hi_a[0] = c1[2];
			assign c1_hi_a[1] = c1[3];
			assign c1_hi_b[0] = c1[2];
			assign c1_hi_b[1] = c1[3];
			assign c1_lo_a[0] = c1[0];
			assign c1_lo_a[1] = c1[1];
			assign c1_lo_b[0] = c1[0];
			assign c1_lo_b[1] = c1[1];
`else
			wire [1:0] co_hi_a = co[3:2], co_hi_b = co[3:2];
			wire [1:0] c1_hi_a = c1[3:2], c1_hi_b = c1[3:2];
			wire [1:0] c1_lo_a = c1[1:0], c1_lo_b = c1[1:0];
`endif
			(* keep = "true" *) wire [3:0] fa_m_nz;
			assign fa_m_nz[0] = (fa_r[5:0]   != 6'd0);
			assign fa_m_nz[1] = (fa_r[11:6]  != 6'd0);
			assign fa_m_nz[2] = (fa_r[17:12] != 6'd0);
			assign fa_m_nz[3] = (fa_r[22:18] != 5'd0);
			wire fa_mant_nz = |fa_m_nz;
			wire fa_is_nan = (fa_r[30:23] == 8'hFF) && fa_mant_nz;
			wire fa_gt_one = fa_r[30] || (fa_r[29:23] == 7'h7F && fa_mant_nz);
			wire zcl_zero_raw = fa_r[31] || fa_is_nan;
`ifdef SYNTH_KINTEX7
			wire [1:0] dc1_buf, zcl_zero_buf, fa_gt1_buf;
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dc1_0 (.I0(dc[1]),        .O(dc1_buf[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dc1_1 (.I0(dc[1]),        .O(dc1_buf[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zz_0  (.I0(zcl_zero_raw), .O(zcl_zero_buf[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zz_1  (.I0(zcl_zero_raw), .O(zcl_zero_buf[1]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fg1_0 (.I0(fa_gt_one),    .O(fa_gt1_buf[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fg1_1 (.I0(fa_gt_one),    .O(fa_gt1_buf[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			wire [1:0] dc1_buf, zcl_zero_buf, fa_gt1_buf;
			assign dc1_buf[0] = dc[1];
			assign dc1_buf[1] = dc[1];
			assign zcl_zero_buf[0] = zcl_zero_raw;
			assign zcl_zero_buf[1] = zcl_zero_raw;
			assign fa_gt1_buf[0] = fa_gt_one;
			assign fa_gt1_buf[1] = fa_gt_one;
`else
			wire [1:0] dc1_buf = {2{dc[1]}};
			wire [1:0] zcl_zero_buf = {2{zcl_zero_raw}};
			wire [1:0] fa_gt1_buf = {2{fa_gt_one}};
`endif
			wire [31:0] zcl = {
				!dc1_buf[1] ? (zcl_zero_buf[1] ? 16'd0 : (fa_gt1_buf[1] ? ONE[31:16] : fa_r[31:16])) : fa_r[31:16],
				!dc1_buf[0] ? (zcl_zero_buf[0] ? 16'd0 : (fa_gt1_buf[0] ? ONE[15:0]  : fa_r[15:0]))  : fa_r[15:0]
			};
			(* keep = "true" *) wire zexp_sel_a = zexp[co_hi_a];
			(* keep = "true" *) wire zexp_sel_b = zexp[co_hi_b];
			(* keep = "true" *) wire [15:0] z_sel_a = z[{co_hi_a, 1'b0, 4'b0000} +: 16];
			(* keep = "true" *) wire [15:0] z_sel_b = z[{co_hi_b, 1'b1, 4'b0000} +: 16];
			wire [31:0] zs4 = {
				zexp_sel_b ? z_sel_b : zcl[31:16],
				zexp_sel_a ? z_sel_a : zcl[15:0]
			};

			/* 2-stage zclamp: Stage 1 -> r_zs_lo_s1/hi_s1 -> Stage 2 -> zs_pipe2_ulo/uhi -> Stage 3 r_zs_final */
			wire [16:0] zs_lo_s1_w, zs_hi_s1_w;
			wire zs_cmp_ulo, zs_cmp_uhi;
			atg_rop_ms_fle_s1 u_zcl_lo_s1 (.a(zs_pipe), .b(zmin), .s1(zs_lo_s1_w));
			atg_rop_ms_fle_s1 u_zcl_hi_s1 (.a(zmax), .b(zs_pipe), .s1(zs_hi_s1_w));
			atg_rop_ms_fle_s2 #(.NAN_TRUE(1)) u_zcl_lo_s2 (.s1(r_zs_lo_s1), .le(zs_cmp_ulo));
			atg_rop_ms_fle_s2 #(.NAN_TRUE(0)) u_zcl_hi_s2 (.s1(r_zs_hi_s1), .le(zs_cmp_uhi));
			wire [31:0] zs_pipe2_final = {
				zs_pipe2_ulo[1] ? zmin[31:16] : (zs_pipe2_uhi[1] ? zmax[31:16] : zs_pipe2[31:16]),
				zs_pipe2_ulo[0] ? zmin[15:0]  : (zs_pipe2_uhi[0] ? zmax[15:0]  : zs_pipe2[15:0])
			};

`ifdef SYNTH_KINTEX7
			wire en_sz_raw = (st == S_Z);
			wire en_sz_fa_buf;
			wire [1:0] en_sz_mid;
			wire [8:0] en_ms_buf;
			genvar g_ms;
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zfa (.I0(en_sz_raw), .O(en_sz_fa_buf));
			for (g_ms = 0; g_ms < 2; g_ms = g_ms + 1) begin : g_sz_mid
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zm (.I0(en_sz_raw), .O(en_sz_mid[g_ms]));
			end
			for (g_ms = 0; g_ms < 9; g_ms = g_ms + 1) begin : g_buf_ms
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bms (
					.I0(en_if_mid[1]), .O(en_ms_buf[g_ms])
				);
			end
			for (g_ms = 0; g_ms < 8; g_ms = g_ms + 1) begin : g_reg_ds
				always @(posedge clk) begin
					if (en_ms_buf[g_ms])
						ds[16 * g_ms +: 16] <= i_aux[20 + 16 * g_ms +: 16];
				end
			end
			always @(posedge clk) begin
				if (en_ms_buf[8])
					zexp <= i_aux[19:16];
			end

			wire [31:0] fa_sz_a = {z[{c1_hi_b, 1'b1, 4'b0000} +: 16], z[{c1_hi_a, 1'b0, 4'b0000} +: 16]};
			wire [31:0] fa_sz_b = {ds[{c1_lo_b, 1'b1, 4'b0000} +: 16], ds[{c1_lo_a, 1'b0, 4'b0000} +: 16]};
			wire fa_ce_raw = zf_prep || en_if_mid[2] || en_sz_fa_buf;
			for (g_ms = 0; g_ms < 2; g_ms = g_ms + 1) begin : g_reg_fa
				wire fa_ce_a, fa_ce_b, fa_zfp_a, fa_zfp_b, fa_sz_en_a, fa_sz_en_b;
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fca (.I0(fa_ce_raw), .O(fa_ce_a));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fcb (.I0(fa_ce_raw), .O(fa_ce_b));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fpa (.I0(zf_prep),   .O(fa_zfp_a));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fpb (.I0(zf_prep),   .O(fa_zfp_b));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fza (.I0(en_sz_fa_buf), .O(fa_sz_en_a));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fzb (.I0(en_sz_fa_buf), .O(fa_sz_en_b));
				always @(posedge clk or posedge rst_sync) begin
					if (rst_sync) begin
						fa_in_a[16 * g_ms +: 16] <= 16'd0;
						fa_in_b[16 * g_ms +: 16] <= 16'd0;
					end else begin
						if (fa_ce_a)
							fa_in_a[16 * g_ms +: 16] <= fa_zfp_a ? vzo[16 * g_ms +: 16] :
								(fa_sz_en_a ? fa_sz_a[16 * g_ms +: 16] : i_depth[16 * g_ms +: 16]);
						if (fa_ce_b)
							fa_in_b[16 * g_ms +: 16] <= fa_zfp_b ? vzs[16 * g_ms +: 16] :
								(fa_sz_en_b ? fa_sz_b[16 * g_ms +: 16] : i_aux[20 + 16 * g_ms +: 16]);
					end
				end
			end

			for (g_ms = 0; g_ms < 16; g_ms = g_ms + 1) begin : g_reg_sdepth
				localparam IS_S0 = ((g_ms % 4) == 0);
				localparam LANE_IDX = g_ms / 4;
				(* keep = "true" *) wire w_z_hit = en_sz_mid[g_ms / 8] && r_zs_en_final && (r_co_final == g_ms[3:0]);
				wire w_en_raw = w_z_hit || en_prep_mid[g_ms / 8] || (IS_S0 && en_d1_mid[g_ms / 8]);
				wire [1:0] w_en_buf, w_z_buf, w_d1_buf;
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_we0 (.I0(w_en_raw), .O(w_en_buf[0]));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_we1 (.I0(w_en_raw), .O(w_en_buf[1]));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wz0 (.I0(w_z_hit), .O(w_z_buf[0]));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wz1 (.I0(w_z_hit), .O(w_z_buf[1]));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wd0 (.I0(en_d1_mid[g_ms / 8]), .O(w_d1_buf[0]));
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wd1 (.I0(en_d1_mid[g_ms / 8]), .O(w_d1_buf[1]));
				always @(posedge clk) begin
					if (w_en_buf[0]) begin
						if (w_z_buf[0])
							r_sdepth_ms[32 * g_ms +: 16] <= r_zs_final[15:0];
						else if (IS_S0 && w_d1_buf[0])
							r_sdepth_ms[32 * g_ms +: 16] <= d1c[32 * LANE_IDX +: 16];
						else
							r_sdepth_ms[32 * g_ms +: 16] <= 16'd0;
					end
					if (w_en_buf[1]) begin
						if (w_z_buf[1])
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= r_zs_final[31:16];
						else if (IS_S0 && w_d1_buf[1])
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= d1c[32 * LANE_IDX + 16 +: 16];
						else
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= 16'd0;
					end
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			wire en_sz_raw = (st == S_Z);
			wire en_sz_fa_buf;
			wire [1:0] en_sz_mid;
			wire [8:0] en_ms_buf;
			genvar g_ms;
			assign en_sz_fa_buf = en_sz_raw;
			for (g_ms = 0; g_ms < 2; g_ms = g_ms + 1) begin : g_sz_mid
				assign en_sz_mid[g_ms] = en_sz_raw;
			end
			for (g_ms = 0; g_ms < 9; g_ms = g_ms + 1) begin : g_buf_ms
				assign en_ms_buf[g_ms] = en_if_mid[1];
			end
			for (g_ms = 0; g_ms < 8; g_ms = g_ms + 1) begin : g_reg_ds
				always @(posedge clk) begin
					if (en_ms_buf[g_ms])
						ds[16 * g_ms +: 16] <= i_aux[20 + 16 * g_ms +: 16];
				end
			end
			always @(posedge clk) begin
				if (en_ms_buf[8])
					zexp <= i_aux[19:16];
			end

			wire [31:0] fa_sz_a = {z[{c1_hi_b, 1'b1, 4'b0000} +: 16], z[{c1_hi_a, 1'b0, 4'b0000} +: 16]};
			wire [31:0] fa_sz_b = {ds[{c1_lo_b, 1'b1, 4'b0000} +: 16], ds[{c1_lo_a, 1'b0, 4'b0000} +: 16]};
			wire fa_ce_raw = zf_prep || en_if_mid[2] || en_sz_fa_buf;
			for (g_ms = 0; g_ms < 2; g_ms = g_ms + 1) begin : g_reg_fa
				wire fa_ce_a, fa_ce_b, fa_zfp_a, fa_zfp_b, fa_sz_en_a, fa_sz_en_b;
				assign fa_ce_a = fa_ce_raw;
				assign fa_ce_b = fa_ce_raw;
				assign fa_zfp_a = zf_prep;
				assign fa_zfp_b = zf_prep;
				assign fa_sz_en_a = en_sz_fa_buf;
				assign fa_sz_en_b = en_sz_fa_buf;
				always @(posedge clk or posedge rst_sync) begin
					if (rst_sync) begin
						fa_in_a[16 * g_ms +: 16] <= 16'd0;
						fa_in_b[16 * g_ms +: 16] <= 16'd0;
					end else begin
						if (fa_ce_a)
							fa_in_a[16 * g_ms +: 16] <= fa_zfp_a ? vzo[16 * g_ms +: 16] :
								(fa_sz_en_a ? fa_sz_a[16 * g_ms +: 16] : i_depth[16 * g_ms +: 16]);
						if (fa_ce_b)
							fa_in_b[16 * g_ms +: 16] <= fa_zfp_b ? vzs[16 * g_ms +: 16] :
								(fa_sz_en_b ? fa_sz_b[16 * g_ms +: 16] : i_aux[20 + 16 * g_ms +: 16]);
					end
				end
			end

			for (g_ms = 0; g_ms < 16; g_ms = g_ms + 1) begin : g_reg_sdepth
				localparam IS_S0 = ((g_ms % 4) == 0);
				localparam LANE_IDX = g_ms / 4;
				(* keep = "true" *) wire w_z_hit = en_sz_mid[g_ms / 8] && r_zs_en_final && (r_co_final == g_ms[3:0]);
				wire w_en_raw = w_z_hit || en_prep_mid[g_ms / 8] || (IS_S0 && en_d1_mid[g_ms / 8]);
				wire [1:0] w_en_buf, w_z_buf, w_d1_buf;
				assign w_en_buf[0] = w_en_raw;
				assign w_en_buf[1] = w_en_raw;
				assign w_z_buf[0] = w_z_hit;
				assign w_z_buf[1] = w_z_hit;
				assign w_d1_buf[0] = en_d1_mid[g_ms / 8];
				assign w_d1_buf[1] = en_d1_mid[g_ms / 8];
				always @(posedge clk) begin
					if (w_en_buf[0]) begin
						if (w_z_buf[0])
							r_sdepth_ms[32 * g_ms +: 16] <= r_zs_final[15:0];
						else if (IS_S0 && w_d1_buf[0])
							r_sdepth_ms[32 * g_ms +: 16] <= d1c[32 * LANE_IDX +: 16];
						else
							r_sdepth_ms[32 * g_ms +: 16] <= 16'd0;
					end
					if (w_en_buf[1]) begin
						if (w_z_buf[1])
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= r_zs_final[31:16];
						else if (IS_S0 && w_d1_buf[1])
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= d1c[32 * LANE_IDX + 16 +: 16];
						else
							r_sdepth_ms[32 * g_ms + 16 +: 16] <= 16'd0;
					end
				end
			end
`endif

			always @(posedge clk) begin
				zs_pipe <= zs4;
				co_pipe <= co;
				zs_pipe1b <= zs_pipe;
				co_pipe1b <= co_pipe;
				r_zs_lo_s1 <= zs_lo_s1_w;
				r_zs_hi_s1 <= zs_hi_s1_w;
				zs_pipe2 <= zs_pipe1b;
				co_pipe2_rep[0] <= co_pipe1b;
				co_pipe2_rep[1] <= co_pipe1b;
				co_pipe2_rep[2] <= co_pipe1b;
				co_pipe2_rep[3] <= co_pipe1b;
				zs_pipe2_ulo <= {2{dc[0] && zs_cmp_ulo}};
				zs_pipe2_uhi <= {2{dc[0] && !zs_cmp_ulo && zs_cmp_uhi}};
				r_zs_final <= zs_pipe2_final;
				r_co_final <= co_pipe2_rep[0];
			end

			integer ml;
			always @(posedge clk or posedge rst_sync) begin
				if (rst_sync) begin
					c <= 5'd0;
					c_next <= 5'd1;
					r_co <= 4'd12;
					zs_pipe_en <= 1'b0;
					zs_pipe1b_en <= 1'b0;
					zs_pipe2_en <= 1'b0;
					r_zs_en_final <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					fa_in_a <= 32'd0;
					fa_in_b <= 32'd0;
`endif
				end else begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					if (zf_prep) begin
						fa_in_a <= vzo;
						fa_in_b <= vzs;
					end
`endif
					case (st)
					S_IDLE:
						if (i_valid && first) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
							ds <= i_aux[147:20];
							zexp <= i_aux[19:16];
							fa_in_a <= i_depth[31:0];
							fa_in_b <= i_aux[20 +: 32];
`endif
						end
					S_PREP: begin
						c <= 5'd0;
						c_next <= 5'd1;
						r_co <= 4'd12;
						zs_pipe_en <= 1'b0;
						zs_pipe1b_en <= 1'b0;
						zs_pipe2_en <= 1'b0;
						r_zs_en_final <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						r_sdepth_ms <= 512'd0;
`endif
					end
					S_D1: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						for (ml = 0; ml < 4; ml = ml + 1)
							r_sdepth_ms[128 * ml +: 32] <= d1c[32 * ml +: 32];
`endif
					end
					S_Z: begin
						c <= c_next;
						c_next <= c1_inc;
						r_co <= r_co_p1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						fa_in_a <= {z[{c1_hi_b, 1'b1, 4'b0000} +: 16], z[{c1_hi_a, 1'b0, 4'b0000} +: 16]};
						fa_in_b <= {ds[{c1_lo_b, 1'b1, 4'b0000} +: 16], ds[{c1_lo_a, 1'b0, 4'b0000} +: 16]};
`endif
						zs_pipe_en <= (c >= 5'd4 && c < 5'd20);
						zs_pipe1b_en <= zs_pipe_en;
						zs_pipe2_en <= zs_pipe1b_en;
						r_zs_en_final <= zs_pipe2_en;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						if (r_zs_en_final)
							r_sdepth_ms[32 * r_co_final +: 32] <= r_zs_final;
`endif
					end
					default: ;
					endcase
				end
			end

			wire ms_unused_ok = &{1'b0, co_pipe2_rep[0], co_pipe2_rep[1], co_pipe2_rep[2], co_pipe2_rep[3], r_co_p1_co_unused, c1_co_unused};
		end else begin : g_no_ms_dp
			reg [31:0] fa_in_a;
			reg [31:0] fa_in_b;
			reg [127:0] r_sdepth_s0;
			assign fa_in_a_w = fa_in_a;
			assign fa_in_b_w = fa_in_b;
			assign fa_sz_ival = 1'b0;
			assign sz_done = 1'b1;

			genvar g_s0;
			for (g_s0 = 0; g_s0 < 4; g_s0 = g_s0 + 1) begin : g_s0_out
				assign o_sdepth[128 * g_s0 +: 32] = r_sdepth_s0[32 * g_s0 +: 32];
				assign o_sdepth[128 * g_s0 + 32 +: 96] = 96'd0;
			end

			always @(posedge clk or posedge rst_sync) begin
				if (rst_sync) begin
					fa_in_a <= 32'd0;
					fa_in_b <= 32'd0;
				end else if (zf_prep) begin
					fa_in_a <= vzo;
					fa_in_b <= vzs;
				end
			end

`ifdef SYNTH_KINTEX7
			for (g_s0 = 0; g_s0 < 4; g_s0 = g_s0 + 1) begin : g_reg_sdepth_s0
				always @(posedge clk) begin
					if (en_prep_mid[g_s0 / 2])
						r_sdepth_s0[32 * g_s0 +: 32] <= 32'd0;
					else if (en_d1_mid[g_s0 / 2])
						r_sdepth_s0[32 * g_s0 +: 32] <= d1c[32 * g_s0 +: 32];
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			for (g_s0 = 0; g_s0 < 4; g_s0 = g_s0 + 1) begin : g_reg_sdepth_s0
				always @(posedge clk) begin
					if (en_prep_mid[g_s0 / 2])
						r_sdepth_s0[32 * g_s0 +: 32] <= 32'd0;
					else if (en_d1_mid[g_s0 / 2])
						r_sdepth_s0[32 * g_s0 +: 32] <= d1c[32 * g_s0 +: 32];
				end
			end
`else
			always @(posedge clk) begin
				if (st == S_PREP)
					r_sdepth_s0 <= 128'd0;
				else if (st == S_D1)
					r_sdepth_s0 <= d1c;
			end
`endif
		end
	endgenerate

	wire unused_ok = &{1'b0, st_wdata[31:10], st_wdata[7:4], fa_o};
endmodule

/* Stage 1 of Pure-LUT fp32 <= comparator (4-bit leaves -> 2-level tree, <= 5 cmos3) */
module atg_rop_ms_fle_s1 (
	input wire [31:0] a,
	input wire [31:0] b,
	output wire [16:0] s1
);
	(* keep = "true" *) wire le0_0 = (a[2:0] <= b[2:0]);
	(* keep = "true" *) wire ge0_0 = (a[2:0] >= b[2:0]);
	(* keep = "true" *) wire [6:1] lt0, gt0, eq0;
	genvar i;
	generate
	for (i = 1; i < 6; i = i + 1) begin : g_s0
		wire [3:0] sa = a[4 * i - 1 +: 4];
		wire [3:0] sb = b[4 * i - 1 +: 4];
		assign eq0[i] = (sa == sb);
		assign lt0[i] = (sa < sb);
		assign gt0[i] = (sa > sb);
	end
	endgenerate
	assign eq0[6] = (a[30:23] == b[30:23]);
	assign lt0[6] = (a[30:23] <  b[30:23]);
	assign gt0[6] = (a[30:23] >  b[30:23]);

	(* keep = "true" *) wire a_e0 = (a[30:26] == 5'h1F);
	(* keep = "true" *) wire a_e1 = (a[25:23] == 3'b111);
	(* keep = "true" *) wire a_z0 = (a[30:26] == 5'd0);
	(* keep = "true" *) wire a_z1 = (a[25:23] == 3'd0);
	(* keep = "true" *) wire [3:0] a_mnz;
	assign a_mnz[0] = (a[5:0]   != 6'd0);
	assign a_mnz[1] = (a[11:6]  != 6'd0);
	assign a_mnz[2] = (a[17:12] != 6'd0);
	assign a_mnz[3] = (a[22:18] != 5'd0);

	(* keep = "true" *) wire b_e0 = (b[30:26] == 5'h1F);
	(* keep = "true" *) wire b_e1 = (b[25:23] == 3'b111);
	(* keep = "true" *) wire b_z0 = (b[30:26] == 5'd0);
	(* keep = "true" *) wire b_z1 = (b[25:23] == 3'd0);
	(* keep = "true" *) wire [3:0] b_mnz;
	assign b_mnz[0] = (b[5:0]   != 6'd0);
	assign b_mnz[1] = (b[11:6]  != 6'd0);
	assign b_mnz[2] = (b[17:12] != 6'd0);
	assign b_mnz[3] = (b[22:18] != 5'd0);

	(* keep = "true" *) wire le1_0 = lt0[1] | (eq0[1] & le0_0);
	(* keep = "true" *) wire ge1_0 = gt0[1] | (eq0[1] & ge0_0);
	(* keep = "true" *) wire [2:1] lt1, gt1, eq1;
	assign eq1[1] = eq0[3] & eq0[2];
	assign lt1[1] = lt0[3] | (eq0[3] & lt0[2]);
	assign gt1[1] = gt0[3] | (eq0[3] & gt0[2]);
	assign eq1[2] = eq0[5] & eq0[4];
	assign lt1[2] = lt0[5] | (eq0[5] & lt0[4]);
	assign gt1[2] = gt0[5] | (eq0[5] & gt0[4]);
	(* keep = "true" *) wire eq_top = eq0[6];
	(* keep = "true" *) wire lt_top = lt0[6];
	(* keep = "true" *) wire gt_top = gt0[6];

	(* keep = "true" *) wire a_nan  = a_e0 & a_e1 & (|a_mnz);
	(* keep = "true" *) wire b_nan  = b_e0 & b_e1 & (|b_mnz);
	(* keep = "true" *) wire a_zero = a_z0 & a_z1 & ~(|a_mnz);
	(* keep = "true" *) wire b_zero = b_z0 & b_z1 & ~(|b_mnz);

	assign s1 = {a[31], b[31], a_nan, b_nan, a_zero, b_zero, eq_top, lt_top, gt_top, eq1, lt1, gt1, le1_0, ge1_0};
endmodule

/* Stage 2 of Pure-LUT fp32 <= comparator (<= 2 LUT6 / <= 6 cmos3 levels) */
module atg_rop_ms_fle_s2 #(
	parameter NAN_TRUE = 0
) (
	input wire [16:0] s1,
	output wire le
);
	wire a_sign   = s1[16];
	wire b_sign   = s1[15];
	wire a_nan    = s1[14];
	wire b_nan    = s1[13];
	wire a_zero   = s1[12];
	wire b_zero   = s1[11];
	wire eq_top   = s1[10];
	wire lt_top   = s1[9];
	wire gt_top   = s1[8];
	wire [2:1] eq1 = s1[7:6];
	wire [2:1] lt1 = s1[5:4];
	wire [2:1] gt1 = s1[3:2];
	wire le1_0    = s1[1];
	wire ge1_0    = s1[0];

	(* keep = "true" *) wire le_26_0  = lt1[2] | (eq1[2] & lt1[1]) | (eq1[2] & eq1[1] & le1_0);
	(* keep = "true" *) wire ge_26_0  = gt1[2] | (eq1[2] & gt1[1]) | (eq1[2] & eq1[1] & ge1_0);
	(* keep = "true" *) wire use_le26 = !a_nan && !b_nan && !a_sign && !b_sign && eq_top;
	(* keep = "true" *) wire use_ge26 = !a_nan && !b_nan &&  a_sign &&  b_sign && eq_top;
	(* keep = "true" *) wire force_top = !a_nan && !b_nan && (a_sign ? (b_sign && gt_top) : (!b_sign && lt_top));
	(* keep = "true" *) wire force_sign = (NAN_TRUE != 0 ? a_nan : 1'b0) ||
		(!a_nan && !b_nan && (a_sign ? !b_sign : (b_sign && a_zero && b_zero)));

	assign le = force_sign || force_top || (use_le26 && le_26_0) || (use_ge26 && ge_26_0);
endmodule

/* Combinational wrapper for slow viewport config comparator (3-bit NOR3 leaves -> 9-bit tree, <= 10 cmos3) */
module atg_rop_ms_fle #(
	parameter NAN_TRUE = 0
) (
	input wire [31:0] a,
	input wire [31:0] b,
	output wire le
);
	/* Pure-LUT 0-XOR signed-magnitude 2->4->8->16->32 AOI3/OAI3 comparator (<= 10 cmos3) */
	wire [30:0] ag_gtn = (~a[30:0]) | b[30:0];
	wire [30:0] ag_gen = (~a[30:0]) & b[30:0];
	wire [30:0] bg_gtn = (~b[30:0]) | a[30:0];
	wire [30:0] bg_gen = (~b[30:0]) & a[30:0];

	wire [15:0] ag_gt2, ag_ge2, bg_gt2, bg_ge2;
	genvar gk;
	generate
	for (gk = 0; gk < 15; gk = gk + 1) begin : g_c2
		assign ag_gt2[gk] = !(ag_gen[2*gk+1] | (ag_gtn[2*gk+1] & ag_gtn[2*gk]));
		assign ag_ge2[gk] = !(ag_gen[2*gk+1] | (ag_gtn[2*gk+1] & ag_gen[2*gk]));
		assign bg_gt2[gk] = !(bg_gen[2*gk+1] | (bg_gtn[2*gk+1] & bg_gtn[2*gk]));
		assign bg_ge2[gk] = !(bg_gen[2*gk+1] | (bg_gtn[2*gk+1] & bg_gen[2*gk]));
	end
	assign ag_gt2[15] = !ag_gtn[30];
	assign ag_ge2[15] = !ag_gen[30];
	assign bg_gt2[15] = !bg_gtn[30];
	assign bg_ge2[15] = !bg_gen[30];

	wire [7:0] ag_gtn4, ag_gen4, bg_gtn4, bg_gen4;
	for (gk = 0; gk < 8; gk = gk + 1) begin : g_c4
		assign ag_gtn4[gk] = !(ag_gt2[2*gk+1] | (ag_ge2[2*gk+1] & ag_gt2[2*gk]));
		assign ag_gen4[gk] = !(ag_gt2[2*gk+1] | (ag_ge2[2*gk+1] & ag_ge2[2*gk]));
		assign bg_gtn4[gk] = !(bg_gt2[2*gk+1] | (bg_ge2[2*gk+1] & bg_gt2[2*gk]));
		assign bg_gen4[gk] = !(bg_gt2[2*gk+1] | (bg_ge2[2*gk+1] & bg_ge2[2*gk]));
	end

	wire [3:0] ag_gt8, ag_ge8, bg_gt8, bg_ge8;
	for (gk = 0; gk < 4; gk = gk + 1) begin : g_c8
		assign ag_gt8[gk] = !(ag_gtn4[2*gk+1] & (ag_gen4[2*gk+1] | ag_gtn4[2*gk]));
		assign ag_ge8[gk] = !(ag_gtn4[2*gk+1] & (ag_gen4[2*gk+1] | ag_gen4[2*gk]));
		assign bg_gt8[gk] = !(bg_gtn4[2*gk+1] & (bg_gen4[2*gk+1] | bg_gtn4[2*gk]));
		assign bg_ge8[gk] = !(bg_gtn4[2*gk+1] & (bg_gen4[2*gk+1] | bg_gen4[2*gk]));
	end

	wire [1:0] ag_gtn16, ag_gen16, bg_gtn16, bg_gen16;
	for (gk = 0; gk < 2; gk = gk + 1) begin : g_c16
		assign ag_gtn16[gk] = !(ag_gt8[2*gk+1] | (ag_ge8[2*gk+1] & ag_gt8[2*gk]));
		assign ag_gen16[gk] = !(ag_gt8[2*gk+1] | (ag_ge8[2*gk+1] & ag_ge8[2*gk]));
		assign bg_gtn16[gk] = !(bg_gt8[2*gk+1] | (bg_ge8[2*gk+1] & bg_gt8[2*gk]));
		assign bg_gen16[gk] = !(bg_gt8[2*gk+1] | (bg_ge8[2*gk+1] & bg_ge8[2*gk]));
	end
	endgenerate

	wire a_gt_b = !(ag_gtn16[1] & (ag_gen16[1] | ag_gtn16[0]));
	wire b_gt_a = !(bg_gtn16[1] & (bg_gen16[1] | bg_gtn16[0]));
	wire unused_fle_ge = ag_gen16[0] & bg_gen16[0];

	/* 3-3-3 NOR3/NAND3 exponent/mantissa flags (<= 5 cmos3, decoupled from comparator) */
	wire [7:0] a_m3_n = {
		!(|a[22:21]), !(|a[20:18]), !(|a[17:15]), !(|a[14:12]),
		!(|a[11:9]),  !(|a[8:6]),   !(|a[5:3]),   !(|a[2:0])
	};
	wire [7:0] b_m3_n = {
		!(|b[22:21]), !(|b[20:18]), !(|b[17:15]), !(|b[14:12]),
		!(|b[11:9]),  !(|b[8:6]),   !(|b[5:3]),   !(|b[2:0])
	};
	wire [2:0] a_m9 = {!(a_m3_n[7] & a_m3_n[6]), !(a_m3_n[5] & a_m3_n[4] & a_m3_n[3]), !(a_m3_n[2] & a_m3_n[1] & a_m3_n[0])};
	wire [2:0] b_m9 = {!(b_m3_n[7] & b_m3_n[6]), !(b_m3_n[5] & b_m3_n[4] & b_m3_n[3]), !(b_m3_n[2] & b_m3_n[1] & b_m3_n[0])};
	wire a_mnz_n = !(a_m9[2] | a_m9[1] | a_m9[0]);
	wire b_mnz_n = !(b_m9[2] | b_m9[1] | b_m9[0]);

	wire a_e_ff_n = !( (&a[30:28]) & (&a[27:25]) & (&a[24:23]) );
	wire b_e_ff_n = !( (&b[30:28]) & (&b[27:25]) & (&b[24:23]) );
	wire a_nan = !(a_e_ff_n | a_mnz_n);
	wire b_nan = !(b_e_ff_n | b_mnz_n);

	wire a_enz = !(!(|a[30:28]) & !(|a[27:25]) & !(|a[24:23]));
	wire b_enz = !(!(|b[30:28]) & !(|b[27:25]) & !(|b[24:23]));
	wire not_bz_hi = !( !(a_m9[2] | a_m9[1] | a_m9[0] | a_enz) & !(b_m9[2] | b_m9[1] | b_m9[0] | b_enz) );

	wire s_11 = a[31] & b[31];
	wire s_00 = !(a[31] | b[31]);
	wire s_01 = (~a[31]) & b[31];
	wire bad_11_n = !(s_11 & b_gt_a);
	wire bad_00_n = !(s_00 & a_gt_b);
	wire not_bad01_n = !(bad_11_n & bad_00_n);
	wire bad_other = (s_01 & not_bz_hi) | (NAN_TRUE != 0 ? b_nan : (a_nan | b_nan));
	wire le_core = !(not_bad01_n | bad_other);
	assign le = (NAN_TRUE != 0) ? (a_nan | le_core) : le_core;
endmodule

`default_nettype wire
