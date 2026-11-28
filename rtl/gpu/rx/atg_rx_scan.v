/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_scan: RX quad traversal & Fine Rasterizer.
 *
 * Traverses bounding boxes via Coarse Tile Rasterizer (atg_rx_coarse,
 * 8x8 pixel tiles) and evaluates coverage of 2x2 quads. Empty tiles outside
 * the triangle are skipped in 1 cycle. Trivially accepted tiles bypass edge
 * testing.
 *
 * In boundary tiles, MSAA 4x coverage (16 sample locations across 4 lanes)
 * is evaluated in parallel using 48 addsub units, resolving fq_scov and
 * fq_ccov in a single clock cycle without serialization.
 *
 * Strict Guardrails: Verilog-2005 only, zero raw arithmetic operators.
 */

`timescale 1ns / 1ps
`default_nettype none

`include "atg_config.vh"
`ifdef CFG_ENABLE_MSAA
`define ATG_RX_DEFAULT_ENABLE_MSAA (`CFG_ENABLE_MSAA != 0)
`else
`define ATG_RX_DEFAULT_ENABLE_MSAA 0
`endif

`ifdef CFG_MSAA_SAMPLES
`define ATG_RX_DEFAULT_MSAA_SAMPLES `CFG_MSAA_SAMPLES
`else
`define ATG_RX_DEFAULT_MSAA_SAMPLES 1
`endif

`ifdef CFG_ENABLE_EARLY_Z
`define ATG_RX_DEFAULT_ENABLE_EARLY_Z (`CFG_ENABLE_EARLY_Z != 0)
`else
`define ATG_RX_DEFAULT_ENABLE_EARLY_Z 0
`endif

`ifdef CFG_COARSE_SCAN
`define ATG_RX_DEFAULT_COARSE_SCAN (`CFG_COARSE_SCAN != 0)
`else
`define ATG_RX_DEFAULT_COARSE_SCAN 0
`endif

module atg_rx_scan #(
	parameter HAS_SU = 1,	/* 1: fragment warps to the SU; 0: quads straight to the ROP with DEBUG_COLOR */
	parameter ENABLE_MSAA = `ATG_RX_DEFAULT_ENABLE_MSAA,
	parameter MSAA_SAMPLES = `ATG_RX_DEFAULT_MSAA_SAMPLES,
	parameter ENABLE_EARLY_Z = `ATG_RX_DEFAULT_ENABLE_EARLY_Z,
	parameter COARSE_SCAN = `ATG_RX_DEFAULT_COARSE_SCAN
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* From atg_rx_setup */
	input wire sp_valid,
	output wire sp_ready,
	input wire [767:0] sp_tri,
	input wire sp_last,
	output reg pt_free,
	output reg [7:0] pt_free_idx,
	/* Covered quads of a non-raw triangle to atg_rx_frag (HAS_SU) */
	output wire fq_valid,
	input wire fq_ready,
	output wire [31:0] fq_xy,
	output wire [15:0] fq_scov,
	output wire [3:0] fq_ccov,
	output wire fq_end,
	input wire f_busy,
	/* Direct fragments to the ROP (no SU: DEBUG_COLOR) */
	output reg dx_valid,
	input wire dx_ready,
	output reg [15:0] dx_seq,
	output reg [31:0] dx_xy,
	output reg [3:0] dx_mask,
	(* extract_reset = "no" *) output reg [127:0] dx_depth,
	output reg dx_last,
	/* Sequence numbers (atg_rx_order) */
	output wire sq_req,
	input wire [15:0] sq_seq,
	output wire busy
);
	`include "ghci_regs.vh"
	localparam [3:0] S_IDLE = 4'd0, S_SEQ = 4'd1, S_TILE_WAIT = 4'd2, S_WALK = 4'd3,
		S_Z = 4'd4, S_EMIT = 4'd5, S_LAST = 4'd6, S_FREE = 4'd7, S_FQ = 4'd8, S_STEP = 4'd9,
		S_SAMP = 4'd10, S_EVAL = 4'd11, S_EVAL0 = 4'd12, S_EVAL1 = 4'd13, S_EVAL2 = 4'd14;
`ifdef SYNTH_KINTEX7
	wire rst_n_inv = !rst_n;
	wire ctv_fsm = coarse_tile_valid;
	wire ctv_edge = coarse_tile_valid;
	wire sp_val_fsm = sp_valid;
	wire sp_val_edge = sp_valid;
	wire sp_val_qx = sp_valid;
	wire sp_val_qy = sp_valid;
	wire we_ent_lo = sp_valid;
	wire we_ent_hi = sp_valid;
	wire sp_val_ctrl = sp_valid;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rst_n_inv = !rst_n;
	wire ctv_fsm = coarse_tile_valid;
	wire ctv_edge = coarse_tile_valid;
	wire sp_val_fsm = sp_valid;
	wire sp_val_edge = sp_valid;
	wire sp_val_qx = sp_valid;
	wire sp_val_qy = sp_valid;
	wire we_ent_lo = sp_valid;
	wire we_ent_hi = sp_valid;
	wire sp_val_ctrl = sp_valid;
`else
	wire rst_n_inv = !rst_n;
	wire ctv_fsm = coarse_tile_valid;
	wire ctv_edge = coarse_tile_valid;
	wire sp_val_fsm = sp_valid;
	wire sp_val_edge = sp_valid;
	wire sp_val_qx = sp_valid;
	wire sp_val_qy = sp_valid;
	wire we_ent_lo = sp_valid;
	wire we_ent_hi = sp_valid;
	wire sp_val_ctrl = sp_valid;
`endif
	(* fsm_encoding = "none", extract_enable = "no", max_fanout = 16 *) reg [3:0] st;
`ifdef SYNTH_KINTEX7
	wire [3:0] st_fsm_0 = st;
	wire [3:0] st_fsm_1 = st;
	wire [3:0] st_fsm_2 = st;
	wire [3:0] st_fsm_3 = st;
	wire [3:0] st_fsm_4 = st;
	wire [3:0] st_fsm_5 = st;
	wire [3:0] st_fsm_6 = st;
	wire [3:0] st_fsm_7 = st;
	wire [3:0] st_fsm_8 = st;
	wire [3:0] st_fsm_main = st;
	wire [3:0] st_fsm_m1 = st;
	wire [3:0] st_fsm_m2 = st;
	wire [3:0] st_fsm_m3 = st;
	wire [3:0] st_fsm_m4 = st;
	wire [3:0] st_fsm_m5 = st;
	wire [3:0] st_fsm_m6 = st;
	wire [3:0] st_fsm_m7 = st;
	wire [4:0] zc_0, zc_1, zc_2, zc_3;
	wire cur_trivial_cov, cur_trivial_scov;
`ifdef SYNTH_KINTEX7
	genvar g_zcb;
	generate
	for (g_zcb = 0; g_zcb < 5; g_zcb = g_zcb + 1) begin : g_zc_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zc0 (.I0(zc[g_zcb]), .O(zc_0[g_zcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zc1 (.I0(zc[g_zcb]), .O(zc_1[g_zcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zc2 (.I0(zc[g_zcb]), .O(zc_2[g_zcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zc3 (.I0(zc[g_zcb]), .O(zc_3[g_zcb]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ctc (.I0(cur_trivial), .O(cur_trivial_cov));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cts (.I0(cur_trivial), .O(cur_trivial_scov));
`else
	assign zc_0 = zc;
	assign zc_1 = zc;
	assign zc_2 = zc;
	assign zc_3 = zc;
	assign cur_trivial_cov = cur_trivial;
	assign cur_trivial_scov = cur_trivial;
`endif
`define SET_ST(val) begin st[1:0] <= (val); end
`define SET_ST_HI(val) begin st[3:2] <= ((val) >> 2); end
`define SET_ZC(val) begin zc <= (val); end
`define SET_TRIV(val) begin cur_trivial <= (val); end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] st_fsm_0 = st;
	wire [3:0] st_fsm_1 = st;
	wire [3:0] st_fsm_2 = st;
	wire [3:0] st_fsm_3 = st;
	wire [3:0] st_fsm_4 = st;
	wire [3:0] st_fsm_5 = st;
	wire [3:0] st_fsm_6 = st;
	wire [3:0] st_fsm_7 = st;
	wire [3:0] st_fsm_8 = st;
	wire [3:0] st_fsm_main = st;
	wire [3:0] st_fsm_m1 = st;
	wire [3:0] st_fsm_m2 = st;
	wire [3:0] st_fsm_m3 = st;
	wire [3:0] st_fsm_m4 = st;
	wire [3:0] st_fsm_m5 = st;
	wire [3:0] st_fsm_m6 = st;
	wire [3:0] st_fsm_m7 = st;
	wire [4:0] zc_0, zc_1, zc_2, zc_3;
	wire cur_trivial_cov, cur_trivial_scov;
	genvar g_zcb;
	generate
	for (g_zcb = 0; g_zcb < 5; g_zcb = g_zcb + 1) begin : g_zc_buf
		assign zc_0[g_zcb] = zc[g_zcb];
		assign zc_1[g_zcb] = zc[g_zcb];
		assign zc_2[g_zcb] = zc[g_zcb];
		assign zc_3[g_zcb] = zc[g_zcb];
	end
	endgenerate
	assign cur_trivial_cov = cur_trivial;
	assign cur_trivial_scov = cur_trivial;
`define SET_ST(val) begin st[1:0] <= (val); end
`define SET_ST_HI(val) begin st[3:2] <= ((val) >> 2); end
`define SET_ZC(val) begin zc <= (val); end
`define SET_TRIV(val) begin cur_trivial <= (val); end
`else
	wire [3:0] st_fsm_0 = st;
	wire [3:0] st_fsm_1 = st;
	wire [3:0] st_fsm_2 = st;
	wire [3:0] st_fsm_3 = st;
	wire [3:0] st_fsm_4 = st;
	wire [3:0] st_fsm_5 = st;
	wire [3:0] st_fsm_6 = st;
	wire [3:0] st_fsm_7 = st;
	wire [3:0] st_fsm_8 = st;
	wire [3:0] st_fsm_main = st;
	wire [3:0] st_fsm_m1 = st;
	wire [3:0] st_fsm_m2 = st;
	wire [3:0] st_fsm_m3 = st;
	wire [3:0] st_fsm_m4 = st;
	wire [3:0] st_fsm_m5 = st;
	wire [3:0] st_fsm_m6 = st;
	wire [3:0] st_fsm_m7 = st;
	wire [4:0] zc_0 = zc;
	wire [4:0] zc_1 = zc;
	wire [4:0] zc_2 = zc;
	wire [4:0] zc_3 = zc;
	wire cur_trivial_cov = cur_trivial;
	wire cur_trivial_scov = cur_trivial;
`define SET_ST(val) begin st <= (val); end
`define SET_ZC(val) begin zc <= (val); end
`define SET_TRIV(val) begin cur_trivial <= (val); end
`endif
	(* max_fanout = 16 *) reg r_tile_row_end;
	(* max_fanout = 16 *) reg r_tile_col_end;
	(* max_fanout = 16 *) reg wm;		/* warp mode: a non-raw triangle with an SU */
	(* max_fanout = 16 *) reg fend;	/* S_FQ: the end marker (no quad) */

	/* The triangle. */
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] a8 [0:2];	/* A << 8: one pixel in x */
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] b8 [0:2];
	(* extract_reset = "no" *) reg signed [47:0] er [0:2];	/* the edges at the tile's current quad row */
	(* extract_reset = "no" *) reg signed [47:0] ec [0:2];	/* at the current quad */
	reg [15:0] bx0, by0, bx1, by1;
	(* extract_enable = "no", extract_reset = "no" *) reg [15:0] qx, qy;
	reg [31:0] az, bz, cz;
	(* extract_enable = "no", extract_reset = "no" *) reg [23:0] ent;
	reg [2:0] fl;
	reg raw;
	reg [1:0] fi;	/* S_FREE: vertex */
	integer k;

	/* Multisampling: MS_CTRL[3:0] = 2 and SAMPLE_KILL[3:0]. */
	reg ms4;
	reg [3:0] kill;

	always @(posedge clk or posedge rst_n_inv)
		if (rst_n_inv) begin
			ms4 <= 1'b0;
			kill <= 4'd0;
		end else if (st_we && st_idx == GHCI_S_MS_CTRL)
			ms4 <= (ENABLE_MSAA != 0) && (st_wdata[3:0] == 4'd2);
		else if (st_we && st_idx == GHCI_S_SAMPLE_KILL)
			kill <= (ENABLE_MSAA != 0) ? st_wdata[3:0] : 4'd0;

	/* Coarse Tile Rasterizer Interface */
	reg coarse_tri_valid;
	wire coarse_tri_ready;
	wire coarse_tile_valid;
	reg coarse_tile_ready;
	wire [15:0] coarse_tile_x, coarse_tile_y;
	wire [47:0] coarse_tile_e0, coarse_tile_e1, coarse_tile_e2;
	wire coarse_tile_trivial, coarse_tile_last, coarse_busy;

	reg [15:0] cur_tx, cur_ty;
	reg cur_trivial;
	reg cur_last_tile;

	/* The edges back to the quad origin (S_IDLE), one quad right (ec + 2 A) and
	 * one row down (er + 2 B); */
	(* extract_reset = "no" *) reg [47:0] r_er_next [0:2];
	(* extract_reset = "no" *) reg [47:0] r_ec_next [0:2];
	reg [2:0] r_er_c24, r_ec_c24;
	wire [47:0] er0 [0:2];
	wire [47:0] ec_n [0:2];
	wire [47:0] er_n [0:2];
	wire [23:0] ec_lo_w [0:2];
	wire [23:0] er_lo_w [0:2];
	wire [23:0] ec_hi_w [0:2];
	wire [23:0] er_hi_w [0:2];
	wire [2:0]  ec_c24_w, er_c24_w;
	wire [11:0] ed_carry_unused;
	genvar ge;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_er
		assign er0[ge] = sp_tri[96 * ge + 48 +: 48];
`ifdef SYNTH_KINTEX7
		am_add #(.W(48)) u_ec_n (.a(ec[ge]), .b(a8[ge] <<< 1), .ci(1'b0), .s(ec_n[ge]),
			.co(ed_carry_unused[4 * ge + 2]));
		am_add #(.W(48)) u_er_n (.a(er[ge]), .b(b8[ge] <<< 1), .ci(1'b0), .s(er_n[ge]),
			.co(ed_carry_unused[4 * ge + 3]));
		assign ec_lo_w[ge] = 24'd0; assign er_lo_w[ge] = 24'd0;
		assign ec_hi_w[ge] = 24'd0; assign er_hi_w[ge] = 24'd0;
		assign ec_c24_w[ge] = 1'b0; assign er_c24_w[ge] = 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
		am_add #(.W(48)) u_ec_n (.a(ec[ge]), .b(a8[ge] <<< 1), .ci(1'b0), .s(ec_n[ge]),
			.co(ed_carry_unused[4 * ge + 2]));
		am_add #(.W(48)) u_er_n (.a(er[ge]), .b(b8[ge] <<< 1), .ci(1'b0), .s(er_n[ge]),
			.co(ed_carry_unused[4 * ge + 3]));
		assign ec_lo_w[ge] = 24'd0; assign er_lo_w[ge] = 24'd0;
		assign ec_hi_w[ge] = 24'd0; assign er_hi_w[ge] = 24'd0;
		assign ec_c24_w[ge] = 1'b0; assign er_c24_w[ge] = 1'b0;
`else
		wire [47:0] step_cx = a8[ge] <<< 1;
		wire [47:0] step_ry = b8[ge] <<< 1;
		wire ec_hi_co_u, er_hi_co_u;
		am_add #(.W(24)) u_ec_lo (.a(ec[ge][23:0]),  .b(step_cx[23:0]),  .ci(1'b0),        .s(ec_lo_w[ge]), .co(ec_c24_w[ge]));
		am_add #(.W(24)) u_ec_hi (.a(ec[ge][47:24]), .b(step_cx[47:24]), .ci(r_ec_c24[ge]), .s(ec_hi_w[ge]), .co(ec_hi_co_u));
		am_add #(.W(24)) u_er_lo (.a(er[ge][23:0]),  .b(step_ry[23:0]),  .ci(1'b0),        .s(er_lo_w[ge]), .co(er_c24_w[ge]));
		am_add #(.W(24)) u_er_hi (.a(er[ge][47:24]), .b(step_ry[47:24]), .ci(r_er_c24[ge]), .s(er_hi_w[ge]), .co(er_hi_co_u));
		assign ec_n[ge] = {ec_hi_w[ge], r_ec_next[ge][23:0]};
		assign er_n[ge] = {er_hi_w[ge], r_er_next[ge][23:0]};
		assign ed_carry_unused[4 * ge + 2] = ec_hi_co_u;
		assign ed_carry_unused[4 * ge + 3] = er_hi_co_u;
`endif
	end
	endgenerate

	/* Quad origin edge correction split across S_IDLE (x offset -> er_a_r) and S_SEQ (y offset -> ec/er) */
	wire [47:0] er_direct_init [0:2];
	generate
	if (COARSE_SCAN == 0) begin : g_er_direct
		for (ge = 0; ge < 3; ge = ge + 1) begin : g_direct_corr
			wire [47:0] ta8 = {{16{sp_tri[96 * ge + 23]}}, sp_tri[96 * ge +: 24], 8'd0};
			wire [47:0] er_a_w;
			(* extract_reset = "no" *) reg [47:0] er_a_r;
			am_sub #(.W(48)) u_er_a (
				.a(er0[ge]),
				.b(sp_tri[288] ? ta8 : 48'd0),
				.bi(1'b0),
				.d(er_a_w),
				.bo(ed_carry_unused[4 * ge])
			);
			always @(posedge clk) begin
				if ((st_fsm_5 == S_IDLE) && sp_val_fsm)
					er_a_r <= er_a_w;
			end
			am_sub #(.W(48)) u_er0 (
				.a(er_a_r),
				.b(by0[0] ? b8[ge] : 48'd0),
				.bi(1'b0),
				.d(er_direct_init[ge]),
				.bo(ed_carry_unused[4 * ge + 1])
			);
		end
	end else begin : g_no_direct_corr
		for (ge = 0; ge < 3; ge = ge + 1) begin : g_direct_raw
			assign er_direct_init[ge] = er0[ge];
			assign ed_carry_unused[4 * ge +: 2] = 2'b00;
		end
	end
	endgenerate

	/* Sample offsets generator */
	wire [143:0] eb_cov_unused;
	wire [143:0] lowmv;
	wire [143:0] cov_op, cov_oq;
`ifdef SYNTH_KINTEX7
	wire we_sp_cv;
	(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'h00010000)) u_wspcv (
		.I0(st_fsm_5[0]), .I1(st_fsm_5[1]), .I2(st_fsm_5[2]), .I3(st_fsm_5[3]),
		.I4(sp_val_fsm), .O(we_sp_cv)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire we_sp_cv;
	(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'h00010000)) u_wspcv (
		.I0(st_fsm_5[0]), .I1(st_fsm_5[1]), .I2(st_fsm_5[2]), .I3(st_fsm_5[3]),
		.I4(sp_val_fsm), .O(we_sp_cv)
	);
`else
	wire we_sp_cv = (st == S_IDLE) && sp_valid;
`endif
	generate
	if (ENABLE_MSAA != 0) begin : g_cov
		wire [2:0] cv_ea_s23;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cvea0 (.I0(sp_tri[23]),  .O(cv_ea_s23[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cvea1 (.I0(sp_tri[119]), .O(cv_ea_s23[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cvea2 (.I0(sp_tri[215]), .O(cv_ea_s23[2]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign cv_ea_s23[0] = sp_tri[23];
		assign cv_ea_s23[1] = sp_tri[119];
		assign cv_ea_s23[2] = sp_tri[215];
`else
		assign cv_ea_s23 = {sp_tri[215], sp_tri[119], sp_tri[23]};
`endif
		atg_rx_cov cv (
			.clk(clk),
			.rst_n(rst_n),
			.i_load(we_sp_cv),
			.i_ab({sp_tri[239:216], cv_ea_s23[2], sp_tri[214:192],
			       sp_tri[143:120], cv_ea_s23[1], sp_tri[118:96],
			       sp_tri[47:24],   cv_ea_s23[0], sp_tri[22:0]}),
			.i_ec({ec[2], ec[1], ec[0]}),
			.i_start(1'b0),
			.i_step(1'b0),
			.i_sc(2'd0),
			.i_samp(1'b0),
			.o_eb(eb_cov_unused),
			.o_lowm(lowmv),
			.o_op(cov_op),
			.o_oq(cov_oq)
		);
	end else begin : g_no_cov
		assign eb_cov_unused = 144'd0;
		assign lowmv = 144'd0;
		assign cov_op = 144'd0;
		assign cov_oq = 144'd0;
	end
	endgenerate

	wire s_e0_a_scan = s_e0_a_scan_ext;
	wire s_e0_b_scan = s_e0_b_scan_ext;
	wire s_e1_a_scan = s_e1_a_scan_ext;
	wire s_e1_b_scan = s_e1_b_scan_ext;
	wire s_e2_a_scan = s_e2_a_scan_ext;
	wire s_e2_b_scan = s_e2_b_scan_ext;

`ifdef SYNTH_KINTEX7
	wire [2:0] se0a_ce_l, se0b_ce_l, se1a_ce_l, se1b_ce_l, se2a_ce_l, se2b_ce_l;
	wire s_e0_a_coarse_reg, s_e0_b_coarse_reg, s_e1_a_coarse_reg, s_e1_b_coarse_reg, s_e2_a_coarse_reg, s_e2_b_coarse_reg;
	wire s_e0_a_scan_ext, s_e0_b_scan_ext, s_e1_a_scan_ext, s_e1_b_scan_ext, s_e2_a_scan_ext, s_e2_b_scan_ext;
	genvar g_seb;
	generate
	for (g_seb = 0; g_seb < 3; g_seb = g_seb + 1) begin : g_se_ce
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se0a (.I0(sp_tri[23]),  .O(se0a_ce_l[g_seb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se0b (.I0(sp_tri[47]),  .O(se0b_ce_l[g_seb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se1a (.I0(sp_tri[119]), .O(se1a_ce_l[g_seb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se1b (.I0(sp_tri[143]), .O(se1b_ce_l[g_seb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se2a (.I0(sp_tri[215]), .O(se2a_ce_l[g_seb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_se2b (.I0(sp_tri[239]), .O(se2b_ce_l[g_seb]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr0a (.I0(sp_tri[23]),  .O(s_e0_a_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr0b (.I0(sp_tri[47]),  .O(s_e0_b_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr1a (.I0(sp_tri[119]), .O(s_e1_a_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr1b (.I0(sp_tri[143]), .O(s_e1_b_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr2a (.I0(sp_tri[215]), .O(s_e2_a_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scr2b (.I0(sp_tri[239]), .O(s_e2_b_coarse_reg));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx0a (.I0(sp_tri[23]),  .O(s_e0_a_scan_ext));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx0b (.I0(sp_tri[47]),  .O(s_e0_b_scan_ext));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx1a (.I0(sp_tri[119]), .O(s_e1_a_scan_ext));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx1b (.I0(sp_tri[143]), .O(s_e1_b_scan_ext));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx2a (.I0(sp_tri[215]), .O(s_e2_a_scan_ext));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssx2b (.I0(sp_tri[239]), .O(s_e2_b_scan_ext));

	wire ms4_misc, ms4_act;
	wire [3:0] ms4_act_q;
	wire [8:0] ms4_lm_b;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ms4m (.I0(ms4), .O(ms4_misc));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ms4a (.I0(ms4), .O(ms4_act));
	genvar g_ms4;
	generate
	for (g_ms4 = 0; g_ms4 < 4; g_ms4 = g_ms4 + 1) begin : g_ms4_q
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_m4q (.I0(ms4_act), .O(ms4_act_q[g_ms4]));
	end
	for (g_ms4 = 0; g_ms4 < 9; g_ms4 = g_ms4 + 1) begin : g_ms4_lm
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_m4l (.I0(ms4), .O(ms4_lm_b[g_ms4]));
	end
	endgenerate
	wire [47:0] lowm0_muxed = {ms4_lm_b[2] ? lowmv[32 +: 16]  : 16'd0, ms4_lm_b[1] ? lowmv[16 +: 16]  : 16'd0, ms4_lm_b[0] ? lowmv[0 +: 16]  : 16'd0};
	wire [47:0] lowm1_muxed = {ms4_lm_b[5] ? lowmv[80 +: 16]  : 16'd0, ms4_lm_b[4] ? lowmv[64 +: 16]  : 16'd0, ms4_lm_b[3] ? lowmv[48 +: 16] : 16'd0};
	wire [47:0] lowm2_muxed = {ms4_lm_b[8] ? lowmv[128 +: 16] : 16'd0, ms4_lm_b[7] ? lowmv[112 +: 16] : 16'd0, ms4_lm_b[6] ? lowmv[96 +: 16] : 16'd0};

	wire we_tile_x_raw = (st_fsm_4 == S_TILE_WAIT && ctv_edge) ||
	                     (st_fsm_4 == S_STEP && r_tile_row_end && r_tile_col_end && !cur_last_tile && ctv_edge) ||
	                     (st_fsm_4 == S_SEQ && COARSE_SCAN == 0);
	wire we_tile_x_lo, we_tile_x_hi, we_tile_y_lo, we_tile_y_hi;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wtxl (.I0(we_tile_x_raw), .O(we_tile_x_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wtxh (.I0(we_tile_x_raw), .O(we_tile_x_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wtyl (.I0(we_tile_x_raw), .O(we_tile_y_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wtyh (.I0(we_tile_x_raw), .O(we_tile_y_hi));

	wire [15:0] next_tile_tx = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? {bx0[15:1], 1'b0} : coarse_tile_x;
	wire [15:0] next_tile_ty = (COARSE_SCAN == 0 && st_fsm_8 == S_SEQ) ? {by0[15:1], 1'b0} : coarse_tile_y;
	wire next_tile_triv = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? 1'b0 : coarse_tile_trivial;
	wire next_tile_last = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? 1'b1 : coarse_tile_last;

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			cur_tx <= 16'd0;
			`SET_TRIV(1'b0);
		end else begin
			if (we_tile_x_lo) cur_tx <= next_tile_tx;
			if (we_tile_x_hi) `SET_TRIV(next_tile_triv);
		end
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			cur_ty <= 16'd0;
			cur_last_tile <= 1'b0;
		end else begin
			if (we_tile_y_lo) cur_ty <= next_tile_ty;
			if (we_tile_y_hi) cur_last_tile <= next_tile_last;
		end
	end

	wire we_fa_raw = (st_fsm_1 == S_Z) && ((zc_0 >= 5'd11 && zc_0 <= 5'd14) || (zc_0 >= 5'd16 && zc_0 <= 5'd19));
	wire [3:0] we_fa_leaf;
	wire [3:0] sel_zc_lo, sel_zc_hi;
	wire sel_zc_b_lo, sel_zc_b_hi;
`ifdef SYNTH_KINTEX7
	wire zc_b_raw = (zc_0 >= 5'd11 && zc_0 <= 5'd14);
	genvar g_zcf;
	generate
	for (g_zcf = 0; g_zcf < 4; g_zcf = g_zcf + 1) begin : g_zc_fbuf
		wire zc_hit_w = (zc_2 == (5'd11 + g_zcf[4:0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wfa  (.I0(we_fa_raw), .O(we_fa_leaf[g_zcf]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_szcl (.I0(zc_hit_w),  .O(sel_zc_lo[g_zcf]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_szch (.I0(zc_hit_w),  .O(sel_zc_hi[g_zcf]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_szbl (.I0(zc_b_raw), .O(sel_zc_b_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_szbh (.I0(zc_b_raw), .O(sel_zc_b_hi));
`else
	assign we_fa_leaf  = {4{we_fa_raw}};
	assign sel_zc_lo   = {zc_2 == 5'd14, zc_2 == 5'd13, zc_2 == 5'd12, zc_2 == 5'd11};
	assign sel_zc_hi   = sel_zc_lo;
	assign sel_zc_b_lo = (zc_0 >= 5'd11 && zc_0 <= 5'd14);
	assign sel_zc_b_hi = sel_zc_b_lo;
`endif

	wire [15:0] next_fa_a_lo = sel_zc_lo[0] ? t1_0[15:0] :
	                           sel_zc_lo[1] ? t1_1[15:0] :
	                           sel_zc_lo[2] ? t1_2[15:0] :
	                           sel_zc_lo[3] ? t1_3[15:0] : fa_r[15:0];
	wire [15:0] next_fa_a_hi = sel_zc_hi[0] ? t1_0[31:16] :
	                           sel_zc_hi[1] ? t1_1[31:16] :
	                           sel_zc_hi[2] ? t1_2[31:16] :
	                           sel_zc_hi[3] ? t1_3[31:16] : fa_r[31:16];

	wire [15:0] next_fa_b_lo = sel_zc_b_lo ? fm_r[15:0] : cz[15:0];
	wire [15:0] next_fa_b_hi = sel_zc_b_hi ? fm_r[31:16] : cz[31:16];

	always @(posedge clk) begin
		if (we_fa_leaf[0]) r_fa_a_lo <= next_fa_a_lo;
		if (we_fa_leaf[1]) r_fa_a_hi <= next_fa_a_hi;
		if (we_fa_leaf[2]) r_fa_b_lo <= next_fa_b_lo;
		if (we_fa_leaf[3]) r_fa_b_hi <= next_fa_b_hi;
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] se0a_ce_l, se0b_ce_l, se1a_ce_l, se1b_ce_l, se2a_ce_l, se2b_ce_l;
	wire s_e0_a_coarse_reg, s_e0_b_coarse_reg, s_e1_a_coarse_reg, s_e1_b_coarse_reg, s_e2_a_coarse_reg, s_e2_b_coarse_reg;
	wire s_e0_a_scan_ext, s_e0_b_scan_ext, s_e1_a_scan_ext, s_e1_b_scan_ext, s_e2_a_scan_ext, s_e2_b_scan_ext;
	genvar g_seb;
	generate
	for (g_seb = 0; g_seb < 3; g_seb = g_seb + 1) begin : g_se_ce
		assign se0a_ce_l[g_seb] = sp_tri[23];
		assign se0b_ce_l[g_seb] = sp_tri[47];
		assign se1a_ce_l[g_seb] = sp_tri[119];
		assign se1b_ce_l[g_seb] = sp_tri[143];
		assign se2a_ce_l[g_seb] = sp_tri[215];
		assign se2b_ce_l[g_seb] = sp_tri[239];
	end
	endgenerate
	assign s_e0_a_coarse_reg = sp_tri[23];
	assign s_e0_b_coarse_reg = sp_tri[47];
	assign s_e1_a_coarse_reg = sp_tri[119];
	assign s_e1_b_coarse_reg = sp_tri[143];
	assign s_e2_a_coarse_reg = sp_tri[215];
	assign s_e2_b_coarse_reg = sp_tri[239];
	assign s_e0_a_scan_ext = sp_tri[23];
	assign s_e0_b_scan_ext = sp_tri[47];
	assign s_e1_a_scan_ext = sp_tri[119];
	assign s_e1_b_scan_ext = sp_tri[143];
	assign s_e2_a_scan_ext = sp_tri[215];
	assign s_e2_b_scan_ext = sp_tri[239];

	wire ms4_misc, ms4_act;
	wire [3:0] ms4_act_q;
	wire [8:0] ms4_lm_b;
	assign ms4_misc = ms4;
	assign ms4_act = ms4;
	genvar g_ms4;
	generate
	for (g_ms4 = 0; g_ms4 < 4; g_ms4 = g_ms4 + 1) begin : g_ms4_q
		assign ms4_act_q[g_ms4] = ms4_act;
	end
	for (g_ms4 = 0; g_ms4 < 9; g_ms4 = g_ms4 + 1) begin : g_ms4_lm
		assign ms4_lm_b[g_ms4] = ms4;
	end
	endgenerate
	wire [47:0] lowm0_muxed = {ms4_lm_b[2] ? lowmv[32 +: 16]  : 16'd0, ms4_lm_b[1] ? lowmv[16 +: 16]  : 16'd0, ms4_lm_b[0] ? lowmv[0 +: 16]  : 16'd0};
	wire [47:0] lowm1_muxed = {ms4_lm_b[5] ? lowmv[80 +: 16]  : 16'd0, ms4_lm_b[4] ? lowmv[64 +: 16]  : 16'd0, ms4_lm_b[3] ? lowmv[48 +: 16] : 16'd0};
	wire [47:0] lowm2_muxed = {ms4_lm_b[8] ? lowmv[128 +: 16] : 16'd0, ms4_lm_b[7] ? lowmv[112 +: 16] : 16'd0, ms4_lm_b[6] ? lowmv[96 +: 16] : 16'd0};

	wire we_tile_x_raw = (st_fsm_4 == S_TILE_WAIT && ctv_edge) ||
	                     (st_fsm_4 == S_STEP && r_tile_row_end && r_tile_col_end && !cur_last_tile && ctv_edge) ||
	                     (st_fsm_4 == S_SEQ && COARSE_SCAN == 0);
	wire we_tile_x_lo, we_tile_x_hi, we_tile_y_lo, we_tile_y_hi;
	assign we_tile_x_lo = we_tile_x_raw;
	assign we_tile_x_hi = we_tile_x_raw;
	assign we_tile_y_lo = we_tile_x_raw;
	assign we_tile_y_hi = we_tile_x_raw;

	wire [15:0] next_tile_tx = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? {bx0[15:1], 1'b0} : coarse_tile_x;
	wire [15:0] next_tile_ty = (COARSE_SCAN == 0 && st_fsm_8 == S_SEQ) ? {by0[15:1], 1'b0} : coarse_tile_y;
	wire next_tile_triv = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? 1'b0 : coarse_tile_trivial;
	wire next_tile_last = (COARSE_SCAN == 0 && st_fsm_4 == S_SEQ) ? 1'b1 : coarse_tile_last;

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			cur_tx <= 16'd0;
			`SET_TRIV(1'b0);
		end else begin
			if (we_tile_x_lo) cur_tx <= next_tile_tx;
			if (we_tile_x_hi) `SET_TRIV(next_tile_triv);
		end
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			cur_ty <= 16'd0;
			cur_last_tile <= 1'b0;
		end else begin
			if (we_tile_y_lo) cur_ty <= next_tile_ty;
			if (we_tile_y_hi) cur_last_tile <= next_tile_last;
		end
	end

	wire we_fa_raw = (st_fsm_1 == S_Z) && ((zc_0 >= 5'd11 && zc_0 <= 5'd14) || (zc_0 >= 5'd16 && zc_0 <= 5'd19));
	wire [3:0] we_fa_leaf;
	wire [3:0] sel_zc_lo, sel_zc_hi;
	wire sel_zc_b_lo, sel_zc_b_hi;
	wire zc_b_raw = (zc_0 >= 5'd11 && zc_0 <= 5'd14);
	genvar g_zcf;
	generate
	for (g_zcf = 0; g_zcf < 4; g_zcf = g_zcf + 1) begin : g_zc_fbuf
		wire zc_hit_w = (zc_2 == (5'd11 + g_zcf[4:0]));
		assign we_fa_leaf[g_zcf] = we_fa_raw;
		assign sel_zc_lo[g_zcf] = zc_hit_w;
		assign sel_zc_hi[g_zcf] = zc_hit_w;
	end
	endgenerate
	assign sel_zc_b_lo = zc_b_raw;
	assign sel_zc_b_hi = zc_b_raw;

	wire [15:0] next_fa_a_lo = sel_zc_lo[0] ? t1_0[15:0] :
	                           sel_zc_lo[1] ? t1_1[15:0] :
	                           sel_zc_lo[2] ? t1_2[15:0] :
	                           sel_zc_lo[3] ? t1_3[15:0] : fa_r[15:0];
	wire [15:0] next_fa_a_hi = sel_zc_hi[0] ? t1_0[31:16] :
	                           sel_zc_hi[1] ? t1_1[31:16] :
	                           sel_zc_hi[2] ? t1_2[31:16] :
	                           sel_zc_hi[3] ? t1_3[31:16] : fa_r[31:16];

	wire [15:0] next_fa_b_lo = sel_zc_b_lo ? fm_r[15:0] : cz[15:0];
	wire [15:0] next_fa_b_hi = sel_zc_b_hi ? fm_r[31:16] : cz[31:16];

	always @(posedge clk) begin
		if (we_fa_leaf[0]) r_fa_a_lo <= next_fa_a_lo;
		if (we_fa_leaf[1]) r_fa_a_hi <= next_fa_a_hi;
		if (we_fa_leaf[2]) r_fa_b_lo <= next_fa_b_lo;
		if (we_fa_leaf[3]) r_fa_b_hi <= next_fa_b_hi;
	end
`else
	wire [2:0] se0a_ce_l = {3{sp_tri[23]}}, se0b_ce_l = {3{sp_tri[47]}};
	wire [2:0] se1a_ce_l = {3{sp_tri[119]}}, se1b_ce_l = {3{sp_tri[143]}};
	wire [2:0] se2a_ce_l = {3{sp_tri[215]}}, se2b_ce_l = {3{sp_tri[239]}};
	wire s_e0_a_coarse_reg = sp_tri[23], s_e0_b_coarse_reg = sp_tri[47];
	wire s_e1_a_coarse_reg = sp_tri[119], s_e1_b_coarse_reg = sp_tri[143];
	wire s_e2_a_coarse_reg = sp_tri[215], s_e2_b_coarse_reg = sp_tri[239];
	wire s_e0_a_scan_ext = sp_tri[23], s_e0_b_scan_ext = sp_tri[47];
	wire s_e1_a_scan_ext = sp_tri[119], s_e1_b_scan_ext = sp_tri[143];
	wire s_e2_a_scan_ext = sp_tri[215], s_e2_b_scan_ext = sp_tri[239];
	wire [47:0] lowm0_muxed = ms4 ? lowmv[0 +: 48] : 48'd0;
	wire [47:0] lowm1_muxed = ms4 ? lowmv[48 +: 48] : 48'd0;
	wire [47:0] lowm2_muxed = ms4 ? lowmv[96 +: 48] : 48'd0;
	wire ms4_misc = ms4;
	wire ms4_act = ms4;
`endif

`ifdef SYNTH_KINTEX7
	wire rst_n_coarse = rst_n;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rst_n_coarse = rst_n;
`else
	wire rst_n_coarse = rst_n;
`endif

	generate
	if (COARSE_SCAN != 0) begin : g_coarse
		atg_rx_coarse u_coarse (
			.clk(clk),
			.rst_n(rst_n_coarse),
			.tri_valid(coarse_tri_valid),
			.tri_ready(coarse_tri_ready),
			.bbox_min_x(sp_tri[303:288]),
			.bbox_min_y(sp_tri[319:304]),
			.bbox_max_x(sp_tri[335:320]),
			.bbox_max_y(sp_tri[351:336]),
			.e0_a8({{5{se0a_ce_l[2]}}, {5{se0a_ce_l[1]}}, {6{se0a_ce_l[0]}}, sp_tri[0 +: 24], 8'd0}),
			.e0_b8({{5{se0b_ce_l[2]}}, {5{se0b_ce_l[1]}}, {6{se0b_ce_l[0]}}, sp_tri[24 +: 24], 8'd0}),
			.e0_er(er0[0]),
			.e1_a8({{5{se1a_ce_l[2]}}, {5{se1a_ce_l[1]}}, {6{se1a_ce_l[0]}}, sp_tri[96 +: 24], 8'd0}),
			.e1_b8({{5{se1b_ce_l[2]}}, {5{se1b_ce_l[1]}}, {6{se1b_ce_l[0]}}, sp_tri[96 + 24 +: 24], 8'd0}),
			.e1_er(er0[1]),
			.e2_a8({{5{se2a_ce_l[2]}}, {5{se2a_ce_l[1]}}, {6{se2a_ce_l[0]}}, sp_tri[192 +: 24], 8'd0}),
			.e2_b8({{5{se2b_ce_l[2]}}, {5{se2b_ce_l[1]}}, {6{se2b_ce_l[0]}}, sp_tri[192 + 24 +: 24], 8'd0}),
			.e2_er(er0[2]),
			.lowm0(lowm0_muxed),
			.lowm1(lowm1_muxed),
			.lowm2(lowm2_muxed),
			.tile_valid(coarse_tile_valid),
			.tile_ready(coarse_tile_ready),
			.tile_x(coarse_tile_x),
			.tile_y(coarse_tile_y),
			.tile_e0(coarse_tile_e0),
			.tile_e1(coarse_tile_e1),
			.tile_e2(coarse_tile_e2),
			.tile_trivial(coarse_tile_trivial),
			.tile_last(coarse_tile_last),
			.busy(coarse_busy)
		);
	end else begin : g_no_coarse
		assign coarse_tri_ready = 1'b1;
		assign coarse_tile_valid = 1'b0;
		assign coarse_tile_x = 16'd0;
		assign coarse_tile_y = 16'd0;
		assign coarse_tile_e0 = 48'd0;
		assign coarse_tile_e1 = 48'd0;
		assign coarse_tile_e2 = 48'd0;
		assign coarse_tile_trivial = 1'b0;
		assign coarse_tile_last = 1'b0;
		assign coarse_busy = 1'b0;
	end
	endgenerate

	/* Hierarchical Early-Z (Hi-Z) RAM */
	wire hiz_cull;
	wire [15:0] hiz_rd_zmax;
	generate
	if (ENABLE_EARLY_Z != 0) begin : g_hiz
		wire [10:0] hiz_rd_addr = {cur_ty[8:3], cur_tx[8:4]};
`ifdef SYNTH_KINTEX7
		wire hiz_wr_en = (st_fsm_5 == S_Z) && (zc_0 == 5'd19);
`elsif SYNTH_KINTEX_ULTRASCALE
		wire hiz_wr_en = (st_fsm_5 == S_Z) && (zc_0 == 5'd19);
`else
		wire hiz_wr_en = (st == S_Z) && (zc == 5'd19);
`endif
		wire [10:0] hiz_wr_addr = {qy[8:3], qx[8:4]};
		wire [15:0] hiz_wr_z = zv[31:16];
		atg_rx_hiz #(
			.DEPTH_W(16),
			.TILE_ADDR_W(11)
		) u_hiz (
			.clk(clk),
			.rst_n(rst_n),
			.clear(st_we && st_idx == GHCI_S_DEPTH),
			.rd_addr(hiz_rd_addr),
			.rd_zmax(hiz_rd_zmax),
			.wr_en(hiz_wr_en),
			.wr_addr(hiz_wr_addr),
			.wr_zmax(hiz_wr_z)
		);
		assign hiz_cull = 1'b0;
	end else begin : g_no_hiz
		assign hiz_cull = 1'b0;
		assign hiz_rd_zmax = 16'd0;
	end
	endgenerate

	/* Quad lane coordinate stepping: qx and qy are always even, so +1 is bit 0 set */
	wire [16:0] qx1 = {1'b0, qx[15:1], 1'b1};
	wire [16:0] qy1 = {1'b0, qy[15:1], 1'b1};
	wire [16:0] qx2, qy2;
	wire [3:0] qstep_carry_unused;
	assign qstep_carry_unused[1:0] = 2'b00;
`ifdef SYNTH_KINTEX7
	am_add #(.W(17)) u_qx2 (.a({1'b0, qx}), .b(17'd2), .ci(1'b0), .s(qx2), .co(qstep_carry_unused[2]));
	am_add #(.W(17)) u_qy2 (.a({1'b0, qy}), .b(17'd2), .ci(1'b0), .s(qy2), .co(qstep_carry_unused[3]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(17)) u_qx2 (.a({1'b0, qx}), .b(17'd2), .ci(1'b0), .s(qx2), .co(qstep_carry_unused[2]));
	am_add #(.W(17)) u_qy2 (.a({1'b0, qy}), .b(17'd2), .ci(1'b0), .s(qy2), .co(qstep_carry_unused[3]));
`else
	/* Pure-LUT 4-bit prefix-AND incrementer on [15:1] for +2 (<= 7 cmos3 vs 14) */
	function [16:0] scan_inc2;
		input [15:0] q;
		reg g4_0, g4_1, g4_2, c5, c9, c13;
		reg [3:0] b0, b1, b2, b3, i0, i1, i2, i3;
		begin
			b0 = q[4:1];
			b1 = q[8:5];
			b2 = q[12:9];
			b3 = {1'b0, q[15:13]};
			g4_0 = (b0[3] & b0[2]) & (b0[1] & b0[0]);
			g4_1 = (b1[3] & b1[2]) & (b1[1] & b1[0]);
			g4_2 = (b2[3] & b2[2]) & (b2[1] & b2[0]);
			c5   = g4_0;
			c9   = g4_1 & g4_0;
			c13  = g4_2 & c9;
			i0 = {b0[3] ^ (b0[2] & b0[1] & b0[0]), b0[2] ^ (b0[1] & b0[0]), b0[1] ^ b0[0], ~b0[0]};
			i1 = {b1[3] ^ (b1[2] & b1[1] & b1[0]), b1[2] ^ (b1[1] & b1[0]), b1[1] ^ b1[0], ~b1[0]};
			i2 = {b2[3] ^ (b2[2] & b2[1] & b2[0]), b2[2] ^ (b2[1] & b2[0]), b2[1] ^ b2[0], ~b2[0]};
			i3 = {b3[3] ^ (b3[2] & b3[1] & b3[0]), b3[2] ^ (b3[1] & b3[0]), b3[1] ^ b3[0], ~b3[0]};
			scan_inc2 = {
				c13 ? i3 : b3,
				c9  ? i2 : b2,
				c5  ? i1 : b1,
				i0,
				q[0]
			};
		end
	endfunction
	assign qx2 = scan_inc2(qx);
	assign qy2 = scan_inc2(qy);
	assign qstep_carry_unused[3:2] = 2'b00;
`endif

	/* Precompute ab8 = a8 + b8 */
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] ab8 [0:2];
	wire signed [47:0] ab8_comb [0:2];
	wire [2:0] ab8_carry_unused;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_ab8
		am_add #(.W(48)) u_ab8 (.a(a8[ge]), .b(b8[ge]), .ci(1'b0), .s(ab8_comb[ge]), .co(ab8_carry_unused[ge]));
	end
	endgenerate

	/* Lane edge values at pixel centre: ec, + A, + B, + (A + B) (2-cycle 24+24 under !SYNTH_KINTEX7) */
	wire [12*48-1:0] lve_comb;
	wire [12*24-1:0] lve_lo_w;
	wire [12*24-1:0] lve_hi_w;
	wire [11:0] lve_c24_w;
	reg  [11:0] r_lve_c24;
	wire [8:0] lve_carry_unused;
	(* extract_reset = "no" *) reg [12*48-1:0] r_lve;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_lv
`ifdef SYNTH_KINTEX7
		assign lve_comb[48 * (4 * ge) +: 48] = ec[ge];
		am_add #(.W(48)) u_la (.a(ec[ge]), .b(a8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 1) +: 48]),
			.co(lve_carry_unused[3 * ge + 0]));
		am_add #(.W(48)) u_lb (.a(ec[ge]), .b(b8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 2) +: 48]),
			.co(lve_carry_unused[3 * ge + 1]));
		am_add #(.W(48)) u_lab (.a(ec[ge]), .b(ab8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 3) +: 48]),
			.co(lve_carry_unused[3 * ge + 2]));
		assign lve_lo_w[24*(4*ge) +: 96] = 96'd0;
		assign lve_hi_w[24*(4*ge) +: 96] = 96'd0;
		assign lve_c24_w[4*ge +: 4] = 4'd0;
`elsif SYNTH_KINTEX_ULTRASCALE
		assign lve_comb[48 * (4 * ge) +: 48] = ec[ge];
		am_add #(.W(48)) u_la (.a(ec[ge]), .b(a8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 1) +: 48]),
			.co(lve_carry_unused[3 * ge + 0]));
		am_add #(.W(48)) u_lb (.a(ec[ge]), .b(b8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 2) +: 48]),
			.co(lve_carry_unused[3 * ge + 1]));
		am_add #(.W(48)) u_lab (.a(ec[ge]), .b(ab8[ge]), .ci(1'b0), .s(lve_comb[48 * (4 * ge + 3) +: 48]),
			.co(lve_carry_unused[3 * ge + 2]));
		assign lve_lo_w[24*(4*ge) +: 96] = 96'd0;
		assign lve_hi_w[24*(4*ge) +: 96] = 96'd0;
		assign lve_c24_w[4*ge +: 4] = 4'd0;
`else
		assign lve_lo_w[24 * (4 * ge) +: 24] = ec[ge][23:0];
		assign lve_hi_w[24 * (4 * ge) +: 24] = ec[ge][47:24];
		assign lve_c24_w[4 * ge] = 1'b0;
		wire la_hi_co_u, lb_hi_co_u, lab_hi_co_u;
		am_add #(.W(24)) u_la_lo  (.a(ec[ge][23:0]),  .b(a8[ge][23:0]),  .ci(1'b0),                  .s(lve_lo_w[24*(4*ge+1) +: 24]), .co(lve_c24_w[4*ge+1]));
		am_add #(.W(24)) u_la_hi  (.a(ec[ge][47:24]), .b(a8[ge][47:24]), .ci(r_lve_c24[4*ge+1]),     .s(lve_hi_w[24*(4*ge+1) +: 24]), .co(la_hi_co_u));
		am_add #(.W(24)) u_lb_lo  (.a(ec[ge][23:0]),  .b(b8[ge][23:0]),  .ci(1'b0),                  .s(lve_lo_w[24*(4*ge+2) +: 24]), .co(lve_c24_w[4*ge+2]));
		am_add #(.W(24)) u_lb_hi  (.a(ec[ge][47:24]), .b(b8[ge][47:24]), .ci(r_lve_c24[4*ge+2]),     .s(lve_hi_w[24*(4*ge+2) +: 24]), .co(lb_hi_co_u));
		am_add #(.W(24)) u_lab_lo (.a(ec[ge][23:0]),  .b(ab8[ge][23:0]), .ci(1'b0),                  .s(lve_lo_w[24*(4*ge+3) +: 24]), .co(lve_c24_w[4*ge+3]));
		am_add #(.W(24)) u_lab_hi (.a(ec[ge][47:24]), .b(ab8[ge][47:24]), .ci(r_lve_c24[4*ge+3]),    .s(lve_hi_w[24*(4*ge+3) +: 24]), .co(lab_hi_co_u));
		assign lve_comb[48*(4*ge+0) +: 48] = {lve_hi_w[24*(4*ge+0) +: 24], lve_lo_w[24*(4*ge+0) +: 24]};
		assign lve_comb[48*(4*ge+1) +: 48] = {lve_hi_w[24*(4*ge+1) +: 24], lve_lo_w[24*(4*ge+1) +: 24]};
		assign lve_comb[48*(4*ge+2) +: 48] = {lve_hi_w[24*(4*ge+2) +: 24], lve_lo_w[24*(4*ge+2) +: 24]};
		assign lve_comb[48*(4*ge+3) +: 48] = {lve_hi_w[24*(4*ge+3) +: 24], lve_lo_w[24*(4*ge+3) +: 24]};
		assign lve_carry_unused[3 * ge + 0] = la_hi_co_u;
		assign lve_carry_unused[3 * ge + 1] = lb_hi_co_u;
		assign lve_carry_unused[3 * ge + 2] = lab_hi_co_u;
`endif
	end
	endgenerate

`ifdef SYNTH_KINTEX7
	wire we_lve_root = (st_fsm_6 == S_WALK);
	wire [2:0] we_lve_mid, we_lve_s23_mid;
	wire [35:0] we_lve_leaf, we_lve_s23_leaf;
	(* keep = "true", dont_touch = "true" *) reg [12*48-1:0] r_lve_s23;
	genvar glvb;
	generate
	for (glvb = 0; glvb < 3; glvb = glvb + 1) begin : g_lve_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lvm   (.I0(we_lve_root), .O(we_lve_mid[glvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lvs23 (.I0(we_lve_root), .O(we_lve_s23_mid[glvb]));
	end
	for (glvb = 0; glvb < 36; glvb = glvb + 1) begin : g_lve_lf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lvlf   (.I0(we_lve_mid[glvb / 12]),     .O(we_lve_leaf[glvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lvs23l (.I0(we_lve_s23_mid[glvb / 12]), .O(we_lve_s23_leaf[glvb]));
		always @(posedge clk) begin
			if (we_lve_leaf[glvb])
				r_lve[16 * glvb +: 16] <= lve_comb[16 * glvb +: 16];
			if (we_lve_s23_leaf[glvb])
				r_lve_s23[16 * glvb +: 16] <= lve_comb[16 * glvb +: 16];
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire we_lve_root = (st_fsm_6 == S_WALK);
	wire [2:0] we_lve_mid, we_lve_s23_mid;
	wire [35:0] we_lve_leaf, we_lve_s23_leaf;
	(* keep = "true", dont_touch = "true" *) reg [12*48-1:0] r_lve_s23;
	genvar glvb;
	generate
	for (glvb = 0; glvb < 3; glvb = glvb + 1) begin : g_lve_mid
		assign we_lve_mid[glvb] = we_lve_root;
		assign we_lve_s23_mid[glvb] = we_lve_root;
	end
	for (glvb = 0; glvb < 36; glvb = glvb + 1) begin : g_lve_lf
		assign we_lve_leaf[glvb] = we_lve_mid[glvb / 12];
		assign we_lve_s23_leaf[glvb] = we_lve_s23_mid[glvb / 12];
		always @(posedge clk) begin
			if (we_lve_leaf[glvb])
				r_lve[16 * glvb +: 16] <= lve_comb[16 * glvb +: 16];
			if (we_lve_s23_leaf[glvb])
				r_lve_s23[16 * glvb +: 16] <= lve_comb[16 * glvb +: 16];
		end
	end
	endgenerate
`else
	integer ilv;
	always @(posedge clk) begin
		if (st == S_WALK) begin
			for (ilv = 0; ilv < 12; ilv = ilv + 1)
				r_lve[48 * ilv +: 24] <= lve_lo_w[24 * ilv +: 24];
			r_lve_c24 <= lve_c24_w;
		end else if (st == S_SAMP) begin
			for (ilv = 0; ilv < 12; ilv = ilv + 1)
				r_lve[48 * ilv + 24 +: 24] <= lve_hi_w[24 * ilv +: 24];
		end
	end
`endif

	/* Parallel sample evaluation: 3 edges x 4 lanes x MSAA_SAMPLES (2-cycle 24+24 under !SYNTH_KINTEX7) */
	wire [47:0] esamp_sign;
	wire [47:0] esamp_c24_w;
	reg  [47:0] r_esamp_c24;
	wire [47:0] esamp_carry_unused;
	wire [48*47-1:0] esamp_unused;
	genvar gk, gs;
	generate
	if (ENABLE_MSAA != 0) begin : g_msaa_eval
		for (ge = 0; ge < 3; ge = ge + 1) begin : g_samp_e
			for (gk = 0; gk < 4; gk = gk + 1) begin : g_samp_k
				(* keep = "true" *) wire signed [47:0] ope_k = cov_op[48 * ge +: 48];
				(* keep = "true" *) wire signed [47:0] oqe_k = cov_oq[48 * ge +: 48];
				wire signed [47:0] lve_val = r_lve[48 * (4 * ge + gk) +: 48];
`ifdef SYNTH_KINTEX7
				wire signed [47:0] lve_val_hi = r_lve_s23[48 * (4 * ge + gk) +: 48];
`elsif SYNTH_KINTEX_ULTRASCALE
				wire signed [47:0] lve_val_hi = r_lve_s23[48 * (4 * ge + gk) +: 48];
`else
				wire signed [47:0] lve_val_hi = lve_val;
`endif
				for (gs = 0; gs < 4; gs = gs + 1) begin : g_samp_s
					localparam integer idx = 16 * ge + 4 * gk + gs;
					if (gs < MSAA_SAMPLES) begin : g_active_sample
						wire signed [47:0] s_off = (gs == 0 || gs == 3) ? ope_k : oqe_k;
`ifdef MUT_SAMPLE_SWAP
						wire s_sub = (gs == 0 || gs == 1);
`else
						wire s_sub = (gs == 0 || gs == 2);
`endif
`ifdef SYNTH_KINTEX7
						wire [47:0] s_val;
						am_addsub #(.W(48)) u_esamp (
							.a((gs >= 2) ? lve_val_hi : lve_val),
							.b(s_off),
							.sub(s_sub),
							.s(s_val),
							.co(esamp_carry_unused[idx])
						);
						assign esamp_sign[idx] = s_val[47];
						assign esamp_c24_w[idx] = 1'b0;
						assign esamp_unused[47 * idx +: 47] = s_val[46:0];
`elsif SYNTH_KINTEX_ULTRASCALE
						wire [47:0] s_val;
						am_addsub #(.W(48)) u_esamp (
							.a((gs >= 2) ? lve_val_hi : lve_val),
							.b(s_off),
							.sub(s_sub),
							.s(s_val),
							.co(esamp_carry_unused[idx])
						);
						assign esamp_sign[idx] = s_val[47];
						assign esamp_c24_w[idx] = 1'b0;
						assign esamp_unused[47 * idx +: 47] = s_val[46:0];
`else
						/* 26-bit CLA carry-out S_EVAL0 (3->9->26 tree, <= 7 cmos3) + 21-bit CLA carry-select S_EVAL1 */
						wire [47:0] s_off_xor = s_sub ? ~s_off : s_off;
						wire [25:0] es_g = lve_val[25:0] & s_off_xor[25:0];
						wire [25:0] es_p = lve_val[25:0] | s_off_xor[25:0];
						wire [7:0] es_g3, es_p3;
						genvar ges;
						for (ges = 0; ges < 8; ges = ges + 1) begin : g_es3
							assign es_p3[ges] = &es_p[3*ges +: 3];
							assign es_g3[ges] = es_g[3*ges+2] | (es_p[3*ges+2] & es_g[3*ges+1]) |
							                    (es_p[3*ges+2] & es_p[3*ges+1] & es_g[3*ges]);
						end
						wire es_g2t = es_g[25] | (es_p[25] & es_g[24]);
						wire es_p2t = es_p[25] & es_p[24];
						wire es_g9_0 = es_g3[2] | (es_p3[2] & es_g3[1]) | (es_p3[2] & es_p3[1] & es_g3[0]);
						wire es_p9_0 = &es_p3[2:0];
						wire es_g9_1 = es_g3[5] | (es_p3[5] & es_g3[4]) | (es_p3[5] & es_p3[4] & es_g3[3]);
						wire es_p9_1 = &es_p3[5:3];
						wire es_g8_2 = es_g2t   | (es_p2t   & es_g3[7]) | (es_p2t   & es_p3[7] & es_g3[6]);
						wire es_p8_2 = es_p2t   & es_p3[7] & es_p3[6];
						assign esamp_c24_w[idx] = es_g8_2 | (es_p8_2 & es_g9_1) | (es_p8_2 & es_p9_1 & es_g9_0) |
						                          (es_p8_2 & es_p9_1 & es_p9_0 & s_sub);
						wire [25:0] s_lo_u = {25'd0, es_g[0] & es_p[0]};

						wire [20:0] hs_g = lve_val[46:26] & s_off_xor[46:26];
						wire [20:0] hs_p = lve_val[46:26] | s_off_xor[46:26];
						wire [6:0] hs_g3, hs_p3;
						genvar ghs;
						for (ghs = 0; ghs < 7; ghs = ghs + 1) begin : g_hsg3
							assign hs_p3[ghs] = &hs_p[3*ghs +: 3];
							assign hs_g3[ghs] = hs_g[3*ghs+2] | (hs_p[3*ghs+2] & hs_g[3*ghs+1]) |
							                    (hs_p[3*ghs+2] & hs_p[3*ghs+1] & hs_g[3*ghs]);
						end
						wire hs_g6_0  = hs_g3[1] | (hs_p3[1] & hs_g3[0]);
						wire hs_p6_0  = hs_p3[1] & hs_p3[0];
						wire hs_g6_1  = hs_g3[3] | (hs_p3[3] & hs_g3[2]);
						wire hs_p6_1  = hs_p3[3] & hs_p3[2];
						wire hs_c38   = hs_g6_1 | (hs_p6_1 & hs_g6_0) | (hs_p6_1 & hs_p6_0 & r_esamp_c24[idx]);

						wire hs_c47_0 = hs_g3[6] | (hs_p3[6] & hs_g3[5]) | (hs_p3[6] & hs_p3[5] & hs_g3[4]);
						wire hs_c47_1 = hs_c47_0 | (&hs_p3[6:4]);
						wire hs_s47   = lve_val[47] ^ s_off_xor[47];
						wire hs_e0    = hs_s47 ^ hs_c47_0;
						wire hs_e1    = hs_s47 ^ hs_c47_1;
						assign esamp_sign[idx] = hs_c38 ? hs_e1 : hs_e0;
						assign esamp_carry_unused[idx] = hs_g[0];
						assign esamp_unused[47 * idx +: 47] = {21'd0, s_lo_u};
`endif
					end else begin : g_inactive_sample
						assign esamp_sign[idx] = 1'b0;
						assign esamp_c24_w[idx] = 1'b0;
						assign esamp_carry_unused[idx] = 1'b0;
						assign esamp_unused[47 * idx +: 47] = 47'd0;
					end
				end
			end
		end
	end else begin : g_no_msaa_eval
		assign esamp_sign = 48'd0;
		assign esamp_c24_w = 48'd0;
		assign esamp_carry_unused = 48'd0;
		assign esamp_unused = { (48*47){1'b0} };
	end
	endgenerate

	/* Registered sample signs and edge signs to break carry chain before coverage logic */
`ifdef SYNTH_KINTEX7
	(* extract_reset = "no" *) reg [47:0] r_esamp_sign;
	(* extract_reset = "no" *) reg [11:0] r_lve_sign;
	(* extract_reset = "no" *) reg [3:0] r_inb;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* extract_reset = "no" *) reg [47:0] r_esamp_sign;
	(* extract_reset = "no" *) reg [11:0] r_lve_sign;
	(* extract_reset = "no" *) reg [3:0] r_inb;
`else
	(* dont_touch = "true", extract_reset = "no" *) reg [47:0] r_esamp_sign;
	(* dont_touch = "true", extract_reset = "no" *) reg [11:0] r_lve_sign;
	(* dont_touch = "true", extract_reset = "no" *) reg [3:0] r_inb;
	(* extract_reset = "no" *) reg [7:0] r_q_lt_b;
`endif

	/* Quad lane in-bounds logic: evaluated at S_WALK via pure-LUT 17-bit comparators, registered to r_inb */
	wire qx0_lt_bx0, qx0_lt_bx1, qx1_lt_bx0, qx1_lt_bx1;
	wire qy0_lt_by0, qy0_lt_by1, qy1_lt_by0, qy1_lt_by1;
`ifdef SYNTH_KINTEX7
	function u16_gt;
		input [15:0] a, b;
		begin
			u16_gt = (a > b);
		end
	endfunction
	wire [135:0] inb_d_unused;
	am_sub #(.W(17)) u_inb_x0l (.a({1'b0, qx}),  .b({1'b0, bx0}), .bi(1'b0), .d(inb_d_unused[0   +: 17]), .bo(qx0_lt_bx0));
	am_sub #(.W(17)) u_inb_x0h (.a({1'b0, qx}),  .b({1'b0, bx1}), .bi(1'b0), .d(inb_d_unused[17  +: 17]), .bo(qx0_lt_bx1));
	am_sub #(.W(17)) u_inb_x1l (.a(qx1),          .b({1'b0, bx0}), .bi(1'b0), .d(inb_d_unused[34  +: 17]), .bo(qx1_lt_bx0));
	am_sub #(.W(17)) u_inb_x1h (.a(qx1),          .b({1'b0, bx1}), .bi(1'b0), .d(inb_d_unused[51  +: 17]), .bo(qx1_lt_bx1));
	am_sub #(.W(17)) u_inb_y0l (.a({1'b0, qy}),  .b({1'b0, by0}), .bi(1'b0), .d(inb_d_unused[68  +: 17]), .bo(qy0_lt_by0));
	am_sub #(.W(17)) u_inb_y0h (.a({1'b0, qy}),  .b({1'b0, by1}), .bi(1'b0), .d(inb_d_unused[85  +: 17]), .bo(qy0_lt_by1));
	am_sub #(.W(17)) u_inb_y1l (.a(qy1),          .b({1'b0, by0}), .bi(1'b0), .d(inb_d_unused[102 +: 17]), .bo(qy1_lt_by0));
	am_sub #(.W(17)) u_inb_y1h (.a(qy1),          .b({1'b0, by1}), .bi(1'b0), .d(inb_d_unused[119 +: 17]), .bo(qy1_lt_by1));
`elsif SYNTH_KINTEX_ULTRASCALE
	function u16_gt;
		input [15:0] a, b;
		begin
			u16_gt = (a > b);
		end
	endfunction
	wire [135:0] inb_d_unused;
	am_sub #(.W(17)) u_inb_x0l (.a({1'b0, qx}),  .b({1'b0, bx0}), .bi(1'b0), .d(inb_d_unused[0   +: 17]), .bo(qx0_lt_bx0));
	am_sub #(.W(17)) u_inb_x0h (.a({1'b0, qx}),  .b({1'b0, bx1}), .bi(1'b0), .d(inb_d_unused[17  +: 17]), .bo(qx0_lt_bx1));
	am_sub #(.W(17)) u_inb_x1l (.a(qx1),          .b({1'b0, bx0}), .bi(1'b0), .d(inb_d_unused[34  +: 17]), .bo(qx1_lt_bx0));
	am_sub #(.W(17)) u_inb_x1h (.a(qx1),          .b({1'b0, bx1}), .bi(1'b0), .d(inb_d_unused[51  +: 17]), .bo(qx1_lt_bx1));
	am_sub #(.W(17)) u_inb_y0l (.a({1'b0, qy}),  .b({1'b0, by0}), .bi(1'b0), .d(inb_d_unused[68  +: 17]), .bo(qy0_lt_by0));
	am_sub #(.W(17)) u_inb_y0h (.a({1'b0, qy}),  .b({1'b0, by1}), .bi(1'b0), .d(inb_d_unused[85  +: 17]), .bo(qy0_lt_by1));
	am_sub #(.W(17)) u_inb_y1l (.a(qy1),          .b({1'b0, by0}), .bi(1'b0), .d(inb_d_unused[102 +: 17]), .bo(qy1_lt_by0));
	am_sub #(.W(17)) u_inb_y1h (.a(qy1),          .b({1'b0, by1}), .bi(1'b0), .d(inb_d_unused[119 +: 17]), .bo(qy1_lt_by1));
`else
	/* Balanced 2->4->8->16->17 pure-gate unsigned < (<= 8 cmos3) */
	function lt17_u;
		input [16:0] a, b;
		reg [15:0] g, p;
		reg [7:0] g2, p2;
		reg [3:0] g4, p4;
		reg [1:0] g8, p8;
		reg lt16;
		integer k2;
		begin
			g = (~a[15:0]) & b[15:0];
			p = (~a[15:0]) | b[15:0];
			for (k2 = 0; k2 < 8; k2 = k2 + 1) begin
				g2[k2] = g[2*k2+1] | (p[2*k2+1] & g[2*k2]);
				p2[k2] = p[2*k2+1] & p[2*k2];
			end
			for (k2 = 0; k2 < 4; k2 = k2 + 1) begin
				g4[k2] = g2[2*k2+1] | (p2[2*k2+1] & g2[2*k2]);
				p4[k2] = p2[2*k2+1] & p2[2*k2];
			end
			g8[0] = g4[1] | (p4[1] & g4[0]); p8[0] = p4[1] & p4[0];
			g8[1] = g4[3] | (p4[3] & g4[2]); p8[1] = p4[3] & p4[2];
			lt16  = g8[1] | (p8[1] & g8[0]);
			lt17_u = (!a[16] & b[16]) | ((!a[16] | b[16]) & lt16);
		end
	endfunction
	function u16_gt;
		input [15:0] a, b;
		begin
			u16_gt = lt17_u({1'b0, b}, {1'b0, a});
		end
	endfunction
	wire inb_d_unused = 1'b0;
	assign qx0_lt_bx0 = lt17_u({1'b0, qx}, {1'b0, bx0});
	assign qx0_lt_bx1 = lt17_u({1'b0, qx}, {1'b0, bx1});
	assign qx1_lt_bx0 = lt17_u(qx1,        {1'b0, bx0});
	assign qx1_lt_bx1 = lt17_u(qx1,        {1'b0, bx1});
	assign qy0_lt_by0 = lt17_u({1'b0, qy}, {1'b0, by0});
	assign qy0_lt_by1 = lt17_u({1'b0, qy}, {1'b0, by1});
	assign qy1_lt_by0 = lt17_u(qy1,        {1'b0, by0});
	assign qy1_lt_by1 = lt17_u(qy1,        {1'b0, by1});
`endif

`ifdef SYNTH_KINTEX7
	wire x0_ok = !qx0_lt_bx0 && qx0_lt_bx1;
	wire x1_ok = !qx1_lt_bx0 && qx1_lt_bx1;
	wire y0_ok = !qy0_lt_by0 && qy0_lt_by1;
	wire y1_ok = !qy1_lt_by0 && qy1_lt_by1;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire x0_ok = !qx0_lt_bx0 && qx0_lt_bx1;
	wire x1_ok = !qx1_lt_bx0 && qx1_lt_bx1;
	wire y0_ok = !qy0_lt_by0 && qy0_lt_by1;
	wire y1_ok = !qy1_lt_by0 && qy1_lt_by1;
`else
	wire x0_ok = !r_q_lt_b[0] && r_q_lt_b[1];
	wire x1_ok = !r_q_lt_b[2] && r_q_lt_b[3];
	wire y0_ok = !r_q_lt_b[4] && r_q_lt_b[5];
	wire y1_ok = !r_q_lt_b[6] && r_q_lt_b[7];
`endif

	wire [3:0] inb_comb;
	assign inb_comb[0] = x0_ok && y0_ok;
	assign inb_comb[1] = x1_ok && y0_ok;
	assign inb_comb[2] = x0_ok && y1_ok;
	assign inb_comb[3] = x1_ok && y1_ok;

	/* Coverage logic: purely random logic evaluated from flops in S_EVAL */
	reg [3:0] cov_edge;
	reg [3:0] ccov_comb;
	reg [15:0] scov_comb;
	integer e, ln, sm;

	always @(*) begin
		for (ln = 0; ln < 4; ln = ln + 1) begin
			cov_edge[ln] = r_inb[ln];
			ccov_comb[ln] = r_inb[ln];
			for (e = 0; e < 3; e = e + 1) begin
				if (r_lve_sign[4 * e + ln]) begin
					cov_edge[ln] = 1'b0;
					ccov_comb[ln] = 1'b0;
				end
			end
			for (sm = 0; sm < 4; sm = sm + 1) begin
				if (ENABLE_MSAA != 0) begin
					scov_comb[4 * ln + sm] = r_inb[ln] && !kill[sm] && (sm < MSAA_SAMPLES) &&
						!r_esamp_sign[16 * 0 + 4 * ln + sm] &&
						!r_esamp_sign[16 * 1 + 4 * ln + sm] &&
						!r_esamp_sign[16 * 2 + 4 * ln + sm];
				end else begin
					scov_comb[4 * ln + sm] = (sm == 0) ? (r_inb[ln] && !kill[0] && ccov_comb[ln]) : 1'b0;
				end
			end
		end
	end

	/* Trivial accept quad coverage */
	reg [3:0] cov_triv;
	reg [15:0] scov_triv;
	always @(*) begin
		for (ln = 0; ln < 4; ln = ln + 1) begin
			cov_triv[ln] = r_inb[ln];
			for (sm = 0; sm < 4; sm = sm + 1) begin
				if (ENABLE_MSAA != 0)
					scov_triv[4 * ln + sm] = r_inb[ln] && !kill[sm] && (sm < MSAA_SAMPLES);
				else
					scov_triv[4 * ln + sm] = (sm == 0) ? (r_inb[ln] && !kill[0]) : 1'b0;
			end
		end
	end

	/* Combined quad evaluation results */
`ifdef SYNTH_KINTEX7
	wire [3:0] cur_cov = cur_trivial_cov ? cov_triv : cov_edge;
	wire [3:0] cur_ccov = cur_trivial_cov ? cov_triv : ccov_comb;
	wire [15:0] cur_scov = cur_trivial_scov ? scov_triv : scov_comb;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] cur_cov = cur_trivial_cov ? cov_triv : cov_edge;
	wire [3:0] cur_ccov = cur_trivial_cov ? cov_triv : ccov_comb;
	wire [15:0] cur_scov = cur_trivial_scov ? scov_triv : scov_comb;
`else
	wire [3:0] cur_cov = cur_trivial ? cov_triv : cov_edge;
	wire [3:0] cur_ccov = cur_trivial ? cov_triv : ccov_comb;
	wire [15:0] cur_scov = cur_trivial ? scov_triv : scov_comb;
`endif
	wire [3:0] cov1 = cur_cov & ~{4{kill[0]}};

	(* extract_enable = "no", extract_reset = "no" *) reg [15:0] scov_reg;
	(* extract_enable = "no", extract_reset = "no" *) reg [3:0] ccov_reg;
	(* extract_enable = "no", extract_reset = "no" *) reg [3:0] cov_reg;
`ifdef SYNTH_KINTEX7
`ifndef SYNTH_KINTEX7
	(* extract_enable = "no", extract_reset = "no", keep = "true", dont_touch = "true" *) reg [3:0] cov_reg_scov;
`else
	(* extract_enable = "no", extract_reset = "no" *) reg [3:0] cov_reg_scov;
`endif
`elsif SYNTH_KINTEX_ULTRASCALE
	(* extract_enable = "no", extract_reset = "no" *) reg [3:0] cov_reg_scov;
`endif

	/* Pre-computed in S_SAMP for shallow <= 4-LUT S_EVAL (decouples kill from S_EVAL critical path) */
	reg [15:0] r_s_act_pre;
	reg [3:0]  r_lve_inb_kill;
	reg [3:0]  r_lve_inb_nokill;
	reg        r_w_empty_triv;
	wire [3:0] cov1_edge = cov_edge & ~{4{kill[0]}};
	wire [3:0] cov1_triv = cov_triv & ~{4{kill[0]}};
	wire w_empty_triv = wm ? (ms4_misc ? (scov_triv == 16'd0) : (cov1_triv == 4'd0)) : (cov_triv == 4'd0);
`ifdef SYNTH_KINTEX7
	wire [15:0] s_active;
	genvar gsi;
	generate
		for (gsi = 0; gsi < 16; gsi = gsi + 1) begin : g_s_act
			localparam integer qln = gsi / 4;
			localparam integer qsm = gsi % 4;
			if (ENABLE_MSAA != 0) begin : g_msaa_s
				assign s_active[gsi] = ms4_act_q[qln] ?
					(r_s_act_pre[gsi] && !r_esamp_sign[gsi] && !r_esamp_sign[16 + gsi] && !r_esamp_sign[32 + gsi]) :
					((qsm == 0) && r_lve_inb_kill[qln]);
			end else begin : g_noms_s
				assign s_active[gsi] = (qsm == 0) && r_lve_inb_kill[qln];
			end
		end
	endgenerate

	wire [3:0] quad_cov_act;
	genvar gqa;
	generate
		for (gqa = 0; gqa < 4; gqa = gqa + 1) begin : g_q_act
			assign quad_cov_act[gqa] = |s_active[4 * gqa +: 4];
		end
	endgenerate

	wire r_w_empty_triv_b;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rwet (.I0(r_w_empty_triv), .O(r_w_empty_triv_b));
	wire any_cov = cur_trivial_cov ? !r_w_empty_triv_b : (|quad_cov_act);
	wire w_empty = !any_cov;
	(* keep = "true", dont_touch = "true" *) reg r_w_empty_st;
	(* keep = "true", dont_touch = "true" *) reg r_w_empty_zm;
	reg [3:0] r_cur_cov_zm;
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			r_w_empty_st <= 1'b1;
			r_w_empty_zm <= 1'b1;
			r_cur_cov_zm <= 4'd0;
		end else if (st_fsm_m6 == S_EVAL2) begin
			r_w_empty_st <= w_empty;
			r_w_empty_zm <= w_empty;
			r_cur_cov_zm <= cur_cov;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [15:0] s_active;
	genvar gsi;
	generate
		for (gsi = 0; gsi < 16; gsi = gsi + 1) begin : g_s_act
			localparam integer qln = gsi / 4;
			localparam integer qsm = gsi % 4;
			if (ENABLE_MSAA != 0) begin : g_msaa_s
				assign s_active[gsi] = ms4_act_q[qln] ?
					(r_s_act_pre[gsi] && !r_esamp_sign[gsi] && !r_esamp_sign[16 + gsi] && !r_esamp_sign[32 + gsi]) :
					((qsm == 0) && r_lve_inb_kill[qln]);
			end else begin : g_noms_s
				assign s_active[gsi] = (qsm == 0) && r_lve_inb_kill[qln];
			end
		end
	endgenerate

	wire [3:0] quad_cov_act;
	genvar gqa;
	generate
		for (gqa = 0; gqa < 4; gqa = gqa + 1) begin : g_q_act
			assign quad_cov_act[gqa] = |s_active[4 * gqa +: 4];
		end
	endgenerate

	wire r_w_empty_triv_b;
	assign r_w_empty_triv_b = r_w_empty_triv;
	wire any_cov = cur_trivial_cov ? !r_w_empty_triv_b : (|quad_cov_act);
	wire w_empty = !any_cov;
	(* keep = "true", dont_touch = "true" *) reg r_w_empty_st;
	(* keep = "true", dont_touch = "true" *) reg r_w_empty_zm;
	reg [3:0] r_cur_cov_zm;
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			r_w_empty_st <= 1'b1;
			r_w_empty_zm <= 1'b1;
			r_cur_cov_zm <= 4'd0;
		end else if (st_fsm_m6 == S_EVAL2) begin
			r_w_empty_st <= w_empty;
			r_w_empty_zm <= w_empty;
			r_cur_cov_zm <= cur_cov;
		end
	end
`else
	reg [3:0] r_quad_cov_ms4;
	reg       r_w_empty_lve;
	wire [15:0] s_act_asic;
	wire [3:0] quad_cov_asic;
	genvar gsi_a;
	generate
		for (gsi_a = 0; gsi_a < 16; gsi_a = gsi_a + 1) begin : g_s_act_a
			assign s_act_asic[gsi_a] = r_s_act_pre[gsi_a] &&
				!(r_esamp_sign[gsi_a] | r_esamp_sign[16 + gsi_a] | r_esamp_sign[32 + gsi_a]);
		end
		for (gsi_a = 0; gsi_a < 4; gsi_a = gsi_a + 1) begin : g_q_act_a
			assign quad_cov_asic[gsi_a] = |s_act_asic[4 * gsi_a +: 4];
		end
	endgenerate
	wire w_empty_edge_ms4 = (r_quad_cov_ms4 == 4'd0);
	wire w_empty_edge_lve = wm ? (r_lve_inb_kill == 4'd0) : (r_lve_inb_nokill == 4'd0);
	wire w_empty_edge = (wm && ms4_misc) ? w_empty_edge_ms4 : r_w_empty_lve;
	wire w_empty = cur_trivial ? r_w_empty_triv : w_empty_edge;
	wire [3:0] unused_cov1_edge = cov1_edge;
`endif

	/* Stepping within 8x8 coarse tile or bounding box (Kogge-Stone am_sub comparators) */
	wire [15:0] cur_tx_end, cur_ty_end;
	wire [1:0] tend_carry_unused;
	generate
	if (COARSE_SCAN != 0) begin : g_coarse_end
		am_add #(.W(16)) u_tx_end (.a(cur_tx), .b(16'd8), .ci(1'b0), .s(cur_tx_end), .co(tend_carry_unused[0]));
		am_add #(.W(16)) u_ty_end (.a(cur_ty), .b(16'd8), .ci(1'b0), .s(cur_ty_end), .co(tend_carry_unused[1]));
	end else begin : g_direct_end
		assign cur_tx_end = bx1;
		assign cur_ty_end = by1;
		assign tend_carry_unused = 2'd0;
	end
	endgenerate

	(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [16:0] r_qx2, r_qy2;
	(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [15:0] r_cur_tx_end, r_cur_ty_end;
	wire qx2_lt_txe, qx2_lt_bx1, qy2_lt_tye, qy2_lt_by1;
`ifdef SYNTH_KINTEX7
	wire [67:0] tre_d_unused;
	am_sub #(.W(17)) u_tre_tx (.a(r_qx2), .b({1'b0, r_cur_tx_end}), .bi(1'b0), .d(tre_d_unused[0  +: 17]), .bo(qx2_lt_txe));
	am_sub #(.W(17)) u_tre_bx (.a(r_qx2), .b({1'b0, bx1}),          .bi(1'b0), .d(tre_d_unused[17 +: 17]), .bo(qx2_lt_bx1));
	am_sub #(.W(17)) u_tce_ty (.a(r_qy2), .b({1'b0, r_cur_ty_end}), .bi(1'b0), .d(tre_d_unused[34 +: 17]), .bo(qy2_lt_tye));
	am_sub #(.W(17)) u_tce_by (.a(r_qy2), .b({1'b0, by1}),          .bi(1'b0), .d(tre_d_unused[51 +: 17]), .bo(qy2_lt_by1));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [67:0] tre_d_unused;
	am_sub #(.W(17)) u_tre_tx (.a(r_qx2), .b({1'b0, r_cur_tx_end}), .bi(1'b0), .d(tre_d_unused[0  +: 17]), .bo(qx2_lt_txe));
	am_sub #(.W(17)) u_tre_bx (.a(r_qx2), .b({1'b0, bx1}),          .bi(1'b0), .d(tre_d_unused[17 +: 17]), .bo(qx2_lt_bx1));
	am_sub #(.W(17)) u_tce_ty (.a(r_qy2), .b({1'b0, r_cur_ty_end}), .bi(1'b0), .d(tre_d_unused[34 +: 17]), .bo(qy2_lt_tye));
	am_sub #(.W(17)) u_tce_by (.a(r_qy2), .b({1'b0, by1}),          .bi(1'b0), .d(tre_d_unused[51 +: 17]), .bo(qy2_lt_by1));
`else
	wire tre_d_unused = 1'b0;
	assign qx2_lt_txe = lt17_u(r_qx2, {1'b0, r_cur_tx_end});
	assign qx2_lt_bx1 = lt17_u(r_qx2, {1'b0, bx1});
	assign qy2_lt_tye = lt17_u(r_qy2, {1'b0, r_cur_ty_end});
	assign qy2_lt_by1 = lt17_u(r_qy2, {1'b0, by1});
`endif
	wire tile_row_end = !qx2_lt_txe || !qx2_lt_bx1;
	wire tile_col_end = !qy2_lt_tye || !qy2_lt_by1;

	/* Depth: z = (Az i + Bz j) + Cz, i = x - bx0, j = y - by0, clamped to [0, 1].
	 * Stage 0 (zc == 0): latch dx0, dx1, dy0, dy1 (17-bit sub).
	 * Stage 1 (zc == 1): 17-bit LZC -> r_z_lz, r_z_is0 (<= 6 cmos3).
	 * Stage 2 (zc == 2): shift+pack fix2f -> r_fdx*, start fmul.
	 * Stage 3..10: fmul.
	 * Stage 11..19: fadd (two phases: Az*i + Bz*j, then + Cz).
	 * Stage 21..24: write zv, transition to S_EMIT at zc == 24.
	 */
	(* extract_enable = "no", extract_reset = "no" *) reg [4:0] zc;
	(* extract_enable = "no", extract_reset = "no" *) reg [24:0] zc_oh;
	reg [1:0] r_zl;
	(* extract_enable = "no", extract_reset = "no" *) reg [3:0] zmask;
	reg [31:0] t1_0, t1_1, t1_2, t1_3;
	reg [127:0] zv;

	wire [16:0] dx0_comb, dy0_comb, dx1_comb, dy1_comb;
	wire [3:0] dxy_carry_unused;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(17)) u_dx0 (.a({1'b0, qx}), .b({1'b0, bx0}), .bi(1'b0), .d(dx0_comb), .bo(dxy_carry_unused[0]));
	am_sub #(.W(17)) u_dy0 (.a({1'b0, qy}), .b({1'b0, by0}), .bi(1'b0), .d(dy0_comb), .bo(dxy_carry_unused[1]));
	am_sub #(.W(17)) u_dx1 (.a(qx1), .b({1'b0, bx0}), .bi(1'b0), .d(dx1_comb), .bo(dxy_carry_unused[2]));
	am_sub #(.W(17)) u_dy1 (.a(qy1), .b({1'b0, by0}), .bi(1'b0), .d(dy1_comb), .bo(dxy_carry_unused[3]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(17)) u_dx0 (.a({1'b0, qx}), .b({1'b0, bx0}), .bi(1'b0), .d(dx0_comb), .bo(dxy_carry_unused[0]));
	am_sub #(.W(17)) u_dy0 (.a({1'b0, qy}), .b({1'b0, by0}), .bi(1'b0), .d(dy0_comb), .bo(dxy_carry_unused[1]));
	am_sub #(.W(17)) u_dx1 (.a(qx1), .b({1'b0, bx0}), .bi(1'b0), .d(dx1_comb), .bo(dxy_carry_unused[2]));
	am_sub #(.W(17)) u_dy1 (.a(qy1), .b({1'b0, by0}), .bi(1'b0), .d(dy1_comb), .bo(dxy_carry_unused[3]));
`else
	/* Zero-XOR 4x4-bit borrow lookahead with speculative 4-bit sum (<= 8 cmos3 vs 14) */
	function [16:0] scan_sub17;
		input [16:0] a;
		input [15:0] b;
		reg [15:0] gb, pb, px;
		reg [7:0] gn2, pn2;
		reg [3:0] g4, p4, c4;
		reg gn8_0, pn8_0, g8_1, p8_1;
		integer m, kb;
		reg [3:0] g4b, p4b, x4b, s0, s1;
		reg c0_1, c0_2, c0_3, c1_1, c1_2, c1_3;
		begin
			gb = (~a[15:0]) & b;
			pb = (~a[15:0]) | b;
			px = a[15:0] ^ b;
			for (kb = 0; kb < 8; kb = kb + 1) begin
				gn2[kb] = !(gb[2*kb+1] | (pb[2*kb+1] & gb[2*kb]));
				pn2[kb] = !(pb[2*kb+1] & pb[2*kb]);
			end
			for (kb = 0; kb < 4; kb = kb + 1) begin
				g4[kb] = !(gn2[2*kb+1] & (pn2[2*kb+1] | gn2[2*kb]));
				p4[kb] = !(pn2[2*kb+1] | pn2[2*kb]);
			end
			c4[0] = 1'b0;
			c4[1] = g4[0];
			gn8_0 = !(g4[1] | (p4[1] & g4[0]));
			pn8_0 = !(p4[1] & p4[0]);
			c4[2] = !gn8_0;
			c4[3] = g4[2] | (p4[2] & g4[1]) | (p4[2] & p4[1] & g4[0]);
			g8_1  = g4[3] | (p4[3] & g4[2]);
			p8_1  = p4[3] & p4[2];
			for (m = 0; m < 4; m = m + 1) begin
				g4b = gb[4*m +: 4];
				p4b = pb[4*m +: 4];
				x4b = px[4*m +: 4];
				c0_1 = g4b[0];
				c0_2 = g4b[1] | (p4b[1] & g4b[0]);
				c0_3 = g4b[2] | (p4b[2] & c0_2);
				s0 = x4b ^ {c0_3, c0_2, c0_1, 1'b0};
				c1_1 = p4b[0];
				c1_2 = g4b[1] | (p4b[1] & p4b[0]);
				c1_3 = g4b[2] | (p4b[2] & c1_2);
				s1 = x4b ^ {c1_3, c1_2, c1_1, 1'b1};
				scan_sub17[4*m +: 4] = c4[m] ? s1 : s0;
			end
			scan_sub17[16] = a[16] ^ (!( !(g8_1 | (p8_1 & !gn8_0)) ) );
		end
	endfunction
	assign dx0_comb = scan_sub17({1'b0, qx}, bx0);
	assign dy0_comb = scan_sub17({1'b0, qy}, by0);
	assign dx1_comb = scan_sub17(qx1,        bx0);
	assign dy1_comb = scan_sub17(qy1,        by0);
	assign dxy_carry_unused = 4'd0;
`endif

	(* extract_reset = "no" *) reg [16:0] r_dx0, r_dy0, r_dx1, r_dy1;

	function [5:0] lzc17;
		input [16:0] v;
		reg [2:0] gnz;
		reg [4:0] lz2, lz1, lz0;
		begin
			gnz[2] = |v[16:12];
			gnz[1] = |v[11:6];
			gnz[0] = |v[5:0];
			casez (v[16:12])
			5'b1????: lz2 = 5'd0;
			5'b01???: lz2 = 5'd1;
			5'b001??: lz2 = 5'd2;
			5'b0001?: lz2 = 5'd3;
			default:  lz2 = 5'd4;
			endcase
			casez (v[11:6])
			6'b1?????: lz1 = 5'd5;
			6'b01????: lz1 = 5'd6;
			6'b001???: lz1 = 5'd7;
			6'b0001??: lz1 = 5'd8;
			6'b00001?: lz1 = 5'd9;
			default:   lz1 = 5'd10;
			endcase
			casez (v[5:0])
			6'b1?????: lz0 = 5'd11;
			6'b01????: lz0 = 5'd12;
			6'b001???: lz0 = 5'd13;
			6'b0001??: lz0 = 5'd14;
			6'b00001?: lz0 = 5'd15;
			default:   lz0 = 5'd16;
			endcase
			lzc17 = {!(|gnz), gnz[2] ? lz2 : gnz[1] ? lz1 : lz0};
		end
	endfunction

	(* keep = "true", dont_touch = "true" *) reg [4:0] r_z_lz0_a, r_z_lz0_b, r_z_lz0_exp;
	(* keep = "true", dont_touch = "true" *) reg [4:0] r_z_lz1, r_z_lz2, r_z_lz3;
	(* keep = "true", dont_touch = "true" *) reg [4:0] r_z_lz1_b, r_z_lz2_b, r_z_lz3_b;
	(* keep = "true", dont_touch = "true" *) reg [4:0] r_z_lz1_exp, r_z_lz2_exp, r_z_lz3_exp;
	reg       r_z_is0_0, r_z_is0_0_b_unused, r_z_is0_0_e_unused, r_z_is0_1, r_z_is0_2, r_z_is0_3;
	wire [5:0] lzc_dx0 = lzc17(r_dx0);
	wire [5:0] lzc_dy0 = lzc17(r_dy0);
	wire [5:0] lzc_dx1 = lzc17(r_dx1);
	wire [5:0] lzc_dy1 = lzc17(r_dy1);

	/* 2-stage coarse+fine LUT left shift by 5 bits (<= 5 cmos3 vs 11) */
	function [16:0] lsh17_lut;
		input [16:0] a;
		input [4:0] amt;
		reg [16:0] s_c;
		integer si;
		begin
			for (si = 0; si < 17; si = si + 1) begin
				case (amt[4:2])
				3'd0:    s_c[si] = a[si];
				3'd1:    s_c[si] = (si >= 4)  ? a[si - 4]  : 1'b0;
				3'd2:    s_c[si] = (si >= 8)  ? a[si - 8]  : 1'b0;
				3'd3:    s_c[si] = (si >= 12) ? a[si - 12] : 1'b0;
				3'd4:    s_c[si] = (si >= 16) ? a[si - 16] : 1'b0;
				default: s_c[si] = 1'b0;
				endcase
			end
			for (si = 0; si < 17; si = si + 1) begin
				case (amt[1:0])
				2'd0: lsh17_lut[si] = s_c[si];
				2'd1: lsh17_lut[si] = (si >= 1) ? s_c[si - 1] : 1'b0;
				2'd2: lsh17_lut[si] = (si >= 2) ? s_c[si - 2] : 1'b0;
				2'd3: lsh17_lut[si] = (si >= 3) ? s_c[si - 3] : 1'b0;
				endcase
			end
		end
	endfunction
`ifdef SYNTH_KINTEX7
	wire [4:0] zlz0_a_g0, zlz0_a_g1, zlz0_a_g2, zlz0_a_g3;
	genvar gzl0;
	generate
	for (gzl0 = 0; gzl0 < 5; gzl0 = gzl0 + 1) begin : g_zlz0_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zl0 (.I0(r_z_lz0_a[gzl0]), .O(zlz0_a_g0[gzl0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zl1 (.I0(r_z_lz0_a[gzl0]), .O(zlz0_a_g1[gzl0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zl2 (.I0(r_z_lz0_a[gzl0]), .O(zlz0_a_g2[gzl0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zl3 (.I0(r_z_lz0_a[gzl0]), .O(zlz0_a_g3[gzl0]));
	end
	endgenerate
	wire [16:0] sh_dx0_lo_0 = lsh17_lut(r_dx0, zlz0_a_g0);
	wire [16:0] sh_dx0_lo_1 = lsh17_lut(r_dx0, zlz0_a_g1);
	wire [16:0] sh_dx0_lo_2 = lsh17_lut(r_dx0, zlz0_a_g2);
	wire [16:0] sh_dx0_lo_3 = lsh17_lut(r_dx0, zlz0_a_g3);
	wire [16:0] sh_dx0_lo = {9'd0, sh_dx0_lo_3[7:6], sh_dx0_lo_2[5:4], sh_dx0_lo_1[3:2], sh_dx0_lo_0[1:0]};
	wire sh_dx0_parts_unused = &{1'b0, sh_dx0_lo_0[16:2], sh_dx0_lo_1[16:4], sh_dx0_lo_1[1:0],
		sh_dx0_lo_2[16:6], sh_dx0_lo_2[3:0], sh_dx0_lo_3[16:8], sh_dx0_lo_3[5:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [4:0] zlz0_a_g0, zlz0_a_g1, zlz0_a_g2, zlz0_a_g3;
	genvar gzl0;
	generate
	for (gzl0 = 0; gzl0 < 5; gzl0 = gzl0 + 1) begin : g_zlz0_buf
		assign zlz0_a_g0[gzl0] = r_z_lz0_a[gzl0];
		assign zlz0_a_g1[gzl0] = r_z_lz0_a[gzl0];
		assign zlz0_a_g2[gzl0] = r_z_lz0_a[gzl0];
		assign zlz0_a_g3[gzl0] = r_z_lz0_a[gzl0];
	end
	endgenerate
	wire [16:0] sh_dx0_lo_0 = lsh17_lut(r_dx0, zlz0_a_g0);
	wire [16:0] sh_dx0_lo_1 = lsh17_lut(r_dx0, zlz0_a_g1);
	wire [16:0] sh_dx0_lo_2 = lsh17_lut(r_dx0, zlz0_a_g2);
	wire [16:0] sh_dx0_lo_3 = lsh17_lut(r_dx0, zlz0_a_g3);
	wire [16:0] sh_dx0_lo = {9'd0, sh_dx0_lo_3[7:6], sh_dx0_lo_2[5:4], sh_dx0_lo_1[3:2], sh_dx0_lo_0[1:0]};
	wire sh_dx0_parts_unused = &{1'b0, sh_dx0_lo_0[16:2], sh_dx0_lo_1[16:4], sh_dx0_lo_1[1:0],
		sh_dx0_lo_2[16:6], sh_dx0_lo_2[3:0], sh_dx0_lo_3[16:8], sh_dx0_lo_3[5:0]};
`else
	wire [16:0] sh_dx0_lo = lsh17_lut(r_dx0, r_z_lz0_a);
	wire sh_dx0_parts_unused = 1'b0;
`endif
	wire [16:0] sh_dx0_hi = lsh17_lut(r_dx0, r_z_lz0_b);
	wire [16:0] sh_dy0_lo = lsh17_lut(r_dy0, r_z_lz1);
	wire [16:0] sh_dy0_hi = lsh17_lut(r_dy0, r_z_lz1_b);
	wire [16:0] sh_dx1_lo = lsh17_lut(r_dx1, r_z_lz2);
	wire [16:0] sh_dx1_hi = lsh17_lut(r_dx1, r_z_lz2_b);
	wire [16:0] sh_dy1_lo = lsh17_lut(r_dy1, r_z_lz3);
	wire [16:0] sh_dy1_hi = lsh17_lut(r_dy1, r_z_lz3_b);
	wire [15:0] sh_dx0 = {sh_dx0_hi[15:8], sh_dx0_lo[7:0]};
	wire [15:0] sh_dy0 = {sh_dy0_hi[15:8], sh_dy0_lo[7:0]};
	wire [15:0] sh_dx1 = {sh_dx1_hi[15:8], sh_dx1_lo[7:0]};
	wire [15:0] sh_dy1 = {sh_dy1_hi[15:8], sh_dy1_lo[7:0]};
	wire unused_sh_z16 = &{1'b0, sh_dx0_parts_unused, sh_dx0_lo[16:8], sh_dx0_hi[16], sh_dx0_hi[7:0],
		sh_dy0_lo[16:8], sh_dy0_hi[16], sh_dy0_hi[7:0],
		sh_dx1_lo[16:8], sh_dx1_hi[16], sh_dx1_hi[7:0],
		sh_dy1_lo[16:8], sh_dy1_hi[16], sh_dy1_hi[7:0]};

	/* Pure-LUT 8'd143 - lz (lz in 0..17 -> 8'h8F - lz, <= 2 cmos3) */
	wire [7:0] exp_dx0 = r_z_lz0_exp[4] ? (r_z_lz0_exp[0] ? 8'h7E : 8'h7F) : {4'h8, ~r_z_lz0_exp[3:0]};
	wire [7:0] exp_dy0 = r_z_lz1_exp[4] ? (r_z_lz1_exp[0] ? 8'h7E : 8'h7F) : {4'h8, ~r_z_lz1_exp[3:0]};
	wire [7:0] exp_dx1 = r_z_lz2_exp[4] ? (r_z_lz2_exp[0] ? 8'h7E : 8'h7F) : {4'h8, ~r_z_lz2_exp[3:0]};
	wire [7:0] exp_dy1 = r_z_lz3_exp[4] ? (r_z_lz3_exp[0] ? 8'h7E : 8'h7F) : {4'h8, ~r_z_lz3_exp[3:0]};
	wire [3:0] exp_z_bo_u = 4'd0;

	wire [31:0] f_dx0_comb = r_z_is0_0 ? 32'd0 : {1'b0, exp_dx0, sh_dx0[15:0], 7'd0};
	wire [31:0] f_dy0_comb = r_z_is0_1 ? 32'd0 : {1'b0, exp_dy0, sh_dy0[15:0], 7'd0};
	wire [31:0] f_dx1_comb = r_z_is0_2 ? 32'd0 : {1'b0, exp_dx1, sh_dx1[15:0], 7'd0};
	wire [31:0] f_dy1_comb = r_z_is0_3 ? 32'd0 : {1'b0, exp_dy1, sh_dy1[15:0], 7'd0};

	(* extract_reset = "no" *) reg [31:0] r_fdx0, r_fdy0, r_fdx1, r_fdy1;

	wire [1:0] r_zl_next;
	wire r_zl_co_unused;
	am_add #(.W(2)) u_zl_inc (.a(r_zl), .b(2'd0), .ci(1'b1), .s(r_zl_next), .co(r_zl_co_unused));

`ifdef SYNTH_KINTEX7
	reg r_is_bz_a, r_is_bz_b;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg r_is_bz_a, r_is_bz_b;
`else
	(* dont_touch = "true" *) reg r_is_bz_a, r_is_bz_b;
`endif
	reg r_fm_v;
	wire fm_v = r_fm_v;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) reg r_is_bz_a_l, r_is_bz_a_h;
	reg r_is_bz_b_l, r_is_bz_b_h0, r_is_bz_b_h1;
	reg [1:0] r_zl_l, r_zl_h;

`define SET_BZA(val) begin r_is_bz_a <= (val); r_is_bz_a_l <= (val); r_is_bz_a_h <= (val); end
`define SET_BZB(val) begin r_is_bz_b <= (val); r_is_bz_b_l <= (val); r_is_bz_b_h0 <= (val); r_is_bz_b_h1 <= (val); end
`define SET_ZL(val)  begin r_zl <= (val); r_zl_l <= (val); r_zl_h <= (val); end

	wire [15:0] fm_a_lo = r_is_bz_a_l ? bz[15:0] : az[15:0];
	wire [15:0] fm_a_hi = r_is_bz_a_h ? bz[31:16] : az[31:16];
	wire [15:0] fm_b_lo = r_is_bz_b_l ? (r_zl_l[1] ? r_fdy1[15:0] : r_fdy0[15:0]) : (r_zl_l[0] ? r_fdx1[15:0] : r_fdx0[15:0]);
	wire [7:0] fm_b_hi_0 = r_is_bz_b_h0 ? (r_zl_h[1] ? r_fdy1[23:16] : r_fdy0[23:16]) : (r_zl_h[0] ? r_fdx1[23:16] : r_fdx0[23:16]);
	wire [7:0] fm_b_hi_1 = r_is_bz_b_h1 ? (r_zl_h[1] ? r_fdy1[31:24] : r_fdy0[31:24]) : (r_zl_h[0] ? r_fdx1[31:24] : r_fdx0[31:24]);
	wire [15:0] fm_b_hi = {fm_b_hi_1, fm_b_hi_0};
	wire [31:0] fm_a = {fm_a_hi, fm_a_lo};
	wire [31:0] fm_b = {fm_b_hi, fm_b_lo};
`else
`define SET_BZA(val) begin r_is_bz_a <= (val); end
`define SET_BZB(val) begin r_is_bz_b <= (val); end
`define SET_ZL(val)  begin r_zl <= (val); end

`ifdef SYNTH_KINTEX7
`else
	reg [31:0] r_fm_a_q, r_fm_b_q;
	wire [31:0] fm_a = r_fm_a_q;
	wire [31:0] fm_b = r_fm_b_q;
`endif
`endif

	reg r_fa_v;
	reg [15:0] r_fa_a_lo, r_fa_a_hi;
	reg [15:0] r_fa_b_lo, r_fa_b_hi;
	wire [31:0] r_fa_a = {r_fa_a_hi, r_fa_a_lo};
	wire [31:0] r_fa_b = {r_fa_b_hi, r_fa_b_lo};
	wire fm_o, fa_o;
	wire [31:0] fm_r, fa_r;

	atg_rx_fmul #(.LAT(4)) fmul (.clk(clk), .rst_n(rst_n), .i_valid(fm_v), .i_a(fm_a), .i_b(fm_b),
		.o_valid(fm_o), .o_r(fm_r));
	atg_rx_fadd #(.LAT(4)) fadd (.clk(clk), .rst_n(rst_n), .i_valid(r_fa_v), .i_a(r_fa_a), .i_b(r_fa_b),
		.i_sub(1'b0), .o_valid(fa_o), .o_r(fa_r));
`ifdef SYNTH_KINTEX7
`ifdef SYNTH_KINTEX7
	wire is_nan_or_neg = fa_r[31] || (fa_r[30:23] == 8'hFF && fa_r[22:0] != 23'd0);
	wire is_sat = fa_r > 32'h3F800000;
	wire is_nan_neg_lo, is_nan_neg_hi, is_sat_lo, is_sat_hi;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_innl (.I0(is_nan_or_neg), .O(is_nan_neg_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_innh (.I0(is_nan_or_neg), .O(is_nan_neg_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_islo (.I0(is_sat),        .O(is_sat_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ishi (.I0(is_sat),        .O(is_sat_hi));
	wire [15:0] zcl_lo = is_nan_neg_lo ? 16'd0 : (is_sat_lo ? 16'h0000 : fa_r[15:0]);
	wire [15:0] zcl_hi = is_nan_neg_hi ? 16'd0 : (is_sat_hi ? 16'h3F80 : fa_r[31:16]);
`else
	wire is_nan_or_neg = fa_r[31] || (fa_r[30:23] == 8'hFF && fa_r[22:0] != 23'd0);
	wire is_sat = fa_r > 32'h3F800000;
	wire [15:0] zcl_lo = is_nan_or_neg ? 16'd0 : (is_sat ? 16'h0000 : fa_r[15:0]);
	wire [15:0] zcl_hi = is_nan_or_neg ? 16'd0 : (is_sat ? 16'h3F80 : fa_r[31:16]);
`endif
	wire [31:0] zcl = {zcl_hi, zcl_lo};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire is_nan_or_neg = fa_r[31] || (fa_r[30:23] == 8'hFF && fa_r[22:0] != 23'd0);
	wire is_sat = fa_r > 32'h3F800000;
	wire is_nan_neg_lo, is_nan_neg_hi, is_sat_lo, is_sat_hi;
	assign is_nan_neg_lo = is_nan_or_neg;
	assign is_nan_neg_hi = is_nan_or_neg;
	assign is_sat_lo = is_sat;
	assign is_sat_hi = is_sat;
	wire [15:0] zcl_lo = is_nan_neg_lo ? 16'd0 : (is_sat_lo ? 16'h0000 : fa_r[15:0]);
	wire [15:0] zcl_hi = is_nan_neg_hi ? 16'd0 : (is_sat_hi ? 16'h3F80 : fa_r[31:16]);
	wire [31:0] zcl = {zcl_hi, zcl_lo};
`else
	wire [31:0] zcl = fa_r[31] || (fa_r[30:23] == 8'hFF && fa_r[22:0] != 23'd0) ? 32'd0 :
		fa_r > 32'h3F800000 ? 32'h3F800000 : fa_r;
`endif

	wire [1:0] fi1;
	wire [4:0] zc1;
	wire [1:0] ct_carry_unused;
	am_add #(.W(2)) u_fi1 (.a(fi), .b(2'd0), .ci(1'b1), .s(fi1), .co(ct_carry_unused[0]));
	am_add #(.W(5)) u_zc1 (.a(zc), .b(5'd0), .ci(1'b1), .s(zc1), .co(ct_carry_unused[1]));

	/* The quad that waits for its successor */
	reg pend;
	reg [31:0] p_xy;
	reg [3:0] p_mask;
	reg [127:0] p_z;
	reg [15:0] seq;

`ifdef SYNTH_KINTEX7
	wire [7:0] clr_zv_leaf;
	wire [7:0] we_zv_leaf;
	wire [7:0] we_pz_leaf;
	wire [7:0] we_dxd_leaf;
	wire [1:0] we_dxy_leaf;
	wire [1:0] we_pxy_leaf;

	wire dx_stall = !(dx_valid && dx_ready);
	wire [2:0] dx_stall_b = {3{dx_stall}};

	wire we_pz_raw = (st_fsm_3 == S_EMIT) && (!pend || !dx_stall_b[0]);
	wire we_dxd_raw = dx_stall_b[1] && (((st_fsm_3 == S_EMIT) && pend) || (st_fsm_3 == S_LAST));
	wire we_dxy_raw = dx_stall_b[2] && (((st_fsm_3 == S_EMIT) && pend) || (st_fsm_3 == S_LAST));
	wire we_pxy_raw = we_pz_raw;

	wire sel_last_raw = (st_fsm_3 == S_LAST) && !pend;
	wire [7:0] sel_last_d_leaf;
	wire [1:0] sel_last_xy_leaf;

`ifdef SYNTH_KINTEX7
	wire clr_zv_raw = (st_fsm_1 == S_Z && zc_3 == 5'd0);
	genvar gzvb;
	generate
	for (gzvb = 0; gzvb < 8; gzvb = gzvb + 1) begin : g_zv_ctrl_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_czv (.I0(clr_zv_raw),   .O(clr_zv_leaf[gzvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wpz (.I0(we_pz_raw),    .O(we_pz_leaf[gzvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdx (.I0(we_dxd_raw),   .O(we_dxd_leaf[gzvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sld (.I0(sel_last_raw), .O(sel_last_d_leaf[gzvb]));
	end
	for (gzvb = 0; gzvb < 2; gzvb = gzvb + 1) begin : g_xy_ctrl_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdy (.I0(we_dxy_raw),   .O(we_dxy_leaf[gzvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wpy (.I0(we_pxy_raw),   .O(we_pxy_leaf[gzvb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sly (.I0(sel_last_raw), .O(sel_last_xy_leaf[gzvb]));
	end
	endgenerate
`else
	assign sel_last_d_leaf  = {8{sel_last_raw}};
	assign sel_last_xy_leaf = {2{sel_last_raw}};
	assign clr_zv_leaf = {8{st_fsm_1 == S_Z && zc_3 == 5'd0}};
	assign we_pz_leaf  = {8{we_pz_raw}};
	assign we_dxd_leaf = {8{we_dxd_raw}};
	assign we_dxy_leaf = {2{we_dxy_raw}};
	assign we_pxy_leaf = {2{we_pxy_raw}};
`endif

	genvar gzs;
	generate
		for (gzs = 0; gzs < 8; gzs = gzs + 1) begin : g_zv_slices
			wire we_zv_raw = (st_fsm_1 == S_Z && zc_3 == (5'd21 + (gzs / 2)) && zmask[gzs / 2]);
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wzv (.I0(we_zv_raw), .O(we_zv_leaf[gzs]));
`else
			assign we_zv_leaf[gzs] = we_zv_raw;
`endif

			always @(posedge clk) begin
				if (clr_zv_leaf[gzs])
					zv[gzs * 16 +: 16] <= 16'd0;
				else if (we_zv_leaf[gzs])
					zv[gzs * 16 +: 16] <= (gzs % 2 == 0) ? zcl_lo : zcl_hi;
			end

			always @(posedge clk) begin
				if (we_pz_leaf[gzs])
					p_z[gzs * 16 +: 16] <= zv[gzs * 16 +: 16];
			end

			always @(posedge clk) begin
				if (we_dxd_leaf[gzs])
					dx_depth[gzs * 16 +: 16] <= sel_last_d_leaf[gzs] ? 16'd0 : p_z[gzs * 16 +: 16];
			end
		end
		for (gzs = 0; gzs < 2; gzs = gzs + 1) begin : g_xy_slices
			always @(posedge clk) begin
				if (we_pxy_leaf[gzs])
					p_xy[gzs * 16 +: 16] <= (gzs == 0) ? qx : qy;
			end

			always @(posedge clk) begin
				if (we_dxy_leaf[gzs])
					dx_xy[gzs * 16 +: 16] <= sel_last_xy_leaf[gzs] ?
						((gzs == 0) ? {bx0[15:1], 1'b0} : {by0[15:1], 1'b0}) :
						p_xy[gzs * 16 +: 16];
			end
		end
	endgenerate

	wire we_seq = (st_fsm_3 == S_SEQ);
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) seq <= 16'd0;
		else if (we_seq) seq <= sq_seq;
	end

`ifdef SYNTH_KINTEX7
	wire we_sp_idle;
	(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'h00010000)) u_wspidle (
		.I0(st_fsm_5[0]), .I1(st_fsm_5[1]), .I2(st_fsm_5[2]), .I3(st_fsm_5[3]),
		.I4(sp_val_fsm), .O(we_sp_idle)
	);
`else
	wire we_sp_idle = (st_fsm_5 == S_IDLE) && sp_val_fsm;
`endif
	wire [3:0] we_sp_branch;
	wire [8:0] we_a8_l, we_b8_l;
	wire [3:0] we_bbox_l;
	wire [5:0] we_z_l;
`ifdef SYNTH_KINTEX7
	genvar gwsp;
	generate
	for (gwsp = 0; gwsp < 4; gwsp = gwsp + 1) begin : g_wsp_br
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wspb (.I0(we_sp_idle), .O(we_sp_branch[gwsp]));
	end
	for (gwsp = 0; gwsp < 9; gwsp = gwsp + 1) begin : g_wsp_ab
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wa8 (.I0(we_sp_branch[gwsp / 3]), .O(we_a8_l[gwsp]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wb8 (.I0(we_sp_branch[gwsp / 3]), .O(we_b8_l[gwsp]));
	end
	for (gwsp = 0; gwsp < 4; gwsp = gwsp + 1) begin : g_wsp_bb
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wbb (.I0(we_sp_branch[3]), .O(we_bbox_l[gwsp]));
	end
	for (gwsp = 0; gwsp < 6; gwsp = gwsp + 1) begin : g_wsp_z
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wz (.I0(we_sp_branch[3]), .O(we_z_l[gwsp]));
	end
	endgenerate
`else
	assign we_sp_branch = {4{we_sp_idle}};
	assign we_a8_l = {3{we_sp_branch[2], we_sp_branch[1], we_sp_branch[0]}};
	assign we_b8_l = we_a8_l;
	assign we_bbox_l = {4{we_sp_branch[0]}};
	assign we_z_l = {6{we_sp_branch[1]}};
`endif

	wire [47:0] in_a8_comb [0:2];
	wire [47:0] in_b8_comb [0:2];
	assign in_a8_comb[0] = {{16{s_e0_a_scan_ext}}, sp_tri[0 +: 24], 8'd0};
	assign in_b8_comb[0] = {{16{s_e0_b_scan_ext}}, sp_tri[24 +: 24], 8'd0};
	assign in_a8_comb[1] = {{16{s_e1_a_scan_ext}}, sp_tri[96 +: 24], 8'd0};
	assign in_b8_comb[1] = {{16{s_e1_b_scan_ext}}, sp_tri[120 +: 24], 8'd0};
	assign in_a8_comb[2] = {{16{s_e2_a_scan_ext}}, sp_tri[192 +: 24], 8'd0};
	assign in_b8_comb[2] = {{16{s_e2_b_scan_ext}}, sp_tri[216 +: 24], 8'd0};

	genvar gsp;
	generate
		for (gsp = 0; gsp < 9; gsp = gsp + 1) begin : g_a8_b8_reg
			localparam integer EK = gsp / 3;
			localparam integer SK = gsp % 3;
			always @(posedge clk) begin
				if (we_a8_l[gsp]) a8[EK][SK*16 +: 16] <= in_a8_comb[EK][SK*16 +: 16];
				if (we_b8_l[gsp]) b8[EK][SK*16 +: 16] <= in_b8_comb[EK][SK*16 +: 16];
			end
		end
	endgenerate

	always @(posedge clk) begin
		if (we_bbox_l[0]) bx0 <= sp_tri[303:288];
		if (we_bbox_l[1]) by0 <= sp_tri[319:304];
		if (we_bbox_l[2]) bx1 <= sp_tri[335:320];
		if (we_bbox_l[3]) by1 <= sp_tri[351:336];
	end

	always @(posedge clk) begin
		if (we_z_l[0]) az[15:0]  <= sp_tri[367:352];
		if (we_z_l[1]) az[31:16] <= sp_tri[383:368];
		if (we_z_l[2]) bz[15:0]  <= sp_tri[399:384];
		if (we_z_l[3]) bz[31:16] <= sp_tri[415:400];
		if (we_z_l[4]) cz[15:0]  <= sp_tri[431:416];
		if (we_z_l[5]) cz[31:16] <= sp_tri[447:432];
	end

	wire we_dx_raw = (st_fsm_1 == S_Z) && (zc_1 == 5'd0);
	wire [7:0] we_dx_leaf;
	wire we_fdx_raw = (st_fsm_1 == S_Z) && (zc_1 == 5'd2);
	wire [7:0] we_fdx_leaf;
`ifdef SYNTH_KINTEX7
	genvar gzdx;
	generate
	for (gzdx = 0; gzdx < 8; gzdx = gzdx + 1) begin : g_zdx_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdxl  (.I0(we_dx_raw),  .O(we_dx_leaf[gzdx]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wfdxl (.I0(we_fdx_raw), .O(we_fdx_leaf[gzdx]));
	end
	endgenerate
`else
	assign we_dx_leaf  = {8{we_dx_raw}};
	assign we_fdx_leaf = {8{we_fdx_raw}};
`endif
	always @(posedge clk) begin
		if (we_dx_leaf[0]) r_dx0[15:0]  <= dx0_comb[15:0];
		if (we_dx_leaf[1]) r_dx0[16]    <= dx0_comb[16];
		if (we_dx_leaf[2]) r_dy0[15:0]  <= dy0_comb[15:0];
		if (we_dx_leaf[3]) r_dy0[16]    <= dy0_comb[16];
		if (we_dx_leaf[4]) r_dx1[15:0]  <= dx1_comb[15:0];
		if (we_dx_leaf[5]) r_dx1[16]    <= dx1_comb[16];
		if (we_dx_leaf[6]) r_dy1[15:0]  <= dy1_comb[15:0];
		if (we_dx_leaf[7]) r_dy1[16]    <= dy1_comb[16];
	end

	wire we_zlz_raw = (st_fsm_1 == S_Z && zc_1 == 5'd1);
	wire [5:0] we_zlz_b;
`ifdef SYNTH_KINTEX7
	genvar gzlw;
	generate
	for (gzlw = 0; gzlw < 6; gzlw = gzlw + 1) begin : g_wzlz_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wzlz (.I0(we_zlz_raw), .O(we_zlz_b[gzlw]));
	end
	endgenerate
`else
	assign we_zlz_b = {6{we_zlz_raw}};
`endif
	always @(posedge clk) begin
		if (we_zlz_b[0]) begin
			{r_z_is0_0, r_z_lz0_a}            <= lzc_dx0;
			{r_z_is0_0_b_unused, r_z_lz0_b}   <= lzc_dx0;
		end
		if (we_zlz_b[1]) begin
			{r_z_is0_0_e_unused, r_z_lz0_exp} <= lzc_dx0;
			{r_z_is0_1, r_z_lz1}              <= lzc_dy0;
		end
		if (we_zlz_b[2]) begin
			r_z_lz1_b   <= lzc_dy0[4:0];
			r_z_lz1_exp <= lzc_dy0[4:0];
		end
		if (we_zlz_b[3]) begin
			{r_z_is0_2, r_z_lz2} <= lzc_dx1;
			r_z_lz2_b            <= lzc_dx1[4:0];
		end
		if (we_zlz_b[4]) begin
			r_z_lz2_exp          <= lzc_dx1[4:0];
			{r_z_is0_3, r_z_lz3} <= lzc_dy1;
		end
		if (we_zlz_b[5]) begin
			r_z_lz3_b   <= lzc_dy1[4:0];
			r_z_lz3_exp <= lzc_dy1[4:0];
		end
		if (we_fdx_leaf[0]) r_fdx0[15:0]  <= f_dx0_comb[15:0];
		if (we_fdx_leaf[1]) r_fdx0[31:16] <= f_dx0_comb[31:16];
		if (we_fdx_leaf[2]) r_fdy0[15:0]  <= f_dy0_comb[15:0];
		if (we_fdx_leaf[3]) r_fdy0[31:16] <= f_dy0_comb[31:16];
		if (we_fdx_leaf[4]) r_fdx1[15:0]  <= f_dx1_comb[15:0];
		if (we_fdx_leaf[5]) r_fdx1[31:16] <= f_dx1_comb[31:16];
		if (we_fdx_leaf[6]) r_fdy1[15:0]  <= f_dy1_comb[15:0];
		if (we_fdx_leaf[7]) r_fdy1[31:16] <= f_dy1_comb[31:16];
	end

	wire [7:0] we_t1_leaf;
	wire we_t1_6 = (st_fsm_1 == S_Z) && (zc_2 == 5'd7);
	wire we_t1_7 = (st_fsm_1 == S_Z) && (zc_2 == 5'd8);
	wire we_t1_8 = (st_fsm_1 == S_Z) && (zc_2 == 5'd9);
	wire we_t1_9 = (st_fsm_1 == S_Z) && (zc_2 == 5'd10);
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt10 (.I0(we_t1_6), .O(we_t1_leaf[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt11 (.I0(we_t1_6), .O(we_t1_leaf[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt12 (.I0(we_t1_7), .O(we_t1_leaf[2]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt13 (.I0(we_t1_7), .O(we_t1_leaf[3]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt14 (.I0(we_t1_8), .O(we_t1_leaf[4]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt15 (.I0(we_t1_8), .O(we_t1_leaf[5]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt16 (.I0(we_t1_9), .O(we_t1_leaf[6]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wt17 (.I0(we_t1_9), .O(we_t1_leaf[7]));
`else
	assign we_t1_leaf[0] = we_t1_6;
	assign we_t1_leaf[1] = we_t1_6;
	assign we_t1_leaf[2] = we_t1_7;
	assign we_t1_leaf[3] = we_t1_7;
	assign we_t1_leaf[4] = we_t1_8;
	assign we_t1_leaf[5] = we_t1_8;
	assign we_t1_leaf[6] = we_t1_9;
	assign we_t1_leaf[7] = we_t1_9;
`endif

	always @(posedge clk) begin
		if (we_t1_leaf[0]) t1_0[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[1]) t1_0[31:16] <= fm_r[31:16];
		if (we_t1_leaf[2]) t1_1[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[3]) t1_1[31:16] <= fm_r[31:16];
		if (we_t1_leaf[4]) t1_2[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[5]) t1_2[31:16] <= fm_r[31:16];
		if (we_t1_leaf[6]) t1_3[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[7]) t1_3[31:16] <= fm_r[31:16];
	end

	wire qte = (r_tile_row_end && r_tile_col_end);
	wire cce = (st_fsm_2 == S_TILE_WAIT && ctv_edge) ||
	           (st_fsm_2 == S_STEP && qte && !cur_last_tile && ctv_edge);
	wire er0_b = (st_fsm_0 == S_SEQ && COARSE_SCAN == 0);
	wire cstep = (st_fsm_0 == S_STEP && !qte);
	wire cstep_r = cstep && r_tile_row_end;
	wire cstep_c = cstep && !r_tile_row_end;

	wire [2:0] cond_coarse_e_b, cond_step_r_b, cond_step_c_b, cond_we_er_b, cond_we_ec_b;
	wire qte_step_c, qte_cce;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_qtes (.I0(qte), .O(qte_step_c));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_qtec (.I0(qte), .O(qte_cce));
`else
	assign qte_step_c = qte;
	assign qte_cce    = qte;
`endif
	wire cce_b   = (st_fsm_2 == S_TILE_WAIT && ctv_edge) ||
	               (st_fsm_2 == S_STEP && qte_cce && !cur_last_tile && ctv_edge);
	wire er0_bb  = (st_fsm_0 == S_SEQ && COARSE_SCAN == 0);
	wire cstep_b = (st_fsm_0 == S_STEP && !qte_step_c);
	wire cstep_r_b = cstep_b && r_tile_row_end;
	wire cstep_c_b = cstep_b && !r_tile_row_end;
`ifdef SYNTH_KINTEX7
	genvar gcb;
	generate
	for (gcb = 0; gcb < 3; gcb = gcb + 1) begin : g_cb_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cbce  (.I0(cce_b),                        .O(cond_coarse_e_b[gcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cbsr  (.I0(cstep_r_b),                    .O(cond_step_r_b[gcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cbsc  (.I0(cstep_c_b),                    .O(cond_step_c_b[gcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cbwer (.I0(er0_bb || cce_b || cstep_r_b), .O(cond_we_er_b[gcb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cbwec (.I0(er0_bb || cce_b || cstep_b),   .O(cond_we_ec_b[gcb]));
	end
	endgenerate
`else
	assign cond_coarse_e_b = {3{cce_b}};
	assign cond_step_r_b   = {3{cstep_r_b}};
	assign cond_step_c_b   = {3{cstep_c_b}};
	assign cond_we_er_b    = {3{er0_bb || cce_b || cstep_r_b}};
	assign cond_we_ec_b    = {3{er0_bb || cce_b || cstep_b}};
`endif

	wire [8:0] sel_step_r_er_l;
	wire [8:0] sel_coarse_er_l;
	wire [8:0] sel_step_c_ec_l;
	wire [8:0] sel_step_r_ec_l;
	wire [8:0] sel_coarse_ec_l;
	wire [8:0] we_er_leaf;
	wire [8:0] we_ec_leaf;
	wire [8:0] we_ab8_leaf;
	genvar gsk;
	generate
		for (gsk = 0; gsk < 9; gsk = gsk + 1) begin : g_slice_er_ec
			localparam integer EK = gsk / 3;
			localparam integer SK = gsk % 3;
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssrer (.I0(cond_step_r_b[EK]),      .O(sel_step_r_er_l[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scer  (.I0(cond_coarse_e_b[EK]),    .O(sel_coarse_er_l[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sscec (.I0(cond_step_c_b[EK]),      .O(sel_step_c_ec_l[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ssrec (.I0(cond_step_r_b[EK]),      .O(sel_step_r_ec_l[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_scec  (.I0(cond_coarse_e_b[EK]),    .O(sel_coarse_ec_l[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_werl  (.I0(cond_we_er_b[EK]),       .O(we_er_leaf[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wecl  (.I0(cond_we_ec_b[EK]),       .O(we_ec_leaf[gsk]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wab8  (.I0(st_fsm_0 == S_SEQ),      .O(we_ab8_leaf[gsk]));
`else
			assign sel_step_r_er_l[gsk] = cond_step_r_b[EK];
			assign sel_coarse_er_l[gsk] = cond_coarse_e_b[EK];
			assign sel_step_c_ec_l[gsk] = cond_step_c_b[EK];
			assign sel_step_r_ec_l[gsk] = cond_step_r_b[EK];
			assign sel_coarse_ec_l[gsk] = cond_coarse_e_b[EK];
			assign we_er_leaf[gsk]      = cond_we_er_b[EK];
			assign we_ec_leaf[gsk]      = cond_we_ec_b[EK];
			assign we_ab8_leaf[gsk]     = (st_fsm_0 == S_SEQ);
`endif

			wire [15:0] in_coarse_e = (EK == 0) ? coarse_tile_e0[SK*16 +: 16] :
			                          (EK == 1) ? coarse_tile_e1[SK*16 +: 16] :
			                                      coarse_tile_e2[SK*16 +: 16];
			wire [15:0] in_er0 = er_direct_init[EK][SK*16 +: 16];
			wire [15:0] in_er_n = er_n[EK][SK*16 +: 16];
			wire [15:0] in_ec_n = ec_n[EK][SK*16 +: 16];

			wire [15:0] next_er = sel_step_r_er_l[gsk] ? in_er_n : sel_coarse_er_l[gsk] ? in_coarse_e : in_er0;
			wire [15:0] next_ec = sel_step_c_ec_l[gsk] ? in_ec_n : sel_step_r_ec_l[gsk] ? in_er_n : sel_coarse_ec_l[gsk] ? in_coarse_e : in_er0;

			always @(posedge clk) begin
				if (we_er_leaf[gsk])
					er[EK][SK*16 +: 16] <= next_er;
				if (we_ec_leaf[gsk])
					ec[EK][SK*16 +: 16] <= next_ec;
				if (we_ab8_leaf[gsk])
					ab8[EK][SK*16 +: 16] <= ab8_comb[EK][SK*16 +: 16];
			end
		end
	endgenerate

	wire we_resamp_raw = (st_fsm_7 == S_IDLE && sp_val_fsm) || (st_fsm_7 == S_SAMP);
	wire [2:0] we_resamp_leaf, we_resamp_clr;
	genvar gres;
	generate
		for (gres = 0; gres < 3; gres = gres + 1) begin : g_we_resamp
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wrs  (.I0(we_resamp_raw),        .O(we_resamp_leaf[gres]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wrsc (.I0(st_fsm_7 == S_IDLE), .O(we_resamp_clr[gres]));
`else
			assign we_resamp_leaf[gres] = we_resamp_raw;
			assign we_resamp_clr[gres]  = (st_fsm_7 == S_IDLE);
`endif
			always @(posedge clk) begin
				if (we_resamp_leaf[gres])
					r_esamp_sign[gres*16 +: 16] <= we_resamp_clr[gres] ? 16'd0 : esamp_sign[gres*16 +: 16];
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] clr_zv_leaf;
	wire [7:0] we_zv_leaf;
	wire [7:0] we_pz_leaf;
	wire [7:0] we_dxd_leaf;
	wire [1:0] we_dxy_leaf;
	wire [1:0] we_pxy_leaf;

	wire dx_stall = !(dx_valid && dx_ready);
	wire [2:0] dx_stall_b = {3{dx_stall}};

	wire we_pz_raw = (st_fsm_3 == S_EMIT) && (!pend || !dx_stall_b[0]);
	wire we_dxd_raw = dx_stall_b[1] && (((st_fsm_3 == S_EMIT) && pend) || (st_fsm_3 == S_LAST));
	wire we_dxy_raw = dx_stall_b[2] && (((st_fsm_3 == S_EMIT) && pend) || (st_fsm_3 == S_LAST));
	wire we_pxy_raw = we_pz_raw;

	wire sel_last_raw = (st_fsm_3 == S_LAST) && !pend;
	wire [7:0] sel_last_d_leaf;
	wire [1:0] sel_last_xy_leaf;

	wire clr_zv_raw = (st_fsm_1 == S_Z && zc_3 == 5'd0);
	genvar gzvb;
	generate
	for (gzvb = 0; gzvb < 8; gzvb = gzvb + 1) begin : g_zv_ctrl_b
		assign clr_zv_leaf[gzvb] = clr_zv_raw;
		assign we_pz_leaf[gzvb] = we_pz_raw;
		assign we_dxd_leaf[gzvb] = we_dxd_raw;
		assign sel_last_d_leaf[gzvb] = sel_last_raw;
	end
	for (gzvb = 0; gzvb < 2; gzvb = gzvb + 1) begin : g_xy_ctrl_b
		assign we_dxy_leaf[gzvb] = we_dxy_raw;
		assign we_pxy_leaf[gzvb] = we_pxy_raw;
		assign sel_last_xy_leaf[gzvb] = sel_last_raw;
	end
	endgenerate

	genvar gzs;
	generate
		for (gzs = 0; gzs < 8; gzs = gzs + 1) begin : g_zv_slices
			wire we_zv_raw = (st_fsm_1 == S_Z && zc_3 == (5'd21 + (gzs / 2)) && zmask[gzs / 2]);
			assign we_zv_leaf[gzs] = we_zv_raw;

			always @(posedge clk) begin
				if (clr_zv_leaf[gzs])
					zv[gzs * 16 +: 16] <= 16'd0;
				else if (we_zv_leaf[gzs])
					zv[gzs * 16 +: 16] <= (gzs % 2 == 0) ? zcl_lo : zcl_hi;
			end

			always @(posedge clk) begin
				if (we_pz_leaf[gzs])
					p_z[gzs * 16 +: 16] <= zv[gzs * 16 +: 16];
			end

			always @(posedge clk) begin
				if (we_dxd_leaf[gzs])
					dx_depth[gzs * 16 +: 16] <= sel_last_d_leaf[gzs] ? 16'd0 : p_z[gzs * 16 +: 16];
			end
		end
		for (gzs = 0; gzs < 2; gzs = gzs + 1) begin : g_xy_slices
			always @(posedge clk) begin
				if (we_pxy_leaf[gzs])
					p_xy[gzs * 16 +: 16] <= (gzs == 0) ? qx : qy;
			end

			always @(posedge clk) begin
				if (we_dxy_leaf[gzs])
					dx_xy[gzs * 16 +: 16] <= sel_last_xy_leaf[gzs] ?
						((gzs == 0) ? {bx0[15:1], 1'b0} : {by0[15:1], 1'b0}) :
						p_xy[gzs * 16 +: 16];
			end
		end
	endgenerate

	wire we_seq = (st_fsm_3 == S_SEQ);
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) seq <= 16'd0;
		else if (we_seq) seq <= sq_seq;
	end

	wire we_sp_idle;
	(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'h00010000)) u_wspidle (
		.I0(st_fsm_5[0]), .I1(st_fsm_5[1]), .I2(st_fsm_5[2]), .I3(st_fsm_5[3]),
		.I4(sp_val_fsm), .O(we_sp_idle)
	);
	wire [3:0] we_sp_branch;
	wire [8:0] we_a8_l, we_b8_l;
	wire [3:0] we_bbox_l;
	wire [5:0] we_z_l;
	genvar gwsp;
	generate
	for (gwsp = 0; gwsp < 4; gwsp = gwsp + 1) begin : g_wsp_br
		assign we_sp_branch[gwsp] = we_sp_idle;
	end
	for (gwsp = 0; gwsp < 9; gwsp = gwsp + 1) begin : g_wsp_ab
		assign we_a8_l[gwsp] = we_sp_branch[gwsp / 3];
		assign we_b8_l[gwsp] = we_sp_branch[gwsp / 3];
	end
	for (gwsp = 0; gwsp < 4; gwsp = gwsp + 1) begin : g_wsp_bb
		assign we_bbox_l[gwsp] = we_sp_branch[3];
	end
	for (gwsp = 0; gwsp < 6; gwsp = gwsp + 1) begin : g_wsp_z
		assign we_z_l[gwsp] = we_sp_branch[3];
	end
	endgenerate

	wire [47:0] in_a8_comb [0:2];
	wire [47:0] in_b8_comb [0:2];
	assign in_a8_comb[0] = {{16{s_e0_a_scan_ext}}, sp_tri[0 +: 24], 8'd0};
	assign in_b8_comb[0] = {{16{s_e0_b_scan_ext}}, sp_tri[24 +: 24], 8'd0};
	assign in_a8_comb[1] = {{16{s_e1_a_scan_ext}}, sp_tri[96 +: 24], 8'd0};
	assign in_b8_comb[1] = {{16{s_e1_b_scan_ext}}, sp_tri[120 +: 24], 8'd0};
	assign in_a8_comb[2] = {{16{s_e2_a_scan_ext}}, sp_tri[192 +: 24], 8'd0};
	assign in_b8_comb[2] = {{16{s_e2_b_scan_ext}}, sp_tri[216 +: 24], 8'd0};

	genvar gsp;
	generate
		for (gsp = 0; gsp < 9; gsp = gsp + 1) begin : g_a8_b8_reg
			localparam integer EK = gsp / 3;
			localparam integer SK = gsp % 3;
			always @(posedge clk) begin
				if (we_a8_l[gsp]) a8[EK][SK*16 +: 16] <= in_a8_comb[EK][SK*16 +: 16];
				if (we_b8_l[gsp]) b8[EK][SK*16 +: 16] <= in_b8_comb[EK][SK*16 +: 16];
			end
		end
	endgenerate

	always @(posedge clk) begin
		if (we_bbox_l[0]) bx0 <= sp_tri[303:288];
		if (we_bbox_l[1]) by0 <= sp_tri[319:304];
		if (we_bbox_l[2]) bx1 <= sp_tri[335:320];
		if (we_bbox_l[3]) by1 <= sp_tri[351:336];
	end

	always @(posedge clk) begin
		if (we_z_l[0]) az[15:0]  <= sp_tri[367:352];
		if (we_z_l[1]) az[31:16] <= sp_tri[383:368];
		if (we_z_l[2]) bz[15:0]  <= sp_tri[399:384];
		if (we_z_l[3]) bz[31:16] <= sp_tri[415:400];
		if (we_z_l[4]) cz[15:0]  <= sp_tri[431:416];
		if (we_z_l[5]) cz[31:16] <= sp_tri[447:432];
	end

	wire we_dx_raw = (st_fsm_1 == S_Z) && (zc_1 == 5'd0);
	wire [7:0] we_dx_leaf;
	wire we_fdx_raw = (st_fsm_1 == S_Z) && (zc_1 == 5'd2);
	wire [7:0] we_fdx_leaf;
	genvar gzdx;
	generate
	for (gzdx = 0; gzdx < 8; gzdx = gzdx + 1) begin : g_zdx_buf
		assign we_dx_leaf[gzdx] = we_dx_raw;
		assign we_fdx_leaf[gzdx] = we_fdx_raw;
	end
	endgenerate
	always @(posedge clk) begin
		if (we_dx_leaf[0]) r_dx0[15:0]  <= dx0_comb[15:0];
		if (we_dx_leaf[1]) r_dx0[16]    <= dx0_comb[16];
		if (we_dx_leaf[2]) r_dy0[15:0]  <= dy0_comb[15:0];
		if (we_dx_leaf[3]) r_dy0[16]    <= dy0_comb[16];
		if (we_dx_leaf[4]) r_dx1[15:0]  <= dx1_comb[15:0];
		if (we_dx_leaf[5]) r_dx1[16]    <= dx1_comb[16];
		if (we_dx_leaf[6]) r_dy1[15:0]  <= dy1_comb[15:0];
		if (we_dx_leaf[7]) r_dy1[16]    <= dy1_comb[16];
	end

	wire we_zlz_raw = (st_fsm_1 == S_Z && zc_1 == 5'd1);
	wire [5:0] we_zlz_b;
	genvar gzlw;
	generate
	for (gzlw = 0; gzlw < 6; gzlw = gzlw + 1) begin : g_wzlz_buf
		assign we_zlz_b[gzlw] = we_zlz_raw;
	end
	endgenerate
	always @(posedge clk) begin
		if (we_zlz_b[0]) begin
			{r_z_is0_0, r_z_lz0_a}            <= lzc_dx0;
			{r_z_is0_0_b_unused, r_z_lz0_b}   <= lzc_dx0;
		end
		if (we_zlz_b[1]) begin
			{r_z_is0_0_e_unused, r_z_lz0_exp} <= lzc_dx0;
			{r_z_is0_1, r_z_lz1}              <= lzc_dy0;
		end
		if (we_zlz_b[2]) begin
			r_z_lz1_b   <= lzc_dy0[4:0];
			r_z_lz1_exp <= lzc_dy0[4:0];
		end
		if (we_zlz_b[3]) begin
			{r_z_is0_2, r_z_lz2} <= lzc_dx1;
			r_z_lz2_b            <= lzc_dx1[4:0];
		end
		if (we_zlz_b[4]) begin
			r_z_lz2_exp          <= lzc_dx1[4:0];
			{r_z_is0_3, r_z_lz3} <= lzc_dy1;
		end
		if (we_zlz_b[5]) begin
			r_z_lz3_b   <= lzc_dy1[4:0];
			r_z_lz3_exp <= lzc_dy1[4:0];
		end
		if (we_fdx_leaf[0]) r_fdx0[15:0]  <= f_dx0_comb[15:0];
		if (we_fdx_leaf[1]) r_fdx0[31:16] <= f_dx0_comb[31:16];
		if (we_fdx_leaf[2]) r_fdy0[15:0]  <= f_dy0_comb[15:0];
		if (we_fdx_leaf[3]) r_fdy0[31:16] <= f_dy0_comb[31:16];
		if (we_fdx_leaf[4]) r_fdx1[15:0]  <= f_dx1_comb[15:0];
		if (we_fdx_leaf[5]) r_fdx1[31:16] <= f_dx1_comb[31:16];
		if (we_fdx_leaf[6]) r_fdy1[15:0]  <= f_dy1_comb[15:0];
		if (we_fdx_leaf[7]) r_fdy1[31:16] <= f_dy1_comb[31:16];
	end

	wire [7:0] we_t1_leaf;
	wire we_t1_6 = (st_fsm_1 == S_Z) && (zc_2 == 5'd7);
	wire we_t1_7 = (st_fsm_1 == S_Z) && (zc_2 == 5'd8);
	wire we_t1_8 = (st_fsm_1 == S_Z) && (zc_2 == 5'd9);
	wire we_t1_9 = (st_fsm_1 == S_Z) && (zc_2 == 5'd10);
	assign we_t1_leaf[0] = we_t1_6;
	assign we_t1_leaf[1] = we_t1_6;
	assign we_t1_leaf[2] = we_t1_7;
	assign we_t1_leaf[3] = we_t1_7;
	assign we_t1_leaf[4] = we_t1_8;
	assign we_t1_leaf[5] = we_t1_8;
	assign we_t1_leaf[6] = we_t1_9;
	assign we_t1_leaf[7] = we_t1_9;

	always @(posedge clk) begin
		if (we_t1_leaf[0]) t1_0[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[1]) t1_0[31:16] <= fm_r[31:16];
		if (we_t1_leaf[2]) t1_1[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[3]) t1_1[31:16] <= fm_r[31:16];
		if (we_t1_leaf[4]) t1_2[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[5]) t1_2[31:16] <= fm_r[31:16];
		if (we_t1_leaf[6]) t1_3[15:0]  <= fm_r[15:0];
		if (we_t1_leaf[7]) t1_3[31:16] <= fm_r[31:16];
	end

	wire qte = (r_tile_row_end && r_tile_col_end);
	wire cce = (st_fsm_2 == S_TILE_WAIT && ctv_edge) ||
	           (st_fsm_2 == S_STEP && qte && !cur_last_tile && ctv_edge);
	wire er0_b = (st_fsm_0 == S_SEQ && COARSE_SCAN == 0);
	wire cstep = (st_fsm_0 == S_STEP && !qte);
	wire cstep_r = cstep && r_tile_row_end;
	wire cstep_c = cstep && !r_tile_row_end;

	wire [2:0] cond_coarse_e_b, cond_step_r_b, cond_step_c_b, cond_we_er_b, cond_we_ec_b;
	wire qte_step_c, qte_cce;
	assign qte_step_c = qte;
	assign qte_cce = qte;
	wire cce_b   = (st_fsm_2 == S_TILE_WAIT && ctv_edge) ||
	               (st_fsm_2 == S_STEP && qte_cce && !cur_last_tile && ctv_edge);
	wire er0_bb  = (st_fsm_0 == S_SEQ && COARSE_SCAN == 0);
	wire cstep_b = (st_fsm_0 == S_STEP && !qte_step_c);
	wire cstep_r_b = cstep_b && r_tile_row_end;
	wire cstep_c_b = cstep_b && !r_tile_row_end;
	genvar gcb;
	generate
	for (gcb = 0; gcb < 3; gcb = gcb + 1) begin : g_cb_buf
		assign cond_coarse_e_b[gcb] = cce_b;
		assign cond_step_r_b[gcb] = cstep_r_b;
		assign cond_step_c_b[gcb] = cstep_c_b;
		assign cond_we_er_b[gcb] = er0_bb || cce_b || cstep_r_b;
		assign cond_we_ec_b[gcb] = er0_bb || cce_b || cstep_b;
	end
	endgenerate

	wire [8:0] sel_step_r_er_l;
	wire [8:0] sel_coarse_er_l;
	wire [8:0] sel_step_c_ec_l;
	wire [8:0] sel_step_r_ec_l;
	wire [8:0] sel_coarse_ec_l;
	wire [8:0] we_er_leaf;
	wire [8:0] we_ec_leaf;
	wire [8:0] we_ab8_leaf;
	genvar gsk;
	generate
		for (gsk = 0; gsk < 9; gsk = gsk + 1) begin : g_slice_er_ec
			localparam integer EK = gsk / 3;
			localparam integer SK = gsk % 3;
			assign sel_step_r_er_l[gsk] = cond_step_r_b[EK];
			assign sel_coarse_er_l[gsk] = cond_coarse_e_b[EK];
			assign sel_step_c_ec_l[gsk] = cond_step_c_b[EK];
			assign sel_step_r_ec_l[gsk] = cond_step_r_b[EK];
			assign sel_coarse_ec_l[gsk] = cond_coarse_e_b[EK];
			assign we_er_leaf[gsk] = cond_we_er_b[EK];
			assign we_ec_leaf[gsk] = cond_we_ec_b[EK];
			assign we_ab8_leaf[gsk] = st_fsm_0 == S_SEQ;

			wire [15:0] in_coarse_e = (EK == 0) ? coarse_tile_e0[SK*16 +: 16] :
			                          (EK == 1) ? coarse_tile_e1[SK*16 +: 16] :
			                                      coarse_tile_e2[SK*16 +: 16];
			wire [15:0] in_er0 = er_direct_init[EK][SK*16 +: 16];
			wire [15:0] in_er_n = er_n[EK][SK*16 +: 16];
			wire [15:0] in_ec_n = ec_n[EK][SK*16 +: 16];

			wire [15:0] next_er = sel_step_r_er_l[gsk] ? in_er_n : sel_coarse_er_l[gsk] ? in_coarse_e : in_er0;
			wire [15:0] next_ec = sel_step_c_ec_l[gsk] ? in_ec_n : sel_step_r_ec_l[gsk] ? in_er_n : sel_coarse_ec_l[gsk] ? in_coarse_e : in_er0;

			always @(posedge clk) begin
				if (we_er_leaf[gsk])
					er[EK][SK*16 +: 16] <= next_er;
				if (we_ec_leaf[gsk])
					ec[EK][SK*16 +: 16] <= next_ec;
				if (we_ab8_leaf[gsk])
					ab8[EK][SK*16 +: 16] <= ab8_comb[EK][SK*16 +: 16];
			end
		end
	endgenerate

	wire we_resamp_raw = (st_fsm_7 == S_IDLE && sp_val_fsm) || (st_fsm_7 == S_SAMP);
	wire [2:0] we_resamp_leaf, we_resamp_clr;
	genvar gres;
	generate
		for (gres = 0; gres < 3; gres = gres + 1) begin : g_we_resamp
			assign we_resamp_leaf[gres] = we_resamp_raw;
			assign we_resamp_clr[gres] = st_fsm_7 == S_IDLE;
			always @(posedge clk) begin
				if (we_resamp_leaf[gres])
					r_esamp_sign[gres*16 +: 16] <= we_resamp_clr[gres] ? 16'd0 : esamp_sign[gres*16 +: 16];
			end
		end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
	assign sp_ready = (st_fsm_7 == S_IDLE);
	assign sq_req = (st_fsm_7 == S_SEQ && !wm);
	assign fq_valid = (st_fsm_7 == S_FQ);
	assign fq_xy = {qy, qx};
	assign fq_scov = fend ? 16'd0 : ms4_misc ? scov_reg :
		{3'd0, cov_reg_scov[3] & ~kill[0], 3'd0, cov_reg_scov[2] & ~kill[0], 3'd0, cov_reg_scov[1] & ~kill[0], 3'd0, cov_reg_scov[0] & ~kill[0]};
	assign fq_ccov = fend ? 4'd0 : ms4_misc ? ccov_reg : cov_reg;
	assign fq_end = fend;
	assign busy = (st_fsm_7 != S_IDLE);
`elsif SYNTH_KINTEX_ULTRASCALE
	assign sp_ready = (st_fsm_7 == S_IDLE);
	assign sq_req = (st_fsm_7 == S_SEQ && !wm);
	assign fq_valid = (st_fsm_7 == S_FQ);
	assign fq_xy = {qy, qx};
	assign fq_scov = fend ? 16'd0 : ms4_misc ? scov_reg :
		{3'd0, cov_reg_scov[3] & ~kill[0], 3'd0, cov_reg_scov[2] & ~kill[0], 3'd0, cov_reg_scov[1] & ~kill[0], 3'd0, cov_reg_scov[0] & ~kill[0]};
	assign fq_ccov = fend ? 4'd0 : ms4_misc ? ccov_reg : cov_reg;
	assign fq_end = fend;
	assign busy = (st_fsm_7 != S_IDLE);
`else
	assign sp_ready = (st == S_IDLE);
	assign sq_req = (st == S_SEQ && !wm);
	assign fq_valid = (st == S_FQ);
	assign fq_xy = {qy, qx};
	assign fq_scov = fend ? 16'd0 : ms4_misc ? scov_reg :
		{3'd0, cov_reg[3] & ~kill[0], 3'd0, cov_reg[2] & ~kill[0], 3'd0, cov_reg[1] & ~kill[0], 3'd0, cov_reg[0] & ~kill[0]};
	assign fq_ccov = fend ? 4'd0 : ms4_misc ? ccov_reg : cov_reg;
	assign fq_end = fend;
	assign busy = (st != S_IDLE);
`endif

	/* Dedicated scov_reg and cov_reg processes to isolate fanout and logic depth */
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			scov_reg <= 16'd0;
			ccov_reg <= 4'd0;
			cov_reg  <= 4'd0;
`ifdef SYNTH_KINTEX7
			cov_reg_scov <= 4'd0;
`elsif SYNTH_KINTEX_ULTRASCALE
			cov_reg_scov <= 4'd0;
`endif
		end else if (st_fsm_6 == S_EVAL) begin
			scov_reg <= cur_scov;
			ccov_reg <= cur_ccov;
			cov_reg  <= cur_cov;
`ifdef SYNTH_KINTEX7
			cov_reg_scov <= cur_cov;
`elsif SYNTH_KINTEX_ULTRASCALE
			cov_reg_scov <= cur_cov;
`endif
		end
	end

	/* Separate qx, qy, ent processes to reduce st_fsm_main fanout */
	always @(posedge clk) begin
		case (st_fsm_m1)
		S_IDLE: if (sp_val_qx) qx[7:0] <= {sp_tri[295:289], 1'b0};
		S_SEQ: if (COARSE_SCAN == 0) qx[7:0] <= {bx0[7:1], 1'b0};
		S_TILE_WAIT: if (ctv_fsm) qx[7:0] <= coarse_tile_x[7:0];
		S_STEP: if (r_tile_row_end && r_tile_col_end) begin
			if (!cur_last_tile && ctv_fsm) qx[7:0] <= coarse_tile_x[7:0];
		end else begin
			qx[7:0] <= r_tile_row_end ? cur_tx[7:0] : qx2[7:0];
		end
		default: ;
		endcase
	end

	always @(posedge clk) begin
		case (st_fsm_m3)
		S_IDLE: if (sp_val_qx) qx[15:8] <= sp_tri[303:296];
		S_SEQ: if (COARSE_SCAN == 0) qx[15:8] <= bx0[15:8];
		S_TILE_WAIT: if (ctv_fsm) qx[15:8] <= coarse_tile_x[15:8];
		S_STEP: if (r_tile_row_end && r_tile_col_end) begin
			if (!cur_last_tile && ctv_fsm) qx[15:8] <= coarse_tile_x[15:8];
		end else begin
			qx[15:8] <= r_tile_row_end ? cur_tx[15:8] : qx2[15:8];
		end
		default: ;
		endcase
	end

	always @(posedge clk) begin
		case (st_fsm_m2)
		S_IDLE: if (sp_val_qy) qy <= {sp_tri[319:305], 1'b0};
		S_SEQ: if (COARSE_SCAN == 0) qy <= {by0[15:1], 1'b0};
		S_TILE_WAIT: if (ctv_fsm) qy <= coarse_tile_y;
		S_STEP: if (r_tile_row_end && r_tile_col_end) begin
			if (!cur_last_tile && ctv_fsm) qy <= coarse_tile_y;
		end else if (r_tile_row_end) begin
			qy <= qy2[15:0];
		end
		default: ;
		endcase
	end

`ifdef SYNTH_KINTEX7
	always @(posedge clk) begin
		if (st_fsm_m3 == S_IDLE) begin
			if (we_ent_lo) ent[11:0] <= sp_tri[747:736];
			if (we_ent_hi) ent[23:12] <= sp_tri[759:748];
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk) begin
		if (st_fsm_m3 == S_IDLE) begin
			if (we_ent_lo) ent[11:0] <= sp_tri[747:736];
			if (we_ent_hi) ent[23:12] <= sp_tri[759:748];
		end
	end
`endif

	wire we_dx_seq_raw = ((st_fsm_m4 == S_EMIT) && pend && !(dx_valid && dx_ready)) ||
		((st_fsm_m4 == S_LAST) && !(dx_valid && dx_ready));
	wire we_dx_seq_lo, we_dx_seq_hi;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdxsq0 (.I0(we_dx_seq_raw), .O(we_dx_seq_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdxsq1 (.I0(we_dx_seq_raw), .O(we_dx_seq_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign we_dx_seq_lo = we_dx_seq_raw;
	assign we_dx_seq_hi = we_dx_seq_raw;
`else
	assign we_dx_seq_lo = we_dx_seq_raw;
	assign we_dx_seq_hi = we_dx_seq_raw;
`endif
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) dx_seq[7:0] <= 8'd0;
		else if (we_dx_seq_lo) dx_seq[7:0] <= seq[7:0];
	end
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) dx_seq[15:8] <= 8'd0;
		else if (we_dx_seq_hi) dx_seq[15:8] <= seq[15:8];
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			dx_valid <= 1'b0;
			dx_mask <= 4'd0;
			dx_last <= 1'b0;
			pend <= 1'b0;
			p_mask <= 4'd0;
		end else case (st_fsm_m4)
		S_IDLE: if (sp_val_ctrl) pend <= 1'b0;
		S_EMIT: begin
			if (pend && !(dx_valid && dx_ready)) begin
				dx_valid <= 1'b1;
				dx_mask <= p_mask;
				dx_last <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				dx_xy <= p_xy;
				dx_depth <= p_z;
`endif
			end else begin
				dx_valid <= 1'b0;
				pend <= 1'b1;
				p_mask <= zmask;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				p_xy <= {qy, qx};
				p_z <= zv;
`endif
			end
		end
		S_LAST: begin
			if (dx_valid && dx_ready) begin
				dx_valid <= 1'b0;
			end else begin
				dx_valid <= 1'b1;
				dx_mask <= pend ? p_mask : 4'd0;
				dx_last <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				dx_xy <= pend ? p_xy : {by0[15:1], 1'b0, bx0[15:1], 1'b0};
				dx_depth <= pend ? p_z : 128'd0;
`endif
			end
		end
		default: ;
		endcase
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			`SET_ZC(5'd0);
			zc_oh <= 25'd1;
			`SET_ZL(2'd0);
			`SET_BZA(1'b0);
			`SET_BZB(1'b0);
			r_fm_v <= 1'b0;
			r_fa_v <= 1'b0;
		end else case (st_fsm_m5)
		S_IDLE: begin
			r_fm_v <= 1'b0;
			`SET_BZA(1'b0);
			`SET_BZB(1'b0);
		end
		S_EVAL: begin
			`SET_ZC(5'd0);
			zc_oh <= 25'd1;
		end
		S_Z: begin
			`SET_ZC(zc1);
			zc_oh <= {zc_oh[23:0], 1'b0};
			if (zc == 5'd2) begin
				`SET_ZL(2'd0);
				`SET_BZA(1'b0);
				`SET_BZB(1'b0);
				r_fm_v <= 1'b1;
`ifdef SYNTH_KINTEX7
`else
				r_fm_a_q <= az;
				r_fm_b_q <= f_dx0_comb;
`endif
			end else if (zc >= 5'd3) begin
				`SET_ZL(r_zl_next);
`ifdef SYNTH_KINTEX7
`else
				case (zc)
				5'd3: begin r_fm_a_q <= az; r_fm_b_q <= r_fdx1; end
				5'd4: begin r_fm_a_q <= az; r_fm_b_q <= r_fdx0; end
				5'd5: begin r_fm_a_q <= az; r_fm_b_q <= r_fdx1; end
				5'd6: begin r_fm_a_q <= bz; r_fm_b_q <= r_fdy0; end
				5'd7: begin r_fm_a_q <= bz; r_fm_b_q <= r_fdy0; end
				5'd8: begin r_fm_a_q <= bz; r_fm_b_q <= r_fdy1; end
				5'd9: begin r_fm_a_q <= bz; r_fm_b_q <= r_fdy1; end
				default: ;
				endcase
`endif
			end
`ifdef SYNTH_KINTEX7
			if (zc_1 == 5'd6) begin
				`SET_BZA(1'b1);
				`SET_BZB(1'b1);
			end
			if (zc_1 == 5'd10) r_fm_v <= 1'b0;
			if (zc_0 == 5'd11 || zc_0 == 5'd12 || zc_0 == 5'd13 || zc_0 == 5'd14 || (zc_0 >= 5'd16 && zc_0 <= 5'd19))
				r_fa_v <= 1'b1;
			if (zc_0 == 5'd15 || zc_0 == 5'd20)
				r_fa_v <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (zc_1 == 5'd6) begin
				`SET_BZA(1'b1);
				`SET_BZB(1'b1);
			end
			if (zc_1 == 5'd10) r_fm_v <= 1'b0;
			if (zc_0 == 5'd11 || zc_0 == 5'd12 || zc_0 == 5'd13 || zc_0 == 5'd14 || (zc_0 >= 5'd16 && zc_0 <= 5'd19))
				r_fa_v <= 1'b1;
			if (zc_0 == 5'd15 || zc_0 == 5'd20)
				r_fa_v <= 1'b0;
`else
			if (zc_oh[6]) begin
				`SET_BZA(1'b1);
				`SET_BZB(1'b1);
			end
			if (zc_oh[10]) r_fm_v <= 1'b0;
			if (zc_oh[11]) begin r_fa_v <= 1'b1; r_fa_a_lo <= t1_0[15:0]; r_fa_a_hi <= t1_0[31:16]; r_fa_b_lo <= fm_r[15:0]; r_fa_b_hi <= fm_r[31:16]; end
			if (zc_oh[12]) begin r_fa_v <= 1'b1; r_fa_a_lo <= t1_1[15:0]; r_fa_a_hi <= t1_1[31:16]; r_fa_b_lo <= fm_r[15:0]; r_fa_b_hi <= fm_r[31:16]; end
			if (zc_oh[13]) begin r_fa_v <= 1'b1; r_fa_a_lo <= t1_2[15:0]; r_fa_a_hi <= t1_2[31:16]; r_fa_b_lo <= fm_r[15:0]; r_fa_b_hi <= fm_r[31:16]; end
			if (zc_oh[14]) begin r_fa_v <= 1'b1; r_fa_a_lo <= t1_3[15:0]; r_fa_a_hi <= t1_3[31:16]; r_fa_b_lo <= fm_r[15:0]; r_fa_b_hi <= fm_r[31:16]; end
			if (zc_oh[15]) begin r_fa_v <= 1'b0; end
			if (|zc_oh[19:16]) begin
				r_fa_v <= 1'b1;
				r_fa_a_lo <= fa_r[15:0];
				r_fa_a_hi <= fa_r[31:16];
				r_fa_b_lo <= cz[15:0];
				r_fa_b_hi <= cz[31:16];
			end
			if (zc_oh[20]) begin r_fa_v <= 1'b0; end
`endif
		end
		default: ;
		endcase
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			zmask <= 4'd0;
			r_lve_sign <= 12'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_inb <= 4'd0;
`endif
			r_tile_row_end <= 1'b0;
			r_tile_col_end <= 1'b0;
			r_lve_inb_nokill <= 4'd0;
			r_w_empty_triv <= 1'b1;
			r_esamp_c24 <= 48'd0;
		end else case (st_fsm_m6)
		S_IDLE: if (sp_val_ctrl) begin
			fl <= sp_tri[762:760];
			raw <= sp_tri[764];
			wm <= (HAS_SU != 0) && !sp_tri[764];
		end
		S_SAMP: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_inb <= inb_comb;
`endif
			r_tile_row_end <= tile_row_end;
			r_tile_col_end <= tile_col_end;
		end
		S_EVAL0: begin
			for (k = 0; k < 12; k = k + 1)
				r_lve_sign[k] <= r_lve[48 * k + 47];
			for (k = 0; k < 4; k = k + 1) begin
				r_lve_inb_nokill[k] <= r_inb[k] && !r_lve[48 * k + 47] && !r_lve[48 * (4 + k) + 47] && !r_lve[48 * (8 + k) + 47];
			end
			r_w_empty_triv <= w_empty_triv;
			r_esamp_c24 <= esamp_c24_w;
		end
		S_EVAL: begin
`ifdef SYNTH_KINTEX7
			if (!r_w_empty_zm && !wm)
				zmask <= r_cur_cov_zm;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (!r_w_empty_zm && !wm)
				zmask <= r_cur_cov_zm;
`else
			if (!w_empty && !wm)
				zmask <= cur_cov;
`endif
		end
		default: ;
		endcase
	end

	wire [4:0] we_walk_b;
`ifdef SYNTH_KINTEX7
	genvar gwk;
	generate
	for (gwk = 0; gwk < 5; gwk = gwk + 1) begin : g_walk_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwlk (.I0(st_fsm_m6 == S_WALK), .O(we_walk_b[gwk]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gwk;
	generate
	for (gwk = 0; gwk < 5; gwk = gwk + 1) begin : g_walk_buf
		assign we_walk_b[gwk] = st_fsm_m6 == S_WALK;
	end
	endgenerate
`else
	assign we_walk_b = {5{(st_fsm_m6 == S_WALK)}};
`endif
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
`ifdef SYNTH_KINTEX7
			r_inb        <= 4'd0;
`elsif SYNTH_KINTEX_ULTRASCALE
			r_inb        <= 4'd0;
`else
			r_q_lt_b     <= 8'd0;
`endif
			r_qx2        <= 17'd0;
			r_qy2        <= 17'd0;
			r_cur_tx_end <= 16'd0;
			r_cur_ty_end <= 16'd0;
		end else begin
`ifdef SYNTH_KINTEX7
			if (we_walk_b[0]) r_inb        <= inb_comb;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (we_walk_b[0]) r_inb        <= inb_comb;
`else
			if (we_walk_b[0]) r_q_lt_b     <= {qy1_lt_by1, qy1_lt_by0, qy0_lt_by1, qy0_lt_by0, qx1_lt_bx1, qx1_lt_bx0, qx0_lt_bx1, qx0_lt_bx0};
`endif
			if (we_walk_b[1]) r_qx2[15:0]  <= qx2[15:0];
			if (we_walk_b[2]) r_qy2[15:0]  <= qy2[15:0];
			if (we_walk_b[0]) begin r_qx2[16] <= qx2[16]; r_qy2[16] <= qy2[16]; end
			if (we_walk_b[3]) r_cur_tx_end <= cur_tx_end;
			if (we_walk_b[4]) r_cur_ty_end <= cur_ty_end;
		end
	end

`ifdef SYNTH_KINTEX7
	wire st_ev0_k, st_ev0_s;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stev0k (.I0(st_fsm_m6 == S_EVAL0), .O(st_ev0_k));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stev0s (.I0(st_fsm_m6 == S_EVAL0), .O(st_ev0_s));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire st_ev0_k, st_ev0_s;
	assign st_ev0_k = st_fsm_m6 == S_EVAL0;
	assign st_ev0_s = st_fsm_m6 == S_EVAL0;
`else
	wire st_ev0_k = (st_fsm_m6 == S_EVAL0);
	wire st_ev0_s = (st_fsm_m6 == S_EVAL0);
`endif
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			r_lve_inb_kill <= 4'd0;
		end else if (st_ev0_k) begin
			for (k = 0; k < 4; k = k + 1) begin
				r_lve_inb_kill[k] <= r_inb[k] && !kill[0] && !r_lve[48 * k + 47] && !r_lve[48 * (4 + k) + 47] && !r_lve[48 * (8 + k) + 47];
			end
		end
	end
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			r_s_act_pre <= 16'd0;
		end else if (st_ev0_s) begin
			for (k = 0; k < 4; k = k + 1) begin
				r_s_act_pre[4 * k + 0] <= r_inb[k] && !kill[0] && (0 < MSAA_SAMPLES);
				r_s_act_pre[4 * k + 1] <= r_inb[k] && !kill[1] && (1 < MSAA_SAMPLES);
				r_s_act_pre[4 * k + 2] <= r_inb[k] && !kill[2] && (2 < MSAA_SAMPLES);
				r_s_act_pre[4 * k + 3] <= r_inb[k] && !kill[3] && (3 < MSAA_SAMPLES);
			end
		end
	end

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			pt_free <= 1'b0;
			coarse_tri_valid <= 1'b0;
			coarse_tile_ready <= 1'b0;
			fend <= 1'b0;
			fi <= 2'd0;
		end else begin
		pt_free <= 1'b0;
		case (st_fsm_m7)
		S_IDLE: begin
			coarse_tile_ready <= 1'b0;
			if (sp_val_ctrl) begin
				fend <= 1'b0;
				fi <= 2'd0;
				if (u16_gt(sp_tri[335:320], sp_tri[303:288]) && u16_gt(sp_tri[351:336], sp_tri[319:304]))
					coarse_tri_valid <= (COARSE_SCAN != 0);
				else
					coarse_tri_valid <= 1'b0;
			end
		end
		S_SEQ: begin
			coarse_tri_valid <= 1'b0;
			coarse_tile_ready <= 1'b0;
		end
		S_TILE_WAIT: begin
			coarse_tile_ready <= 1'b0;
			if (ctv_fsm) coarse_tile_ready <= 1'b1;
			else if (!coarse_busy) fend <= wm;
		end
		S_WALK: begin
			coarse_tile_ready <= 1'b0;
		end
		S_STEP: begin
			if (r_tile_row_end && r_tile_col_end) begin
				if (cur_last_tile) fend <= wm;
				else if (ctv_fsm) coarse_tile_ready <= 1'b1;
			end
		end
		S_FQ: begin
			coarse_tile_ready <= 1'b0;
		end
		S_FREE: if (!f_busy) begin
			fi <= fi1;
			if (fi < 2'd3 && fl[fi] && !raw) begin
				pt_free <= 1'b1;
				pt_free_idx <= ent[8 * fi +: 8];
			end
		end
		default: ;
		endcase
		end
	end

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			`SET_ST_HI(S_IDLE);
		end else case (st_fsm_8)
		S_IDLE: if (sp_val_ctrl) begin
			if (u16_gt(sp_tri[335:320], sp_tri[303:288]) && u16_gt(sp_tri[351:336], sp_tri[319:304]))
				`SET_ST_HI(S_SEQ)
			else
				`SET_ST_HI(S_FREE)
		end
		S_SEQ: `SET_ST_HI((COARSE_SCAN != 0) ? S_TILE_WAIT : S_WALK)
		S_TILE_WAIT: begin
			if (ctv_fsm) `SET_ST_HI(S_WALK)
			else if (!coarse_busy) `SET_ST_HI(wm ? S_FQ : S_LAST)
		end
		S_WALK: `SET_ST_HI(S_SAMP)
		S_SAMP: `SET_ST_HI(S_EVAL0)
		S_EVAL0: `SET_ST_HI(S_EVAL1)
		S_EVAL1: `SET_ST_HI(S_EVAL2)
		S_EVAL2: `SET_ST_HI(S_EVAL)
		S_EVAL: begin
`ifdef SYNTH_KINTEX7
			if (!r_w_empty_st && wm) `SET_ST_HI(S_FQ)
			else if (!r_w_empty_st) `SET_ST_HI(S_Z)
`else
			if (!w_empty && wm) `SET_ST_HI(S_FQ)
			else if (!w_empty) `SET_ST_HI(S_Z)
`endif
			else `SET_ST_HI(S_STEP)
		end
		S_STEP: begin
			if (r_tile_row_end && r_tile_col_end) begin
				if (cur_last_tile) `SET_ST_HI(wm ? S_FQ : S_LAST)
				else if (ctv_fsm) `SET_ST_HI(S_WALK)
				else `SET_ST_HI(S_TILE_WAIT)
			end else begin
				`SET_ST_HI(S_WALK)
			end
		end
		S_FQ: if (fq_ready) `SET_ST_HI(fend ? S_FREE : S_STEP)
		S_Z: if (zc_0 == 5'd24) `SET_ST_HI(S_EMIT)
		S_EMIT: if (!(pend && !(dx_valid && dx_ready))) `SET_ST_HI(S_STEP)
		S_LAST: if (dx_valid && dx_ready) `SET_ST_HI(S_FREE)
		default: if (!f_busy && (fi == 2'd3)) `SET_ST_HI(S_IDLE)
		endcase
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			`SET_ST_HI(S_IDLE);
		end else case (st_fsm_8)
		S_IDLE: if (sp_val_ctrl) begin
			if (u16_gt(sp_tri[335:320], sp_tri[303:288]) && u16_gt(sp_tri[351:336], sp_tri[319:304]))
				`SET_ST_HI(S_SEQ)
			else
				`SET_ST_HI(S_FREE)
		end
		S_SEQ: `SET_ST_HI((COARSE_SCAN != 0) ? S_TILE_WAIT : S_WALK)
		S_TILE_WAIT: begin
			if (ctv_fsm) `SET_ST_HI(S_WALK)
			else if (!coarse_busy) `SET_ST_HI(wm ? S_FQ : S_LAST)
		end
		S_WALK: `SET_ST_HI(S_SAMP)
		S_SAMP: `SET_ST_HI(S_EVAL0)
		S_EVAL0: `SET_ST_HI(S_EVAL1)
		S_EVAL1: `SET_ST_HI(S_EVAL2)
		S_EVAL2: `SET_ST_HI(S_EVAL)
		S_EVAL: begin
			if (!r_w_empty_st && wm) `SET_ST_HI(S_FQ)
			else if (!r_w_empty_st) `SET_ST_HI(S_Z)
			else `SET_ST_HI(S_STEP)
		end
		S_STEP: begin
			if (r_tile_row_end && r_tile_col_end) begin
				if (cur_last_tile) `SET_ST_HI(wm ? S_FQ : S_LAST)
				else if (ctv_fsm) `SET_ST_HI(S_WALK)
				else `SET_ST_HI(S_TILE_WAIT)
			end else begin
				`SET_ST_HI(S_WALK)
			end
		end
		S_FQ: if (fq_ready) `SET_ST_HI(fend ? S_FREE : S_STEP)
		S_Z: if (zc_0 == 5'd24) `SET_ST_HI(S_EMIT)
		S_EMIT: if (!(pend && !(dx_valid && dx_ready))) `SET_ST_HI(S_STEP)
		S_LAST: if (dx_valid && dx_ready) `SET_ST_HI(S_FREE)
		default: if (!f_busy && (fi == 2'd3)) `SET_ST_HI(S_IDLE)
		endcase
	end
`endif

	always @(posedge clk or posedge rst_n_inv) begin
		if (rst_n_inv) begin
			`SET_ST(S_IDLE);
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			cur_tx <= 16'd0;
			cur_ty <= 16'd0;
			cur_trivial <= 1'b0;
			cur_last_tile <= 1'b0;
			seq <= 16'd0;
			r_esamp_sign <= 48'd0;
`endif
`ifdef SYNTH_KINTEX7
		end else case (st_fsm_main)
`elsif SYNTH_KINTEX_ULTRASCALE
		end else case (st_fsm_main)
`else
		end else case (st)
`endif
		S_IDLE: begin
			if (sp_val_ctrl) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				a8[0] <= {{16{s_e0_a_scan}}, sp_tri[0 +: 24], 8'd0};
				b8[0] <= {{16{s_e0_b_scan}}, sp_tri[24 +: 24], 8'd0};
				er[0] <= er0[0];
				ec[0] <= er0[0];
				a8[1] <= {{16{s_e1_a_scan}}, sp_tri[96 +: 24], 8'd0};
				b8[1] <= {{16{s_e1_b_scan}}, sp_tri[120 +: 24], 8'd0};
				er[1] <= er0[1];
				ec[1] <= er0[1];
				a8[2] <= {{16{s_e2_a_scan}}, sp_tri[192 +: 24], 8'd0};
				b8[2] <= {{16{s_e2_b_scan}}, sp_tri[216 +: 24], 8'd0};
				er[2] <= er0[2];
				ec[2] <= er0[2];
				bx0 <= sp_tri[303:288];
				by0 <= sp_tri[319:304];
				bx1 <= sp_tri[335:320];
				by1 <= sp_tri[351:336];
				{cz, bz, az} <= sp_tri[447:352];
				ent <= sp_tri[759:736];
`endif
				if (u16_gt(sp_tri[335:320], sp_tri[303:288]) && u16_gt(sp_tri[351:336], sp_tri[319:304])) begin
					`SET_ST(S_SEQ);
				end else begin
					`SET_ST(S_FREE);
				end
			end
		end

		S_SEQ: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			seq <= sq_seq;
			ab8[0] <= ab8_comb[0];
			ab8[1] <= ab8_comb[1];
			ab8[2] <= ab8_comb[2];
`endif
			if (COARSE_SCAN != 0) begin
				`SET_ST(S_TILE_WAIT);
			end else begin
				/* Direct bounding box traversal bypassing coarse scan */
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				cur_tx <= {bx0[15:1], 1'b0};
				cur_ty <= {by0[15:1], 1'b0};
				cur_trivial <= 1'b0;
				cur_last_tile <= 1'b1;
				er[0] <= er_direct_init[0]; ec[0] <= er_direct_init[0];
				er[1] <= er_direct_init[1]; ec[1] <= er_direct_init[1];
				er[2] <= er_direct_init[2]; ec[2] <= er_direct_init[2];
`endif
				`SET_ST(S_WALK);
			end
		end

		S_TILE_WAIT: begin
			if (ctv_fsm) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				cur_tx <= coarse_tile_x;
				cur_ty <= coarse_tile_y;
				cur_trivial <= coarse_tile_trivial;
				cur_last_tile <= coarse_tile_last;
				er[0] <= coarse_tile_e0; ec[0] <= coarse_tile_e0;
				er[1] <= coarse_tile_e1; ec[1] <= coarse_tile_e1;
				er[2] <= coarse_tile_e2; ec[2] <= coarse_tile_e2;
`endif
				`SET_ST(S_WALK);
			end else if (!coarse_busy) begin
				/* Traversal of all tiles complete */
				`SET_ST(wm ? S_FQ : S_LAST);
			end
		end

		S_WALK: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			for (k = 0; k < 3; k = k + 1) begin
				r_ec_next[k][23:0] <= ec_lo_w[k];
				r_er_next[k][23:0] <= er_lo_w[k];
				r_ec_c24[k]        <= ec_c24_w[k];
				r_er_c24[k]        <= er_c24_w[k];
			end
`endif
			`SET_ST(S_SAMP);
		end

		S_SAMP: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			for (k = 0; k < 3; k = k + 1) begin
				r_ec_next[k][47:24] <= ec_hi_w[k];
				r_er_next[k][47:24] <= er_hi_w[k];
			end
`endif
			`SET_ST(S_EVAL0);
		end

		S_EVAL0: begin
			`SET_ST(S_EVAL1);
		end

		S_EVAL1: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_esamp_sign <= esamp_sign;
`endif
			`SET_ST(S_EVAL2);
		end

		S_EVAL2: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_quad_cov_ms4 <= quad_cov_asic;
			r_w_empty_lve  <= w_empty_edge_lve;
`endif
			`SET_ST(S_EVAL);
		end

		S_EVAL: begin
`ifdef SYNTH_KINTEX7
			if (!r_w_empty_st && wm) begin
				`SET_ST(S_FQ);
			end else if (!r_w_empty_st) begin
				`SET_ST(S_Z);
			end else begin
				`SET_ST(S_STEP);
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (!r_w_empty_st && wm) begin
				`SET_ST(S_FQ);
			end else if (!r_w_empty_st) begin
				`SET_ST(S_Z);
			end else begin
				`SET_ST(S_STEP);
			end
`else
			if (!w_empty && wm) begin
				`SET_ST(S_FQ);
			end else if (!w_empty) begin
				`SET_ST(S_Z);
			end else begin
				`SET_ST(S_STEP);
			end
`endif
		end

		S_STEP: begin
			if (r_tile_row_end && r_tile_col_end) begin
				/* End of quads in this coarse tile */
				if (cur_last_tile) begin
					`SET_ST(wm ? S_FQ : S_LAST);
				end else if (ctv_fsm) begin
					/* Immediate switch to next pre-fetched tile */
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					cur_tx <= coarse_tile_x;
					cur_ty <= coarse_tile_y;
					cur_trivial <= coarse_tile_trivial;
					cur_last_tile <= coarse_tile_last;
					er[0] <= coarse_tile_e0; ec[0] <= coarse_tile_e0;
					er[1] <= coarse_tile_e1; ec[1] <= coarse_tile_e1;
					er[2] <= coarse_tile_e2; ec[2] <= coarse_tile_e2;
`endif
					`SET_ST(S_WALK);
				end else begin
					`SET_ST(S_TILE_WAIT);
				end
			end else begin
				/* Step to next quad within tile */
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				if (r_tile_row_end) begin
					for (k = 0; k < 3; k = k + 1) begin
						er[k] <= r_er_next[k];
						ec[k] <= r_er_next[k];
					end
				end else begin
					for (k = 0; k < 3; k = k + 1)
						ec[k] <= r_ec_next[k];
				end
`endif
				`SET_ST(S_WALK);
			end
		end

		S_FQ: begin
			if (fq_ready) begin
				if (fend) begin
					`SET_ST(S_FREE);
				end else begin
					`SET_ST(S_STEP);
				end
			end
		end

		S_Z: begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			if (zc == 5'd0) begin
				zv <= 128'd0;
				r_dx0 <= dx0_comb;
				r_dy0 <= dy0_comb;
				r_dx1 <= dx1_comb;
				r_dy1 <= dy1_comb;
			end
			if (zc == 5'd1) begin
				{r_z_is0_0, r_z_lz0_a}   <= lzc_dx0;
				{r_z_is0_0_b_unused, r_z_lz0_b}   <= lzc_dx0;
				{r_z_is0_0_e_unused, r_z_lz0_exp} <= lzc_dx0;
				{r_z_is0_1, r_z_lz1} <= lzc_dy0;
				r_z_lz1_b   <= lzc_dy0[4:0];
				r_z_lz1_exp <= lzc_dy0[4:0];
				{r_z_is0_2, r_z_lz2} <= lzc_dx1;
				r_z_lz2_b   <= lzc_dx1[4:0];
				r_z_lz2_exp <= lzc_dx1[4:0];
				{r_z_is0_3, r_z_lz3} <= lzc_dy1;
				r_z_lz3_b   <= lzc_dy1[4:0];
				r_z_lz3_exp <= lzc_dy1[4:0];
			end
			if (zc == 5'd2) begin
				r_fdx0 <= f_dx0_comb;
				r_fdy0 <= f_dy0_comb;
				r_fdx1 <= f_dx1_comb;
				r_fdy1 <= f_dy1_comb;
			end
			if (zc == 5'd7)  t1_0 <= fm_r;
			if (zc == 5'd8)  t1_1 <= fm_r;
			if (zc == 5'd9)  t1_2 <= fm_r;
			if (zc == 5'd10) t1_3 <= fm_r;
			if (zc == 5'd21 && zmask[0]) zv[31:0] <= zcl;
			if (zc == 5'd22 && zmask[1]) zv[63:32] <= zcl;
			if (zc == 5'd23 && zmask[2]) zv[95:64] <= zcl;
			if (zc == 5'd24 && zmask[3]) zv[127:96] <= zcl;
`endif
`ifdef SYNTH_KINTEX7
			if (zc_0 == 5'd24)
`elsif SYNTH_KINTEX_ULTRASCALE
			if (zc_0 == 5'd24)
`else
			if (zc == 5'd24)
`endif
				`SET_ST(S_EMIT);
		end

		S_EMIT: begin
			if (!(pend && !(dx_valid && dx_ready))) begin
				`SET_ST(S_STEP);
			end
		end

		S_LAST: begin
			if (dx_valid && dx_ready) begin
				`SET_ST(S_FREE);
			end
		end

		default: if (!f_busy && (fi == 2'd3)) begin	/* S_FREE */
			`SET_ST(S_IDLE);
		end
		endcase
	end

	wire unused_ok = &{1'b0, st_wdata[31:4], sp_last, fm_o, fa_o,
		sp_tri[767:765], sp_tri[735:448], qx1[16], qy1[16], qx2[16], qy2[16],
		HAS_SU != 0, coarse_tri_ready, cur_last_tile, eb_cov_unused,
		esamp_carry_unused, esamp_unused, lve_carry_unused, qstep_carry_unused, tend_carry_unused,
		dxy_carry_unused, r_zl_co_unused, ab8_carry_unused, ed_carry_unused, inb_d_unused, tre_d_unused,
		hiz_cull, hiz_rd_zmax, coarse_tri_valid, coarse_tile_ready, cur_ty, cov_op, cov_oq,
		exp_z_bo_u,
		lve_comb, lve_lo_w, lve_hi_w, ec_n[0], ec_n[1], ec_n[2], er_n[0], er_n[1], er_n[2],
		ec_lo_w[0], ec_lo_w[1], ec_lo_w[2], er_lo_w[0], er_lo_w[1], er_lo_w[2],
		ec_hi_w[0], ec_hi_w[1], ec_hi_w[2], er_hi_w[0], er_hi_w[1], er_hi_w[2],
		ec_c24_w, er_c24_w, r_ec_next[0], r_ec_next[1], r_ec_next[2],
		r_er_next[0], r_er_next[1], r_er_next[2], r_ec_c24, r_er_c24, r_lve_c24,
		r_z_is0_0_b_unused, r_z_is0_0_e_unused};
endmodule

`default_nettype wire
