/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_exp: the fragment export buffer: exp of the colour attachments
 * 0-3 (0x100 + 16 k + c, CAPS[30]) and DEPTH (0x104) by fragment warps
 * into the warp's buffer, drained to the ROP after the warp's end, in
 * sequence-number order, by atg_su_exp_drain (one beat per exported
 * attachment of each quad); the slot release (xr_*) of every ended warp.
 * The PT and its allocator are atg_su_pt.
 *
 * The warp's quads come from the RX's register writes (fl_*): r3 (z) of
 * every lane is the depth unless the shader exports 0x104, r4/r5 (the
 * pixel centre, x + 0.5 and y + 0.5) of lane 4q give quad q's position;
 * fl_go gives the coverage and the sequence number (fl_info).  A quad of
 * the warp is one with coverage (RX packs the quads from lane 0).
 * ex_mask = the covered lanes that some fragment exp wrote under the
 * side-effect mask (EXEC & coverage & ~KILLED).  Storage: NPHYS lane
 * banks × 18 words (R, G, B, A of 4 attachments, depth, z) × NWARPS×32/NPHYS
 * rows, written one word per bank per cycle, read asynchronously (a
 * quad's 4 lanes are in 4 banks of one row).  The warp's attachments
 * (rtw) are those some fragment exp of a colour wrote under a non-empty
 * side-effect mask; every beat of a quad carries the same ex_mask.
 *
 * Multisampling (docs/ghci.md section 12.11): the RX's r6 (the lane's
 * samples after the sample mask test) is snooped like r3 (a lane the RX
 * did not write takes 1111); exp 0x105 (SAMPLE_MASK) keeps the last value
 * written per lane; the quad's sample mask is r6 AND that value (1111 when
 * none), lane l at ex_aux bits 4 l + s.  A lane whose mask is 0 does not go
 * out (ex_mask).  ex_aux also carries the lanes that exported DEPTH and the
 * warp's d_s (the RX's table words 0xFF8-0xFFB, one per sample; 0 for a
 * warp without them: the slot's copy is cleared at its release).
 * ex_aux = {rt[1:0], qlast, qfirst, nocol, front, d_s, zexp, smask}: the
 * beat's attachment, its place in the quad, no colour, the warp's facing.
 * scrub (session protection, the unit idle) writes 0 to every row of every
 * bank through each array's write port, one row per cycle, while
 * scrub_busy (a component a fragment does not export is not another
 * session's).
 * See spec/gpu/su/atg_su_exp.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_exp #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter LDS_BYTES = 4096,	/* Shared memory bytes */
	parameter LDS_AW = $clog2(LDS_BYTES / 4),
`ifdef CFG_ENABLE_MESH
	parameter HAS_MESH = (`CFG_ENABLE_MESH != 0),
`else
	parameter HAS_MESH = 0,
`endif
	parameter MRT_COUNT = 8,	/* Colour attachments supported (1..8) */
	parameter DUAL_SOURCE = 1	/* Dual-source color1 export support (0/1) */
) (
	input wire clk,
	input wire rst_n,
	/* Issued instruction, one lane group per cycle (from atg_su_opd, shared by all units) */
	input wire x_valid,
	input wire [WARP_W-1:0] x_warp,
	input wire [GRP_W-1:0] x_grp,
	input wire x_last,
	input wire [SBE_W:0] x_sbe,
	input wire [15:0] x_pc,
	input wire [UOP_W-1:0] x_uop,
	input wire [NPHYS*32-1:0] x_a,
	input wire [NPHYS*32-1:0] x_b,
	input wire [NPHYS*32-1:0] x_c,
	input wire [2*NPHYS-1:0] x_mask,
	input wire [31:0] x_info,
	output wire x_ready,
	/* Export to the ROP (atg_rop) */
	output wire ex_valid,
	input wire ex_ready,
	output wire [15:0] ex_seq,
	output wire [31:0] ex_xy,
	output wire [3:0] ex_mask,
	output wire [511:0] ex_color,
	output wire [511:0] ex_color1,
	output wire ex_has_src1,
	output wire [127:0] ex_depth,
	output wire ex_last,
	input wire [15:0] ex_next,
	/* Fragment warps from the RX (snooped: atg_su fl_*) */
	input wire fl_we,
	input wire [WARP_W-1:0] fl_slot,
	input wire [5:0] fl_reg,
	input wire [4:0] fl_lane,
	input wire [31:0] fl_data,
	input wire fl_go,
	input wire [63:0] fl_info,
	input wire fl_pw_we,
	input wire [11:0] fl_pw_addr,
	input wire [31:0] fl_pw_data,
	/* The beat's attachment and multisampling to the ROP */
	output wire [154:0] ex_aux,	/* {rt, qlast, qfirst, nocol, front, d_s, zexp, smask} */
	/* Slot release (atg_su_sched) */
	output wire xr_valid,
	output wire [WARP_W-1:0] xr_warp,
	input wire wc_end,
	input wire [WARP_W-1:0] wc_warp,
	/* Session protection: clear the buffer */
	input wire scrub,
	output wire scrub_busy,
	/* Meshlet Primitive Export to RX (atg_rx_setup) */
	output wire mesh_valid,
	input wire mesh_ready,
	output wire [701:0] mesh_tri,
	output wire mesh_last,
	output wire [15:0] mesh_id,
	output wire mesh_busy,
	/* LDS Port B Read Interface */
	output wire mesh_lds_rd_en,
	output wire [LDS_AW-1:0] mesh_lds_rd_addr,
	input wire [31:0] mesh_lds_rd_data
);
`include "atg_su_uop.vh"

	wire [WARP_W-1:0] fl_slot_top, fl_slot_gw_in;
`ifdef SYNTH_KINTEX7
	genvar gfs;
	generate
		for (gfs = 0; gfs < WARP_W; gfs = gfs + 1) begin : g_fls_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flst (.I0(fl_slot[gfs]), .O(fl_slot_top[gfs]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flsg (.I0(fl_slot[gfs]), .O(fl_slot_gw_in[gfs]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gfs;
	generate
		for (gfs = 0; gfs < WARP_W; gfs = gfs + 1) begin : g_fls_buf
			assign fl_slot_top[gfs] = fl_slot[gfs];
			assign fl_slot_gw_in[gfs] = fl_slot[gfs];
		end
	endgenerate
`else
	assign fl_slot_top   = fl_slot;
	assign fl_slot_gw_in = fl_slot;
`endif


	localparam G = 32 / NPHYS;
	localparam ROWS = NWARPS * G;
	localparam RW = ROWS > 1 ? $clog2(ROWS) : 1;
	localparam G_W = $clog2(G);	/* G is a power of two: warp G + group are disjoint bits */
	localparam RT_SLOTS = (MRT_COUNT <= 1) ? 1 : ((MRT_COUNT <= 2) ? 2 : ((MRT_COUNT <= 4) ? 4 : 8));
	localparam RT_AW = (RT_SLOTS == 1) ? 0 : ((RT_SLOTS == 2) ? 1 : ((RT_SLOTS == 4) ? 2 : 3));
	localparam C_DEPTH = ROWS * RT_SLOTS;
	localparam CAW = RW + RT_AW;
	localparam [2:0] RT_MAX = (MRT_COUNT >= 8) ? 3'd7 : ((MRT_COUNT <= 1) ? 3'd0 : (MRT_COUNT - 1));

	/* ---- Per warp. */
	reg [NWARPS-1:0] fr;	/* the warp's facing (fl_info[48]) */
	reg [NWARPS*8-1:0] rtw;	/* the attachments exported (8 attachments) */
	wire [NWARPS-1:0] has_src1_w;	/* whether the warp exported src1 (COLOR1) */
	wire [NWARPS*32-1:0] cov, xp, dw;	/* coverage, exported lanes, depth written */
	wire [NWARPS*32-1:0] smw, r6w;	/* SAMPLE_MASK exported, r6 written by the RX */
	wire [NWARPS*128-1:0] dsv;	/* the warp's d_s */
	wire [NWARPS*8*16-1:0] qx, qy;

	/*
	 * 1-cycle input pipeline stage on x_* to isolate OPD -> EXP long route from LUTRAM write ports.
	 * Timing invariant (0-cycle margin):
	 *   - With r_x_*, LUTRAM/rtw/has_src1_r/mesh writes take effect 2 cycles after exp issues.
	 *   - wc_end arrives from atg_su_exec's registered wc_v 1 cycle after END issues.
	 *   - Single-issue from opd guarantees the last exp issues >= 1 cycle before END, so its
	 *     writes complete by wc_end before drain starts reading. Dual-issue or an earlier wc_end
	 *     would violate this ordering and require delaying wc_end (or bypassing r_x_*).
	 */
	wire rst_exp_p = !rst_n;
	reg r_x_valid;
	reg [WARP_W-1:0] r_x_warp;
	reg [GRP_W-1:0] r_x_grp;
	reg [3:0] r_x_unit;
	reg [31:0] r_x_tgt;
	reg [NPHYS-1:0] r_x_side;
	reg [17:0] r_x_info;
	(* keep = "true" *) reg [NPHYS*32-1:0] r_x_b;
	always @(posedge clk or posedge rst_exp_p) begin
		if (rst_exp_p) begin
			r_x_valid <= 1'b0;
			r_x_warp  <= {WARP_W{1'b0}};
			r_x_grp   <= {GRP_W{1'b0}};
			r_x_unit  <= 4'd0;
			r_x_tgt   <= 32'd0;
			r_x_side  <= {NPHYS{1'b0}};
			r_x_info  <= 18'd0;
		end else begin
			r_x_valid <= x_valid;
			r_x_warp  <= x_warp;
			r_x_grp   <= x_grp;
			r_x_unit  <= x_uop[ATG_UOP_UNIT +: 4];
			r_x_tgt   <= x_uop[ATG_UOP_IMM +: 32];
			r_x_side  <= x_mask[2*NPHYS-1:NPHYS];
			r_x_info  <= x_info[17:0];
		end
	end
	always @(posedge clk) begin
		r_x_b <= x_b;
	end

	/* ---- Exports of fragment warps: colour k component c at 0x100 + 16 k + c;
	 * 0x108 + c is the second colour (src1) of attachment 0 for dual-source blending. */
	wire [31:0] tgt = r_x_tgt;
	wire ex_frag = r_x_valid && r_x_unit == ATG_UNIT_EXP && r_x_info[17:16] == 2'd1;
	wire [2:0] tk = tgt[6:4];
	wire ex_col  = ex_frag && tgt[31:7] == 25'd2 && tgt[3:2] == 2'd0 &&
		((MRT_COUNT >= 8) || (tk <= RT_MAX));
	wire ex_col1 = (DUAL_SOURCE != 0) && ex_frag && tgt[31:4] == 28'h010 && tgt[3:2] == 2'b10;
	wire ex_dep  = ex_frag && tgt == 32'h104;
	wire ex_sm   = ex_frag && tgt == 32'h105;
	wire ex_op   = ex_col || ex_dep || ex_sm;
	wire [2:0] tk_xcrow, tk_rtw;
	genvar gtk;
	generate
		for (gtk = 0; gtk < 3; gtk = gtk + 1) begin : g_tk_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tkxc (.I0(tk[gtk]), .O(tk_xcrow[gtk]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tkrt (.I0(tk[gtk]), .O(tk_rtw[gtk]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign tk_xcrow[gtk] = tk[gtk];
			assign tk_rtw[gtk] = tk[gtk];
`else
			assign tk_xcrow[gtk] = tk[gtk];
			assign tk_rtw[gtk]   = tk[gtk];
`endif
		end
	endgenerate
	wire [1:0] comp = tgt[1:0];

	wire [WARP_W-1:0] x_warp_row;
	wire [WARP_W-1:0] x_warp_ctrl;
	wire [WARP_W-1:0] x_warp_rtw;
	genvar gxwb;
	generate
		for (gxwb = 0; gxwb < WARP_W; gxwb = gxwb + 1) begin : g_xwarp_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xwr (.I0(r_x_warp[gxwb]), .O(x_warp_row[gxwb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xwc (.I0(r_x_warp[gxwb]), .O(x_warp_ctrl[gxwb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xwt (.I0(r_x_warp[gxwb]), .O(x_warp_rtw[gxwb]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign x_warp_row[gxwb] = r_x_warp[gxwb];
			assign x_warp_ctrl[gxwb] = r_x_warp[gxwb];
			assign x_warp_rtw[gxwb] = r_x_warp[gxwb];
`else
			assign x_warp_row[gxwb]  = r_x_warp[gxwb];
			assign x_warp_ctrl[gxwb] = r_x_warp[gxwb];
			assign x_warp_rtw[gxwb]  = r_x_warp[gxwb];
`endif
		end
	endgenerate

	wire [GRP_W-1:0] x_grp_row;
	wire [GRP_W-1:0] x_grp_sw0;
	wire [GRP_W-1:0] x_grp_sw1;
	genvar gxgb;
	generate
		for (gxgb = 0; gxgb < GRP_W; gxgb = gxgb + 1) begin : g_xgrp_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xg_row (.I0(r_x_grp[gxgb]), .O(x_grp_row[gxgb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xg_sw0 (.I0(r_x_grp[gxgb]), .O(x_grp_sw0[gxgb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xg_sw1 (.I0(r_x_grp[gxgb]), .O(x_grp_sw1[gxgb]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign x_grp_row[gxgb] = r_x_grp[gxgb];
			assign x_grp_sw0[gxgb] = r_x_grp[gxgb];
			assign x_grp_sw1[gxgb] = r_x_grp[gxgb];
`else
			assign x_grp_row[gxgb] = r_x_grp[gxgb];
			assign x_grp_sw0[gxgb] = r_x_grp[gxgb];
			assign x_grp_sw1[gxgb] = r_x_grp[gxgb];
`endif
		end
	endgenerate

	wire [31:0] xrow = ({{(32-WARP_W){1'b0}}, x_warp_row} << G_W) | ({{(32-GRP_W){1'b0}}, x_grp_row} % G);
	wire [NPHYS-1:0] side = r_x_side;
	wire [NPHYS-1:0] side_shf;
	wire [NPHYS-1:0] side_cmp;
	wire [NPHYS-1:0] side_bnk;
	genvar gsb;
	generate
		for (gsb = 0; gsb < NPHYS; gsb = gsb + 1) begin : g_side_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sshf (.I0(side[gsb]), .O(side_shf[gsb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_scmp (.I0(side[gsb]), .O(side_cmp[gsb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sbnk (.I0(side[gsb]), .O(side_bnk[gsb]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign side_shf[gsb] = side[gsb];
			assign side_cmp[gsb] = side[gsb];
			assign side_bnk[gsb] = side[gsb];
`else
			assign side_shf[gsb] = side[gsb];
			assign side_cmp[gsb] = side[gsb];
			assign side_bnk[gsb] = side[gsb];
`endif
		end
	endgenerate
	wire [31:0] lane0 = {{(32-GRP_W){1'b0}}, x_grp_row} * NPHYS;
	wire [31:0] side_w_raw, side_w;
	genvar gg;
	generate
		if (G == 1) begin : g_sw_g1
			assign side_w_raw = side_shf;
		end else begin : g_sw_gn
			for (gg = 0; gg < G; gg = gg + 1) begin : g_sw_loop
				wire [GRP_W-1:0] xg_sw = (gg < G / 2) ? x_grp_sw0 : x_grp_sw1;
				assign side_w_raw[gg * NPHYS +: NPHYS] = (xg_sw == gg[GRP_W-1:0]) ? side_shf : {NPHYS{1'b0}};
			end
		end
		for (gg = 0; gg < 32; gg = gg + 1) begin : g_sw_out
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sw_o (.I0(side_w_raw[gg]), .O(side_w[gg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign side_w[gg] = side_w_raw[gg];
`else
			assign side_w[gg] = side_w_raw[gg];
`endif
		end
	endgenerate
	wire [4:0] fl_lane_z;
	genvar gzln;
	generate
		for (gzln = 0; gzln < 5; gzln = gzln + 1) begin : g_zlane_buf

			assign fl_lane_z[gzln] = fl_lane[gzln];

		end
	endgenerate
	wire [31:0] zrow = ({{(32-WARP_W){1'b0}}, fl_slot_top} << G_W) | ({27'd0, fl_lane_z} / NPHYS);
	wire [31:0] zbank = {27'd0, fl_lane_z} % NPHYS;
	wire [31:0] fq = {{(25-WARP_W){1'b0}}, fl_slot_top, fl_lane_z[4:2], 4'd0};

	/* ---- The drain (atg_su_exp_drain): the beat of warp dwarp, quad dq, attachment drt. */
	wire [WARP_W-1:0] dwarp;
	wire [2:0] dq;
	wire [2:0] drt;
	wire [44*WARP_W-1:0] dwarp_rep;
	wire [26*3-1:0] dq_rep;
	wire dfirst, dlastb, dnocol;
	wire [31:0] dcov, dxp_raw, ddw, dsm, dr6;
	genvar gwm;
	generate
		for (gwm = 0; gwm < 2; gwm = gwm + 1) begin : g_wmux_sl
			wire [WARP_W-1:0] dw_cov_s;
			wire [WARP_W-1:0] dw_xp_s;
			wire [WARP_W-1:0] dw_dw_s;
			wire [WARP_W-1:0] dw_sm_s;
			wire [WARP_W-1:0] dw_r6_s;
			genvar gb;
			for (gb = 0; gb < WARP_W; gb = gb + 1) begin : g_wm_buf

				assign dw_cov_s[gb] = dwarp_rep[(16 + gwm*5 + 0)*WARP_W + gb];
				assign dw_xp_s[gb]  = dwarp_rep[(16 + gwm*5 + 1)*WARP_W + gb];
				assign dw_dw_s[gb]  = dwarp_rep[(16 + gwm*5 + 2)*WARP_W + gb];
				assign dw_sm_s[gb]  = dwarp_rep[(16 + gwm*5 + 3)*WARP_W + gb];
				assign dw_r6_s[gb]  = dwarp_rep[(16 + gwm*5 + 4)*WARP_W + gb];

			end
			assign dcov[gwm*16 +: 16]    = cov[dw_cov_s*32 + gwm*16 +: 16];
			assign dxp_raw[gwm*16 +: 16] = xp[dw_xp_s*32 + gwm*16 +: 16];
			assign ddw[gwm*16 +: 16]     = dw[dw_dw_s*32 + gwm*16 +: 16];
			assign dsm[gwm*16 +: 16]     = smw[dw_sm_s*32 + gwm*16 +: 16];
			assign dr6[gwm*16 +: 16]     = r6w[dw_r6_s*32 + gwm*16 +: 16];
		end
	endgenerate
	wire [WARP_W-1:0] dw_rtw;
	genvar gbr;
	generate
		for (gbr = 0; gbr < WARP_W; gbr = gbr + 1) begin : g_dw_rtw_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dwrtw (.I0(dwarp_rep[26*WARP_W + gbr]), .O(dw_rtw[gbr]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign dw_rtw[gbr] = dwarp_rep[26*WARP_W + gbr];
`else
			assign dw_rtw[gbr] = dwarp_rep[26*WARP_W + gbr];
`endif
		end
	endgenerate
	atg_su_exp_drain #(.NWARPS(NWARPS), .WARP_W(WARP_W)) drain (
		.clk(clk), .rst_n(rst_n), .fl_go(fl_go), .fl_slot(fl_slot_top), .fl_seq(fl_info[47:32]),
		.wc_end(wc_end), .wc_warp(wc_warp), .d_cov(dcov), .d_rtw(rtw[dw_rtw*8 +: 8]),
		.d_warp(dwarp), .d_q(dq), .d_rt(drt), .d_warp_rep(dwarp_rep), .d_q_rep(dq_rep),
		.d_first(dfirst), .d_lastb(dlastb),
		.d_nocol(dnocol), .ex_valid(ex_valid), .ex_ready(ex_ready), .ex_seq(ex_seq),
		.ex_last(ex_last), .ex_next(ex_next), .xr_valid(xr_valid), .xr_warp(xr_warp)
	);

	wire [2:0] dq_dl, dq_dl_dqs, dq_dl_rsm, dq_dl_zsm;
	wire [2:0] dq_dl_zsm_0, dq_dl_zsm_1;
`ifdef SYNTH_KINTEX7
	genvar gdqdl;
	generate
		for (gdqdl = 0; gdqdl < 3; gdqdl = gdqdl + 1) begin : g_dqdl_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dqdl0 (.I0(dq_rep[4*3 + gdqdl]), .O(dq_dl[gdqdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dqdl1 (.I0(dq_rep[4*3 + gdqdl]), .O(dq_dl_dqs[gdqdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dqdl2 (.I0(dq_rep[5*3 + gdqdl]), .O(dq_dl_rsm[gdqdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dqdl3 (.I0(dq_rep[5*3 + gdqdl]), .O(dq_dl_zsm_0[gdqdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dqdl4 (.I0(dq_rep[5*3 + gdqdl]), .O(dq_dl_zsm_1[gdqdl]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gdqdl;
	generate
		for (gdqdl = 0; gdqdl < 3; gdqdl = gdqdl + 1) begin : g_dqdl_buf
			assign dq_dl[gdqdl] = dq_rep[4*3 + gdqdl];
			assign dq_dl_dqs[gdqdl] = dq_rep[4*3 + gdqdl];
			assign dq_dl_rsm[gdqdl] = dq_rep[5*3 + gdqdl];
			assign dq_dl_zsm_0[gdqdl] = dq_rep[5*3 + gdqdl];
			assign dq_dl_zsm_1[gdqdl] = dq_rep[5*3 + gdqdl];
		end
	endgenerate
`else
	assign dq_dl       = dq_rep[4*3 +: 3];
	assign dq_dl_dqs   = dq_rep[4*3 +: 3];
	assign dq_dl_rsm   = dq_rep[5*3 +: 3];
	assign dq_dl_zsm_0 = dq_rep[5*3 +: 3];
	assign dq_dl_zsm_1 = dq_rep[5*3 +: 3];
`endif

	wire [4:0] dl0       = {dq_dl, 2'b00};
	wire [4:0] dl0_rsm   = {dq_dl_rsm, 2'b00};
	wire [4:0] dl0_zsm_0 = {dq_dl_zsm_0, 2'b00};
	wire [4:0] dl0_zsm_1 = {dq_dl_zsm_1, 2'b00};
	wire [31:0] drow = {{(32-WARP_W){1'b0}}, dwarp_rep[26*WARP_W +: WARP_W]} * G + {27'd0, dl0} / NPHYS;
	wire [CAW-1:0] dcrow, xcrow;
	generate
		if (RT_AW == 0) begin : g_crow_single
			assign dcrow = drow[RW-1:0];
			assign xcrow = xrow[RW-1:0];
		end else begin : g_crow_mrt
			assign dcrow = {drow[RW-1:0], drt[RT_AW-1:0]};
			assign xcrow = {xrow[RW-1:0], tk_xcrow[RT_AW-1:0]};
		end
	endgenerate
	wire [31:0] dqs = ({{(32-WARP_W){1'b0}}, dwarp_rep[26*WARP_W +: WARP_W]} * 8 + {29'd0, dq_dl_dqs}) * 16;
	wire [31:0] dxp = dxp_raw & dcov;

	wire [3:0] col_comp_match;
	wire [3:0] col1_comp_match;
	genvar gcm;
	generate
		for (gcm = 0; gcm < 4; gcm = gcm + 1) begin : g_comp_match
			assign col_comp_match[gcm]  = ex_col  && (comp == gcm[1:0]);
			assign col1_comp_match[gcm] = ex_col1 && (comp == gcm[1:0]);
		end
	endgenerate

	wire [3:0] col_match_q [0:1];
	wire [3:0] col1_match_q [0:1];
	wire dep_q [0:1];
	wire sm_q [0:1];
	wire [CAW-1:0] xcrow_q [0:1];
	wire [RW-1:0] xrow_q [0:1];
	genvar gmq, gmc, gq_bit;
	generate
		for (gmq = 0; gmq < 2; gmq = gmq + 1) begin : g_match_q
			for (gmc = 0; gmc < 4; gmc = gmc + 1) begin : g_match_c
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cm  (.I0(col_comp_match[gmc]),  .O(col_match_q[gmq][gmc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cm1 (.I0(col1_comp_match[gmc]), .O(col1_match_q[gmq][gmc]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign col_match_q[gmq][gmc] = col_comp_match[gmc];
				assign col1_match_q[gmq][gmc] = col1_comp_match[gmc];
`else
				assign col_match_q[gmq][gmc]  = col_comp_match[gmc];
				assign col1_match_q[gmq][gmc] = col1_comp_match[gmc];
`endif
			end
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dpq (.I0(ex_dep), .O(dep_q[gmq]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_smq (.I0(ex_sm),  .O(sm_q[gmq]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign dep_q[gmq] = ex_dep;
			assign sm_q[gmq] = ex_sm;
`else
			assign dep_q[gmq] = ex_dep;
			assign sm_q[gmq]  = ex_sm;
`endif
			for (gq_bit = 0; gq_bit < CAW; gq_bit = gq_bit + 1) begin : g_quad_xcbit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qxc (.I0(xcrow[gq_bit]), .O(xcrow_q[gmq][gq_bit]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign xcrow_q[gmq][gq_bit] = xcrow[gq_bit];
`else
				assign xcrow_q[gmq][gq_bit] = xcrow[gq_bit];
`endif
			end
			for (gq_bit = 0; gq_bit < RW; gq_bit = gq_bit + 1) begin : g_quad_xrbit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qxr (.I0(xrow[gq_bit]), .O(xrow_q[gmq][gq_bit]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign xrow_q[gmq][gq_bit] = xrow[gq_bit];
`else
				assign xrow_q[gmq][gq_bit] = xrow[gq_bit];
`endif
			end
		end
	endgenerate

	/* The scrub: rows of the attachments' arrays; the others take its row / RT_SLOTS. */
	wire sc_on;
	wire [CAW-1:0] sc_a;

	atg_su_scrub #(.DEPTH(C_DEPTH), .AW(CAW)) sc (
		.clk(clk), .rst_n(rst_n), .start(scrub), .on(sc_on), .a(sc_a)
	);

	assign scrub_busy = sc_on;

	/* NPHYS lane banks × {R, G, B, A of RT_SLOTS attachments, depth, z} and {SAMPLE_MASK, r6}. */
	wire [NPHYS*192-1:0] rdat;
	wire [NPHYS*128-1:0] rdat1;
	wire [NPHYS*8-1:0] rsm;
	genvar b, bs, bh;
	generate
	wire [NPHYS-1:0] sc_on_bnk;
	genvar gscb;
	for (gscb = 0; gscb < NPHYS; gscb = gscb + 1) begin : g_sc_buf
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sconb (.I0(sc_on), .O(sc_on_bnk[gscb]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sc_on_bnk[gscb] = sc_on;
`else
		assign sc_on_bnk[gscb] = sc_on;
`endif
	end

	wire [CAW-1:0] sc_a_pair [0:3];
	genvar gsp, gsa_b;
	for (gsp = 0; gsp < 4; gsp = gsp + 1) begin : g_sca_pair
		for (gsa_b = 0; gsa_b < CAW; gsa_b = gsa_b + 1) begin : g_sca_pb
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_scap (.I0(sc_a[gsa_b]), .O(sc_a_pair[gsp][gsa_b]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign sc_a_pair[gsp][gsa_b] = sc_a[gsa_b];
`else
			assign sc_a_pair[gsp][gsa_b] = sc_a[gsa_b];
`endif
		end
	end

	wire [CAW-1:0] dcrow_pair [0:NPHYS-1];
	wire [RW-1:0] drow_pair [0:NPHYS-1];
	localparam integer N_DRTP = (NPHYS + 3) / 4;
	wire [2:0] drt_pair [0:N_DRTP-1];
	genvar gdp, gdpr;
	for (gdpr = 0; gdpr < N_DRTP; gdpr = gdpr + 1) begin : g_drt_pair
		genvar gdrb;
		for (gdrb = 0; gdrb < 3; gdrb = gdrb + 1) begin : g_drt_b
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_drtp (.I0(drt[gdrb]), .O(drt_pair[gdpr][gdrb]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign drt_pair[gdpr][gdrb] = drt[gdrb];
`else
			assign drt_pair[gdpr][gdrb] = drt[gdrb];
`endif
		end
	end
	for (gdp = 0; gdp < NPHYS; gdp = gdp + 1) begin : g_dr_pair
		wire [WARP_W-1:0] dw_b = dwarp_rep[gdp*WARP_W +: WARP_W];
		wire [2:0] dq_b = dq_rep[(gdp / 4)*3 +: 3];
		wire [31:0] drow_b = {{(32-WARP_W){1'b0}}, dw_b} * G + {27'd0, dq_b, 2'b00} / NPHYS;
		assign drow_pair[gdp] = drow_b[RW-1:0];
		if (RT_AW == 0) begin : g_dcr_s
			assign dcrow_pair[gdp] = drow_b[RW-1:0];
		end else begin : g_dcr_m
			assign dcrow_pair[gdp] = {drow_b[RW-1:0], drt_pair[gdp / 4][RT_AW-1:0]};
		end
	end

	wire [RW-1:0] zrow_g [0:NPHYS-1];
	wire [NPHYS-1:0] fl_mr6_en_g;
	wire fl_mr6_en_raw = fl_we && (fl_reg == 6'd6);
	genvar gzg, gzb;
	for (gzg = 0; gzg < NPHYS; gzg = gzg + 1) begin : g_zrow_g
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fm6g (.I0(fl_mr6_en_raw), .O(fl_mr6_en_g[gzg]));
		for (gzb = 0; gzb < RW; gzb = gzb + 1) begin : g_zrow_gb
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_zrg (.I0(zrow[gzb]), .O(zrow_g[gzg][gzb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		assign fl_mr6_en_g[gzg] = fl_mr6_en_raw;
		for (gzb = 0; gzb < RW; gzb = gzb + 1) begin : g_zrow_gb
			assign zrow_g[gzg][gzb] = zrow[gzb];
		end
`else
		assign fl_mr6_en_g[gzg] = fl_mr6_en_raw;
		assign zrow_g[gzg]      = zrow[RW-1:0];
`endif
	end

	for (b = 0; b < NPHYS; b = b + 1) begin : g_bank
		wire b_sc_on_c0_we, b_sc_on_c0_sl;
		wire b_sc_on_c1_we, b_sc_on_c1_sl;
		wire b_sc_on_r0_we, b_sc_on_r1_we;
		wire [3:0] b_sc_on_r_sl;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_c0w (.I0(sc_on_bnk[b]), .O(b_sc_on_c0_we));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_c0s (.I0(sc_on_bnk[b]), .O(b_sc_on_c0_sl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_c1w (.I0(sc_on_bnk[b]), .O(b_sc_on_c1_we));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_c1s (.I0(sc_on_bnk[b]), .O(b_sc_on_c1_sl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_r0w (.I0(sc_on_bnk[b]), .O(b_sc_on_r0_we));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_r1w (.I0(sc_on_bnk[b]), .O(b_sc_on_r1_we));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_rs0 (.I0(sc_on_bnk[b]), .O(b_sc_on_r_sl[0]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_rs1 (.I0(sc_on_bnk[b]), .O(b_sc_on_r_sl[1]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_rs2 (.I0(sc_on_bnk[b]), .O(b_sc_on_r_sl[2]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc_rs3 (.I0(sc_on_bnk[b]), .O(b_sc_on_r_sl[3]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign b_sc_on_c0_we = sc_on_bnk[b];
		assign b_sc_on_c0_sl = sc_on_bnk[b];
		assign b_sc_on_c1_we = sc_on_bnk[b];
		assign b_sc_on_c1_sl = sc_on_bnk[b];
		assign b_sc_on_r0_we = sc_on_bnk[b];
		assign b_sc_on_r1_we = sc_on_bnk[b];
		assign b_sc_on_r_sl[0] = sc_on_bnk[b];
		assign b_sc_on_r_sl[1] = sc_on_bnk[b];
		assign b_sc_on_r_sl[2] = sc_on_bnk[b];
		assign b_sc_on_r_sl[3] = sc_on_bnk[b];
`else
		assign b_sc_on_c0_we = sc_on_bnk[b];
		assign b_sc_on_c0_sl = sc_on_bnk[b];
		assign b_sc_on_c1_we = sc_on_bnk[b];
		assign b_sc_on_c1_sl = sc_on_bnk[b];
		assign b_sc_on_r0_we = sc_on_bnk[b];
		assign b_sc_on_r1_we = sc_on_bnk[b];
		assign b_sc_on_r_sl  = {4{sc_on_bnk[b]}};
`endif

		localparam b_half = (b >= NPHYS/2) ? 1 : 0;
		localparam b_quad = (b * 4) / NPHYS;
		wire [CAW-1:0] b_sc_a_c0, b_sc_a_c1;
		wire [RW-1:0] b_sc_ar_0, b_sc_ar_1;
		genvar gsa;
		for (gsa = 0; gsa < CAW; gsa = gsa + 1) begin : g_b_sca
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsca0 (.I0(sc_a_pair[b_quad][gsa]), .O(b_sc_a_c0[gsa]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsca1 (.I0(sc_a_pair[b_quad][gsa]), .O(b_sc_a_c1[gsa]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_sc_a_c0[gsa] = sc_a_pair[b_quad][gsa];
			assign b_sc_a_c1[gsa] = sc_a_pair[b_quad][gsa];
`else
			assign b_sc_a_c0[gsa] = sc_a_pair[b_quad][gsa];
			assign b_sc_a_c1[gsa] = sc_a_pair[b_quad][gsa];
`endif
		end
		for (gsa = 0; gsa < RW; gsa = gsa + 1) begin : g_b_scar
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bscar0 (.I0(sc_a_pair[b_quad][gsa + RT_AW]), .O(b_sc_ar_0[gsa]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bscar1 (.I0(sc_a_pair[b_quad][gsa + RT_AW]), .O(b_sc_ar_1[gsa]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_sc_ar_0[gsa] = sc_a_pair[b_quad][gsa + RT_AW];
			assign b_sc_ar_1[gsa] = sc_a_pair[b_quad][gsa + RT_AW];
`else
			assign b_sc_ar_0[gsa] = sc_a_pair[b_quad][gsa + RT_AW];
			assign b_sc_ar_1[gsa] = sc_a_pair[b_quad][gsa + RT_AW];
`endif
		end
		wire [CAW-1:0] b_xcrow_0, b_xcrow_1;
		wire [RW-1:0] b_xrow_0, b_xrow_1;
		genvar gxc, gxr;
		for (gxc = 0; gxc < CAW; gxc = gxc + 1) begin : g_b_xc
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxc0 (.I0(xcrow_q[b_half][gxc]), .O(b_xcrow_0[gxc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxc1 (.I0(xcrow_q[b_half][gxc]), .O(b_xcrow_1[gxc]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_xcrow_0[gxc] = xcrow_q[b_half][gxc];
			assign b_xcrow_1[gxc] = xcrow_q[b_half][gxc];
`else
			assign b_xcrow_0[gxc] = xcrow_q[b_half][gxc];
			assign b_xcrow_1[gxc] = xcrow_q[b_half][gxc];
`endif
		end
		for (gxr = 0; gxr < RW; gxr = gxr + 1) begin : g_b_xr
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxr0 (.I0(xrow_q[b_half][gxr]), .O(b_xrow_0[gxr]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxr1 (.I0(xrow_q[b_half][gxr]), .O(b_xrow_1[gxr]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_xrow_0[gxr] = xrow_q[b_half][gxr];
			assign b_xrow_1[gxr] = xrow_q[b_half][gxr];
`else
			assign b_xrow_0[gxr] = xrow_q[b_half][gxr];
			assign b_xrow_1[gxr] = xrow_q[b_half][gxr];
`endif
		end
		wire [RW-1:0] b_zrow = zrow_g[b];
		wire [RW-1:0] b_zrow_m6, b_zrow_0, b_zrow_1;
		genvar gbzw;
		for (gbzw = 0; gbzw < RW; gbzw = gbzw + 1) begin : g_bzw_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bzwm (.I0(b_zrow[gbzw]), .O(b_zrow_m6[gbzw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bzw0 (.I0(b_zrow[gbzw]), .O(b_zrow_0[gbzw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bzw1 (.I0(b_zrow[gbzw]), .O(b_zrow_1[gbzw]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_zrow_m6[gbzw] = b_zrow[gbzw];
			assign b_zrow_0[gbzw] = b_zrow[gbzw];
			assign b_zrow_1[gbzw] = b_zrow[gbzw];
`else
			assign b_zrow_m6[gbzw] = b_zrow[gbzw];
			assign b_zrow_0[gbzw]  = b_zrow[gbzw];
			assign b_zrow_1[gbzw]  = b_zrow[gbzw];
`endif
		end
		wire [CAW-1:0] b_dcrow_0, b_dcrow_1;
		wire [RW-1:0] b_drow_0, b_drow_1, b_drow_rsm;
		genvar gbd;
		for (gbd = 0; gbd < CAW; gbd = gbd + 1) begin : g_b_dcrow
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bdcr0 (.I0(dcrow_pair[b][gbd]), .O(b_dcrow_0[gbd]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bdcr1 (.I0(dcrow_pair[b][gbd]), .O(b_dcrow_1[gbd]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_dcrow_0[gbd] = dcrow_pair[b][gbd];
			assign b_dcrow_1[gbd] = dcrow_pair[b][gbd];
`else
			assign b_dcrow_0[gbd] = dcrow_pair[b][gbd];
			assign b_dcrow_1[gbd] = dcrow_pair[b][gbd];
`endif
		end
		for (gbd = 0; gbd < RW; gbd = gbd + 1) begin : g_b_drow
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bdr0 (.I0(drow_pair[b][gbd]), .O(b_drow_0[gbd]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bdr1 (.I0(drow_pair[b][gbd]), .O(b_drow_1[gbd]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bdrr (.I0(drow_pair[b][gbd]), .O(b_drow_rsm[gbd]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_drow_0[gbd] = drow_pair[b][gbd];
			assign b_drow_1[gbd] = drow_pair[b][gbd];
			assign b_drow_rsm[gbd] = drow_pair[b][gbd];
`else
			assign b_drow_0[gbd] = drow_pair[b][gbd];
			assign b_drow_1[gbd] = drow_pair[b][gbd];
			assign b_drow_rsm[gbd] = drow_pair[b][gbd];
`endif
		end
		wire b_we_0 = b_sc_on_c0_we || (side_bnk[b] && col_match_q[b_half][0]);
		wire b_we_1 = b_sc_on_c0_we || (side_bnk[b] && col_match_q[b_half][1]);
		wire b_we_2 = b_sc_on_c1_we || (side_bnk[b] && col_match_q[b_half][2]);
		wire b_we_3 = b_sc_on_c1_we || (side_bnk[b] && col_match_q[b_half][3]);
		wire b_we_s0 = b_sc_on_r0_we || (side_bnk[b] && col1_match_q[b_half][0]);
		wire b_we_s1 = b_sc_on_r0_we || (side_bnk[b] && col1_match_q[b_half][1]);
		wire b_we_s2 = b_sc_on_r1_we || (side_bnk[b] && col1_match_q[b_half][2]);
		wire b_we_s3 = b_sc_on_r1_we || (side_bnk[b] && col1_match_q[b_half][3]);
		wire b_we_m4 = b_sc_on_r0_we || (side_bnk[b] && dep_q[b_half]);
		wire b_we_mz = b_sc_on_r1_we || (fl_we && fl_reg == 6'd3 && zbank == b);

		wire [RW-1:0] r_wa_m5  = b_sc_on_r1_we ? b_sc_ar_1 : b_xrow_1;
		wire [RW-1:0] z_wa_mr6 = b_sc_on_r1_we ? b_sc_ar_1 : b_zrow_m6;
		wire [RW-1:0] z_wa_mr6_0, z_wa_mr6_1;
`ifdef SYNTH_KINTEX7
		genvar gzw;
		for (gzw = 0; gzw < RW; gzw = gzw + 1) begin : g_zmr6_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_zm0 (.I0(z_wa_mr6[gzw]), .O(z_wa_mr6_0[gzw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_zm1 (.I0(z_wa_mr6[gzw]), .O(z_wa_mr6_1[gzw]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		genvar gzw;
		for (gzw = 0; gzw < RW; gzw = gzw + 1) begin : g_zmr6_buf
			assign z_wa_mr6_0[gzw] = z_wa_mr6[gzw];
			assign z_wa_mr6_1[gzw] = z_wa_mr6[gzw];
		end
`else
		assign z_wa_mr6_0 = z_wa_mr6;
		assign z_wa_mr6_1 = z_wa_mr6;
`endif
		wire [RW-1:0] drow_rsm = b_drow_rsm;
		wire we_m5 = b_sc_on_r1_we || (side_bnk[b] && sm_q[b_half]);
		wire we_mr6 = b_sc_on_r1_we || (fl_mr6_en_g[b] && (zbank == b));
		wire we_mr6_0, we_mr6_1;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wmr60 (.I0(we_mr6), .O(we_mr6_0));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wmr61 (.I0(we_mr6), .O(we_mr6_1));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign we_mr6_0 = we_mr6;
		assign we_mr6_1 = we_mr6;
`else
		assign we_mr6_0 = we_mr6;
		assign we_mr6_1 = we_mr6;
`endif
		wire [3:0] d_m5 = b_sc_on_r1_we ? 4'd0 : r_x_b[32 * b +: 4];
		wire [3:0] fd_mr6 = b_sc_on_r1_we ? 4'd0 : fl_data[3:0];

		(* dont_touch = "true" *) reg w_we_m5_r, w_we_mr6_0_r, w_we_mr6_1_r;
		(* shreg_extract = "no", dont_touch = "true" *) reg [RW-1:0] w_r_wa_m5_r, w_z_wa_mr6_0_r, w_z_wa_mr6_1_r;
		(* shreg_extract = "no" *) reg [3:0] w_d_m5_r, w_fd_mr6_r;
		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				w_we_m5_r      <= 1'b0;
				w_we_mr6_0_r   <= 1'b0;
				w_we_mr6_1_r   <= 1'b0;
				w_r_wa_m5_r    <= {RW{1'b0}};
				w_z_wa_mr6_0_r <= {RW{1'b0}};
				w_z_wa_mr6_1_r <= {RW{1'b0}};
			end else begin
				w_we_m5_r      <= we_m5;
				w_we_mr6_0_r   <= we_mr6_0;
				w_we_mr6_1_r   <= we_mr6_1;
				w_r_wa_m5_r    <= r_wa_m5;
				w_z_wa_mr6_0_r <= z_wa_mr6_0;
				w_z_wa_mr6_1_r <= z_wa_mr6_1;
			end
		end
		always @(posedge clk) begin
			w_d_m5_r   <= d_m5;
			w_fd_mr6_r <= fd_mr6;
		end

		reg [3:0] m5 [0:ROWS-1];
		reg [1:0] mr6_lo [0:ROWS-1];
		reg [1:0] mr6_hi [0:ROWS-1];
		always @(posedge clk) begin
			if (w_we_m5_r)    m5[w_r_wa_m5_r]        <= w_d_m5_r;
			if (w_we_mr6_0_r) mr6_lo[w_z_wa_mr6_0_r] <= w_fd_mr6_r[1:0];
			if (w_we_mr6_1_r) mr6_hi[w_z_wa_mr6_1_r] <= w_fd_mr6_r[3:2];
		end
		assign rsm[8 * b +: 8] = {m5[drow_rsm], mr6_hi[drow_rsm], mr6_lo[drow_rsm]};

		for (bs = 0; bs < 4; bs = bs + 1) begin : g_col_sl
			wire b_scon_raw = (bs < 2) ? b_sc_on_c0_sl : b_sc_on_c1_sl;
			wire b_scon_a, b_scon_d;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_b_scon_a (.I0(b_scon_raw), .O(b_scon_a));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_b_scon_d (.I0(b_scon_raw), .O(b_scon_d));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign b_scon_a = b_scon_raw;
			assign b_scon_d = b_scon_raw;
`else
			assign b_scon_a = b_scon_raw;
			assign b_scon_d = b_scon_raw;
`endif
			wire [CAW-1:0] b_sca_raw = (bs < 2) ? b_sc_a_c0 : b_sc_a_c1;
			wire [CAW-1:0] b_xcr = (bs < 2) ? b_xcrow_0 : b_xcrow_1;
			wire [CAW-1:0] b_dcr = (bs < 2) ? b_dcrow_0 : b_dcrow_1;
			wire [CAW-1:0] c_wa_raw = b_scon_a ? b_sca_raw : b_xcr;
			wire [CAW-1:0] dcrow_0_lo, dcrow_0_hi, dcrow_1_lo, dcrow_1_hi;
			wire [CAW-1:0] dcrow_2_lo, dcrow_2_hi, dcrow_3_lo, dcrow_3_hi;
`ifdef SYNTH_KINTEX7
			genvar gbc;
			for (gbc = 0; gbc < CAW; gbc = gbc + 1) begin : g_dc_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr0_lo (.I0(b_dcr[gbc]), .O(dcrow_0_lo[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr0_hi (.I0(b_dcr[gbc]), .O(dcrow_0_hi[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr1_lo (.I0(b_dcr[gbc]), .O(dcrow_1_lo[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr1_hi (.I0(b_dcr[gbc]), .O(dcrow_1_hi[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr2_lo (.I0(b_dcr[gbc]), .O(dcrow_2_lo[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr2_hi (.I0(b_dcr[gbc]), .O(dcrow_2_hi[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr3_lo (.I0(b_dcr[gbc]), .O(dcrow_3_lo[gbc]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dcr3_hi (.I0(b_dcr[gbc]), .O(dcrow_3_hi[gbc]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gbc;
			for (gbc = 0; gbc < CAW; gbc = gbc + 1) begin : g_dc_buf
				assign dcrow_0_lo[gbc] = b_dcr[gbc];
				assign dcrow_0_hi[gbc] = b_dcr[gbc];
				assign dcrow_1_lo[gbc] = b_dcr[gbc];
				assign dcrow_1_hi[gbc] = b_dcr[gbc];
				assign dcrow_2_lo[gbc] = b_dcr[gbc];
				assign dcrow_2_hi[gbc] = b_dcr[gbc];
				assign dcrow_3_lo[gbc] = b_dcr[gbc];
				assign dcrow_3_hi[gbc] = b_dcr[gbc];
			end
`else
			assign dcrow_0_lo = b_dcr;
			assign dcrow_0_hi = b_dcr;
			assign dcrow_1_lo = b_dcr;
			assign dcrow_1_hi = b_dcr;
			assign dcrow_2_lo = b_dcr;
			assign dcrow_2_hi = b_dcr;
			assign dcrow_3_lo = b_dcr;
			assign dcrow_3_hi = b_dcr;
`endif
			wire [7:0] d_sl = b_scon_d ? 8'd0 : r_x_b[32 * b + bs * 8 +: 8];

			/* 4-bit nibbles keep LUTRAM addr fanout <= 16 under Vivado Rule 2. */
			(* dont_touch = "true" *) reg w_we_0_lo_r, w_we_0_hi_r, w_we_1_lo_r, w_we_1_hi_r;
			(* dont_touch = "true" *) reg w_we_2_lo_r, w_we_2_hi_r, w_we_3_lo_r, w_we_3_hi_r;
			(* dont_touch = "true", shreg_extract = "no" *) reg [CAW-1:0] w_c_wa_0_lo_r, w_c_wa_0_hi_r;
			(* dont_touch = "true", shreg_extract = "no" *) reg [CAW-1:0] w_c_wa_1_lo_r, w_c_wa_1_hi_r;
			(* dont_touch = "true", shreg_extract = "no" *) reg [CAW-1:0] w_c_wa_2_lo_r, w_c_wa_2_hi_r;
			(* dont_touch = "true", shreg_extract = "no" *) reg [CAW-1:0] w_c_wa_3_lo_r, w_c_wa_3_hi_r;
			(* shreg_extract = "no" *) reg [7:0] w_d_sl_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					w_we_0_lo_r   <= 1'b0; w_we_0_hi_r   <= 1'b0;
					w_we_1_lo_r   <= 1'b0; w_we_1_hi_r   <= 1'b0;
					w_we_2_lo_r   <= 1'b0; w_we_2_hi_r   <= 1'b0;
					w_we_3_lo_r   <= 1'b0; w_we_3_hi_r   <= 1'b0;
					w_c_wa_0_lo_r <= {CAW{1'b0}}; w_c_wa_0_hi_r <= {CAW{1'b0}};
					w_c_wa_1_lo_r <= {CAW{1'b0}}; w_c_wa_1_hi_r <= {CAW{1'b0}};
					w_c_wa_2_lo_r <= {CAW{1'b0}}; w_c_wa_2_hi_r <= {CAW{1'b0}};
					w_c_wa_3_lo_r <= {CAW{1'b0}}; w_c_wa_3_hi_r <= {CAW{1'b0}};
				end else begin
					w_we_0_lo_r   <= b_we_0; w_we_0_hi_r   <= b_we_0;
					w_we_1_lo_r   <= b_we_1; w_we_1_hi_r   <= b_we_1;
					w_we_2_lo_r   <= b_we_2; w_we_2_hi_r   <= b_we_2;
					w_we_3_lo_r   <= b_we_3; w_we_3_hi_r   <= b_we_3;
					w_c_wa_0_lo_r <= c_wa_raw; w_c_wa_0_hi_r <= c_wa_raw;
					w_c_wa_1_lo_r <= c_wa_raw; w_c_wa_1_hi_r <= c_wa_raw;
					w_c_wa_2_lo_r <= c_wa_raw; w_c_wa_2_hi_r <= c_wa_raw;
					w_c_wa_3_lo_r <= c_wa_raw; w_c_wa_3_hi_r <= c_wa_raw;
				end
			end
			always @(posedge clk) begin
				w_d_sl_r <= d_sl;
			end

			reg [3:0] m0_lo [0:C_DEPTH-1], m0_hi [0:C_DEPTH-1];
			reg [3:0] m1_lo [0:C_DEPTH-1], m1_hi [0:C_DEPTH-1];
			reg [3:0] m2_lo [0:C_DEPTH-1], m2_hi [0:C_DEPTH-1];
			reg [3:0] m3_lo [0:C_DEPTH-1], m3_hi [0:C_DEPTH-1];
			always @(posedge clk) begin
				if (w_we_0_lo_r) m0_lo[w_c_wa_0_lo_r] <= w_d_sl_r[3:0];
				if (w_we_0_hi_r) m0_hi[w_c_wa_0_hi_r] <= w_d_sl_r[7:4];
				if (w_we_1_lo_r) m1_lo[w_c_wa_1_lo_r] <= w_d_sl_r[3:0];
				if (w_we_1_hi_r) m1_hi[w_c_wa_1_hi_r] <= w_d_sl_r[7:4];
				if (w_we_2_lo_r) m2_lo[w_c_wa_2_lo_r] <= w_d_sl_r[3:0];
				if (w_we_2_hi_r) m2_hi[w_c_wa_2_hi_r] <= w_d_sl_r[7:4];
				if (w_we_3_lo_r) m3_lo[w_c_wa_3_lo_r] <= w_d_sl_r[3:0];
				if (w_we_3_hi_r) m3_hi[w_c_wa_3_hi_r] <= w_d_sl_r[7:4];
			end
			assign rdat[192 * b + 0   + bs * 8 +: 8] = {m0_hi[dcrow_0_hi], m0_lo[dcrow_0_lo]};
			assign rdat[192 * b + 32  + bs * 8 +: 8] = {m1_hi[dcrow_1_hi], m1_lo[dcrow_1_lo]};
			assign rdat[192 * b + 64  + bs * 8 +: 8] = {m2_hi[dcrow_2_hi], m2_lo[dcrow_2_lo]};
			assign rdat[192 * b + 96  + bs * 8 +: 8] = {m3_hi[dcrow_3_hi], m3_lo[dcrow_3_lo]};
		end

		for (bh = 0; bh < 4; bh = bh + 1) begin : g_row_sl
			wire b_scon_raw = b_sc_on_r_sl[bh];
			wire [RW-1:0] b_scar = (bh < 2) ? b_sc_ar_0 : b_sc_ar_1;
			wire [RW-1:0] b_xr   = (bh < 2) ? b_xrow_0  : b_xrow_1;
			wire [RW-1:0] b_dr   = (bh < 2) ? b_drow_0  : b_drow_1;
			wire [RW-1:0] r_wa_raw = b_scon_raw ? b_scar : b_xr;
			wire [RW-1:0] z_wa_raw = b_scon_raw ? b_scar : ((bh < 2) ? b_zrow_0 : b_zrow_1);
			wire [RW-1:0] drow_s01, drow_s23, drow_m4z;
`ifdef SYNTH_KINTEX7
			genvar gbr;
			for (gbr = 0; gbr < RW; gbr = gbr + 1) begin : g_dr_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_drs01 (.I0(b_dr[gbr]), .O(drow_s01[gbr]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_drs23 (.I0(b_dr[gbr]), .O(drow_s23[gbr]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_drm4z (.I0(b_dr[gbr]), .O(drow_m4z[gbr]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gbr;
			for (gbr = 0; gbr < RW; gbr = gbr + 1) begin : g_dr_buf
				assign drow_s01[gbr] = b_dr[gbr];
				assign drow_s23[gbr] = b_dr[gbr];
				assign drow_m4z[gbr] = b_dr[gbr];
			end
`else
			assign drow_s01 = b_dr;
			assign drow_s23 = b_dr;
			assign drow_m4z = b_dr;
`endif
			wire we_s0 = b_we_s0;
			wire we_s1 = b_we_s1;
			wire we_s2 = b_we_s2;
			wire we_s3 = b_we_s3;
			wire we_m4 = b_we_m4;
			wire we_mz = b_we_mz;

			wire [7:0] d_h  = b_scon_raw ? 8'd0 : r_x_b[32 * b + bh * 8 +: 8];
			wire [7:0] fd_h = b_scon_raw ? 8'd0 : fl_data[bh * 8 +: 8];

			(* dont_touch = "true" *) reg w_we_s0_r, w_we_s1_r, w_we_s2_r, w_we_s3_r, w_we_m4_r, w_we_mz_r;
			(* dont_touch = "true", shreg_extract = "no" *) reg [RW-1:0] w_r_wa_s0_r, w_r_wa_s1_r, w_r_wa_s2_r, w_r_wa_s3_r, w_r_wa_m4_r, w_z_wa_mz_r;
			(* shreg_extract = "no" *) reg [7:0] w_d_h_r, w_fd_h_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					w_we_s0_r   <= 1'b0;
					w_we_s1_r   <= 1'b0;
					w_we_s2_r   <= 1'b0;
					w_we_s3_r   <= 1'b0;
					w_we_m4_r   <= 1'b0;
					w_we_mz_r   <= 1'b0;
					w_r_wa_s0_r <= {RW{1'b0}};
					w_r_wa_s1_r <= {RW{1'b0}};
					w_r_wa_s2_r <= {RW{1'b0}};
					w_r_wa_s3_r <= {RW{1'b0}};
					w_r_wa_m4_r <= {RW{1'b0}};
					w_z_wa_mz_r <= {RW{1'b0}};
				end else begin
					w_we_s0_r   <= we_s0;
					w_we_s1_r   <= we_s1;
					w_we_s2_r   <= we_s2;
					w_we_s3_r   <= we_s3;
					w_we_m4_r   <= we_m4;
					w_we_mz_r   <= we_mz;
					w_r_wa_s0_r <= r_wa_raw;
					w_r_wa_s1_r <= r_wa_raw;
					w_r_wa_s2_r <= r_wa_raw;
					w_r_wa_s3_r <= r_wa_raw;
					w_r_wa_m4_r <= r_wa_raw;
					w_z_wa_mz_r <= z_wa_raw;
				end
			end
			always @(posedge clk) begin
				w_d_h_r  <= d_h;
				w_fd_h_r <= fd_h;
			end

			if (DUAL_SOURCE != 0) begin : g_ms1
				reg [7:0] ms1_0 [0:ROWS-1];
				reg [7:0] ms1_1 [0:ROWS-1];
				reg [7:0] ms1_2 [0:ROWS-1];
				reg [7:0] ms1_3 [0:ROWS-1];
				always @(posedge clk) begin
					if (w_we_s0_r) ms1_0[w_r_wa_s0_r] <= w_d_h_r;
					if (w_we_s1_r) ms1_1[w_r_wa_s1_r] <= w_d_h_r;
					if (w_we_s2_r) ms1_2[w_r_wa_s2_r] <= w_d_h_r;
					if (w_we_s3_r) ms1_3[w_r_wa_s3_r] <= w_d_h_r;
				end
				assign rdat1[128 * b + 0  + bh * 8 +: 8] = ms1_0[drow_s01];
				assign rdat1[128 * b + 32 + bh * 8 +: 8] = ms1_1[drow_s01];
				assign rdat1[128 * b + 64 + bh * 8 +: 8] = ms1_2[drow_s23];
				assign rdat1[128 * b + 96 + bh * 8 +: 8] = ms1_3[drow_s23];
			end else begin : g_no_ms1
				assign rdat1[128 * b + 0  + bh * 8 +: 8] = 8'd0;
				assign rdat1[128 * b + 32 + bh * 8 +: 8] = 8'd0;
				assign rdat1[128 * b + 64 + bh * 8 +: 8] = 8'd0;
				assign rdat1[128 * b + 96 + bh * 8 +: 8] = 8'd0;
			end
			reg [7:0] m4 [0:ROWS-1];
			reg [7:0] mz [0:ROWS-1];
			always @(posedge clk) begin
				if (w_we_m4_r) m4[w_r_wa_m4_r] <= w_d_h_r;
				if (w_we_mz_r) mz[w_z_wa_mz_r] <= w_fd_h_r;
			end
			assign rdat[192 * b + 128 + bh * 8 +: 8] = m4[drow_m4z];
			assign rdat[192 * b + 160 + bh * 8 +: 8] = mz[drow_m4z];
		end
	end
	endgenerate

	/* The quad's lanes i = 0..3 are lanes 4 dq + i, banks (4 dq + i) mod NPHYS. */
	wire [511:0] col, col1;
	wire [127:0] dep;
	reg [7:0] ls;
	reg [15:0] smask;
	reg [3:0] zexp;
	reg [3:0] smok;
	integer i;

	genvar gi, gch;
	generate
		for (gi = 0; gi < 4; gi = gi + 1) begin : g_qmux
			wire [2:0] dq_col  = dq_rep[(8 + gi*3 + 0)*3 +: 3];
			wire [2:0] dq_col1 = dq_rep[(8 + gi*3 + 1)*3 +: 3];
			wire [2:0] dq_dep  = dq_rep[(8 + gi*3 + 2)*3 +: 3];
			wire [4:0] lane_col  = {dq_col,  gi[1:0]} & (NPHYS - 1);
			wire [4:0] lane_col1 = {dq_col1, gi[1:0]} & (NPHYS - 1);
			wire [4:0] lane_dep  = {dq_dep,  gi[1:0]} & (NPHYS - 1);
			wire q_ddw = ddw[{dq_dep, gi[1:0]}];
			wire q_ddw_ch0, q_ddw_ch1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qddw0 (.I0(q_ddw), .O(q_ddw_ch0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qddw1 (.I0(q_ddw), .O(q_ddw_ch1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign q_ddw_ch0 = q_ddw;
			assign q_ddw_ch1 = q_ddw;
`else
			assign q_ddw_ch0 = q_ddw;
			assign q_ddw_ch1 = q_ddw;
`endif


			for (gch = 0; gch < 8; gch = gch + 1) begin : g_c_sl
				wire [4:0] lane_col_ch_lo, lane_col_ch_hi;
				wire [4:0] lane_col1_ch_lo, lane_col1_ch_hi;
				genvar glb;
				for (glb = 0; glb < 5; glb = glb + 1) begin : g_lbuf
`ifdef SYNTH_KINTEX7
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lc_lo  (.I0(lane_col[glb]),  .O(lane_col_ch_lo[glb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lc_hi  (.I0(lane_col[glb]),  .O(lane_col_ch_hi[glb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lc1_lo (.I0(lane_col1[glb]), .O(lane_col1_ch_lo[glb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lc1_hi (.I0(lane_col1[glb]), .O(lane_col1_ch_hi[glb]));
`elsif SYNTH_KINTEX_ULTRASCALE
					assign lane_col_ch_lo[glb] = lane_col[glb];
					assign lane_col_ch_hi[glb] = lane_col[glb];
					assign lane_col1_ch_lo[glb] = lane_col1[glb];
					assign lane_col1_ch_hi[glb] = lane_col1[glb];
`else
					assign lane_col_ch_lo[glb]  = lane_col[glb];
					assign lane_col_ch_hi[glb]  = lane_col[glb];
					assign lane_col1_ch_lo[glb] = lane_col1[glb];
					assign lane_col1_ch_hi[glb] = lane_col1[glb];
`endif
				end
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(0), .CH_IDX(gch)) u_col_lo (
					.all_rdat(rdat), .l_idx(lane_col_ch_lo),
					.byte_out(col[128 * gi + gch * 16 +: 8])
				);
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(1), .CH_IDX(gch)) u_col_hi (
					.all_rdat(rdat), .l_idx(lane_col_ch_hi),
					.byte_out(col[128 * gi + gch * 16 + 8 +: 8])
				);
				if (DUAL_SOURCE != 0) begin : g_col1_mux
					atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(128), .HI(0), .CH_IDX(gch)) u_col1_lo (
						.all_rdat(rdat1), .l_idx(lane_col1_ch_lo),
						.byte_out(col1[128 * gi + gch * 16 +: 8])
					);
					atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(128), .HI(1), .CH_IDX(gch)) u_col1_hi (
						.all_rdat(rdat1), .l_idx(lane_col1_ch_hi),
						.byte_out(col1[128 * gi + gch * 16 + 8 +: 8])
					);
				end else begin : g_no_col1_mux
					assign col1[128 * gi + gch * 16 +: 16] = 16'd0;
				end
			end
			for (gch = 0; gch < 2; gch = gch + 1) begin : g_d_sl
				wire [4:0] lane_dep_m4_lo, lane_dep_m4_hi;
				wire [4:0] lane_dep_mz_lo, lane_dep_mz_hi;
				genvar gdb;
				for (gdb = 0; gdb < 5; gdb = gdb + 1) begin : g_dbuf
`ifdef SYNTH_KINTEX7
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dm4l (.I0(lane_dep[gdb]), .O(lane_dep_m4_lo[gdb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dm4h (.I0(lane_dep[gdb]), .O(lane_dep_m4_hi[gdb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dmzl (.I0(lane_dep[gdb]), .O(lane_dep_mz_lo[gdb]));
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dmzh (.I0(lane_dep[gdb]), .O(lane_dep_mz_hi[gdb]));
`elsif SYNTH_KINTEX_ULTRASCALE
					assign lane_dep_m4_lo[gdb] = lane_dep[gdb];
					assign lane_dep_m4_hi[gdb] = lane_dep[gdb];
					assign lane_dep_mz_lo[gdb] = lane_dep[gdb];
					assign lane_dep_mz_hi[gdb] = lane_dep[gdb];
`else
					assign lane_dep_m4_lo[gdb] = lane_dep[gdb];
					assign lane_dep_m4_hi[gdb] = lane_dep[gdb];
					assign lane_dep_mz_lo[gdb] = lane_dep[gdb];
					assign lane_dep_mz_hi[gdb] = lane_dep[gdb];
`endif
				end
				wire [7:0] w_m4_lo, w_m4_hi, w_mz_lo, w_mz_hi;
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(0), .CH_IDX(8 + gch)) u_m4_lo (
					.all_rdat(rdat), .l_idx(lane_dep_m4_lo),
					.byte_out(w_m4_lo)
				);
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(1), .CH_IDX(8 + gch)) u_m4_hi (
					.all_rdat(rdat), .l_idx(lane_dep_m4_hi),
					.byte_out(w_m4_hi)
				);
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(0), .CH_IDX(10 + gch)) u_mz_lo (
					.all_rdat(rdat), .l_idx(lane_dep_mz_lo),
					.byte_out(w_mz_lo)
				);
				atg_su_exp_ch_mux #(.NPHYS(NPHYS), .CH_W(192), .HI(1), .CH_IDX(10 + gch)) u_mz_hi (
					.all_rdat(rdat), .l_idx(lane_dep_mz_hi),
					.byte_out(w_mz_hi)
				);
				wire q_ddw_cur = (gch == 0) ? q_ddw_ch0 : q_ddw_ch1;
				assign dep[32 * gi + gch * 16 +: 8]      = q_ddw_cur ? w_m4_lo : w_mz_lo;
				assign dep[32 * gi + gch * 16 + 8 +: 8]  = q_ddw_cur ? w_m4_hi : w_mz_hi;
			end
		end
	endgenerate

	wire [4:0] dl0_rsm_a, dl0_rsm_b;
`ifdef SYNTH_KINTEX7
	genvar gdrb;
	generate
		for (gdrb = 0; gdrb < 5; gdrb = gdrb + 1) begin : g_dl0rsm_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_drsma (.I0(dl0_rsm[gdrb]), .O(dl0_rsm_a[gdrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_drsmb (.I0(dl0_rsm[gdrb]), .O(dl0_rsm_b[gdrb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gdrb;
	generate
		for (gdrb = 0; gdrb < 5; gdrb = gdrb + 1) begin : g_dl0rsm_buf
			assign dl0_rsm_a[gdrb] = dl0_rsm[gdrb];
			assign dl0_rsm_b[gdrb] = dl0_rsm[gdrb];
		end
	endgenerate
`else
	assign dl0_rsm_a = dl0_rsm;
	assign dl0_rsm_b = dl0_rsm;
`endif
	always @(*) begin
		ls = 8'd0;
		for (i = 0; i < 4; i = i + 1) begin
			ls = rsm[8 * (({27'd0, (i < 2) ? dl0_rsm_a : dl0_rsm_b} + i) % NPHYS) +: 8];
			zexp[i] = ddw[((i < 2) ? dl0_zsm_0 : dl0_zsm_1) + i[4:0]];
`ifdef MUT_SMASK_AND
			smask[4 * i +: 4] = (dr6[((i < 2) ? dl0_zsm_0 : dl0_zsm_1) + i[4:0]] ? ls[3:0] : 4'hF) &
				(dsm[((i < 2) ? dl0_zsm_0 : dl0_zsm_1) + i[4:0]] ? ls[7:4] & ls[3:0] : 4'hF);
`else
			smask[4 * i +: 4] = (dr6[((i < 2) ? dl0_zsm_0 : dl0_zsm_1) + i[4:0]] ? ls[3:0] : 4'hF) &
				(dsm[((i < 2) ? dl0_zsm_0 : dl0_zsm_1) + i[4:0]] ? ls[7:4] : 4'hF);
`endif
			smok[i] = smask[4 * i +: 4] != 4'd0;
		end
	end

	wire [127:0] dsv_mux;
	genvar gds;
	generate
		for (gds = 0; gds < 8; gds = gds + 1) begin : g_dsv_sl
			wire [WARP_W-1:0] dw_sl_lo, dw_sl_hi;
			genvar gwb;
			for (gwb = 0; gwb < WARP_W; gwb = gwb + 1) begin : g_dw_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dw_lo (.I0(dwarp_rep[(27 + gds)*WARP_W + gwb]), .O(dw_sl_lo[gwb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dw_hi (.I0(dwarp_rep[(27 + gds)*WARP_W + gwb]), .O(dw_sl_hi[gwb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign dw_sl_lo[gwb] = dwarp_rep[(27 + gds)*WARP_W + gwb];
				assign dw_sl_hi[gwb] = dwarp_rep[(27 + gds)*WARP_W + gwb];
`else
				assign dw_sl_lo[gwb] = dwarp_rep[(27 + gds)*WARP_W + gwb];
				assign dw_sl_hi[gwb] = dwarp_rep[(27 + gds)*WARP_W + gwb];
`endif
			end
			assign dsv_mux[gds * 16 +: 8]     = dsv[{dw_sl_lo, gds[2:0], 4'd0} +: 8];
			assign dsv_mux[gds * 16 + 8 +: 8] = dsv[{dw_sl_hi, gds[2:0], 4'd8} +: 8];
		end
	endgenerate

	wire [8*WARP_W-1:0] dwarp_qx_sl, dwarp_qy_sl;
	wire [8*3-1:0]      dq_qx_sl, dq_qy_sl;
	genvar gq_b, gq_s;
	generate
		for (gq_s = 0; gq_s < 4; gq_s = gq_s + 1) begin : g_qs_rep
			localparam integer DW_X_I = (gq_s == 0) ? 35 : (35 + gq_s);
			localparam integer DW_Y_I = 39 + gq_s;
			localparam integer DQ_X_I = (gq_s == 0) ? 5 : (19 + gq_s);
			localparam integer DQ_Y_I = (gq_s == 0) ? 6 : (22 + gq_s);
			for (gq_b = 0; gq_b < WARP_W; gq_b = gq_b + 1) begin : g_dwb
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dx0 (.I0(dwarp_rep[DW_X_I*WARP_W + gq_b]), .O(dwarp_qx_sl[(gq_s*2+0)*WARP_W + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dx1 (.I0(dwarp_rep[DW_X_I*WARP_W + gq_b]), .O(dwarp_qx_sl[(gq_s*2+1)*WARP_W + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dy0 (.I0(dwarp_rep[DW_Y_I*WARP_W + gq_b]), .O(dwarp_qy_sl[(gq_s*2+0)*WARP_W + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dy1 (.I0(dwarp_rep[DW_Y_I*WARP_W + gq_b]), .O(dwarp_qy_sl[(gq_s*2+1)*WARP_W + gq_b]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign dwarp_qx_sl[(gq_s*2+0)*WARP_W + gq_b] = dwarp_rep[DW_X_I*WARP_W + gq_b];
				assign dwarp_qx_sl[(gq_s*2+1)*WARP_W + gq_b] = dwarp_rep[DW_X_I*WARP_W + gq_b];
				assign dwarp_qy_sl[(gq_s*2+0)*WARP_W + gq_b] = dwarp_rep[DW_Y_I*WARP_W + gq_b];
				assign dwarp_qy_sl[(gq_s*2+1)*WARP_W + gq_b] = dwarp_rep[DW_Y_I*WARP_W + gq_b];
`else
				assign dwarp_qx_sl[(gq_s*2+0)*WARP_W + gq_b] = dwarp_rep[DW_X_I*WARP_W + gq_b];
				assign dwarp_qx_sl[(gq_s*2+1)*WARP_W + gq_b] = dwarp_rep[DW_X_I*WARP_W + gq_b];
				assign dwarp_qy_sl[(gq_s*2+0)*WARP_W + gq_b] = dwarp_rep[DW_Y_I*WARP_W + gq_b];
				assign dwarp_qy_sl[(gq_s*2+1)*WARP_W + gq_b] = dwarp_rep[DW_Y_I*WARP_W + gq_b];
`endif
			end
			for (gq_b = 0; gq_b < 3; gq_b = gq_b + 1) begin : g_dqb
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qx0 (.I0(dq_rep[DQ_X_I*3 + gq_b]), .O(dq_qx_sl[(gq_s*2+0)*3 + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qx1 (.I0(dq_rep[DQ_X_I*3 + gq_b]), .O(dq_qx_sl[(gq_s*2+1)*3 + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qy0 (.I0(dq_rep[DQ_Y_I*3 + gq_b]), .O(dq_qy_sl[(gq_s*2+0)*3 + gq_b]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qy1 (.I0(dq_rep[DQ_Y_I*3 + gq_b]), .O(dq_qy_sl[(gq_s*2+1)*3 + gq_b]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign dq_qx_sl[(gq_s*2+0)*3 + gq_b] = dq_rep[DQ_X_I*3 + gq_b];
				assign dq_qx_sl[(gq_s*2+1)*3 + gq_b] = dq_rep[DQ_X_I*3 + gq_b];
				assign dq_qy_sl[(gq_s*2+0)*3 + gq_b] = dq_rep[DQ_Y_I*3 + gq_b];
				assign dq_qy_sl[(gq_s*2+1)*3 + gq_b] = dq_rep[DQ_Y_I*3 + gq_b];
`else
				assign dq_qx_sl[(gq_s*2+0)*3 + gq_b] = dq_rep[DQ_X_I*3 + gq_b];
				assign dq_qx_sl[(gq_s*2+1)*3 + gq_b] = dq_rep[DQ_X_I*3 + gq_b];
				assign dq_qy_sl[(gq_s*2+0)*3 + gq_b] = dq_rep[DQ_Y_I*3 + gq_b];
				assign dq_qy_sl[(gq_s*2+1)*3 + gq_b] = dq_rep[DQ_Y_I*3 + gq_b];
`endif
			end
		end
	endgenerate
	assign x_ready = 1'b1;

	wire [15:0] qx_out, qy_out;
	genvar gq_sl;
	generate
		for (gq_sl = 0; gq_sl < 8; gq_sl = gq_sl + 1) begin : g_qx_mux
			wire [WARP_W+2:0] dqs_x_sl = {dwarp_qx_sl[gq_sl*WARP_W +: WARP_W], dq_qx_sl[gq_sl*3 +: 3]};
			wire [WARP_W+2:0] dqs_y_sl = {dwarp_qy_sl[gq_sl*WARP_W +: WARP_W], dq_qy_sl[gq_sl*3 +: 3]};
			assign qx_out[gq_sl*2 +: 2] = qx[{dqs_x_sl, gq_sl[2:0], 1'b0} +: 2];
			assign qy_out[gq_sl*2 +: 2] = qy[{dqs_y_sl, gq_sl[2:0], 1'b0} +: 2];
		end
	endgenerate
	assign ex_xy = {qy_out, qx_out};
	assign ex_mask = dxp[dl0 +: 4] & smok;
	wire [2:0] drt_ex;
`ifdef SYNTH_KINTEX7
	genvar gdx;
	generate
		for (gdx = 0; gdx < 3; gdx = gdx + 1) begin : g_drtex
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_drte (.I0(drt[gdx]), .O(drt_ex[gdx]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gdx;
	generate
		for (gdx = 0; gdx < 3; gdx = gdx + 1) begin : g_drtex
			assign drt_ex[gdx] = drt[gdx];
		end
	endgenerate
`else
	assign drt_ex = drt;
`endif
`ifdef MUT_EXP_FRONT
	assign ex_aux = {drt_ex, dlastb, dfirst, dnocol, 1'b1, dsv_mux,
		zexp & dxp[dl0 +: 4], smask};
`else
	assign ex_aux = {drt_ex, dlastb, dfirst, dnocol, fr[dw_rtw], dsv_mux,
		zexp & dxp[dl0 +: 4], smask};
`endif
	assign ex_color = col;
	assign ex_color1 = col1;
	assign ex_has_src1 = (DUAL_SOURCE != 0) ? has_src1_w[dw_rtw] : 1'b0;
	assign ex_depth = dep;

	localparam integer NG_FL = (NWARPS + 3) / 4;
	wire [4:0] fl_lane_q [0:NG_FL-1];
	genvar gw, gq, gflq, gflqn;
	generate
		for (gflq = 0; gflq < NG_FL; gflq = gflq + 1) begin : g_fl_lane_q
			for (gflqn = 0; gflqn < 5; gflqn = gflqn + 1) begin : g_fl_lane_qb
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flq (.I0(fl_lane[gflqn]), .O(fl_lane_q[gflq][gflqn]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign fl_lane_q[gflq][gflqn] = fl_lane[gflqn];
`else
				assign fl_lane_q[gflq][gflqn] = fl_lane[gflqn];
`endif
			end
		end
		for (gw = 0; gw < NWARPS; gw = gw + 1) begin : g_w_ctrl
			wire [4:0] fl_lane_g = fl_lane_q[gw / 4];
			wire rst_w = !rst_n;
			wire [23:0] w_ff_f_sh, w_ff_f_cmp;
			genvar gwf;
			for (gwf = 0; gwf < 24; gwf = gwf + 1) begin : g_wff_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wfs (.I0(fl_data[8+gwf]), .O(w_ff_f_sh[gwf]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wfc (.I0(fl_data[8+gwf]), .O(w_ff_f_cmp[gwf]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign w_ff_f_sh[gwf] = fl_data[8+gwf];
				assign w_ff_f_cmp[gwf] = fl_data[8+gwf];
`else
				assign w_ff_f_sh[gwf]  = fl_data[8+gwf];
				assign w_ff_f_cmp[gwf] = fl_data[8+gwf];
`endif
			end
			wire [3:0] w_ff_f_exp;
			genvar gwe;
			for (gwe = 0; gwe < 4; gwe = gwe + 1) begin : g_wexp_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wfe (.I0(fl_data[8+15+gwe]), .O(w_ff_f_exp[gwe]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign w_ff_f_exp[gwe] = fl_data[8+15+gwe];
`else
				assign w_ff_f_exp[gwe] = fl_data[8+15+gwe];
`endif
			end
			wire [3:0] w_ff_sh_raw;
			wire [3:0] w_ff_sh;
			genvar gsh;
			for (gsh = 0; gsh < 4; gsh = gsh + 1) begin : g_sh_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wsh (.I0(w_ff_sh_raw[gsh]), .O(w_ff_sh[gsh]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign w_ff_sh[gsh] = w_ff_sh_raw[gsh];
`else
				assign w_ff_sh[gsh] = w_ff_sh_raw[gsh];
`endif
			end
			wire [15:0] w_ff_m;
			wire w_fs_bo_unused;
			wire w_unused_ok = &{1'b0, w_fs_bo_unused};
			am_sub #(.W(4)) u_w_ff_sh (.a(4'd14), .b(w_ff_f_exp), .bi(1'b0), .d(w_ff_sh_raw), .bo(w_fs_bo_unused));
			am_shift #(.W(16), .SW(4), .DIR(1)) u_w_ff_m (.a({1'b1, w_ff_f_sh[14:0]}), .amt(w_ff_sh), .left(1'b0), .arith(1'b0),
				.y(w_ff_m));
			wire [15:0] w_fx_floor =
				(w_ff_f_cmp[23] || w_ff_f_cmp[22:15] < 8'd127) ? 16'd0 :
				(w_ff_f_cmp[22:15] > 8'd142) ? 16'hFFFF : w_ff_m;
			wire [15:0] w_fx_floor_qx, w_fx_floor_qy;
			genvar gfxb;
			for (gfxb = 0; gfxb < 16; gfxb = gfxb + 1) begin : g_fxf_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxf_x (.I0(w_fx_floor[gfxb]), .O(w_fx_floor_qx[gfxb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxf_y (.I0(w_fx_floor[gfxb]), .O(w_fx_floor_qy[gfxb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign w_fx_floor_qx[gfxb] = w_fx_floor[gfxb];
				assign w_fx_floor_qy[gfxb] = w_fx_floor[gfxb];
`else
				assign w_fx_floor_qx[gfxb] = w_fx_floor[gfxb];
				assign w_fx_floor_qy[gfxb] = w_fx_floor[gfxb];
`endif
			end

			wire [4:0] w_fl_lane_h0;
			wire [4:0] w_fl_lane_h1;
			wire [4:0] w_fl_lane_q;
			genvar gln;
			for (gln = 0; gln < 5; gln = gln + 1) begin : g_lane_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wfl0 (.I0(fl_lane_g[gln]), .O(w_fl_lane_h0[gln]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wfl1 (.I0(fl_lane_g[gln]), .O(w_fl_lane_h1[gln]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wflq (.I0(fl_lane_g[gln]), .O(w_fl_lane_q[gln]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign w_fl_lane_h0[gln] = fl_lane_g[gln];
				assign w_fl_lane_h1[gln] = fl_lane_g[gln];
				assign w_fl_lane_q[gln] = fl_lane_g[gln];
`else
				assign w_fl_lane_h0[gln] = fl_lane_g[gln];
				assign w_fl_lane_h1[gln] = fl_lane_g[gln];
				assign w_fl_lane_q[gln]  = fl_lane_g[gln];
`endif
			end
			wire fl_slot_gw0, fl_slot_gw1, fl_go_gw0, fl_go_gw1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fsgw0 (.I0(fl_slot == gw[WARP_W-1:0]), .O(fl_slot_gw0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fsgw1 (.I0(fl_slot == gw[WARP_W-1:0]), .O(fl_slot_gw1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fggw0 (.I0(fl_go), .O(fl_go_gw0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fggw1 (.I0(fl_go), .O(fl_go_gw1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign fl_slot_gw0 = fl_slot == gw[WARP_W-1:0];
			assign fl_slot_gw1 = fl_slot == gw[WARP_W-1:0];
			assign fl_go_gw0 = fl_go;
			assign fl_go_gw1 = fl_go;
`else
			assign fl_slot_gw0 = (fl_slot == gw[WARP_W-1:0]);
			assign fl_slot_gw1 = (fl_slot == gw[WARP_W-1:0]);
			assign fl_go_gw0   = fl_go;
			assign fl_go_gw1   = fl_go;
`endif

			wire w_qx_en  = fl_we && (w_fl_lane_q[1:0] == 2'd0) && (fl_reg == 6'd4) && fl_slot_gw0;
			wire w_qy_en  = fl_we && (w_fl_lane_q[1:0] == 2'd0) && (fl_reg == 6'd5) && fl_slot_gw0;
			wire w_dsv_en = fl_pw_we && (fl_pw_addr[11:2] == 10'h3FE) && fl_slot_gw0;
			wire w_warp_hit = (x_warp_ctrl == gw[WARP_W-1:0]);
			wire w_ex_op_w  = ex_op && w_warp_hit;
			wire w_ex_dep_w = ex_dep && w_warp_hit;
			wire w_ex_sm_w  = ex_sm && w_warp_hit;

			for (gq = 0; gq < 2; gq = gq + 1) begin : g_wh
				wire [4:0] wh_lane = (gq == 0) ? w_fl_lane_h0 : w_fl_lane_h1;
				wire fl_slot_gq = (gq == 0) ? fl_slot_gw0 : fl_slot_gw1;
				wire fl_go_gq   = (gq == 0) ? fl_go_gw0   : fl_go_gw1;
				wire xr_clr_wh, cov_we_s, xp_clr_s, dw_clr_s, smw_clr_s;
				wire w_ex_op_s0, w_ex_op_s1, w_ex_dep_s0, w_ex_dep_s1, w_ex_sm_s0, w_ex_sm_s1;
				wire xp_clr_s0, xp_clr_s1, dw_clr_s0, dw_clr_s1, smw_clr_s0, smw_clr_s1;
				wire w_r6_we_s  = fl_we && (fl_reg == 6'd6) && fl_slot_gq && (wh_lane[4] == gq[0]);
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xrclr (.I0(xr_valid && (xr_warp == gw[WARP_W-1:0])), .O(xr_clr_wh));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_covwe (.I0(fl_go_gq && fl_slot_gq), .O(cov_we_s));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xpc0  (.I0(fl_go_gq && fl_slot_gq), .O(xp_clr_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xpc1  (.I0(fl_go_gq && fl_slot_gq), .O(xp_clr_s1));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dwc0  (.I0(fl_go_gq && fl_slot_gq), .O(dw_clr_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dwc1  (.I0(fl_go_gq && fl_slot_gq), .O(dw_clr_s1));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_smc0  (.I0(fl_go_gq && fl_slot_gq), .O(smw_clr_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_smc1  (.I0(fl_go_gq && fl_slot_gq), .O(smw_clr_s1));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exop0 (.I0(w_ex_op_w),  .O(w_ex_op_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exop1 (.I0(w_ex_op_w),  .O(w_ex_op_s1));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exdp0 (.I0(w_ex_dep_w), .O(w_ex_dep_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exdp1 (.I0(w_ex_dep_w), .O(w_ex_dep_s1));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exsm0 (.I0(w_ex_sm_w),  .O(w_ex_sm_s0));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_exsm1 (.I0(w_ex_sm_w),  .O(w_ex_sm_s1));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign xr_clr_wh = xr_valid && (xr_warp == gw[WARP_W-1:0]);
				assign cov_we_s = fl_go_gq && fl_slot_gq;
				assign xp_clr_s0 = fl_go_gq && fl_slot_gq;
				assign xp_clr_s1 = fl_go_gq && fl_slot_gq;
				assign dw_clr_s0 = fl_go_gq && fl_slot_gq;
				assign dw_clr_s1 = fl_go_gq && fl_slot_gq;
				assign smw_clr_s0 = fl_go_gq && fl_slot_gq;
				assign smw_clr_s1 = fl_go_gq && fl_slot_gq;
				assign w_ex_op_s0 = w_ex_op_w;
				assign w_ex_op_s1 = w_ex_op_w;
				assign w_ex_dep_s0 = w_ex_dep_w;
				assign w_ex_dep_s1 = w_ex_dep_w;
				assign w_ex_sm_s0 = w_ex_sm_w;
				assign w_ex_sm_s1 = w_ex_sm_w;
`else
				assign xr_clr_wh   = (xr_valid && (xr_warp == gw[WARP_W-1:0]));
				assign cov_we_s    = fl_go_gq && fl_slot_gq;
				assign xp_clr_s0   = fl_go_gq && fl_slot_gq;
				assign xp_clr_s1   = fl_go_gq && fl_slot_gq;
				assign dw_clr_s0   = fl_go_gq && fl_slot_gq;
				assign dw_clr_s1   = fl_go_gq && fl_slot_gq;
				assign smw_clr_s0  = fl_go_gq && fl_slot_gq;
				assign smw_clr_s1  = fl_go_gq && fl_slot_gq;
				assign w_ex_op_s0  = w_ex_op_w;
				assign w_ex_op_s1  = w_ex_op_w;
				assign w_ex_dep_s0 = w_ex_dep_w;
				assign w_ex_dep_s1 = w_ex_dep_w;
				assign w_ex_sm_s0  = w_ex_sm_w;
				assign w_ex_sm_s1  = w_ex_sm_w;
`endif
				assign xp_clr_s  = xp_clr_s0;
				assign dw_clr_s  = dw_clr_s0;
				assign smw_clr_s = smw_clr_s0;
				(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] r6w_sl, cov_sl, xp_sl, dw_sl, smw_sl;
				wire [15:0] side_w_sl;
				genvar gsw;
				for (gsw = 0; gsw < 16; gsw = gsw + 1) begin : g_sw_buf
`ifdef SYNTH_KINTEX7
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_swsl (.I0(side_w[gq*16 + gsw]), .O(side_w_sl[gsw]));
`elsif SYNTH_KINTEX_ULTRASCALE
					assign side_w_sl[gsw] = side_w[gq*16 + gsw];
`else
					assign side_w_sl[gsw] = side_w[gq*16 + gsw];
`endif
				end
				integer ib;
				always @(posedge clk or posedge rst_w) begin
					if (rst_w) begin
						r6w_sl <= 16'd0;
						cov_sl <= 16'd0;
						xp_sl  <= 16'd0;
						dw_sl  <= 16'd0;
						smw_sl <= 16'd0;
					end else begin
						if (xr_clr_wh) begin
							r6w_sl <= 16'd0;
						end else if (w_r6_we_s) begin
							for (ib = 0; ib < 16; ib = ib + 1)
								if (wh_lane[3:0] == ib[3:0])
									r6w_sl[ib] <= 1'b1;
						end
						if (cov_we_s)
							cov_sl <= fl_info[gq*16 +: 16];
						if (xp_clr_s)
							xp_sl[7:0] <= 8'd0;
						else if (w_ex_op_s0)
							xp_sl[7:0] <= xp_sl[7:0] | side_w_sl[7:0];
						if (xp_clr_s1)
							xp_sl[15:8] <= 8'd0;
						else if (w_ex_op_s1)
							xp_sl[15:8] <= xp_sl[15:8] | side_w_sl[15:8];
						if (dw_clr_s)
							dw_sl[7:0] <= 8'd0;
						else if (w_ex_dep_s0)
							dw_sl[7:0] <= dw_sl[7:0] | side_w_sl[7:0];
						if (dw_clr_s1)
							dw_sl[15:8] <= 8'd0;
						else if (w_ex_dep_s1)
							dw_sl[15:8] <= dw_sl[15:8] | side_w_sl[15:8];
						if (smw_clr_s)
							smw_sl[7:0] <= 8'd0;
						else if (w_ex_sm_s0)
							smw_sl[7:0] <= smw_sl[7:0] | side_w_sl[7:0];
						if (smw_clr_s1)
							smw_sl[15:8] <= 8'd0;
						else if (w_ex_sm_s1)
							smw_sl[15:8] <= smw_sl[15:8] | side_w_sl[15:8];
					end
				end
				assign r6w[gw*32 + gq*16 +: 16] = r6w_sl;
				assign cov[gw*32 + gq*16 +: 16] = cov_sl;
				assign xp[gw*32 + gq*16 +: 16]  = xp_sl;
				assign dw[gw*32 + gq*16 +: 16]  = dw_sl;
				assign smw[gw*32 + gq*16 +: 16] = smw_sl;
			end
			for (gq = 0; gq < 8; gq = gq + 1) begin : g_wq
				wire xr_clr_wq, dsv_we_s, qx_we_s, qy_we_s;
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xrcq (.I0(xr_valid && (xr_warp == gw[WARP_W-1:0])), .O(xr_clr_wq));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dsvw (.I0(w_dsv_en && (fl_pw_addr[1:0] == gq[2:1])), .O(dsv_we_s));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qxw  (.I0(w_qx_en && (w_fl_lane_q[4:2] == gq[2:0])), .O(qx_we_s));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qyw  (.I0(w_qy_en && (w_fl_lane_q[4:2] == gq[2:0])), .O(qy_we_s));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign xr_clr_wq = xr_valid && (xr_warp == gw[WARP_W-1:0]);
				assign dsv_we_s = w_dsv_en && (fl_pw_addr[1:0] == gq[2:1]);
				assign qx_we_s = w_qx_en && (w_fl_lane_q[4:2] == gq[2:0]);
				assign qy_we_s = w_qy_en && (w_fl_lane_q[4:2] == gq[2:0]);
`else
				assign xr_clr_wq = (xr_valid && (xr_warp == gw[WARP_W-1:0]));
				assign dsv_we_s  = w_dsv_en && (fl_pw_addr[1:0] == gq[2:1]);
				assign qx_we_s   = w_qx_en && (w_fl_lane_q[4:2] == gq[2:0]);
				assign qy_we_s   = w_qy_en && (w_fl_lane_q[4:2] == gq[2:0]);
`endif
				(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] dsv_sl, qx_sl, qy_sl;
				always @(posedge clk or posedge rst_w) begin
					if (rst_w) begin
						dsv_sl <= 16'd0;
						qx_sl  <= 16'd0;
						qy_sl  <= 16'd0;
					end else begin
						if (xr_clr_wq)
							dsv_sl <= 16'd0;
						else if (dsv_we_s)
							dsv_sl <= fl_pw_data[(gq[0] * 16) +: 16];
						if (qx_we_s)
							qx_sl <= w_fx_floor_qx;
						if (qy_we_s)
							qy_sl <= w_fx_floor_qy;
					end
				end
				assign dsv[gw*128 + gq*16 +: 16] = dsv_sl;
				assign qx[(gw*8 + gq)*16 +: 16]  = qx_sl;
				assign qy[(gw*8 + gq)*16 +: 16]  = qy_sl;
			end
		end
	endgenerate

	wire side_any = (side_cmp != {NPHYS{1'b0}});
	wire side_any_rtw0, side_any_rtw1, side_any_has1;
	wire ex_col_w0, ex_col_w1;
	wire [WARP_W-1:0] x_warp_rtw_g [0:7];
	wire [WARP_W-1:0] x_warp_rtw_b;
	wire fl_go_fr, fl_go_has1;
	wire [2:0] tk_rtw_g [0:NWARPS-1];
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sart0 (.I0(side_any), .O(side_any_rtw0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sart1 (.I0(side_any), .O(side_any_rtw1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sahas (.I0(side_any), .O(side_any_has1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_excw0 (.I0(ex_col),   .O(ex_col_w0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_excw1 (.I0(ex_col),   .O(ex_col_w1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fgfr  (.I0(fl_go),    .O(fl_go_fr));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fgh1  (.I0(fl_go),    .O(fl_go_has1));
	genvar gxwr, gxwg;
	generate
		for (gxwg = 0; gxwg < NWARPS; gxwg = gxwg + 1) begin : g_tkrtw_g
			for (gxwr = 0; gxwr < 3; gxwr = gxwr + 1) begin : g_tkrtw_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tkrg (.I0(tk_rtw[gxwr]), .O(tk_rtw_g[gxwg][gxwr]));
			end
		end
		for (gxwg = 0; gxwg < 8; gxwg = gxwg + 1) begin : g_xwrtw_g
			for (gxwr = 0; gxwr < WARP_W; gxwr = gxwr + 1) begin : g_xwrtw_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xwra (.I0(x_warp_rtw[gxwr]), .O(x_warp_rtw_g[gxwg][gxwr]));
			end
		end
		for (gxwr = 0; gxwr < WARP_W; gxwr = gxwr + 1) begin : g_xwrtw_ab
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xwrb (.I0(x_warp_rtw[gxwr]), .O(x_warp_rtw_b[gxwr]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	assign side_any_rtw0 = side_any;
	assign side_any_rtw1 = side_any;
	assign side_any_has1 = side_any;
	assign ex_col_w0 = ex_col;
	assign ex_col_w1 = ex_col;
	assign fl_go_fr = fl_go;
	assign fl_go_has1 = fl_go;
	genvar gxwr, gxwg;
	generate
		for (gxwg = 0; gxwg < NWARPS; gxwg = gxwg + 1) begin : g_tkrtw_g
			for (gxwr = 0; gxwr < 3; gxwr = gxwr + 1) begin : g_tkrtw_b
				assign tk_rtw_g[gxwg][gxwr] = tk_rtw[gxwr];
			end
		end
		for (gxwg = 0; gxwg < 8; gxwg = gxwg + 1) begin : g_xwrtw_g
			for (gxwr = 0; gxwr < WARP_W; gxwr = gxwr + 1) begin : g_xwrtw_b
				assign x_warp_rtw_g[gxwg][gxwr] = x_warp_rtw[gxwr];
			end
		end
		for (gxwr = 0; gxwr < WARP_W; gxwr = gxwr + 1) begin : g_xwrtw_ab
			assign x_warp_rtw_b[gxwr] = x_warp_rtw[gxwr];
		end
	endgenerate
`else
	assign side_any_rtw0 = side_any;
	assign side_any_rtw1 = side_any;
	assign side_any_has1 = side_any;
	assign ex_col_w0     = ex_col;
	assign ex_col_w1     = ex_col;
	assign fl_go_fr      = fl_go;
	assign fl_go_has1    = fl_go;
	genvar gxwg;
	generate
		for (gxwg = 0; gxwg < NWARPS; gxwg = gxwg + 1) begin : g_tkrtw_g
			assign tk_rtw_g[gxwg] = tk_rtw;
		end
		for (gxwg = 0; gxwg < 8; gxwg = gxwg + 1) begin : g_xwrtw_g
			assign x_warp_rtw_g[gxwg] = x_warp_rtw;
		end
	endgenerate
	assign x_warp_rtw_b  = x_warp_rtw;
`endif


	wire rst_p = !rst_n;
	genvar grtw;
	generate
		for (grtw = 0; grtw < NWARPS; grtw = grtw + 1) begin : g_rtw_w
			wire [WARP_W-1:0] xwr_s = x_warp_rtw_g[grtw / 2];
			wire [2:0] tk_rtw_s = tk_rtw_g[grtw];
			wire [7:0] tk_rtw_oh = {
				(tk_rtw_s == 3'd7), (tk_rtw_s == 3'd6), (tk_rtw_s == 3'd5), (tk_rtw_s == 3'd4),
				(tk_rtw_s == 3'd3), (tk_rtw_s == 3'd2), (tk_rtw_s == 3'd1), (tk_rtw_s == 3'd0)
			};
			wire rtw_set = (xwr_s[1] ? ex_col_w1 : ex_col_w0) &&
				(xwr_s[1] ? side_any_rtw1 : side_any_rtw0) && (xwr_s == grtw[WARP_W-1:0]);
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					fr[grtw] <= 1'b0;
					rtw[grtw*8 +: 8] <= 8'd0;
				end else begin
					if (fl_go_fr && (fl_slot_top == grtw[WARP_W-1:0]))
						fr[grtw] <= fl_info[48];
					if ((fl_go_fr && (fl_slot_top == grtw[WARP_W-1:0])) || rtw_set) begin
						rtw[grtw*8 +: 8] <= (fl_go_fr && (fl_slot_top == grtw[WARP_W-1:0]))
							? (rtw_set ? tk_rtw_oh : 8'd0)
							: (rtw[grtw*8 +: 8] | tk_rtw_oh);
					end
				end
			end
		end
	endgenerate

	generate
		if (DUAL_SOURCE != 0) begin : g_has_src1_reg
			reg [NWARPS-1:0] has_src1_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					has_src1_r <= {NWARPS{1'b0}};
				end else begin
					if (fl_go_has1)
						has_src1_r[fl_slot_top] <= 1'b0;
					if (ex_col1 && side_any_has1)
						has_src1_r[x_warp_rtw_b] <= 1'b1;
				end
			end
			assign has_src1_w = has_src1_r;
		end else begin : g_no_has_src1_reg
			assign has_src1_w = {NWARPS{1'b0}};
		end
	endgenerate

	/* ---- Unified Meshlet Primitive Export Sequencer */
	wire ex_mesh = r_x_valid && r_x_unit == ATG_UNIT_EXP && tgt == 32'h200;
	wire [7:0] mesh_k_prim = r_x_b[7:0];
	wire [7:0] mesh_k_vert = r_x_b[15:8];
	wire [9:0] mesh_pb_10 = 10'd512;
	wire [LDS_AW-1:0] mesh_k_pbase = {{(LDS_AW > 10 ? LDS_AW - 10 : 0){1'b0}}, mesh_pb_10[LDS_AW > 10 ? 9 : LDS_AW-1 : 0]};

	wire mesh_done_unused;

	wire [9:0] mesh_vb_10 = 10'd256;
	wire [LDS_AW-1:0] mesh_k_vbase = {{(LDS_AW > 10 ? LDS_AW - 10 : 0){1'b0}}, mesh_vb_10[LDS_AW > 10 ? 9 : LDS_AW-1 : 0]};

	generate
	if (HAS_MESH != 0) begin : g_mesh
		atg_su_exp_mesh #(.AW(LDS_AW), .DATA_W(32)) u_mesh (
			.clk(clk), .rst_n(rst_n),
			.kick_valid(ex_mesh),
			.kick_mesh_id(r_x_info[15:0]),
			.kick_prim_count(mesh_k_prim),
			.kick_vert_count(mesh_k_vert),
			.kick_prim_base(mesh_k_pbase),
			.kick_pos_base({LDS_AW{1'b0}}),
			.kick_var_base(mesh_k_vbase),
			.busy(mesh_busy),
			.done(mesh_done_unused),
			.lds_rd_en(mesh_lds_rd_en),
			.lds_rd_addr(mesh_lds_rd_addr),
			.lds_rd_data(mesh_lds_rd_data),
			.mesh_valid(mesh_valid),
			.mesh_ready(mesh_ready),
			.mesh_tri(mesh_tri),
			.mesh_last(mesh_last),
			.mesh_id(mesh_id)
		);
	end else begin : g_no_mesh
		assign mesh_busy = 1'b0;
		assign mesh_done_unused = 1'b0;
		assign mesh_lds_rd_en = 1'b0;
		assign mesh_lds_rd_addr = {LDS_AW{1'b0}};
		assign mesh_valid = 1'b0;
		assign mesh_tri = 702'd0;
		assign mesh_last = 1'b0;
		assign mesh_id = 16'd0;
		wire unused_no_mesh = &{1'b0, ex_mesh, mesh_k_prim, mesh_k_vert, mesh_k_pbase,
			mesh_k_vbase, mesh_ready, mesh_lds_rd_data};
	end
	endgenerate

	wire unused_ok = &{1'b0, x_last, x_sbe, x_pc, x_a, x_c, x_mask[NPHYS-1:0], x_info[31:18],
		x_uop, fl_info[63:49], fl_data[7:0], tgt, xrow, lane0, zrow, drow, fq, dqs, mesh_done_unused,
		mesh_pb_10, mesh_vb_10, dwarp, dq, dcrow, dq_rep[7*3 +: 3]};
endmodule

module atg_su_exp_ch_mux #(
	parameter NPHYS = 8,
	parameter CH_W = 192,
	parameter HI = 0,
	parameter integer CH_IDX = 0
) (
	input wire [NPHYS*CH_W-1:0] all_rdat,
	input wire [4:0] l_idx,
	output reg [7:0] byte_out
);
	localparam OFF = CH_IDX * 16 + (HI ? 8 : 0);
	integer k;
	always @(*) begin
		byte_out = 8'd0;
		for (k = 0; k < NPHYS; k = k + 1) begin
			if (l_idx[4:0] == k[4:0])
				byte_out = all_rdat[k * CH_W + OFF +: 8];
		end
	end
endmodule

`default_nettype wire
