/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_lds: Shared memory (LDS): lds/sts/atom (the shared form, atom sub[4])
 * per lane, LDS_BYTES of block RAM per SU (4 KiB by default, 16 KiB KC705),
 * word addressed; returns as long beats.
 *
 * One lane group at a time (x_ready is 1 only in IDLE).  The word
 * address of lane l is ((rs1 + imm) >> 2) mod LDS_BYTES/4 (atom: rs1, no
 * immediate).  The lanes go through the RAM's one port one per cycle
 * (lane 0 first; for sts a later lane wins on the same word): lds reads
 * the EXEC lanes (the read data is registered, one more cycle at the
 * end), sts writes the side-effect lanes; atom takes two cycles per
 * side-effect lane, the read and then the write of the new value
 * (docs/ghci.md section 12.5: sub[3:0] the operation, rs2 the operand, rs3
 * cmpxchg's compare value), so each lane's read-modify-write is whole
 * before the next lane's read: lanes on the same word act in lane order,
 * and as the unit takes one group at a time, every atom is indivisible
 * with respect to every other LDS access.  Then one beat: lds l_wr 1 with
 * the EXEC lanes, atom l_wr = the uop's WRD with the side-effect lanes
 * (the old values), sts l_wr 0; l_last = x_last, class m.  A group without
 * lanes returns at once.
 *
 * States: IDLE →(group)→ ACCESS (NPHYS cycles, atom 2 NPHYS) → DRAIN →
 * BEAT →(l_ready)→ IDLE.  scrub (session protection, the unit idle) writes
 * 0 to every word through the RAM's write port, one per cycle, while
 * scrub_busy.
*/

`timescale 1ns / 1ps
`default_nettype none

module atg_su_lds #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter LDS_BYTES = 4096	/* Size in bytes */
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
	/* Long result to atg_su_wb (any time; wb may refuse with l_ready = 0) */
	output wire l_valid,
	input wire l_ready,
	output wire [WARP_W-1:0] l_warp,
	output wire [GRP_W-1:0] l_grp,
	output wire [6:0] l_rd,
	output wire [NPHYS*32-1:0] l_data,
	output wire [NPHYS-1:0] l_lanes,
	output wire l_wr,
	output wire l_last,
	output wire [SBE_W:0] l_sbe,
	output wire [2:0] l_cls,
	/* Session protection: clear the contents */
	input wire scrub,
	output wire scrub_busy,
	/* Port B: Mesh Read Port */
	input wire mesh_rd_en,
	input wire [$clog2(LDS_BYTES/4)-1:0] mesh_rd_addr,
	output wire [31:0] mesh_rd_data
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam NW = LDS_BYTES / 4;
	localparam AW = $clog2(NW);
	localparam LW = NPHYS > 1 ? $clog2(NPHYS) : 1;
	localparam [2:0] IDLE = 3'd0, ACCESS = 3'd1, DRAIN1 = 3'd2, DRAIN2 = 3'd3,
		DRAIN3 = 3'd4, BEAT = 3'd5;

`ifdef SYNTH_KINTEX7
	(* ram_style = "block" *)
	/* synthesis syn_ramstyle = "block" */
`elsif SYNTH_KINTEX_ULTRASCALE
	(* ram_style = "block" *)
	/* synthesis syn_ramstyle = "block" */
`else
	(* ram_style = "block" *)
`endif
	reg [31:0] ram [0:NW-1];
	integer init_i;
	initial begin
		for (init_i = 0; init_i < NW; init_i = init_i + 1)
			ram[init_i] = 32'd0;
	end
	wire rst_p = !rst_n;
	wire scrub_sc, scrub_we, scrub_ctl;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_scsc  (.I0(scrub), .O(scrub_sc));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_scwe  (.I0(scrub), .O(scrub_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_scctl (.I0(scrub), .O(scrub_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign scrub_sc = scrub;
	assign scrub_we = scrub;
	assign scrub_ctl = scrub;
`else
	assign scrub_sc  = scrub;
	assign scrub_we  = scrub;
	assign scrub_ctl = scrub;
`endif
	wire lds_take_raw = !scrub_ctl && !sc_on && (st == IDLE) && mine && !ram_we_r;
	wire lds_take_g0, lds_take_g1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ltk_g0 (.I0(lds_take_raw), .O(lds_take_g0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ltk_g1 (.I0(lds_take_raw), .O(lds_take_g1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lds_take_g0 = lds_take_raw;
	assign lds_take_g1 = lds_take_raw;
`else
	assign lds_take_g0 = lds_take_raw;
	assign lds_take_g1 = lds_take_raw;
`endif
	(* max_fanout = 12 *) reg [2:0] st;
	(* keep = "true", dont_touch = "true" *) reg [NPHYS-1:0] lds_idle_rep;
	wire [NPHYS*AW-1:0] radr;
	wire [DW-1:0] rdat, rcmp, res;
	(* max_fanout = 12 *) reg [NPHYS-1:0] lanes;
	(* shreg_extract = "no" *) reg [15:0] atom_b_lo, atom_b_hi, atom_c_lo, atom_c_hi;
	wire [31:0] atom_b = {atom_b_hi, atom_b_lo};
	wire [31:0] atom_c = {atom_c_hi, atom_c_lo};
	(* max_fanout = 12 *) reg rsts, ratom, rwrd;
	(* max_fanout = 12 *) reg [2:0] aph;
	(* max_fanout = 12 *) reg [3:0] rsub;
	reg [6:0] rrd;
	reg [WARP_W-1:0] rwarp;
	reg [GRP_W-1:0] rgrp;
	reg rlast;
	reg [SBE_W:0] rsbe;
	(* shreg_extract = "no", max_fanout = 12 *) reg [LW-1:0] li, pli_1, pli_2, pli_3;
	(* shreg_extract = "no", keep = "true" *) reg [LW-1:0] li_radr, li_lane, li_rdat_lo, li_rdat_hi, li_rcmp_lo, li_rcmp_hi, li_atom;
	reg [NPHYS-1:0] li_radr_oh;
	(* dont_touch = "true" *) reg [NPHYS-1:0] li_radr_oh_hi;
	reg sc_done_r;
	reg [31:0] asum_r, w_other_r;
	(* shreg_extract = "no", max_fanout = 12 *) reg pval_1, pval_2, pval_3;
	reg [AW-1:0] ram_addr_r;
	(* max_fanout = 12 *) reg ram_we_r;
	wire [31:0] ram_wd_r;
	reg [31:0] q;
	(* dont_touch = "true" *) reg [31:0] q_pipe;

	(* max_fanout = 12 *) wire mine = x_valid && x_uop[ATG_UOP_UNIT +: 4] == ATG_UNIT_LDS;
	(* max_fanout = 12 *) wire is_sts = x_uop[ATG_UOP_OP +: 8] == ATG_LDS_STS;
	(* max_fanout = 12 *) wire is_atom = x_uop[ATG_UOP_OP +: 8] == ATG_LDS_ATOM;
	wire [NPHYS-1:0] xl = is_sts || is_atom ? x_mask[2*NPHYS-1:NPHYS] : x_mask[NPHYS-1:0];
	(* max_fanout = 12 *) wire [31:0] imm = is_atom ? 32'd0 : x_uop[ATG_UOP_IMM +: 32];
	/* Lane li's word address (parallel 4-lane group tree using pre-decoded 1-hot li_radr_oh). */
	localparam integer NG_ADR = (NPHYS >= 32) ? 8 : (NPHYS >= 16) ? 4 : (NPHYS >= 8) ? 2 : 1;
	wire [NG_ADR*AW-1:0] a_grp;
	genvar gg_a, gl_a;
	generate
		for (gg_a = 0; gg_a < NG_ADR; gg_a = gg_a + 1) begin : g_adr_grp
			wire [3:0] gm;
			for (gl_a = 0; gl_a < 4; gl_a = gl_a + 1) begin : g_adr_ln
				localparam integer LI_A = gg_a * 4 + gl_a;
				assign gm[gl_a] = (LI_A < NPHYS) && ((NG_ADR > 1 && gg_a < (NG_ADR/2)) ? li_radr_oh[LI_A] : li_radr_oh_hi[LI_A]);
			end
			assign a_grp[gg_a*AW +: AW] =
				(gm[0] ? radr[((gg_a*4 + 0) < NPHYS ? (gg_a*4 + 0) : 0)*AW +: AW] : {AW{1'b0}}) |
				(gm[1] ? radr[((gg_a*4 + 1) < NPHYS ? (gg_a*4 + 1) : 0)*AW +: AW] : {AW{1'b0}}) |
				(gm[2] ? radr[((gg_a*4 + 2) < NPHYS ? (gg_a*4 + 2) : 0)*AW +: AW] : {AW{1'b0}}) |
				(gm[3] ? radr[((gg_a*4 + 3) < NPHYS ? (gg_a*4 + 3) : 0)*AW +: AW] : {AW{1'b0}});
		end
	endgenerate
	reg [AW-1:0] a_mux;
	integer ga;
	always @(*) begin
		a_mux = {AW{1'b0}};
		for (ga = 0; ga < NG_ADR; ga = ga + 1)
			a_mux = a_mux | a_grp[ga*AW +: AW];
	end
	(* max_fanout = 12 *) wire [AW-1:0] a = a_mux;
	(* max_fanout = 12 *) wire lane_on = lanes[li_lane];
	/* An atom lane's read-modify-write uses q_pipe (dedicated fabric register after BRAM q). */
	wire [LW-1:0] li_rdat_lo0, li_rdat_lo1, li_rdat_hi0, li_rdat_hi1;
	wire [LW-1:0] li_rcmp_lo0, li_rcmp_lo1, li_rcmp_hi0, li_rcmp_hi1;
`ifdef SYNTH_KINTEX7
	genvar glrb;
	generate
		for (glrb = 0; glrb < LW; glrb = glrb + 1) begin : g_lird_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrdl0 (.I0(li_rdat_lo[glrb]), .O(li_rdat_lo0[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrdl1 (.I0(li_rdat_lo[glrb]), .O(li_rdat_lo1[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrdh0 (.I0(li_rdat_hi[glrb]), .O(li_rdat_hi0[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrdh1 (.I0(li_rdat_hi[glrb]), .O(li_rdat_hi1[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrcl0 (.I0(li_rcmp_lo[glrb]), .O(li_rcmp_lo0[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrcl1 (.I0(li_rcmp_lo[glrb]), .O(li_rcmp_lo1[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrch0 (.I0(li_rcmp_hi[glrb]), .O(li_rcmp_hi0[glrb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lrch1 (.I0(li_rcmp_hi[glrb]), .O(li_rcmp_hi1[glrb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar glrb;
	generate
		for (glrb = 0; glrb < LW; glrb = glrb + 1) begin : g_lird_buf
			assign li_rdat_lo0[glrb] = li_rdat_lo[glrb];
			assign li_rdat_lo1[glrb] = li_rdat_lo[glrb];
			assign li_rdat_hi0[glrb] = li_rdat_hi[glrb];
			assign li_rdat_hi1[glrb] = li_rdat_hi[glrb];
			assign li_rcmp_lo0[glrb] = li_rcmp_lo[glrb];
			assign li_rcmp_lo1[glrb] = li_rcmp_lo[glrb];
			assign li_rcmp_hi0[glrb] = li_rcmp_hi[glrb];
			assign li_rcmp_hi1[glrb] = li_rcmp_hi[glrb];
		end
	endgenerate
`else
	assign li_rdat_lo0 = li_rdat_lo;
	assign li_rdat_lo1 = li_rdat_lo;
	assign li_rdat_hi0 = li_rdat_hi;
	assign li_rdat_hi1 = li_rdat_hi;
	assign li_rcmp_lo0 = li_rcmp_lo;
	assign li_rcmp_lo1 = li_rcmp_lo;
	assign li_rcmp_hi0 = li_rcmp_hi;
	assign li_rcmp_hi1 = li_rcmp_hi;
`endif
	wire [31:0] cur_rdat = {
		rdat[{li_rdat_hi1, 5'd24} +: 8], rdat[{li_rdat_hi0, 5'd16} +: 8],
		rdat[{li_rdat_lo1, 5'd8}  +: 8], rdat[{li_rdat_lo0, 5'd0}  +: 8]
	};
	wire [31:0] cur_rcmp = {
		rcmp[{li_rcmp_hi1, 5'd24} +: 8], rcmp[{li_rcmp_hi0, 5'd16} +: 8],
		rcmp[{li_rcmp_lo1, 5'd8}  +: 8], rcmp[{li_rcmp_lo0, 5'd0}  +: 8]
	};
	wire [15:0] asum_lo_s, asum_hi_s;
	wire        asum_lo_co, asum_hi_co_unused;
	(* dont_touch = "true" *) reg  [15:0] asum_lo_r;
	(* dont_touch = "true" *) reg         asum_cy_r;
	wire        atom_is_sub = (rsub == 4'd5);
	wire [31:0] atom_b_as = atom_is_sub ? ~atom_b : atom_b;
	am_add #(.W(16)) u_asum_lo (.a(q_pipe[15:0]),  .b(atom_b_as[15:0]),  .ci(atom_is_sub), .s(asum_lo_s), .co(asum_lo_co));
	am_add #(.W(16)) u_asum_hi (.a(q_pipe[31:16]), .b(atom_b_as[31:16]), .ci(asum_cy_r),   .s(asum_hi_s), .co(asum_hi_co_unused));
	wire [31:0] asum = {asum_hi_s, asum_lo_r};

	/* Atomic comparisons (fanout = 2 each into the two 16-bit write-data slices). */
`ifdef MUT_LDS_ATOM_NOCMP
	wire [31:0] atom_cmp_rhs = atom_b;
`else
	wire [31:0] atom_cmp_rhs = atom_c;
`endif
	/* Parallel 4-byte unsigned/signed compare registered at aph==3, reduced at aph==4 (<= 9 cmos3). */
	wire [3:0] cmp_eq_b, cmp_lt_b;
	genvar gcb;
	generate
		for (gcb = 0; gcb < 4; gcb = gcb + 1) begin : g_atom_cmp_b
			assign cmp_eq_b[gcb] = (q_pipe[gcb*8 +: 8] == atom_b[gcb*8 +: 8]);
			assign cmp_lt_b[gcb] = (q_pipe[gcb*8 +: 8] <  atom_b[gcb*8 +: 8]);
		end
	endgenerate
	reg [3:0] cmp_eq_b_r, cmp_lt_b_r;
	reg       cmp_eq_r, cmp_sign_xor_r, cmp_old31_r;
	reg       rsub_is_sgt_r, rsub_is_slt_r, rsub_is_ugt_r, rsub_is_ult_r, rsub_is_cas_r, rsub_is_const1_r;
	wire cmp_ab_eq = &cmp_eq_b_r;
	wire cmp_ult_t3 = cmp_lt_b_r[3];
	wire cmp_ult_t2 = cmp_eq_b_r[3] && cmp_lt_b_r[2];
	wire cmp_ult_t1 = cmp_eq_b_r[3] && cmp_eq_b_r[2] && cmp_lt_b_r[1];
	wire cmp_ult_t0 = cmp_eq_b_r[3] && cmp_eq_b_r[2] && cmp_eq_b_r[1] && cmp_lt_b_r[0];
	wire cmp_ult = cmp_ult_t3 || cmp_ult_t2 || cmp_ult_t1 || cmp_ult_t0;
	wire cmp_want_ult   = rsub_is_ult_r || (rsub_is_slt_r && !cmp_sign_xor_r);
	wire cmp_want_ugt   = rsub_is_ugt_r || (rsub_is_sgt_r && !cmp_sign_xor_r);
	wire cmp_want_c1    = rsub_is_const1_r || (rsub_is_cas_r && !cmp_eq_r) ||
		(cmp_sign_xor_r && ((rsub_is_sgt_r && !cmp_old31_r) || (rsub_is_slt_r && cmp_old31_r)));
	wire use_old_flat   = cmp_want_c1 || (cmp_want_ult && cmp_ult) || (cmp_want_ugt && !cmp_ult && !cmp_ab_eq);
	wire cmp_eq  = cmp_eq_r;
	wire cmp_eq_w = (q_pipe == atom_cmp_rhs);
	wire unused_cmp_diff = 1'b0;

`ifdef MUT_LDS_ATOM_RAW
	wire atom_active = 1'b0;
`else
	wire atom_active = ratom && lane_on;
`endif
	assign x_ready = st == IDLE && !ram_we_r;

	/* Session protection scrub counter. */
	wire sc_on;
	wire [AW-1:0] sc_a;

	atg_su_scrub #(.DEPTH(NW), .AW(AW)) sc (
		.clk(clk), .rst_n(rst_n), .start(scrub_sc), .on(sc_on), .a(sc_a)
	);

	/* The word addresses of the group on x_*: (rs1 + imm) / 4, its low AW bits. */
	(* dont_touch = "true" *) wire lds_atom_base = !scrub_ctl && !sc_on && (st == ACCESS) && ratom && (aph == 3'd3);
	wire [NPHYS*AW-1:0] xadr;
	genvar l, glh;
	generate
		for (l = 0; l < NPHYS; l = l + 1) begin : g_adr
			wire [AW+1:0] imm_l;
`ifdef SYNTH_KINTEX7
			genvar gbimm;
			for (gbimm = 0; gbimm < AW+2; gbimm = gbimm + 1) begin : g_buf_imm
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_imm (.I0(imm[gbimm]), .O(imm_l[gbimm]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gbimm;
			for (gbimm = 0; gbimm < AW+2; gbimm = gbimm + 1) begin : g_buf_imm
				assign imm_l[gbimm] = imm[gbimm];
			end
`else
			assign imm_l = imm[AW+1:0];
`endif
			wire [AW+1:0] ba;
			wire co_unused;
			am_add #(.W(AW + 2)) u_ba (.a(x_a[l*32 +: AW + 2]), .b(imm_l), .ci(1'b0), .s(ba),
				.co(co_unused));
			assign xadr[l*AW +: AW] = ba[AW+1:2];
			wire unused_b = &{1'b0, ba[1:0]};

			wire lds_take_l;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_a (.I0((l < NPHYS/2) ? lds_take_g0 : lds_take_g1), .O(lds_take_l));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign lds_take_l = (l < NPHYS/2) ? lds_take_g0 : lds_take_g1;
`else
			assign lds_take_l  = lds_idle_rep[l];
`endif
			(* shreg_extract = "no" *) reg [AW-1:0] radr_l_r;
			always @(posedge clk)
				if (lds_take_l)
					radr_l_r <= xadr[l*AW +: AW];
			assign radr[l*AW +: AW] = radr_l_r;

			for (glh = 0; glh < 2; glh = glh + 1) begin : g_lds_half
				wire lds_take_dat, lds_take_cmp, lds_take_res;
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_dat (.I0(lds_take_l), .O(lds_take_dat));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_cmp (.I0(lds_take_l), .O(lds_take_cmp));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_res (.I0(lds_take_l), .O(lds_take_res));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign lds_take_dat = lds_take_l;
				assign lds_take_cmp = lds_take_l;
				assign lds_take_res = lds_take_l;
`else
				assign lds_take_dat = lds_take_l;
				assign lds_take_cmp = lds_take_l;
				assign lds_take_res = lds_take_l;
`endif
				wire lds_atom_we_h = lds_atom_base && lanes[l] && (li == l[LW-1:0]);
				wire lds_rd_we_h   = pval_3 && (pli_3 == l[LW-1:0]);
				(* dont_touch = "true" *) wire lds_q_we_h = lds_rd_we_h || lds_atom_we_h;
				(* shreg_extract = "no" *) reg [15:0] rdat_h_r, rcmp_h_r, res_h_r;
				always @(posedge clk) begin
					if (lds_take_dat)
						rdat_h_r <= x_b[l*32 + glh*16 +: 16];
					if (lds_take_cmp)
						rcmp_h_r <= x_c[l*32 + glh*16 +: 16];
					if (lds_take_res)
						res_h_r <= 16'd0;
					else if (lds_q_we_h)
						res_h_r <= q_pipe[glh*16 +: 16];
				end
				assign rdat[l*32 + glh*16 +: 16] = rdat_h_r;
				assign rcmp[l*32 + glh*16 +: 16] = rcmp_h_r;
				assign res[l*32 + glh*16 +: 16]  = res_h_r;
			end
		end

		for (glh = 0; glh < 2; glh = glh + 1) begin : g_wd_half
			(* dont_touch = "true" *) wire [3:0] rsub_h = rsub;
			(* dont_touch = "true" *) wire use_old_h = use_old_flat;
			wire [15:0] old_h = q_pipe[glh*16 +: 16];
			wire [15:0] b_h   = atom_b[glh*16 +: 16];
			(* dont_touch = "true" *) wire [15:0] w_other_h =
				(rsub_h == 4'd0) ? (old_h & b_h) :
				(rsub_h == 4'd1) ? (old_h | b_h) :
				(rsub_h == 4'd2) ? (old_h ^ b_h) :
				(use_old_h ? old_h : b_h);
			wire wd_zero_h, wd_atom_h, wd_sts_h, is_addsub_h, wr_atom_en_0, wr_atom_en_1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ias  (.I0(rsub_h == 4'd4 || rsub_h == 4'd5),                    .O(is_addsub_h));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wdz  (.I0(scrub_we || sc_on),                                .O(wd_zero_h));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wda  (.I0((st == ACCESS) && atom_active && (aph == 3'd5)),       .O(wd_atom_h));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wds  (.I0((st == ACCESS) && !atom_active && !ratom && rsts && lane_on), .O(wd_sts_h));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wrat0 (.I0((st == ACCESS) && atom_active && (aph == 3'd4)),      .O(wr_atom_en_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wrat1 (.I0((st == ACCESS) && atom_active && (aph == 3'd4)),      .O(wr_atom_en_1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign is_addsub_h = rsub_h == 4'd4 || rsub_h == 4'd5;
			assign wd_zero_h = scrub_we || sc_on;
			assign wd_atom_h = (st == ACCESS) && atom_active && (aph == 3'd5);
			assign wd_sts_h = (st == ACCESS) && !atom_active && !ratom && rsts && lane_on;
			assign wr_atom_en_0 = (st == ACCESS) && atom_active && (aph == 3'd4);
			assign wr_atom_en_1 = (st == ACCESS) && atom_active && (aph == 3'd4);
`else
			assign is_addsub_h = (rsub_h == 4'd4 || rsub_h == 4'd5);
			assign wd_zero_h = scrub_we || sc_on;
			assign wd_atom_h = (st == ACCESS) && atom_active && (aph == 3'd5);
			assign wd_sts_h  = (st == ACCESS) && !atom_active && !ratom && rsts && lane_on;
			assign wr_atom_en_0 = (st == ACCESS) && atom_active && (aph == 3'd4);
			assign wr_atom_en_1 = (st == ACCESS) && atom_active && (aph == 3'd4);
`endif
			wire [15:0] w_atom_h = is_addsub_h ? asum_r[glh*16 +: 16] : w_other_r[glh*16 +: 16];
			(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] ram_wd_h_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					ram_wd_h_r <= 16'd0;
				else if (wd_zero_h)
					ram_wd_h_r <= 16'd0;
				else if (wd_atom_h)
					ram_wd_h_r <= w_atom_h;
				else if (wd_sts_h)
					ram_wd_h_r <= cur_rdat[glh*16 +: 16];
			end
			assign ram_wd_r[glh*16 +: 16] = ram_wd_h_r;
			always @(posedge clk) begin
				if (wr_atom_en_0)
					asum_r[glh*16 +: 16]    <= asum[glh*16 +: 16];
				if (wr_atom_en_1)
					w_other_r[glh*16 +: 16] <= w_other_h;
			end
		end
	endgenerate

	/* The next lane. */
	wire [LW-1:0] li_next, li_next_b;
	wire li_co_unused;
	am_add #(.W(LW)) u_li (.a(li), .b({LW{1'b0}}), .ci(1'b1), .s(li_next), .co(li_co_unused));
`ifdef SYNTH_KINTEX7
	genvar gli;
	generate
		for (gli = 0; gli < LW; gli = gli + 1) begin : g_libuf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lib (.I0(li_next[gli]), .O(li_next_b[gli]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gli;
	generate
		for (gli = 0; gli < LW; gli = gli + 1) begin : g_libuf
			assign li_next_b[gli] = li_next[gli];
		end
	endgenerate
`else
	assign li_next_b = li_next;
`endif

	wire [AW-1:0] sc_a_next;
	wire sc_a_co_unused;
	am_add #(.W(AW)) u_sc_a_next (.a(sc_a), .b({AW{1'b0}}), .ci(1'b1), .s(sc_a_next), .co(sc_a_co_unused));

	assign scrub_busy = sc_on;

	wire lds_idle_ce_a0, lds_idle_ce_a1, lds_idle_ce_b;
	wire atom_bc_ce_raw = !scrub_ctl && !sc_on && (st == ACCESS) && atom_active && (aph == 3'd0);
	wire atom_bc_ce_0, atom_bc_ce_1, atom_bc_ce_2, atom_bc_ce_3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldicea0 (.I0(lds_take_raw), .O(lds_idle_ce_a0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldicea1 (.I0(lds_take_raw), .O(lds_idle_ce_a1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldiceb  (.I0(lds_take_raw), .O(lds_idle_ce_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_atb0  (.I0(atom_bc_ce_raw), .O(atom_bc_ce_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_atb1  (.I0(atom_bc_ce_raw), .O(atom_bc_ce_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_atc0  (.I0(atom_bc_ce_raw), .O(atom_bc_ce_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_atc1  (.I0(atom_bc_ce_raw), .O(atom_bc_ce_3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lds_idle_ce_a0 = lds_take_raw;
	assign lds_idle_ce_a1 = lds_take_raw;
	assign lds_idle_ce_b = lds_take_raw;
	assign atom_bc_ce_0 = atom_bc_ce_raw;
	assign atom_bc_ce_1 = atom_bc_ce_raw;
	assign atom_bc_ce_2 = atom_bc_ce_raw;
	assign atom_bc_ce_3 = atom_bc_ce_raw;
`else
	assign lds_idle_ce_a0 = lds_take_raw;
	assign lds_idle_ce_a1 = lds_take_raw;
	assign lds_idle_ce_b  = lds_take_raw;
	assign atom_bc_ce_0 = atom_bc_ce_raw;
	assign atom_bc_ce_1 = atom_bc_ce_raw;
	assign atom_bc_ce_2 = atom_bc_ce_raw;
	assign atom_bc_ce_3 = atom_bc_ce_raw;
`endif

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= IDLE;
			lds_idle_rep <= {NPHYS{1'b1}};
			li <= {LW{1'b0}};
			li_radr <= {LW{1'b0}};
			li_lane <= {LW{1'b0}};
			li_rdat_lo <= {LW{1'b0}};
			li_rdat_hi <= {LW{1'b0}};
			li_rcmp_lo <= {LW{1'b0}};
			li_rcmp_hi <= {LW{1'b0}};
			li_atom <= {LW{1'b0}};
			li_radr_oh <= {{(NPHYS-1){1'b0}}, 1'b1};
			li_radr_oh_hi <= {{(NPHYS-1){1'b0}}, 1'b1};
			sc_done_r <= 1'b0;
			pli_1 <= {LW{1'b0}};
			pli_2 <= {LW{1'b0}};
			pli_3 <= {LW{1'b0}};
			pval_1 <= 1'b0;
			pval_2 <= 1'b0;
			pval_3 <= 1'b0;
			aph <= 3'd0;
			ram_addr_r <= {AW{1'b0}};
			ram_we_r <= 1'b0;
			lanes <= {NPHYS{1'b0}};
			rsts <= 1'b0;
			ratom <= 1'b0;
			rwrd <= 1'b0;
			rsub <= 4'd0;
			rrd <= 7'd0;
			rwarp <= {WARP_W{1'b0}};
			rgrp <= {GRP_W{1'b0}};
			rlast <= 1'b0;
			rsbe <= {(SBE_W+1){1'b0}};
		end else begin
			if (lds_idle_ce_a0) begin
				lanes <= xl;
				rsts <= is_sts;
				ratom <= is_atom;
				rwrd <= x_uop[ATG_UOP_WRD];
			end
			if (lds_idle_ce_a1) begin
				rsub <= x_uop[ATG_UOP_SUB +: 4];
				rsub_is_sgt_r    <= (x_uop[ATG_UOP_SUB +: 4] == 4'd6);
				rsub_is_slt_r    <= (x_uop[ATG_UOP_SUB +: 4] == 4'd7);
				rsub_is_ugt_r    <= (x_uop[ATG_UOP_SUB +: 4] == 4'd8);
				rsub_is_ult_r    <= (x_uop[ATG_UOP_SUB +: 4] == 4'd9);
				rsub_is_cas_r    <= (x_uop[ATG_UOP_SUB +: 4] == 4'd10);
				rsub_is_const1_r <= x_uop[ATG_UOP_SUB + 3] &&
					(x_uop[ATG_UOP_SUB + 2] || (x_uop[ATG_UOP_SUB + 1] && x_uop[ATG_UOP_SUB + 0]));
			end
			if (lds_idle_ce_b) begin
				rrd <= x_uop[ATG_UOP_RD +: 7];
				rwarp <= x_warp;
				rgrp <= x_grp;
				rlast <= x_last;
				rsbe <= x_sbe;
			end

			pval_1 <= 1'b0;
			pval_2 <= pval_1;
			pval_3 <= pval_2;
			pli_2 <= pli_1;
			pli_3 <= pli_2;
			ram_we_r <= 1'b0;
			sc_done_r <= !scrub_we && sc_on && ({{(32-AW){1'b0}}, sc_a_next} == NW - 1);

			if (scrub_we) begin
				ram_we_r <= 1'b1;
				ram_addr_r <= {AW{1'b0}};
			end else if (sc_on) begin
				if (!sc_done_r) begin
					ram_we_r <= 1'b1;
					ram_addr_r <= sc_a_next;
				end
			end else begin
				case (st)
				IDLE: if (mine && !ram_we_r) begin
					li <= {LW{1'b0}};
					li_radr <= {LW{1'b0}};
					li_lane <= {LW{1'b0}};
					li_rdat_lo <= {LW{1'b0}};
					li_rdat_hi <= {LW{1'b0}};
					li_rcmp_lo <= {LW{1'b0}};
					li_rcmp_hi <= {LW{1'b0}};
					li_atom <= {LW{1'b0}};
					li_radr_oh <= {{(NPHYS-1){1'b0}}, 1'b1};
			li_radr_oh_hi <= {{(NPHYS-1){1'b0}}, 1'b1};
					aph <= 3'd0;
					st <= xl == {NPHYS{1'b0}} ? BEAT : ACCESS;
					lds_idle_rep <= {NPHYS{1'b0}};
				end
				ACCESS: if (atom_active) begin
					case (aph)
					3'd0: begin
						ram_addr_r <= a;
						aph <= 3'd1;
					end
					3'd1: aph <= 3'd2;
					3'd2: aph <= 3'd3;
					3'd3: begin
						asum_lo_r  <= asum_lo_s;
						asum_cy_r  <= asum_lo_co;
						cmp_eq_b_r <= cmp_eq_b;
						cmp_lt_b_r <= cmp_lt_b;
						cmp_eq_r   <= cmp_eq_w;
						cmp_sign_xor_r <= (q_pipe[31] != atom_b[31]);
						cmp_old31_r    <= q_pipe[31];
						aph <= 3'd4;
					end
					3'd4: aph <= 3'd5;
					3'd5: begin
						ram_we_r <= 1'b1;
						ram_addr_r <= a;
						aph <= 3'd6;
					end
					default: begin
						aph <= 3'd0;
						li <= li_next;
						li_radr <= li_next;
						li_lane <= li_next;
						li_rdat_lo <= li_next_b;
						li_rdat_hi <= li_next_b;
						li_rcmp_lo <= li_next_b;
						li_rcmp_hi <= li_next_b;
						li_atom <= li_next_b;
						li_radr_oh <= {li_radr_oh[NPHYS-2:0], 1'b0};
						li_radr_oh_hi <= {li_radr_oh_hi[NPHYS-2:0], 1'b0};
						if ({{(32-LW){1'b0}}, li_atom} == NPHYS - 1)
							st <= DRAIN1;
					end
					endcase
				end else begin
					if (!ratom) begin
						ram_addr_r <= a;
						if (rsts && lane_on)
							ram_we_r <= 1'b1;
					end
					aph <= 3'd0;
					pval_1 <= lane_on && !rsts && !ratom;
					pli_1 <= li;
					li <= li_next;
					li_radr <= li_next;
					li_lane <= li_next;
					li_rdat_lo <= li_next_b;
					li_rdat_hi <= li_next_b;
					li_rcmp_lo <= li_next_b;
					li_rcmp_hi <= li_next_b;
					li_atom <= li_next_b;
					li_radr_oh <= {li_radr_oh[NPHYS-2:0], 1'b0};
					li_radr_oh_hi <= {li_radr_oh_hi[NPHYS-2:0], 1'b0};
					if ({{(32-LW){1'b0}}, li_atom} == NPHYS - 1)
						st <= DRAIN1;
				end
				DRAIN1: st <= DRAIN2;
				DRAIN2: st <= DRAIN3;
				DRAIN3: st <= BEAT;
				default: if (l_ready) begin
					st <= IDLE;
					lds_idle_rep <= {NPHYS{1'b1}};
				end
				endcase
			end
		end
	end

	/* Datapath atomic operands: free-running without reset */
	always @(posedge clk) begin
		if (atom_bc_ce_0) atom_b_lo <= cur_rdat[15:0];
		if (atom_bc_ce_1) atom_b_hi <= cur_rdat[31:16];
		if (atom_bc_ce_2) atom_c_lo <= cur_rcmp[15:0];
		if (atom_bc_ce_3) atom_c_hi <= cur_rcmp[31:16];
	end

	/* Port A: Registered address + dedicated fabric output register q_pipe (Rule 4). */
	always @(posedge clk) begin
		if (ram_we_r)
			ram[ram_addr_r] <= ram_wd_r;
		q <= ram[ram_addr_r];
		q_pipe <= q;
	end

	/* Port B: Registered read + dedicated fabric output register mesh_rd_data_r (Rule 4). */
	reg [31:0] mesh_rd_q;
	(* dont_touch = "true" *) reg [31:0] mesh_rd_data_r;
	always @(posedge clk) begin
		if (mesh_rd_en)
			mesh_rd_q <= ram[mesh_rd_addr];
		mesh_rd_data_r <= mesh_rd_q;
	end
	assign mesh_rd_data = mesh_rd_data_r;

	assign l_valid = st == BEAT;
	assign l_warp = rwarp;
	assign l_grp = rgrp;
	assign l_rd = rrd;
	assign l_data = res;
	assign l_lanes = lanes;
	assign l_wr = ratom ? rwrd : !rsts;
	assign l_last = rlast;
	assign l_sbe = rsbe;
	assign l_cls = ATG_CLS_M;

	wire unused_ok = &{1'b0, x_pc, x_info, x_uop, imm, x_a, sc_a_co_unused, (NWARPS != 0)};
endmodule

`default_nettype wire
