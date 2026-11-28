/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_pt: the post-transform buffer (PT) of the SU: vertex warps' exp
 * of the position (0x000-0x003) and the varyings (0x010 + k) land in the
 * vertex entry base + lane, RX reads it back word by word; the allocator
 * hands out 32-entry blocks to vertex warps.
 *
 * Storage: NPHYS banks (bank = entry mod NPHYS, so one lane group writes
 * one word into each bank), WPV = 4 + VARY_WORDS words per entry, read
 * data registered with 3-cycle latency (pt_rdata valid 3 cycles after pt_rd:
 * address register -> BRAM synchronous read -> output pipeline register).
 *
 * Allocator: each block counts its entries in use.
 * pt_take (with the warp's slot pt_warp) takes the lowest block whose
 * count is 0 and sets 32; vl_go returns the entries of the lanes outside
 * vl_mask (also for vl_mask 0); pt_free returns one entry.  pt_done[b]:
 * the vertex warp that took block b has ended (its exports are in), so RX
 * may read the block's primitives.  See spec/gpu/su/atg_su_pt.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_pt #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS) (1 for NPHYS 32) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W) */
	parameter PT_ENTRIES = 64,	/* PT vertices: a multiple of 32, at most 256 */
	parameter VARY_WORDS = 16,	/* Varying words per vertex, at most 64 (KC705 64) */
	parameter SHARED_PT = 0		/* 0: local inside SU; 1: Spine PT driven by pt_exp_* */
) (
	input wire clk,
	input wire rst_n,
	/* Issued instruction (from atg_su_opd; only exp of vertex warps is taken) */
	input wire x_valid,
	input wire [WARP_W-1:0] x_warp,
	input wire [GRP_W-1:0] x_grp,
	input wire [UOP_W-1:0] x_uop,
	input wire [NPHYS*32-1:0] x_b,
	input wire [2*NPHYS-1:0] x_mask,
	input wire [31:0] x_info,
	/* Shared Spine stream input (used when SHARED_PT != 0) */
	input wire pt_exp_valid,
	output wire pt_exp_ready,
	input wire pt_exp_end,
	input wire [2:0] pt_exp_blk,
	input wire [GRP_W-1:0] pt_exp_grp,
	input wire [6:0] pt_exp_word,
	input wire [NPHYS-1:0] pt_exp_mask,
	input wire [NPHYS*32-1:0] pt_exp_data,
	/* PT read port (atg_rx) */
	input wire pt_rd,
	input wire [7:0] pt_idx,
	input wire [6:0] pt_word,
	output wire [31:0] pt_rdata,
	input wire pt_free,
	input wire [7:0] pt_free_idx,
	/* Allocation (atg_su_sched, the VF) */
	output wire pt_ok,
	input wire pt_take,
	input wire [WARP_W-1:0] pt_warp,
	output wire [7:0] pt_base,
	input wire vl_go,
	input wire [31:0] vl_mask,
	output wire [7:0] pt_done,
	/* Warp ends (atg_su_exec) */
	input wire wc_end,
	input wire [WARP_W-1:0] wc_warp,
	/* Session protection: clear the table (the unit idle), one word per cycle */
	input wire scrub,
	output wire scrub_busy
);
`include "atg_su_uop.vh"

	localparam NB = PT_ENTRIES / 32;	/* blocks */
	localparam BW = NB > 1 ? $clog2(NB) : 1;
	localparam WPV = 4 + VARY_WORDS;	/* words per entry */
	localparam ROWS = PT_ENTRIES / NPHYS;	/* entries per bank */
	localparam G = 32 / NPHYS;	/* lane groups per warp (= rows per block per bank) */
	localparam DEPTH = ROWS * WPV;
	localparam AW = $clog2(DEPTH);
	localparam G_W = $clog2(G);	/* G is a power of two: block G + group are disjoint bits */
	/* verilator lint_off WIDTHTRUNC */
	localparam [7:0] WPV_V = WPV;
	/* verilator lint_on WIDTHTRUNC */

	/* ---- The exp writes of vertex warps. */
	wire rst_p = !rst_n;
	(* dont_touch = "true" *) wire scrub_i = scrub;
	(* dont_touch = "true" *) wire vl_go_pt = vl_go;
	(* dont_touch = "true" *) wire [31:0] vl_mask_pt = vl_mask;
	wire [31:0] tgt = x_uop[ATG_UOP_IMM +: 32];
	wire is_pos, is_var;
	wire [31:0] wword;
	wire [31:0] wrow;
	wire wr;
	wire [NPHYS-1:0] wr_mask_eff;
	wire [NPHYS*32-1:0] wr_data_eff;
	wire tv_bo_unused;
	/* The block of each vertex warp (set at pt_take in SHARED_PT=0 mode). */
	reg [NWARPS*BW-1:0] wblk;
	reg [NWARPS-1:0] wv;	/* a vertex warp that owns a block */

	generate
		if (SHARED_PT != 0) begin : g_shared_pt_wr
			wire [BW-1:0] s_xblk = pt_exp_blk[BW-1:0];
			assign is_pos = 1'b0;
			assign is_var = 1'b0;
			assign tv_bo_unused = 1'b0;
			assign wword = {25'd0, pt_exp_word};
			assign pt_exp_ready = !(scrub_i || sc_on);
			assign wr = pt_exp_valid && pt_exp_ready && !pt_exp_end;
			assign wrow = ({{(32-BW){1'b0}}, s_xblk} << G_W) | ({{(32-GRP_W){1'b0}}, pt_exp_grp} % G);
			assign wr_mask_eff = pt_exp_mask;
			assign wr_data_eff = pt_exp_data;
		end else begin : g_local_pt_wr
			wire [BW-1:0] xblk = wblk[x_warp*BW +: BW];
			assign pt_exp_ready = 1'b0;
			if (32'h10 + VARY_WORDS <= 32'd128) begin : g_tgt_narrow
				wire tgt_top0 = (tgt[31:7] == 25'd0);
				wire is_pos_lo = (tgt[6:2] == 5'd0);
				wire [6:0] tgt_v7;
				am_sub #(.W(7)) u_tgt_v (.a(tgt[6:0]), .b(7'd12), .bi(1'b0), .d(tgt_v7), .bo(tv_bo_unused));
				wire [6:0] wword7 = is_pos_lo ? tgt[6:0] : tgt_v7;
				assign is_pos = tgt_top0 && is_pos_lo;
				assign is_var = tgt_top0 && (tgt[6:0] >= 7'h10) && ({1'b0, tgt[6:0]} < (8'h10 + VARY_WORDS[7:0]));
				assign wword = {25'd0, wword7};	/* varying v at word 4 + v */
			end else begin : g_tgt_wide
				wire [31:0] tgt_v;
				am_sub #(.W(32)) u_tgt_v (.a(tgt), .b(32'd12), .bi(1'b0), .d(tgt_v), .bo(tv_bo_unused));
				assign is_pos = tgt < 32'd4;
				assign is_var = tgt >= 32'h10 && tgt < 32'h10 + VARY_WORDS;
				assign wword = is_pos ? tgt : tgt_v;	/* varying v at word 4 + v */
			end
			assign wr = x_valid && x_uop[ATG_UOP_UNIT +: 4] == ATG_UNIT_EXP && x_info[17:16] == 2'd0 &&
				(is_pos || is_var);
			/* the row (entry / NPHYS) of lane l of this group */
			assign wrow = ({{(32-BW){1'b0}}, xblk} << G_W) | ({{(32-GRP_W){1'b0}}, x_grp} % G);
			assign wr_mask_eff = x_mask[2*NPHYS-1:NPHYS];
			assign wr_data_eff = x_b;
		end
	endgenerate
	wire [31:0] rrow = {24'd0, pt_idx} / NPHYS;

	/*
	 * The word addresses row WPV + word, modulo 2^AW (the bank's):
	 * ROWS = PT_ENTRIES / NPHYS is small (e.g. 8 or 16), so row * WPV is a pure
	 * constant-table lookup of row[$clog2(ROWS)-1:0] (1 LUT level, 0 CARRY4s),
	 * followed by a single am_add with wword / pt_word.
	 */
	localparam RW = ROWS > 1 ? $clog2(ROWS) : 1;
	reg [31:0] wrow_w32, rrow_w32;
	integer ri;
	always @(*) begin
		wrow_w32 = 32'd0;
		rrow_w32 = 32'd0;
		for (ri = 0; ri < ROWS; ri = ri + 1) begin
			if (wrow[RW-1:0] == ri[RW-1:0])
				wrow_w32 = ri * WPV;
			if (rrow[RW-1:0] == ri[RW-1:0])
				rrow_w32 = ri * WPV;
		end
	end
	wire [AW-1:0] wrow_w_aw = wrow_w32[AW-1:0];
	wire [AW-1:0] rrow_w_aw = rrow_w32[AW-1:0];

	wire [AW-1:0] waddr_a;
	wire [AW-1:0] raddr_a;
	wire [31:0] pt_word32 = {25'd0, pt_word};
	wire ra_co_unused;
	/* Pipeline Stage P1 before u_waddr to cut x_warp -> wblk -> wrow_w32 -> u_waddr (24 cmos3 -> <=10). */
	reg wr_p1;
	reg [AW-1:0] wrow_w_p1;
	reg [AW-1:0] wword_p1;
	reg [NPHYS-1:0] wr_mask_p1;
	reg [NPHYS*32-1:0] wr_data_p1;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			wr_p1      <= 1'b0;
			wrow_w_p1  <= {AW{1'b0}};
			wword_p1   <= {AW{1'b0}};
			wr_mask_p1 <= {NPHYS{1'b0}};
			wr_data_p1 <= {(NPHYS*32){1'b0}};
		end else begin
			wr_p1      <= wr;
			wrow_w_p1  <= wrow_w_aw;
			wword_p1   <= wword[AW-1:0];
			wr_mask_p1 <= wr_mask_eff;
			wr_data_p1 <= wr_data_eff;
		end
	end
	wire wa_co_unused;
	generate
		if (AW > 6) begin : g_wa_csel
			localparam W_LO = 6;
			localparam W_HI = AW - W_LO;
			wire [W_LO-1:0] wa_lo;
			wire wa_lo_co;
			wire [W_HI-1:0] wa_hi_0, wa_hi_1;
			wire wa_co0_unused, wa_co1_unused;
			am_add #(.W(W_LO)) u_waddr_lo (.a(wrow_w_p1[W_LO-1:0]), .b(wword_p1[W_LO-1:0]), .ci(1'b0), .s(wa_lo), .co(wa_lo_co));
			am_add #(.W(W_HI)) u_waddr_hi0 (.a(wrow_w_p1[AW-1:W_LO]), .b(wword_p1[AW-1:W_LO]), .ci(1'b0), .s(wa_hi_0), .co(wa_co0_unused));
			am_add #(.W(W_HI)) u_waddr_hi1 (.a(wrow_w_p1[AW-1:W_LO]), .b(wword_p1[AW-1:W_LO]), .ci(1'b1), .s(wa_hi_1), .co(wa_co1_unused));
			assign waddr_a = {wa_lo_co ? wa_hi_1 : wa_hi_0, wa_lo};
			assign wa_co_unused = &{1'b0, wa_co0_unused, wa_co1_unused};
		end else begin : g_wa_flat
			am_add #(.W(AW)) u_waddr (.a(wrow_w_p1), .b(wword_p1), .ci(1'b0), .s(waddr_a), .co(wa_co_unused));
		end
	endgenerate
	am_add #(.W(AW)) u_raddr (.a(rrow_w_aw), .b(pt_word32[AW-1:0]), .ci(1'b0), .s(raddr_a),
		.co(ra_co_unused));
	wire [31:0] waddr = {{(32-AW){1'b0}}, waddr_a};
	wire [31:0] raddr = {{(32-AW){1'b0}}, raddr_a};
	wire unused_addr = &{1'b0, wword[31:AW], wrow[31:RW], rrow[31:RW], wrow_w32[31:AW], rrow_w32[31:AW], pt_word32[31:AW], WPV_V, wa_co_unused};
	wire [31:0] rbank = {24'd0, pt_idx} % NPHYS;
	(* shreg_extract = "no", max_fanout = 12 *) reg [7:0] pt_rd_r1_rep;
	wire pt_rd_r1 = pt_rd_r1_rep[0];
	(* dont_touch = "true" *) reg [AW-1:0] raddr_r;
	(* shreg_extract = "no", max_fanout = 12 *) reg [$clog2(NPHYS)-1:0] rb_r1;
	(* keep = "true", dont_touch = "true" *) reg [$clog2(NPHYS)-1:0] rb_r2_sl [0:3];
	(* keep = "true", dont_touch = "true" *) reg [$clog2(NPHYS)-1:0] rb_r3_sl [0:3];
	wire [NPHYS*32-1:0] rd_q;

	/* The banks' write port: the vertex warps' exp, or the scrub's zeros (registered for 0-LUT BRAM ADDR, Rule 4). */
	wire sc_on_raw;
	wire sc_on_busy, sc_on_ctl, sc_on_ctl_wa, sc_on_wd_lo, sc_on_wd_hi;
	wire [AW-1:0] sc_a;
	wire scrub_sc, scrub_ctl0, scrub_ctl1, scrub_wd;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsc_sc (.I0(scrub_i),   .O(scrub_sc));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsc_c0 (.I0(scrub_i),   .O(scrub_ctl0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsc_c1 (.I0(scrub_i),   .O(scrub_ctl1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsc_wd (.I0(scrub_i),   .O(scrub_wd));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsn_bs (.I0(sc_on_raw), .O(sc_on_busy));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsn_ct (.I0(sc_on_raw), .O(sc_on_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsn_cw (.I0(sc_on_raw), .O(sc_on_ctl_wa));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsn_wl (.I0(sc_on_raw), .O(sc_on_wd_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ptsn_wh (.I0(sc_on_raw), .O(sc_on_wd_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign scrub_sc = scrub_i;
	assign scrub_ctl0 = scrub_i;
	assign scrub_ctl1 = scrub_i;
	assign scrub_wd = scrub_i;
	assign sc_on_busy = sc_on_raw;
	assign sc_on_ctl = sc_on_raw;
	assign sc_on_ctl_wa = sc_on_raw;
	assign sc_on_wd_lo = sc_on_raw;
	assign sc_on_wd_hi = sc_on_raw;
`else
	assign scrub_sc     = scrub_i;
	assign scrub_ctl0   = scrub_i;
	assign scrub_ctl1   = scrub_i;
	assign scrub_wd     = scrub_i;
	assign sc_on_busy   = sc_on_raw;
	assign sc_on_ctl    = sc_on_raw;
	assign sc_on_ctl_wa = sc_on_raw;
	assign sc_on_wd_lo  = sc_on_raw;
	assign sc_on_wd_hi  = sc_on_raw;
`endif
	wire sc_on = sc_on_ctl;

	atg_su_scrub #(.DEPTH(DEPTH), .AW(AW)) sc (
		.clk(clk), .rst_n(rst_n), .start(scrub_sc), .on(sc_on_raw), .a(sc_a)
	);

	wire [AW-1:0] sc_a_next;
	wire sc_a_co_unused;
	am_add #(.W(AW)) u_sc_a_next (.a(sc_a), .b({AW{1'b0}}), .ci(1'b1), .s(sc_a_next), .co(sc_a_co_unused));

	assign scrub_busy = sc_on_busy;
	(* max_fanout = 12 *) reg [NPHYS-1:0] m_we_r;
	(* max_fanout = 8 *) reg [AW-1:0] m_wa_r;
	wire [NPHYS*32-1:0] m_wd_r;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			m_we_r <= {NPHYS{1'b0}};
		end else if (scrub_ctl0) begin
			m_we_r <= {NPHYS{1'b1}};
		end else if (sc_on_ctl) begin
			if ({{(32-AW){1'b0}}, sc_a} == DEPTH - 1)
				m_we_r <= {NPHYS{1'b0}};
			else
				m_we_r <= {NPHYS{1'b1}};
		end else begin
			m_we_r <= wr_p1 ? wr_mask_p1 : {NPHYS{1'b0}};
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			m_wa_r <= {AW{1'b0}};
		end else if (scrub_ctl1) begin
			m_wa_r <= {AW{1'b0}};
		end else if (sc_on_ctl_wa) begin
			if ({{(32-AW){1'b0}}, sc_a} != DEPTH - 1)
				m_wa_r <= sc_a_next;
		end else begin
			m_wa_r <= waddr[AW-1:0];
		end
	end

	genvar b, gbh;
	generate
	for (b = 0; b < NPHYS; b = b + 1) begin : g_bank
		for (gbh = 0; gbh < 2; gbh = gbh + 1) begin : g_wd_half
			(* shreg_extract = "no" *) reg [15:0] m_wd_h;
			wire wd_zero_h;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wdz (.I0(scrub_wd || (gbh == 0 ? sc_on_wd_lo : sc_on_wd_hi)), .O(wd_zero_h));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wd_zero_h = scrub_wd || (gbh == 0 ? sc_on_wd_lo : sc_on_wd_hi);
`else
			assign wd_zero_h = scrub_wd || sc_on;
`endif
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					m_wd_h <= 16'd0;
				else if (wd_zero_h)
					m_wd_h <= 16'd0;
				else
					m_wd_h <= wr_data_p1[b*32 + gbh*16 +: 16];
			end
			assign m_wd_r[b*32 + gbh*16 +: 16] = m_wd_h;
		end
		(* ram_style = "block" *) reg [31:0] m [0:DEPTH-1];
		reg [31:0] rd_q_b;
		(* keep = "true", dont_touch = "true" *) reg [31:0] rd_q_pipe_b;
		(* dont_touch = "true" *) wire pt_rd_b = pt_rd_r1_rep[(b * 8) / NPHYS];
		always @(posedge clk) begin
			if (m_we_r[b])
				m[m_wa_r] <= m_wd_r[32 * b +: 32];
			if (pt_rd_b)
				rd_q_b <= m[raddr_r];
			rd_q_pipe_b <= rd_q_b;
		end
		assign rd_q[32 * b +: 32] = rd_q_pipe_b;
	end
	endgenerate

	wire [31:0] pt_rdata_nxt;
	generate
	for (gbh = 0; gbh < 4; gbh = gbh + 1) begin : g_rdata_sl
		wire [NPHYS*8-1:0] rdata_b;
		genvar grbi;
		for (grbi = 0; grbi < NPHYS; grbi = grbi + 1) begin : g_rbi
			assign rdata_b[grbi*8 +: 8] = rd_q[grbi*32 + gbh*8 +: 8];
		end
		assign pt_rdata_nxt[gbh*8 +: 8] = rdata_b[rb_r3_sl[gbh]*8 +: 8];
	end
	endgenerate

	integer grs;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pt_rd_r1_rep <= 8'd0;
			raddr_r      <= {AW{1'b0}};
			rb_r1        <= {$clog2(NPHYS){1'b0}};
			for (grs = 0; grs < 4; grs = grs + 1) begin
				rb_r2_sl[grs] <= {$clog2(NPHYS){1'b0}};
				rb_r3_sl[grs] <= {$clog2(NPHYS){1'b0}};
			end
		end else begin
			pt_rd_r1_rep <= {8{pt_rd}};
			if (pt_rd) begin
				raddr_r <= raddr[AW-1:0];
				rb_r1   <= rbank[$clog2(NPHYS)-1:0];
			end
			if (pt_rd_r1) begin
				for (grs = 0; grs < 4; grs = grs + 1)
					rb_r2_sl[grs] <= rb_r1;
			end
			for (grs = 0; grs < 4; grs = grs + 1)
				rb_r3_sl[grs] <= rb_r2_sl[grs];
		end
	end
	assign pt_rdata = pt_rdata_nxt;

	/* ---- The allocator. */
	reg [NB*6-1:0] cnt;	/* entries in use, 0..32 */
	reg [NB-1:0]   cnt_z_r;
	reg            pt_ok_r;
	reg [NB*WARP_W-1:0] own;	/* the vertex warp that took the block */
	reg [NB-1:0] done;
	reg [BW-1:0] last_blk;	/* the block of the latest grant (for vl_go) */

	reg [BW-1:0] fb;
	integer i;
	always @(*) begin
		fb = {BW{1'b0}};
		for (i = NB - 1; i >= 0; i = i - 1)
			if (cnt_z_r[i])
				fb = i[BW-1:0];
	end
	assign pt_ok = pt_ok_r;
	assign pt_base = {{(3-BW){1'b0}}, fb, 5'd0};
	assign pt_done = {{(8-NB){1'b0}}, done};

	/*
	 * Pipelined popcount of vl_mask:
	 * - On vl_go with vl_mask == 0, cnt[last_blk] becomes 0 immediately.
	 * - On vl_go with vl_mask != 0, two 16-bit popcounts are latched into
	 *   p16_lo_r and p16_hi_r, and on the next cycle u_pop32 sets cnt[last_blk_d1].
	 */
	wire [2:0] p4_lo [0:3];
	wire [2:0] p4_hi [0:3];
	genvar gp4;
	generate
		for (gp4 = 0; gp4 < 4; gp4 = gp4 + 1) begin : g_p4
			atg_su_pt_pop4 u_p4_lo (.x(vl_mask_pt[gp4*4 +: 4]),       .pop(p4_lo[gp4]));
			atg_su_pt_pop4 u_p4_hi (.x(vl_mask_pt[16 + gp4*4 +: 4]),  .pop(p4_hi[gp4]));
		end
	endgenerate

	wire [3:0] p16_lo_s0, p16_lo_s1;
	wire [3:0] p16_hi_s0, p16_hi_s1;
	wire [4:0] p16_lo_comb, p16_hi_comb;

	am_add #(.W(3)) u_p16_lo_add0 (.a(p4_lo[0]), .b(p4_lo[1]), .ci(1'b0), .s(p16_lo_s0[2:0]), .co(p16_lo_s0[3]));
	am_add #(.W(3)) u_p16_lo_add1 (.a(p4_lo[2]), .b(p4_lo[3]), .ci(1'b0), .s(p16_lo_s1[2:0]), .co(p16_lo_s1[3]));
	am_add #(.W(4)) u_p16_lo_add2 (.a(p16_lo_s0), .b(p16_lo_s1), .ci(1'b0), .s(p16_lo_comb[3:0]), .co(p16_lo_comb[4]));

	am_add #(.W(3)) u_p16_hi_add0 (.a(p4_hi[0]), .b(p4_hi[1]), .ci(1'b0), .s(p16_hi_s0[2:0]), .co(p16_hi_s0[3]));
	am_add #(.W(3)) u_p16_hi_add1 (.a(p4_hi[2]), .b(p4_hi[3]), .ci(1'b0), .s(p16_hi_s1[2:0]), .co(p16_hi_s1[3]));
	am_add #(.W(4)) u_p16_hi_add2 (.a(p16_hi_s0), .b(p16_hi_s1), .ci(1'b0), .s(p16_hi_comb[3:0]), .co(p16_hi_comb[4]));

	wire [5:0] p16_lo_m1_comb;
	wire p16_m1_bo_unused;
	am_sub #(.W(6)) u_p16_lo_m1 (
		.a({1'b0, p16_lo_comb}), .b(6'd1), .bi(1'b0), .d(p16_lo_m1_comb), .bo(p16_m1_bo_unused)
	);

	reg vl_go_d1;
	reg [BW-1:0] last_blk_d1;
	reg [4:0] p16_lo_r, p16_hi_r;
	reg [5:0] p16_lo_m1_r;
	wire [5:0] pop32_s, pop32_dec;
	wire pop32_co_unused, pop32_dec_co_unused;
	am_add #(.W(6)) u_pop32 (
		.a({1'b0, p16_lo_r}),
		.b({1'b0, p16_hi_r}),
		.ci(1'b0),
		.s(pop32_s),
		.co(pop32_co_unused)
	);
	am_add #(.W(6)) u_pop32_dec (
		.a(p16_lo_m1_r),
		.b({1'b0, p16_hi_r}),
		.ci(1'b0),
		.s(pop32_dec),
		.co(pop32_dec_co_unused)
	);
	wire pop32_is1 = ((p16_lo_r == 5'd1) && (p16_hi_r == 5'd0)) ||
		((p16_lo_r == 5'd0) && (p16_hi_r == 5'd1));

	wire [BW-1:0] free_blk = pt_free_idx[5 +: BW];
	wire vl_mask_zero = (vl_mask_pt == 32'd0);

	wire [NB*6-1:0] cnt_dec_blk;
	wire [NB-1:0]   cnt_is1_blk;
	wire [NB-1:0]   cnt_dec_bo_unused;
	genvar gcb;
	generate
		for (gcb = 0; gcb < NB; gcb = gcb + 1) begin : g_cnt_dec
			wire [5:0] cnt_blk_cur = cnt[gcb*6 +: 6];
			am_sub #(.W(6)) u_cnt_blk_dec (
				.a(cnt_blk_cur), .b(6'd1), .bi(1'b0),
				.d(cnt_dec_blk[gcb*6 +: 6]), .bo(cnt_dec_bo_unused[gcb])
			);
			assign cnt_is1_blk[gcb] = (cnt_blk_cur == 6'd1);
		end
	endgenerate

	wire [BW-1:0] end_blk = (SHARED_PT != 0) ? pt_exp_blk[BW-1:0] : wblk[wc_warp*BW +: BW];
	wire end_own = (SHARED_PT != 0) ? (pt_exp_valid && pt_exp_ready && pt_exp_end) :
		(wc_end && wv[wc_warp] && own[end_blk*WARP_W +: WARP_W] == wc_warp);

	reg [NB-1:0] nxt_cnt_z;
	integer cz_i;
	always @(*) begin
		for (cz_i = 0; cz_i < NB; cz_i = cz_i + 1) begin
			if (pt_take && fb == cz_i[BW-1:0])
				nxt_cnt_z[cz_i] = 1'b0;
			else if (vl_go_pt && vl_mask_zero && last_blk == cz_i[BW-1:0])
				nxt_cnt_z[cz_i] = 1'b1;
			else if (vl_go_d1 && last_blk_d1 == cz_i[BW-1:0])
				nxt_cnt_z[cz_i] = (pt_free && free_blk == cz_i[BW-1:0]) ? pop32_is1 : 1'b0;
			else if (pt_free && free_blk == cz_i[BW-1:0])
				nxt_cnt_z[cz_i] = cnt_is1_blk[cz_i];
			else
				nxt_cnt_z[cz_i] = cnt_z_r[cz_i];
		end
	end

	integer cb_i;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			cnt <= {(NB*6){1'b0}};
			cnt_z_r <= {NB{1'b1}};
			pt_ok_r <= 1'b1;
			own <= {(NB*WARP_W){1'b0}};
			done <= {NB{1'b0}};
			last_blk <= {BW{1'b0}};
			last_blk_d1 <= {BW{1'b0}};
			vl_go_d1 <= 1'b0;
			p16_lo_r <= 5'd0;
			p16_hi_r <= 5'd0;
			p16_lo_m1_r <= 6'd0;
			wblk <= {(NWARPS*BW){1'b0}};
			wv <= {NWARPS{1'b0}};
		end else begin
			cnt_z_r <= nxt_cnt_z;
			pt_ok_r <= |nxt_cnt_z;
			vl_go_d1 <= vl_go_pt && !vl_mask_zero;
			if (vl_go_pt) begin
				last_blk_d1 <= last_blk;
				p16_lo_r <= p16_lo_comb;
				p16_hi_r <= p16_hi_comb;
				p16_lo_m1_r <= p16_lo_m1_comb;
			end
			for (cb_i = 0; cb_i < NB; cb_i = cb_i + 1) begin
				if (pt_take && fb == cb_i[BW-1:0])
					cnt[cb_i*6 +: 6] <= 6'd32;
				else if (vl_go_pt && vl_mask_zero && last_blk == cb_i[BW-1:0])
					cnt[cb_i*6 +: 6] <= 6'd0;
				else if (vl_go_d1 && last_blk_d1 == cb_i[BW-1:0])
					cnt[cb_i*6 +: 6] <= (pt_free && free_blk == cb_i[BW-1:0]) ? pop32_dec : pop32_s;
				else if (pt_free && free_blk == cb_i[BW-1:0])
					cnt[cb_i*6 +: 6] <= cnt_dec_blk[cb_i*6 +: 6];
			end
			if (end_own)
				done[end_blk] <= 1'b1;
			if (wc_end && SHARED_PT == 0)
				wv[wc_warp] <= 1'b0;
			if (pt_take) begin
				own[fb*WARP_W +: WARP_W] <= pt_warp;
				done[fb] <= 1'b0;
				last_blk <= fb;
				if (SHARED_PT == 0) begin
					wblk[pt_warp*BW +: BW] <= fb;
					wv[pt_warp] <= 1'b1;
				end
			end
		end
	end

	wire unused_ok = &{1'b0, waddr, raddr, rbank, pt_free_idx, x_valid, x_warp, x_grp, x_b, x_mask, x_info, x_uop,
		pt_exp_valid, pt_exp_end, pt_exp_blk, pt_exp_grp, pt_exp_word, pt_exp_mask, pt_exp_data,
		pt_warp, wc_end, wc_warp, tv_bo_unused, is_pos, is_var,
		sc_a_co_unused, pop32_co_unused, pop32_dec_co_unused, p16_m1_bo_unused, cnt_dec_bo_unused};
endmodule

module atg_su_pt_pop4 (
	input wire [3:0] x,
	output reg [2:0] pop
);
	always @(*) begin
		case (x)
		4'b0000: pop = 3'd0;
		4'b0001, 4'b0010, 4'b0100, 4'b1000: pop = 3'd1;
		4'b0011, 4'b0101, 4'b0110, 4'b1001, 4'b1010, 4'b1100: pop = 3'd2;
		4'b0111, 4'b1011, 4'b1101, 4'b1110: pop = 3'd3;
		default: pop = 3'd4;
		endcase
	end
endmodule

`default_nettype wire
