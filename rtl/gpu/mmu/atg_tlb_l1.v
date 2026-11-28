/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tlb_l1: 16-entry 2-way set-associative L1 TLB per client (8 sets x 2 ways).
 * Hit latency: 1 clock cycle (clean <= 4 LUT levels).
 * Translates virtual page number (VPN) to physical frame number (PFN).
 * See spec/gpu/mmu/atg_mmu.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tlb_l1 #(
	parameter ENTRIES = 16,
	parameter VA_BITS = 32,
	parameter PA_BITS = 32,
	parameter VPN_BITS = VA_BITS - 12,
	parameter PPN_BITS = PA_BITS - 12,
	parameter SET_W = 3,
	parameter SETS = 8,
	parameter REM_VPN = VPN_BITS - SET_W,	/* 20 - 3 = 17 bits */
	parameter TAG_W = 4 + REM_VPN		/* 4 + 17 = 21 bits */
) (
	input wire clk,
	input wire rst_active,

	/* Translation Lookup Request (Combinatorial 1-cycle hit) */
	input  wire                req_valid,
	input  wire [3:0]          req_session,
	input  wire [VPN_BITS-1:0] req_vpn,
	input  wire [SET_W-1:0]    req_set_pfn_lo,
	input  wire [SET_W-1:0]    req_set_pfn_hi,
	input  wire [SET_W-1:0]    req_set_flg,
	input  wire [SET_W-1:0]    req_set_hit,

	(* max_fanout = 16 *) output wire                 hit,
	(* max_fanout = 16 *) output wire                 miss,
	output wire [PPN_BITS-1:0] hit_pfn,
	output wire [5:0]          hit_flags,	/* [0]=V, [1]=LOCAL, [2]=W, [3]=X, [4]=D, [5]=A */
	output wire                hit_w1,
	output wire [PPN_BITS-1:0] pfn_w0,
	output wire [PPN_BITS-1:0] pfn_w1,
	output wire [5:0]          flags_w0,
	output wire [5:0]          flags_w1,

	/* Refill Interface (from L2 or Walker) */
	input wire                refill_we,
	input wire [3:0]          refill_session,
	input wire [VPN_BITS-1:0] refill_vpn,
	input wire [PPN_BITS-1:0] refill_pfn,
	input wire [5:0]          refill_flags,

	/* Invalidation Interface */
	input wire        flush_trigger,
	input wire        flush_all,
	input wire [3:0]  flush_session
);
	/* Clock and reset net naming to prevent anonymous net violations */
	(* keep = "true" *) wire clk_l1 = clk;
	(* keep = "true" *) wire rst_l1_int = rst_active;

	wire [TAG_W-1:0] req_tag = {req_session, req_vpn[VPN_BITS-1:SET_W]};

	wire [SET_W-1:0] refill_set = refill_vpn[SET_W-1:0];
	wire [TAG_W-1:0] refill_tag = {refill_session, refill_vpn[VPN_BITS-1:SET_W]};
`ifdef SYNTH_KINTEX7
	wire [TAG_W-1:0] refill_tag_w0, refill_tag_w1;
	genvar grt;
	generate
		for (grt = 0; grt < TAG_W; grt = grt + 1) begin : g_rt_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_rtw0 (.I0(refill_tag[grt]), .O(refill_tag_w0[grt]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_rtw1 (.I0(refill_tag[grt]), .O(refill_tag_w1[grt]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [TAG_W-1:0] refill_tag_w0, refill_tag_w1;
	genvar grt;
	generate
		for (grt = 0; grt < TAG_W; grt = grt + 1) begin : g_rt_buf
			assign refill_tag_w0[grt] = refill_tag[grt];
			assign refill_tag_w1[grt] = refill_tag[grt];
		end
	endgenerate
`else
	wire [TAG_W-1:0] refill_tag_w0 = refill_tag;
	wire [TAG_W-1:0] refill_tag_w1 = refill_tag;
`endif

	/* 3:8 Set Active Decoder (limits refill_set fanout to <= 8) */
	(* dont_touch = "true" *) wire [SETS-1:0] set_active;
	assign set_active[0] = (refill_set == 3'd0);
	assign set_active[1] = (refill_set == 3'd1);
	assign set_active[2] = (refill_set == 3'd2);
	assign set_active[3] = (refill_set == 3'd3);
	assign set_active[4] = (refill_set == 3'd4);
	assign set_active[5] = (refill_set == 3'd5);
	assign set_active[6] = (refill_set == 3'd6);
	assign set_active[7] = (refill_set == 3'd7);

	/* Per-set match and read signals */
	wire [SETS-1:0]      set_hit;
	wire [SETS-1:0]      set_miss;
	wire [SETS-1:0]      set_hit_w1;
	wire [9:0]           set_w0_pfn_lo [0:SETS-1];
	wire [PPN_BITS-11:0] set_w0_pfn_hi [0:SETS-1];
	wire [5:0]           set_w0_flags  [0:SETS-1];
	wire [9:0]           set_w1_pfn_lo [0:SETS-1];
	wire [PPN_BITS-11:0] set_w1_pfn_hi [0:SETS-1];
	wire [5:0]           set_w1_flags  [0:SETS-1];

	genvar gs;
	generate
	for (gs = 0; gs < SETS; gs = gs + 1) begin : g_set
		wire cur_set_active = set_active[gs];

		(* keep = "true", equivalent_register_removal = "no" *) reg w0_valid;
		(* keep = "true", equivalent_register_removal = "no" *) reg w1_valid;
		(* keep = "true", equivalent_register_removal = "no" *) reg set_alloc_ptr;

		(* keep = "true", equivalent_register_removal = "no" *) reg [TAG_W-1:0]     w0_tag;
		(* keep = "true", equivalent_register_removal = "no" *) reg [TAG_W-1:0]     w1_tag;
		(* keep = "true", equivalent_register_removal = "no" *) reg [TAG_W-1:0]     w0_tag_n;
		(* keep = "true", equivalent_register_removal = "no" *) reg [TAG_W-1:0]     w1_tag_n;
		(* keep = "true", equivalent_register_removal = "no" *) reg [9:0]           w0_pfn_lo;
		(* keep = "true", equivalent_register_removal = "no" *) reg [PPN_BITS-11:0] w0_pfn_hi;
		(* keep = "true", equivalent_register_removal = "no" *) reg [9:0]           w1_pfn_lo;
		(* keep = "true", equivalent_register_removal = "no" *) reg [PPN_BITS-11:0] w1_pfn_hi;
		(* keep = "true", equivalent_register_removal = "no" *) reg [5:0]           w0_flags;
		(* keep = "true", equivalent_register_removal = "no" *) reg [5:0]           w1_flags;

		/* Replicated write enables per field to strictly maintain <= 11 fanout */
		(* keep = "true", equivalent_register_removal = "no" *) wire we0_tag_lo = refill_we & cur_set_active & (set_alloc_ptr == 1'b0);
		(* keep = "true", equivalent_register_removal = "no" *) wire we0_tag_hi = refill_we & cur_set_active & (set_alloc_ptr == 1'b0);
		(* keep = "true", equivalent_register_removal = "no" *) wire we0_pfn_lo = refill_we & cur_set_active & (set_alloc_ptr == 1'b0);
		(* keep = "true", equivalent_register_removal = "no" *) wire we0_pfn_hi = refill_we & cur_set_active & (set_alloc_ptr == 1'b0);
		(* keep = "true", equivalent_register_removal = "no" *) wire we0_flg    = refill_we & cur_set_active & (set_alloc_ptr == 1'b0);

		(* keep = "true", equivalent_register_removal = "no" *) wire we1_tag_lo = refill_we & cur_set_active & (set_alloc_ptr == 1'b1);
		(* keep = "true", equivalent_register_removal = "no" *) wire we1_tag_hi = refill_we & cur_set_active & (set_alloc_ptr == 1'b1);
		(* keep = "true", equivalent_register_removal = "no" *) wire we1_pfn_lo = refill_we & cur_set_active & (set_alloc_ptr == 1'b1);
		(* keep = "true", equivalent_register_removal = "no" *) wire we1_pfn_hi = refill_we & cur_set_active & (set_alloc_ptr == 1'b1);
		(* keep = "true", equivalent_register_removal = "no" *) wire we1_flg    = refill_we & cur_set_active & (set_alloc_ptr == 1'b1);

		always @(posedge clk_l1 or posedge rst_l1_int) begin
			if (rst_l1_int) begin
				w0_valid      <= 1'b0;
				w1_valid      <= 1'b0;
				set_alloc_ptr <= 1'b0;
			end else begin
				if (flush_all) begin
					w0_valid <= 1'b0;
					w1_valid <= 1'b0;
				end else if (flush_trigger) begin
					if (w0_tag[TAG_W-1:TAG_W-4] == flush_session) w0_valid <= 1'b0;
					if (w1_tag[TAG_W-1:TAG_W-4] == flush_session) w1_valid <= 1'b0;
				end

				if (refill_we && cur_set_active) begin
					set_alloc_ptr <= ~set_alloc_ptr;
					if (set_alloc_ptr == 1'b0) begin
						w0_valid <= 1'b1;
					end else begin
						w1_valid <= 1'b1;
					end
				end
			end
		end

		always @(posedge clk_l1 or posedge rst_l1_int) begin
			if (rst_l1_int) begin
				w0_tag[10:0]          <= 11'd0;
				w0_tag[TAG_W-1:11]    <= {(TAG_W-11){1'b0}};
				w0_tag_n[10:0]        <= 11'h7FF;
				w0_tag_n[TAG_W-1:11]  <= {(TAG_W-11){1'b1}};
				w0_pfn_lo             <= 10'd0;
				w0_pfn_hi             <= {(PPN_BITS-10){1'b0}};
				w0_flags              <= 6'd0;

				w1_tag[10:0]          <= 11'd0;
				w1_tag[TAG_W-1:11]    <= {(TAG_W-11){1'b0}};
				w1_tag_n[10:0]        <= 11'h7FF;
				w1_tag_n[TAG_W-1:11]  <= {(TAG_W-11){1'b1}};
				w1_pfn_lo             <= 10'd0;
				w1_pfn_hi             <= {(PPN_BITS-10){1'b0}};
				w1_flags              <= 6'd0;
			end else begin
				if (we0_tag_lo) begin
					w0_tag[10:0]          <= refill_tag_w0[10:0];
					w0_tag_n[10:0]        <= ~refill_tag_w0[10:0];
				end
				if (we0_tag_hi) begin
					w0_tag[TAG_W-1:11]    <= refill_tag_w0[TAG_W-1:11];
					w0_tag_n[TAG_W-1:11]  <= ~refill_tag_w0[TAG_W-1:11];
				end
				if (we0_pfn_lo) w0_pfn_lo             <= refill_pfn[9:0];
				if (we0_pfn_hi) w0_pfn_hi             <= refill_pfn[PPN_BITS-1:10];
				if (we0_flg)    w0_flags              <= refill_flags;

				if (we1_tag_lo) begin
					w1_tag[10:0]          <= refill_tag_w1[10:0];
					w1_tag_n[10:0]        <= ~refill_tag_w1[10:0];
				end
				if (we1_tag_hi) begin
					w1_tag[TAG_W-1:11]    <= refill_tag_w1[TAG_W-1:11];
					w1_tag_n[TAG_W-1:11]  <= ~refill_tag_w1[TAG_W-1:11];
				end
				if (we1_pfn_lo) w1_pfn_lo             <= refill_pfn[9:0];
				if (we1_pfn_hi) w1_pfn_hi             <= refill_pfn[PPN_BITS-1:10];
				if (we1_flg)    w1_flags              <= refill_flags;
			end
		end

`ifdef SYNTH_KINTEX7
		/* Level 1: ceil(TAG_W/3) groups of <=3-bit equality checks per way (1 LUT each) */
		localparam N_TEQ    = (TAG_W + 2) / 3;
		localparam N_TEQ_LO = (N_TEQ + 1) / 2;
		wire [N_TEQ-1:0] eq0_c, eq1_c;
		genvar gtc;
		for (gtc = 0; gtc < N_TEQ; gtc = gtc + 1) begin : g_teq
			localparam TB = gtc * 3;
			localparam TW = (TB + 3 <= TAG_W) ? 3 : (TAG_W - TB);
			assign eq0_c[gtc] = (w0_tag[TB +: TW] == req_tag[TB +: TW]);
			assign eq1_c[gtc] = (w1_tag[TB +: TW] == req_tag[TB +: TW]);
		end

		/* Level 2: combine into hi/lo intermediate terms */
		wire m0_lo = &eq0_c[N_TEQ_LO-1:0];
		wire m0_hi = (&eq0_c[N_TEQ-1:N_TEQ_LO]) & w0_valid;

		wire m1_lo = &eq1_c[N_TEQ_LO-1:0];
		wire m1_hi = (&eq1_c[N_TEQ-1:N_TEQ_LO]) & w1_valid;

		/* Level 3: per-set hit/miss indicators (<= 5 inputs -> 1 LUT each, folds req_valid) */
		assign set_hit[gs]       = req_valid & ((m0_lo & m0_hi) | (m1_lo & m1_hi));
		assign set_miss[gs]      = req_valid & ~((m0_lo & m0_hi) | (m1_lo & m1_hi));
		assign set_hit_w1[gs]    = (m1_lo & m1_hi);
`elsif SYNTH_KINTEX_ULTRASCALE
		/* Level 1: ceil(TAG_W/3) groups of <=3-bit equality checks per way (1 LUT each) */
		localparam N_TEQ    = (TAG_W + 2) / 3;
		localparam N_TEQ_LO = (N_TEQ + 1) / 2;
		wire [N_TEQ-1:0] eq0_c, eq1_c;
		genvar gtc;
		for (gtc = 0; gtc < N_TEQ; gtc = gtc + 1) begin : g_teq
			localparam TB = gtc * 3;
			localparam TW = (TB + 3 <= TAG_W) ? 3 : (TAG_W - TB);
			assign eq0_c[gtc] = (w0_tag[TB +: TW] == req_tag[TB +: TW]);
			assign eq1_c[gtc] = (w1_tag[TB +: TW] == req_tag[TB +: TW]);
		end

		/* Level 2: combine into hi/lo intermediate terms */
		wire m0_lo = &eq0_c[N_TEQ_LO-1:0];
		wire m0_hi = (&eq0_c[N_TEQ-1:N_TEQ_LO]) & w0_valid;

		wire m1_lo = &eq1_c[N_TEQ_LO-1:0];
		wire m1_hi = (&eq1_c[N_TEQ-1:N_TEQ_LO]) & w1_valid;

		/* Level 3: per-set hit/miss indicators (<= 5 inputs -> 1 LUT each, folds req_valid) */
		assign set_hit[gs]       = req_valid & ((m0_lo & m0_hi) | (m1_lo & m1_hi));
		assign set_miss[gs]      = req_valid & ~((m0_lo & m0_hi) | (m1_lo & m1_hi));
		assign set_hit_w1[gs]    = (m1_lo & m1_hi);
`else
		wire set_en_gs = (req_set_hit == gs[SET_W-1:0]) & req_valid;
		/* 1 AOI4 gate from w0/w1_tag & w0/w1_tag_n (1 = match, 0 = mismatch) -> 11 cmos3 total! */
		(* keep = "true" *) wire [23:0] eq0_pad, eq1_pad;
		genvar gtk;
		for (gtk = 0; gtk < 24; gtk = gtk + 1) begin : g_teq_pad
			if (gtk < TAG_W) begin : g_t
				assign eq0_pad[gtk] = ~((w0_tag[gtk] & ~req_tag[gtk]) | (w0_tag_n[gtk] & req_tag[gtk]));
				assign eq1_pad[gtk] = ~((w1_tag[gtk] & ~req_tag[gtk]) | (w1_tag_n[gtk] & req_tag[gtk]));
			end else if (gtk == TAG_W) begin : g_v
				assign eq0_pad[gtk] = w0_valid;
				assign eq1_pad[gtk] = w1_valid;
			end else if (gtk == TAG_W + 1) begin : g_s
				assign eq0_pad[gtk] = set_en_gs;
				assign eq1_pad[gtk] = set_en_gs;
			end else begin : g_c
				assign eq0_pad[gtk] = 1'b1;
				assign eq1_pad[gtk] = 1'b1;
			end
		end
		/* Pure 2-input NAND2/NOR2 tree starting from active-HIGH match bits (t=1..2). */
		wire [11:0] d0_n1, d1_n1;
		wire [5:0]  d0_o2, d1_o2;
		wire [2:0]  d0_n3, d1_n3;
		wire [1:0]  d0_o4, d1_o4;
		for (gtk = 0; gtk < 12; gtk = gtk + 1) begin : g_n1
			assign d0_n1[gtk] = ~(eq0_pad[gtk*2] & eq0_pad[gtk*2+1]);
			assign d1_n1[gtk] = ~(eq1_pad[gtk*2] & eq1_pad[gtk*2+1]);
		end
		for (gtk = 0; gtk < 6; gtk = gtk + 1) begin : g_o2
			assign d0_o2[gtk] = ~(d0_n1[gtk*2] | d0_n1[gtk*2+1]);
			assign d1_o2[gtk] = ~(d1_n1[gtk*2] | d1_n1[gtk*2+1]);
		end
		for (gtk = 0; gtk < 3; gtk = gtk + 1) begin : g_n3
			assign d0_n3[gtk] = ~(d0_o2[gtk*2] & d0_o2[gtk*2+1]);
			assign d1_n3[gtk] = ~(d1_o2[gtk*2] & d1_o2[gtk*2+1]);
		end
		assign d0_o4[0] = ~(d0_n3[0] | d0_n3[1]);
		assign d0_o4[1] = ~d0_n3[2];
		assign d1_o4[0] = ~(d1_n3[0] | d1_n3[1]);
		assign d1_o4[1] = ~d1_n3[2];
		wire w0_miss_gs = ~(d0_o4[0] & d0_o4[1]);
		wire w1_miss_gs = ~(d1_o4[0] & d1_o4[1]);
		assign set_hit[gs]    = ~(w0_miss_gs & w1_miss_gs);
		assign set_miss[gs]   = ~w1_miss_gs;
		assign set_hit_w1[gs] = 1'b0;
`endif
		assign set_w0_pfn_lo[gs] = w0_pfn_lo;
		assign set_w0_pfn_hi[gs] = w0_pfn_hi;
		assign set_w0_flags[gs]  = w0_flags;
		assign set_w1_pfn_lo[gs] = w1_pfn_lo;
		assign set_w1_pfn_hi[gs] = w1_pfn_hi;
		assign set_w1_flags[gs]  = w1_flags;
	end
	endgenerate

	/* Decoupled 8:1 Multiplexers (<= 20 fanout per select bit) */
	wire [SET_W-1:0] sel_pfn_lo_a, sel_pfn_lo_b;
	wire [SET_W-1:0] sel_pfn_hi_a, sel_pfn_hi_b;
`ifdef SYNTH_KINTEX7
	LUT1 #(.INIT(2'h2)) u_b_plo_a0 (.I0(req_set_pfn_lo[0]), .O(sel_pfn_lo_a[0]));
	LUT1 #(.INIT(2'h2)) u_b_plo_a1 (.I0(req_set_pfn_lo[1]), .O(sel_pfn_lo_a[1]));
	LUT1 #(.INIT(2'h2)) u_b_plo_a2 (.I0(req_set_pfn_lo[2]), .O(sel_pfn_lo_a[2]));
	LUT1 #(.INIT(2'h2)) u_b_plo_b0 (.I0(req_set_pfn_lo[0]), .O(sel_pfn_lo_b[0]));
	LUT1 #(.INIT(2'h2)) u_b_plo_b1 (.I0(req_set_pfn_lo[1]), .O(sel_pfn_lo_b[1]));
	LUT1 #(.INIT(2'h2)) u_b_plo_b2 (.I0(req_set_pfn_lo[2]), .O(sel_pfn_lo_b[2]));

	LUT1 #(.INIT(2'h2)) u_b_phi_a0 (.I0(req_set_pfn_hi[0]), .O(sel_pfn_hi_a[0]));
	LUT1 #(.INIT(2'h2)) u_b_phi_a1 (.I0(req_set_pfn_hi[1]), .O(sel_pfn_hi_a[1]));
	LUT1 #(.INIT(2'h2)) u_b_phi_a2 (.I0(req_set_pfn_hi[2]), .O(sel_pfn_hi_a[2]));
	LUT1 #(.INIT(2'h2)) u_b_phi_b0 (.I0(req_set_pfn_hi[0]), .O(sel_pfn_hi_b[0]));
	LUT1 #(.INIT(2'h2)) u_b_phi_b1 (.I0(req_set_pfn_hi[1]), .O(sel_pfn_hi_b[1]));
	LUT1 #(.INIT(2'h2)) u_b_phi_b2 (.I0(req_set_pfn_hi[2]), .O(sel_pfn_hi_b[2]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign sel_pfn_lo_a[0] = req_set_pfn_lo[0];
	assign sel_pfn_lo_a[1] = req_set_pfn_lo[1];
	assign sel_pfn_lo_a[2] = req_set_pfn_lo[2];
	assign sel_pfn_lo_b[0] = req_set_pfn_lo[0];
	assign sel_pfn_lo_b[1] = req_set_pfn_lo[1];
	assign sel_pfn_lo_b[2] = req_set_pfn_lo[2];

	assign sel_pfn_hi_a[0] = req_set_pfn_hi[0];
	assign sel_pfn_hi_a[1] = req_set_pfn_hi[1];
	assign sel_pfn_hi_a[2] = req_set_pfn_hi[2];
	assign sel_pfn_hi_b[0] = req_set_pfn_hi[0];
	assign sel_pfn_hi_b[1] = req_set_pfn_hi[1];
	assign sel_pfn_hi_b[2] = req_set_pfn_hi[2];
`else
	assign sel_pfn_lo_a = req_set_pfn_lo;
	assign sel_pfn_lo_b = req_set_pfn_lo;
	assign sel_pfn_hi_a = req_set_pfn_hi;
	assign sel_pfn_hi_b = req_set_pfn_hi;
`endif

	wire [9:0]           w0_pfn_lo_sel = set_w0_pfn_lo[sel_pfn_lo_a];
	wire [PPN_BITS-11:0] w0_pfn_hi_sel = set_w0_pfn_hi[sel_pfn_hi_a];
	wire [9:0]           w1_pfn_lo_sel = set_w1_pfn_lo[sel_pfn_lo_b];
	wire [PPN_BITS-11:0] w1_pfn_hi_sel = set_w1_pfn_hi[sel_pfn_hi_b];
	wire [5:0]           w0_flags_sel  = set_w0_flags[req_set_flg];
	wire [5:0]           w1_flags_sel  = set_w1_flags[req_set_flg];

	assign pfn_w0   = {w0_pfn_hi_sel, w0_pfn_lo_sel};
	assign pfn_w1   = {w1_pfn_hi_sel, w1_pfn_lo_sel};
	assign flags_w0 = w0_flags_sel;
	assign flags_w1 = w1_flags_sel;

`ifdef SYNTH_KINTEX7
	assign hit_w1    = set_hit_w1[req_set_hit];
`elsif SYNTH_KINTEX_ULTRASCALE
	assign hit_w1    = set_hit_w1[req_set_hit];
`else
	wire [3:0] match_o0, match_o1;
	wire [1:0] match_n0, match_n1;
	wire match_w0_c, match_w1_c;
	generate
		if (SETS == 8) begin : g_match8
			genvar gm;
			for (gm = 0; gm < 4; gm = gm + 1) begin : g_m_o
				assign match_o0[gm] = ~(set_hit[gm*2]  | set_hit[gm*2+1]);	/* 0 if either set hit */
				assign match_o1[gm] = ~(set_miss[gm*2] | set_miss[gm*2+1]);
			end
			assign match_n0[0] = ~(match_o0[0] & match_o0[1]);	/* 1 if hit in 0..3 */
			assign match_n0[1] = ~(match_o0[2] & match_o0[3]);	/* 1 if hit in 4..7 */
			assign match_n1[0] = ~(match_o1[0] & match_o1[1]);
			assign match_n1[1] = ~(match_o1[2] & match_o1[3]);
			assign match_w0_c  = ~(match_n0[0] | match_n0[1]);	/* active-LOW hit (1 = no hit) */
			assign match_w1_c  = ~(match_n1[0] | match_n1[1]);	/* active-LOW w1 hit */
		end else begin : g_match_gen
			assign match_o0   = 4'd0;
			assign match_o1   = 4'd0;
			assign match_n0   = 2'd0;
			assign match_n1   = 2'd0;
			assign match_w0_c = ~(|set_hit);
			assign match_w1_c = ~(|set_miss);
		end
	endgenerate
	assign hit_w1 = ~match_w1_c;
`endif
`ifdef SYNTH_KINTEX7
	generate
		if (SETS == 8) begin : g_hit_mux8
			wire hit_lo, hit_hi, miss_lo, miss_hi;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_hit_lo (
				.I0(set_hit[0]), .I1(set_hit[1]), .I2(set_hit[2]), .I3(set_hit[3]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_hit_hi (
				.I0(set_hit[4]), .I1(set_hit[5]), .I2(set_hit[6]), .I3(set_hit[7]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(hit_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_hit_f7 (.I0(hit_lo), .I1(hit_hi), .S(req_set_hit[2]), .O(hit));

			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_miss_lo (
				.I0(set_miss[0]), .I1(set_miss[1]), .I2(set_miss[2]), .I3(set_miss[3]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(miss_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_miss_hi (
				.I0(set_miss[4]), .I1(set_miss[5]), .I2(set_miss[6]), .I3(set_miss[7]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(miss_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_miss_f7 (.I0(miss_lo), .I1(miss_hi), .S(req_set_hit[2]), .O(miss));
		end else begin : g_hit_mux_gen
			assign hit  = set_hit[req_set_hit];
			assign miss = set_miss[req_set_hit];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	generate
		if (SETS == 8) begin : g_hit_mux8
			wire hit_lo, hit_hi, miss_lo, miss_hi;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_hit_lo (
				.I0(set_hit[0]), .I1(set_hit[1]), .I2(set_hit[2]), .I3(set_hit[3]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_hit_hi (
				.I0(set_hit[4]), .I1(set_hit[5]), .I2(set_hit[6]), .I3(set_hit[7]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(hit_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_hit_f7 (.I0(hit_lo), .I1(hit_hi), .S(req_set_hit[2]), .O(hit));

			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_miss_lo (
				.I0(set_miss[0]), .I1(set_miss[1]), .I2(set_miss[2]), .I3(set_miss[3]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(miss_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_miss_hi (
				.I0(set_miss[4]), .I1(set_miss[5]), .I2(set_miss[6]), .I3(set_miss[7]),
				.I4(req_set_hit[0]), .I5(req_set_hit[1]), .O(miss_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_miss_f7 (.I0(miss_lo), .I1(miss_hi), .S(req_set_hit[2]), .O(miss));
		end else begin : g_hit_mux_gen
			assign hit  = set_hit[req_set_hit];
			assign miss = set_miss[req_set_hit];
		end
	endgenerate
`else
	assign hit   = ~match_w0_c;
	assign miss  = req_valid & match_w0_c;
	wire [SETS-1:0] unused_sh_w1 = set_hit_w1;
`endif
	assign hit_pfn   = hit_w1 ? pfn_w1 : pfn_w0;
	assign hit_flags = hit_w1 ? flags_w1 : flags_w0;
endmodule

`default_nettype wire
