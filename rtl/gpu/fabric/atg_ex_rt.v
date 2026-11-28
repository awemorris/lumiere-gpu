/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_ex_rt: K-way pipelined fragment export router (SU -> ROP).
 *
 * Bridges K child fragment export buses (ex_*) up to one parent port toward
 * spine atg_rop with strict flop-to-flop am_skid boundaries.
 * Arbitrates with ex_seq == ex_next priority (prevents head-of-line stalling
 * across multiple SUs) and fair round-robin fallback, locking each multi-beat
 * quad export packet until ex_last passes.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_ex_rt #(
	parameter K = 4
) (
	input wire clk,
	input wire rst_n,

	/* Child SU export ports (packed, child 0 in low bits) */
	input wire [K-1:0] c_ex_valid,
	output wire [K-1:0] c_ex_ready,
	input wire [K*16-1:0] c_ex_seq,
	input wire [K*32-1:0] c_ex_xy,
	input wire [K*4-1:0] c_ex_mask,
	input wire [K*512-1:0] c_ex_color,
	input wire [K*512-1:0] c_ex_color1,
	input wire [K-1:0] c_ex_has_src1,
	input wire [K*128-1:0] c_ex_depth,
	input wire [K-1:0] c_ex_last,
	input wire [K*155-1:0] c_ex_aux,
	output reg [15:0] c_ex_next,

	/* Parent port toward Spine ROP */
	output wire p_ex_valid,
	input wire p_ex_ready,
	output wire [15:0] p_ex_seq,
	output wire [31:0] p_ex_xy,
	output wire [3:0] p_ex_mask,
	output wire [511:0] p_ex_color,
	output wire [511:0] p_ex_color1,
	output wire p_ex_has_src1,
	output wire [127:0] p_ex_depth,
	output wire p_ex_last,
	output wire [154:0] p_ex_aux,
	input wire [15:0] p_ex_next
);
	localparam KW   = (K > 2) ? 2 : 1;
	localparam EX_W = 16 + 32 + 4 + 512 + 512 + 1 + 128 + 155 + 1; /* 1361 bits */

	wire [K-1:0] sk_v, sk_r;
	wire [K*EX_W-1:0] sk_d_flat;
	wire [K-1:0] sk_match_next;
	wire [K-1:0] sk_last;

	genvar gk;
	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_ex_sk
			wire [EX_W-1:0] in_vec = {
				c_ex_seq[gk*16 +: 16],
				c_ex_xy[gk*32 +: 32],
				c_ex_mask[gk*4 +: 4],
				c_ex_color[gk*512 +: 512],
				c_ex_color1[gk*512 +: 512],
				c_ex_has_src1[gk],
				c_ex_depth[gk*128 +: 128],
				c_ex_aux[gk*155 +: 155],
				c_ex_last[gk]
			};
			atg_ex_rt_skid #(.WIDTH(EX_W), .SL_S(10)) u_ex_sk (
				.clk(clk), .rst_n(rst_n),
				.in_valid(c_ex_valid[gk]), .in_ready(c_ex_ready[gk]),
				.in_data(in_vec),
				.out_valid(sk_v[gk]), .out_ready(sk_r[gk]),
				.out_data(sk_d_flat[gk*EX_W +: EX_W])
			);
			/* 16-bit equality as 4x 4-bit AND-reduced groups (<= 6 cmos3) */
			wire [15:0] sk_seq = sk_d_flat[gk*EX_W + EX_W - 1 -: 16];
			wire [3:0] eq_nib;
			assign eq_nib[0] = (sk_seq[3:0]   == c_ex_next[3:0]);
			assign eq_nib[1] = (sk_seq[7:4]   == c_ex_next[7:4]);
			assign eq_nib[2] = (sk_seq[11:8]  == c_ex_next[11:8]);
			assign eq_nib[3] = (sk_seq[15:12] == c_ex_next[15:12]);
			assign sk_match_next[gk] = sk_v[gk] && (&eq_nib);
			assign sk_last[gk]       = sk_d_flat[gk*EX_W];
		end
	endgenerate

	/* Registered grant stage: breaks c_ex_next == seq -> RR -> mux -> am_skid (25 cmos3 -> <= 11 cmos3). */
	wire [K-1:0] cand = sk_match_next;
	reg          grant_v_r;
	reg [K-1:0]  grant_oh_r;
	reg [K-1:0]  rr_ge_r;

	wire [K-1:0] cand_hi = cand & rr_ge_r;
	wire [K-1:0] win_lo_1h, win_hi_1h;
	assign win_lo_1h[0] = cand[0];
	assign win_hi_1h[0] = cand_hi[0];
	generate
		for (gk = 1; gk < K; gk = gk + 1) begin : g_ex_pe
			assign win_lo_1h[gk] = cand[gk] && !(|cand[gk-1:0]);
			assign win_hi_1h[gk] = cand_hi[gk] && !(|cand_hi[gk-1:0]);
		end
	endgenerate
	wire         any_cand      = |cand;
	wire         any_hi        = |cand_hi;
	wire [K-1:0] next_grant_oh = any_hi ? win_hi_1h : win_lo_1h;

	wire [K-1:0] next_rr_ge;
	assign next_rr_ge[0] = next_grant_oh[K - 1];
	generate
		for (gk = 1; gk < K; gk = gk + 1) begin : g_ex_nrr
			assign next_rr_ge[gk] = next_grant_oh[K - 1] || (|next_grant_oh[gk-1:0]);
		end
	endgenerate

	wire pick_valid = grant_v_r && (|(grant_oh_r & sk_v));
	wire pick_last  = |(grant_oh_r & sk_last);

	wire out_in_ready;
	wire fire = pick_valid && out_in_ready;

	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_ex_rdy
			assign sk_r[gk] = out_in_ready && grant_v_r && grant_oh_r[gk];
		end
	endgenerate

	localparam integer SL_W = 8;
	localparam integer N_SL = (EX_W + SL_W - 1) / SL_W;
	localparam integer N_M1 = (N_SL + 11) / 12;
	wire [N_M1*K-1:0] sel_m1;
	genvar gm1, gmk;
	generate
		for (gm1 = 0; gm1 < N_M1; gm1 = gm1 + 1) begin : g_ex_m1
			for (gmk = 0; gmk < K; gmk = gmk + 1) begin : g_m1_k
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m1 (.I0(grant_oh_r[gmk]), .O(sel_m1[gm1*K + gmk]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_m1[gm1*K + gmk] = grant_oh_r[gmk];
`else
				assign sel_m1[gm1*K + gmk] = grant_oh_r[gmk];
`endif
			end
		end
	endgenerate
	wire [EX_W-1:0] mux_ex;
	genvar gs;
	generate
		for (gs = 0; gs < N_SL; gs = gs + 1) begin : g_ex_mux
			localparam integer B0 = gs * SL_W;
			localparam integer BW = (B0 + SL_W <= EX_W) ? SL_W : (EX_W - B0);
			(* keep = "true" *) wire [K-1:0] sel_oh_sl;
			for (gk = 0; gk < K; gk = gk + 1) begin : g_sel_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sel (.I0(sel_m1[(gs / 12)*K + gk]), .O(sel_oh_sl[gk]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_oh_sl[gk] = sel_m1[(gs / 12)*K + gk];
`else
				assign sel_oh_sl[gk] = sel_m1[(gs / 12)*K + gk];
`endif
			end
			reg [BW-1:0] sl_v;
			integer mi;
			always @(*) begin
				sl_v = {BW{1'b0}};
				for (mi = 0; mi < K; mi = mi + 1)
					if (sel_oh_sl[mi])
						sl_v = sl_v | sk_d_flat[mi*EX_W + B0 +: BW];
			end
			assign mux_ex[B0 +: BW] = sl_v;
		end
	endgenerate

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			grant_v_r  <= 1'b0;
			grant_oh_r <= {K{1'b0}};
			rr_ge_r    <= {K{1'b1}};
			c_ex_next  <= 16'd0;
		end else begin
			c_ex_next <= p_ex_next;
			if (!grant_v_r) begin
				if (any_cand) begin
					grant_v_r  <= 1'b1;
					grant_oh_r <= next_grant_oh;
					rr_ge_r    <= next_rr_ge;
				end
			end else if (fire && pick_last) begin
				grant_v_r  <= 1'b0;
				grant_oh_r <= {K{1'b0}};
			end
		end
	end

	atg_ex_rt_skid #(.WIDTH(EX_W), .SL_S(7)) u_ex_out_sk (
		.clk(clk), .rst_n(rst_n),
		.in_valid(pick_valid), .in_ready(out_in_ready),
		.in_data(mux_ex),
		.out_valid(p_ex_valid), .out_ready(p_ex_ready),
		.out_data({p_ex_seq, p_ex_xy, p_ex_mask, p_ex_color, p_ex_color1,
		           p_ex_has_src1, p_ex_depth, p_ex_aux, p_ex_last})
	);
	wire unused_ex_kw = (KW != 0);
endmodule

/* Fanout-bounded wide skid slice (SL_S <= 10 keeps Vivado FO <= 20). */
module atg_ex_rt_skid #(
	parameter WIDTH = 32,
	parameter SL_S = 7
) (
	input wire clk,
	input wire rst_n,
	input wire in_valid,
	(* keep = "true" *) output reg in_ready,
	input wire [WIDTH-1:0] in_data,
	(* keep = "true" *) output reg out_valid,
	input wire out_ready,
	output wire [WIDTH-1:0] out_data
);
	localparam N_SL = (WIDTH + SL_S - 1) / SL_S;
	localparam N_G1 = (N_SL + 11) / 12;
	localparam N_G0 = (N_G1 + 11) / 12;
	wire [N_G0-1:0] ordy_g0, ival_g0;
	wire [N_G1-1:0] ordy_g1, ival_g1;
	wire [N_SL-1:0] out_ready_g;
	wire [N_SL-1:0] in_valid_g;
	wire rst_p = !rst_n;

	genvar gg0, gg1, gs;
	generate
		for (gg0 = 0; gg0 < N_G0; gg0 = gg0 + 1) begin : g_sk_g0
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g0_rdy (.I0(out_ready), .O(ordy_g0[gg0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g0_val (.I0(in_valid),  .O(ival_g0[gg0]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ordy_g0[gg0] = out_ready;
			assign ival_g0[gg0] = in_valid;
`else
			assign ordy_g0[gg0] = out_ready;
			assign ival_g0[gg0] = in_valid;
`endif
		end
		for (gg1 = 0; gg1 < N_G1; gg1 = gg1 + 1) begin : g_sk_g1
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g1_rdy (.I0(ordy_g0[gg1 / 12]), .O(ordy_g1[gg1]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g1_val (.I0(ival_g0[gg1 / 12]), .O(ival_g1[gg1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ordy_g1[gg1] = ordy_g0[gg1 / 12];
			assign ival_g1[gg1] = ival_g0[gg1 / 12];
`else
			assign ordy_g1[gg1] = ordy_g0[gg1 / 12];
			assign ival_g1[gg1] = ival_g0[gg1 / 12];
`endif
		end
		for (gs = 0; gs < N_SL; gs = gs + 1) begin : g_sk_sl
			localparam SL_B = gs * SL_S;
			localparam SL_W = (SL_B + SL_S <= WIDTH) ? SL_S : (WIDTH - SL_B);
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ordy (.I0(ordy_g1[gs / 12]), .O(out_ready_g[gs]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ival (.I0(ival_g1[gs / 12]), .O(in_valid_g[gs]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign out_ready_g[gs] = ordy_g1[gs / 12];
			assign in_valid_g[gs] = ival_g1[gs / 12];
`else
			assign out_ready_g[gs] = ordy_g1[gs / 12];
			assign in_valid_g[gs]  = ival_g1[gs / 12];
`endif
			(* shreg_extract = "no" *) reg [SL_W-1:0] out_d_sl;
			(* shreg_extract = "no" *) reg [SL_W-1:0] skid_d_sl;
			reg out_v_sl, skid_v_sl, in_r_sl;
			wire load_out_s  = skid_v_sl ? out_ready_g[gs] : (in_valid_g[gs] && (out_ready_g[gs] || !out_v_sl));
			wire skid_load_s = in_valid_g[gs] && in_r_sl;
			wire [SL_W-1:0] next_out_s = skid_v_sl ? skid_d_sl : in_data[SL_B +: SL_W];

			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					out_v_sl  <= 1'b0;
					skid_v_sl <= 1'b0;
					in_r_sl   <= 1'b1;
				end else begin
					if (skid_v_sl) begin
						if (out_ready_g[gs]) begin
							skid_v_sl <= 1'b0;
							in_r_sl   <= 1'b1;
						end
					end else if (in_valid_g[gs] && out_v_sl && !out_ready_g[gs]) begin
						skid_v_sl <= 1'b1;
						in_r_sl   <= 1'b0;
					end else if (in_valid_g[gs] && !out_v_sl) begin
						out_v_sl <= 1'b1;
					end else if (out_ready_g[gs] && !in_valid_g[gs]) begin
						out_v_sl <= 1'b0;
					end
				end
			end
			always @(posedge clk) begin
				if (load_out_s)
					out_d_sl <= next_out_s;
				if (skid_load_s)
					skid_d_sl <= in_data[SL_B +: SL_W];
			end
			assign out_data[SL_B +: SL_W] = out_d_sl;
		end
	endgenerate

	reg skid_valid_r;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			out_valid    <= 1'b0;
			skid_valid_r <= 1'b0;
			in_ready     <= 1'b1;
		end else if (skid_valid_r) begin
			if (out_ready) begin
				skid_valid_r <= 1'b0;
				in_ready     <= 1'b1;
			end
		end else if (in_valid && out_valid && !out_ready) begin
			skid_valid_r <= 1'b1;
			in_ready     <= 1'b0;
		end else if (in_valid && !out_valid) begin
			out_valid <= 1'b1;
		end else if (out_ready && !in_valid) begin
			out_valid <= 1'b0;
		end
	end
endmodule

`default_nettype wire
