/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_fword: a lane's four words in atg_tu_filter (P18, combinational):
 * the conversions i_rn, then RGB565's texf alpha 1.0, a depth texel
 * without compare (D, 0, 0, 1.0; D32F the word itself), gather (texels 2,
 * 3, 1, 0: the compare's 1.0 or 0.0, the swizzle's constant, a depth
 * component, the byte's conversion), else the component swizzle; the
 * queries (texsz: the tc_value word; texqlod: the two signed values); 0
 * outside the mask.
 * See spec/gpu/tu/atg_tu_filter.md section 4.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_fword (
	input wire clk,
	input wire [127:0] i_rn,	/* the four conversions (P19) */
	input wire [127:0] i_lt18,	/* the lane's texels (P18) */
	input wire [31:0] i_vw18,	/* the lane's tc_value word (P18) */
	input wire [3:0] i_ok18,	/* the compares (P18) */
	input wire [49:0] i_ctl18,
	input wire i_mk18,
	output wire [127:0] o_res
);
	/* The component that swizzle code 3-6 (R, G, B, A) takes, by its bits 1:0. */
	function [1:0] zsrc;
		input [1:0] s;
		case (s)
		2'd3: zsrc = 2'd0;
		2'd0: zsrc = 2'd1;
		2'd1: zsrc = 2'd2;
		default: zsrc = 2'd3;
		endcase
	endfunction

	/* The texel of word g of a gather: 2, 3, 1, 0. */
	function [1:0] gtx;
		input [1:0] g;
		gtx = g == 2'd0 ? 2'd2 : g == 2'd1 ? 2'd3 : g == 2'd2 ? 2'd1 : 2'd0;
	endfunction

	/* P17 -> P18 pre-stage (from dl[18]): swizzle + gsw/gsrc decode, registered to P18_mid */
	wire d_nocomp18    = i_ctl18[8] && !i_ctl18[9];
	wire d32_sel18     = i_ctl18[1];
	wire rgb565_texf18 = i_ctl18[19] && i_ctl18[3];

	wire [3:0] base_use_rn;
	assign base_use_rn[0] = !(d_nocomp18 && d32_sel18);
	assign base_use_rn[1] = !d_nocomp18;
	assign base_use_rn[2] = !d_nocomp18;
	assign base_use_rn[3] = !(d_nocomp18 || rgb565_texf18);

	wire [2:0] gsw_m18 = (i_ctl18[15:14] == 2'd0) ? i_ctl18[38:36] :
	                     (i_ctl18[15:14] == 2'd1) ? i_ctl18[41:39] :
	                     (i_ctl18[15:14] == 2'd2) ? i_ctl18[44:42] : i_ctl18[47:45];
	wire [1:0] gsrc_m18 = (gsw_m18 >= 3'd3 && gsw_m18 <= 3'd6) ? zsrc(gsw_m18[1:0]) : i_ctl18[15:14];

	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [2:0]  gsw_r19 [0:3];
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [1:0]  gsrc_r19 [0:3];
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [3:0] c13_r19_lo, c13_r19_hi, c9_r19_lo, c9_r19_hi;
	(* extract_reset = "no" *) reg        c8_r19, c1_r19;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [3:0] q_mode_r19, s_mode_r19_lo, s_mode_r19_hi, mk_r19;
	(* extract_reset = "no" *) reg [3:0]  ok_r19;
	(* extract_reset = "no" *) reg [31:0] vw_r19;

	integer kc19;
	always @(posedge clk) begin
		for (kc19 = 0; kc19 < 4; kc19 = kc19 + 1) begin
			gsw_r19[kc19]  <= gsw_m18;
			gsrc_r19[kc19] <= gsrc_m18;
		end
		c13_r19_lo    <= {4{i_ctl18[13]}};
		c13_r19_hi    <= {4{i_ctl18[13]}};
		c9_r19_lo     <= {4{i_ctl18[9]}};
		c9_r19_hi     <= {4{i_ctl18[9]}};
		c8_r19        <= i_ctl18[8];
		c1_r19        <= i_ctl18[1];
		q_mode_r19    <= {4{i_ctl18[2] &&  i_ctl18[16]}};
		s_mode_r19_lo <= {4{i_ctl18[2] && !i_ctl18[16]}};
		s_mode_r19_hi <= {4{i_ctl18[2] && !i_ctl18[16]}};
		mk_r19        <= {4{i_mk18}};
		ok_r19        <= i_ok18;
		vw_r19        <= i_vw18;
	end

	(* extract_reset = "no" *) reg        swz_use_rn_r19 [0:3];
	(* extract_reset = "no" *) reg [1:0]  swz_src_r19    [0:3];
	(* extract_reset = "no" *) reg [31:0] swz_alt_r19    [0:3];
	(* extract_reset = "no" *) reg [31:0] gw_m_r19       [0:3];

	(* keep = "true", equivalent_register_removal = "no" *) reg        use_rn_lo_q [0:3], use_rn_hi_q [0:3];
	(* keep = "true", equivalent_register_removal = "no" *) reg [1:0]  sel_rn_lo_q [0:3], sel_rn_hi_q [0:3];
	(* extract_reset = "no" *) reg [15:0] alt_val_lo_q [0:3], alt_val_hi_q [0:3];
	(* extract_reset = "no" *) reg        msb_ovr_q   [0:1];
	(* extract_reset = "no" *) reg        msb_val_q   [0:1];

	genvar gc;
	generate
	for (gc = 0; gc < 4; gc = gc + 1) begin : g_ch
		localparam [1:0] CH = gc;
		wire [2:0] sc = i_ctl18[36 + 3*gc +: 3];
		wire [1:0] swz_src = (sc >= 3'd3 && sc <= 3'd6) ? zsrc(sc[1:0]) : CH;
		wire swz_const_zero = (sc == 3'd1);
		wire swz_const_one  = (sc == 3'd2);
		wire swz_use_rn = !swz_const_zero && !swz_const_one && base_use_rn[swz_src];
		wire [31:0] swz_alt = swz_const_zero ? 32'd0 :
		                      swz_const_one  ? 32'h3F800000 :
		                      (swz_src == 2'd0) ? i_lt18[31:0] :
		                      (swz_src == 2'd3) ? 32'h3F800000 : 32'd0;

		always @(posedge clk) begin
			swz_use_rn_r19[gc] <= swz_use_rn;
			swz_src_r19[gc]    <= swz_src;
			swz_alt_r19[gc]    <= swz_alt;
			gw_m_r19[gc]       <= i_lt18[gtx(CH)*32 +: 32];
		end

		/* P18_mid -> P19 (<= 7 cmos3) */
		wire c13_use_rn = !c9_r19_lo[gc] && (gsw_r19[gc] != 3'd1) && (gsw_r19[gc] != 3'd2) && (!c8_r19 || (gsrc_r19[gc] == 2'd0 && !c1_r19));
		wire [15:0] c13_alt_lo = c9_r19_lo[gc] ? 16'd0 :
		                         (gsw_r19[gc] == 3'd1) ? 16'd0 :
		                         (gsw_r19[gc] == 3'd2) ? 16'd0 :
		                         (gsrc_r19[gc] == 2'd0) ? gw_m_r19[gc][15:0] : 16'd0;
		wire [15:0] c13_alt_hi = c9_r19_hi[gc] ? (ok_r19[gtx(CH)] ? 16'h3F80 : 16'd0) :
		                         (gsw_r19[gc] == 3'd1) ? 16'd0 :
		                         (gsw_r19[gc] == 3'd2) ? 16'h3F80 :
		                         (gsrc_r19[gc] == 2'd0) ? gw_m_r19[gc][31:16] :
		                         (gsrc_r19[gc] == 2'd3) ? 16'h3F80 : 16'd0;

		wire mid_use_rn = c13_r19_lo[gc] ? c13_use_rn : swz_use_rn_r19[gc];
		wire [1:0] mid_sel = c13_r19_lo[gc] ? CH : swz_src_r19[gc];
		wire [15:0] mid_alt_lo = c13_r19_lo[gc] ? c13_alt_lo : swz_alt_r19[gc][15:0];
		wire [15:0] mid_alt_hi = c13_r19_hi[gc] ? c13_alt_hi : swz_alt_r19[gc][31:16];

		wire fin_use_rn = mk_r19[gc] && (q_mode_r19[gc] ? (gc < 2) : (!s_mode_r19_lo[gc] && mid_use_rn));
		wire [1:0] fin_sel = q_mode_r19[gc] ? CH : mid_sel;
		(* dont_touch = "true" *) wire fin_zero_lo_a = !mk_r19[gc] || q_mode_r19[gc];
		(* dont_touch = "true" *) wire fin_zero_lo_b = !mk_r19[gc] || q_mode_r19[gc];
		(* dont_touch = "true" *) wire fin_zero_hi_a = !mk_r19[gc] || q_mode_r19[gc];
		(* dont_touch = "true" *) wire fin_zero_hi_b = !mk_r19[gc] || q_mode_r19[gc];
		wire [15:0] fin_alt_lo = {
			fin_zero_lo_b ? 8'd0 : (s_mode_r19_lo[gc] ? ((gc == 0) ? vw_r19[15:8] : 8'd0) : mid_alt_lo[15:8]),
			fin_zero_lo_a ? 8'd0 : (s_mode_r19_lo[gc] ? ((gc == 0) ? vw_r19[7:0]  : 8'd0) : mid_alt_lo[7:0])
		};
		wire [15:0] fin_alt_hi = {
			fin_zero_hi_b ? 8'd0 : (s_mode_r19_hi[gc] ? ((gc == 0) ? vw_r19[31:24] : 8'd0) : mid_alt_hi[15:8]),
			fin_zero_hi_a ? 8'd0 : (s_mode_r19_hi[gc] ? ((gc == 0) ? vw_r19[23:16] : 8'd0) : mid_alt_hi[7:0])
		};

		always @(posedge clk) begin
			use_rn_lo_q[gc]  <= fin_use_rn;
			use_rn_hi_q[gc]  <= fin_use_rn;
			sel_rn_lo_q[gc]  <= fin_sel;
			sel_rn_hi_q[gc]  <= fin_sel;
			alt_val_lo_q[gc] <= fin_alt_lo;
			alt_val_hi_q[gc] <= fin_alt_hi;
		end

		if (gc < 2) begin : g_q_msb
			always @(posedge clk) begin
				msb_ovr_q[gc] <= mk_r19[gc] && q_mode_r19[gc];
				msb_val_q[gc] <= (gc == 0) ? vw_r19[15] : vw_r19[31];
			end
		end

		wire [31:0] rn_lo_w = (sel_rn_lo_q[gc] == 2'd0) ? i_rn[31:0]   :
		                      (sel_rn_lo_q[gc] == 2'd1) ? i_rn[63:32]  :
		                      (sel_rn_lo_q[gc] == 2'd2) ? i_rn[95:64]  : i_rn[127:96];
		wire [31:0] rn_hi_w = (sel_rn_hi_q[gc] == 2'd0) ? i_rn[31:0]   :
		                      (sel_rn_hi_q[gc] == 2'd1) ? i_rn[63:32]  :
		                      (sel_rn_hi_q[gc] == 2'd2) ? i_rn[95:64]  : i_rn[127:96];
		assign o_res[gc*32 +: 16]      = use_rn_lo_q[gc] ? rn_lo_w[15:0]  : alt_val_lo_q[gc];
		assign o_res[gc*32 + 16 +: 15] = use_rn_hi_q[gc] ? rn_hi_w[30:16] : alt_val_hi_q[gc][14:0];
		if (gc < 2) begin : g_msb31
			assign o_res[gc*32 + 31] = msb_ovr_q[gc] ? msb_val_q[gc] :
			                           (use_rn_hi_q[gc] ? rn_hi_w[31] : alt_val_hi_q[gc][15]);
		end else begin : g_nomsb31
			assign o_res[gc*32 + 31] = use_rn_hi_q[gc] ? rn_hi_w[31] : alt_val_hi_q[gc][15];
		end
	end
	endgenerate

	wire unused_ok = &{1'b0, i_ctl18[49:48], i_ctl18[35:20], i_ctl18[18:17], i_ctl18[12:10],
		i_ctl18[7:4], i_ctl18[0]};
endmodule

`default_nettype wire
