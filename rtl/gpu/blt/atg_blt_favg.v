/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_favg: the floating-point and sRGB averages of the blitter's
 * RESOLVE (docs/ghci.md sections 8.2 and 8.5), one group of up to four
 * pixels at a time (atg_blt_rsv_px stalls while it works).  The input and
 * output layouts are atg_blt_avg's (sample s of pixel p at slot 4 p + s of
 * 2^psl bytes; pixel p at bits 8 ps p).
 *
 *   fmt 3 SRGB8_A8  32-bit pixels: bytes 0-2 decoded to linear fp32
 *                   (atg_srgb_dec), averaged, encoded to the nearest code
 *                   (atg_fmt_srgbenc); byte 3 as UNORM8 (atg_blt_avg's rule)
 *   fmt 4 SFLOAT16  every 16 bits of 16-, 32- and 64-bit pixels: to fp32
 *                   exactly, averaged, to binary16 to nearest even
 *                   (am_fp_f2h)
 *   fmt 5 SFLOAT32  every 32 bits of 32-, 64- and 128-bit pixels, averaged
 *
 * The average of a channel is ((s0 + s1) + (s2 + s3)) x 0.25 in fp32, each
 * step one fused multiply-add (am_fp_fma_p; round to nearest
 * even, denormals flushed, NaN 0x7FC00000), four steps a channel (and the
 * encoder's nine cycles for sRGB).  i_valid/i_ready: a group, taken in
 * IDLE (npx pixels); o_valid/o_ready: its averages, held until taken.
 * See spec/gpu/blt/atg_blt_favg.md.
 *
 * States: IDLE →(i_valid)→ RUN (channel by channel: steps 0-3, sRGB: ENC)
 * → OUT →(o_ready)→ IDLE.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_blt_favg (
	input wire clk,
	input wire rst_n,
	input wire i_valid,
	output wire i_ready,
	input wire [2:0] fmt,
	input wire [2:0] psl,
	input wire [2:0] npx,	/* pixels */
	input wire [511:0] in_s,
	output wire o_valid,
	input wire o_ready,
	output reg [127:0] out_p
);
	`include "atg_srgb.vh"
	localparam [3:0] IDLE = 4'd0, RUN = 4'd1, ENC_START = 4'd2, ENC = 4'd3,
		FH0 = 4'd4, FH1 = 4'd5, FH2 = 4'd6, FH3 = 4'd7, COMMIT = 4'd8, OUT = 4'd9;
	localparam [31:0] ONE = 32'h3F800000, QUARTER = 32'h3E800000;

	reg [3:0] st;
	reg [511:0] s_in;
	(* max_fanout = 16 *) reg [2:0] m_fmt, m_psl;
	reg [2:0] m_n;
	(* max_fanout = 16 *) reg [1:0] p;		/* the pixel */
	(* max_fanout = 16 *) reg [2:0] j;		/* its channel */
	reg [2:0] step_d;
	(* max_fanout = 16 *) reg [1:0] step;
	reg [4:0] fma_wait;
	reg [31:0] r01, r23, sm, r_final;
	reg [31:0] r_fa, r_fb, r_fc;
	reg        r_fa_nrm;
	reg [31:0] r_dec_a, r_dec_c;
	reg [127:0] r_xa_128, r_xc_128;
	(* max_fanout = 16 *) reg [31:0] r_xa, r_xc;
	(* dont_touch = "true" *) reg [31:0] r_srgb_a_rom, r_srgb_c_rom;
	(* dont_touch = "true" *) reg [31:0] r_srgb_a, r_srgb_c;
	reg [39:0] aq_s_r, aq_c_r;
	reg [31:0] aq_r;
	reg        r_ha_exp_sp0, r_ha_zero0, r_ha_sign0, r_hc_exp_sp0, r_hc_zero0, r_hc_sign0;
	reg [9:0]  r_ha_man0, r_hc_man0;
	reg [3:0]  r_ha_lz0, r_hc_lz0;
	reg [4:0]  r_ha_exp0, r_hc_exp0;
	(* equivalent_register_removal = "no" *) reg [15:0] ca_sl_rep, cc_sl_rep;
	reg [5:0] ia_s_r, ic_s_r;
	reg [4:0] ia_h_r, ic_h_r;
	reg [3:0] ia_f_r, ic_f_r;
	reg [2:0] cpp_last_r, n_last_r;

	/* The pixel's bytes, 2^psl in 5 bits (0 from 32 bytes), by table. */
	function [4:0] psl_to_ps;
		input [2:0] pl;
		case (pl)
		3'd0: psl_to_ps = 5'd1;
		3'd1: psl_to_ps = 5'd2;
		3'd2: psl_to_ps = 5'd4;
		3'd3: psl_to_ps = 5'd8;
		3'd4: psl_to_ps = 5'd16;
		default: psl_to_ps = 5'd0;
		endcase
	endfunction
	wire [4:0] ps_in = psl_to_ps(psl);
	wire [2:0] cpp_in = (fmt == 3'd3) ? 3'd3 : (fmt == 3'd4) ? ps_in[3:1] : ps_in[4:2];
	wire [2:0] cpp_last_in, n_last_in;
	wire [1:0] last_co_unused;
	am_sub #(.W(3)) s_cpp (.a(cpp_in), .b(3'd1), .bi(1'b0), .d(cpp_last_in), .bo(last_co_unused[0]));
	am_sub #(.W(3)) s_n (.a(npx), .b(3'd1), .bi(1'b0), .d(n_last_in), .bo(last_co_unused[1]));

	/* Steps 0 and 1 add two samples each: (s0 + s1), then (s2 + s3); only
	 * those two samples are decoded. */
`ifdef MUT_FAVG_PAIR
	wire [1:0] sn_a = step == 2'd1 ? 2'd2 : 2'd0;
	wire [1:0] sn_c = step == 2'd1 ? 2'd1 : 2'd2;
`else
	wire [1:0] sn_a = step == 2'd1 ? 2'd2 : 2'd0;
	wire [1:0] sn_c = step == 2'd1 ? 2'd3 : 2'd1;
`endif
	wire [5:0] ta_sh, tc_sh, ta_ps, tc_ps, ia_s, ic_s;
	wire [4:0] ia_h, ic_h;
	wire [3:0] ia_f, ic_f;
	wire [5:0] idx_co_unused;

	am_shift #(.W(6), .SW(3)) h_ta (.a({2'd0, p, sn_a}), .amt(m_psl), .left(1'b1), .arith(1'b0), .y(ta_sh));
	am_shift #(.W(6), .SW(3)) h_tc (.a({2'd0, p, sn_c}), .amt(m_psl), .left(1'b1), .arith(1'b0), .y(tc_sh));
	assign ta_ps = (m_psl > 3'd4) ? 6'd0 : ta_sh;
	assign tc_ps = (m_psl > 3'd4) ? 6'd0 : tc_sh;
	am_add #(.W(6)) a_ias (.a({p, sn_a, 2'b00}), .b({3'd0, j}), .ci(1'b0), .s(ia_s), .co(idx_co_unused[0]));
	am_add #(.W(6)) a_ics (.a({p, sn_c, 2'b00}), .b({3'd0, j}), .ci(1'b0), .s(ic_s), .co(idx_co_unused[1]));
	am_add #(.W(5)) a_iah (.a(ta_ps[5:1]), .b({2'd0, j}), .ci(1'b0), .s(ia_h), .co(idx_co_unused[2]));
	am_add #(.W(5)) a_ich (.a(tc_ps[5:1]), .b({2'd0, j}), .ci(1'b0), .s(ic_h), .co(idx_co_unused[3]));
	am_add #(.W(4)) a_iaf (.a(ta_ps[5:2]), .b({1'd0, j}), .ci(1'b0), .s(ia_f), .co(idx_co_unused[4]));
	am_add #(.W(4)) a_icf (.a(tc_ps[5:2]), .b({1'd0, j}), .ci(1'b0), .s(ic_f), .co(idx_co_unused[5]));

	/* Sample sn of channel j of pixel p: Stage 1 selects 128b chunk (with replicated selector); Stage 2 extracts; Stage 3 decodes in 2 cycles. */
	wire [1:0] chk_a_n = (m_fmt == 3'd3) ? ia_s[5:4] : (m_fmt == 3'd4) ? ia_h[4:3] : ia_f[3:2];
	wire [1:0] chk_c_n = (m_fmt == 3'd3) ? ic_s[5:4] : (m_fmt == 3'd4) ? ic_h[4:3] : ic_f[3:2];
	wire [127:0] raw_xa_128, raw_xc_128;
	genvar gc128;
	generate
		for (gc128 = 0; gc128 < 8; gc128 = gc128 + 1) begin : g_chk128
			reg [15:0] xas_m, xcs_m;
			wire [1:0] ca_s_l = ca_sl_rep[gc128*2 +: 2];
			wire [1:0] cc_s_l = cc_sl_rep[gc128*2 +: 2];
			always @(*) begin
				case (ca_s_l)
				2'd0: xas_m = s_in[  0 + gc128*16 +: 16];
				2'd1: xas_m = s_in[128 + gc128*16 +: 16];
				2'd2: xas_m = s_in[256 + gc128*16 +: 16];
				default: xas_m = s_in[384 + gc128*16 +: 16];
				endcase
				case (cc_s_l)
				2'd0: xcs_m = s_in[  0 + gc128*16 +: 16];
				2'd1: xcs_m = s_in[128 + gc128*16 +: 16];
				2'd2: xcs_m = s_in[256 + gc128*16 +: 16];
				default: xcs_m = s_in[384 + gc128*16 +: 16];
				endcase
			end
			assign raw_xa_128[gc128*16 +: 16] = xas_m;
			assign raw_xc_128[gc128*16 +: 16] = xcs_m;
		end
	endgenerate

	wire [31:0] raw_xa = (m_fmt == 3'd3) ? {24'd0, r_xa_128[{ia_s_r[3:0], 3'b000} +: 8]} :
	                     (m_fmt == 3'd4) ? {16'd0, r_xa_128[{ia_h_r[2:0], 4'b0000} +: 16]} :
	                                       r_xa_128[{ia_f_r[1:0], 5'b00000} +: 32];
	wire [31:0] raw_xc = (m_fmt == 3'd3) ? {24'd0, r_xc_128[{ic_s_r[3:0], 3'b000} +: 8]} :
	                     (m_fmt == 3'd4) ? {16'd0, r_xc_128[{ic_h_r[2:0], 4'b0000} +: 16]} :
	                                       r_xc_128[{ic_f_r[1:0], 5'b00000} +: 32];

	/* 2-stage fp16 -> fp32 with denormal normalization (<= 8 cmos3 per stage). */
	function [3:0] fp16_lz;
		input [9:0] m;
		casez (m)
		10'b1?????????: fp16_lz = 4'd0;
		10'b01????????: fp16_lz = 4'd1;
		10'b001???????: fp16_lz = 4'd2;
		10'b0001??????: fp16_lz = 4'd3;
		10'b00001?????: fp16_lz = 4'd4;
		10'b000001????: fp16_lz = 4'd5;
		10'b0000001???: fp16_lz = 4'd6;
		10'b00000001??: fp16_lz = 4'd7;
		10'b000000001?: fp16_lz = 4'd8;
		default:        fp16_lz = 4'd9;
		endcase
	endfunction
	wire [3:0] ha_lz_w = fp16_lz(r_xa[9:0]);
	wire [3:0] hc_lz_w = fp16_lz(r_xc[9:0]);
	wire [3:0] ha_lz1_w, hc_lz1_w;
	wire [9:0] ha_fr_w, hc_fr_w;
	wire [7:0] ha_ed_w, ha_en_w, hc_ed_w, hc_en_w;
	wire [5:0] h2f_co_unused;
	am_add #(.W(4)) u_halz (.a(r_ha_lz0), .b(4'd0), .ci(1'b1), .s(ha_lz1_w), .co(h2f_co_unused[0]));
	am_shift #(.W(10), .SW(4), .DIR(2)) u_hafr (.a(r_ha_man0), .amt(ha_lz1_w), .left(1'b1), .arith(1'b0), .y(ha_fr_w));
	am_sub #(.W(8)) u_haed (.a(8'd112), .b({4'd0, r_ha_lz0}), .bi(1'b0), .d(ha_ed_w), .bo(h2f_co_unused[1]));
	am_add #(.W(8)) u_haen (.a({3'd0, r_ha_exp0}), .b(8'd112), .ci(1'b0), .s(ha_en_w), .co(h2f_co_unused[2]));
	am_add #(.W(4)) u_hclz (.a(r_hc_lz0), .b(4'd0), .ci(1'b1), .s(hc_lz1_w), .co(h2f_co_unused[3]));
	am_shift #(.W(10), .SW(4), .DIR(2)) u_hcfr (.a(r_hc_man0), .amt(hc_lz1_w), .left(1'b1), .arith(1'b0), .y(hc_fr_w));
	am_sub #(.W(8)) u_hced (.a(8'd112), .b({4'd0, r_hc_lz0}), .bi(1'b0), .d(hc_ed_w), .bo(h2f_co_unused[4]));
	am_add #(.W(8)) u_hcen (.a({3'd0, r_hc_exp0}), .b(8'd112), .ci(1'b0), .s(hc_en_w), .co(h2f_co_unused[5]));
	wire [31:0] ha = r_ha_exp_sp0 ? ((r_ha_man0 != 10'd0) ? 32'h7FC00000 : {r_ha_sign0, 31'h7F800000}) :
		r_ha_zero0 ? ((r_ha_man0 == 10'd0) ? {r_ha_sign0, 31'd0} : {r_ha_sign0, ha_ed_w, ha_fr_w, 13'd0}) :
		{r_ha_sign0, ha_en_w, r_ha_man0, 13'd0};
	wire [31:0] hc = r_hc_exp_sp0 ? ((r_hc_man0 != 10'd0) ? 32'h7FC00000 : {r_hc_sign0, 31'h7F800000}) :
		r_hc_zero0 ? ((r_hc_man0 == 10'd0) ? {r_hc_sign0, 31'd0} : {r_hc_sign0, hc_ed_w, hc_fr_w, 13'd0}) :
		{r_hc_sign0, hc_en_w, r_hc_man0, 13'd0};
	wire [31:0] xa = (m_fmt == 3'd3) ? r_srgb_a : (m_fmt == 3'd4) ? ha : r_xa;
	wire [31:0] xc = (m_fmt == 3'd3) ? r_srgb_c : (m_fmt == 3'd4) ? hc : r_xc;
	/* The fma: fb is 1.0 or 0.25 (a significand of 2^23), so the product of
	 * the significands is fa's shifted (no multiplier). */
	wire [31:0] fr;
	am_fp_fma_p #(.LAT(16)) u_fma (.clk(clk), .rst_n(rst_n), .en(1'b1), .a0(r_fa), .b0(r_fb), .c0(r_fc), .rm(3'd0), .noc(1'b0),
		.p({1'b0, r_fa_nrm ? {1'b1, r_fa[22:0]} : 24'd0, 23'd0}), .r(fr));

	/*
	 * 4-cycle pipelined fp32 -> fp16 conversion (exact match to am_fp_f2h)
	 * so every stage stays <= 12 cmos3 levels instead of 63 cmos3.
	 */
	wire [23:0] fh_m = {1'b1, r_final[22:0]};
	wire [11:0] fh_q_w;
	wire [4:0] fh_sh_w, fh_sh1_w;
	wire [5:0] fh_co_unused;
	am_add #(.W(12)) u_fh_q (.a({1'b0, fh_m[23:13]}), .b(12'd0),
		.ci(r_final[12] && (r_final[11:0] != 12'd0 || r_final[13])), .s(fh_q_w), .co(fh_co_unused[0]));
	am_sub #(.W(5)) u_fh_sh (.a(5'd30), .b(r_final[27:23]), .bi(1'b0), .d(fh_sh_w), .bo(fh_co_unused[1]));
	am_sub #(.W(5)) u_fh_sh1 (.a(5'd29), .b(r_final[27:23]), .bi(1'b0), .d(fh_sh1_w), .bo(fh_co_unused[2]));

	reg        fh0_sign_r, fh0_ef_r, fh0_mnz_r, fh0_ge113_r, fh0_gt142_r, fh0_le101_r;
	reg [11:0] fh0_q_r;
	reg [4:0]  fh0_sh_r, fh0_sh1_r;
	wire [7:0] fh_ex_w;
	wire [23:0] fh_ds_w, fh_hi_w;
	am_sub #(.W(8)) u_fh_ex (.a(r_final[30:23]), .b(fh0_q_r[11] ? 8'd111 : 8'd112), .bi(1'b0),
		.d(fh_ex_w), .bo(fh_co_unused[3]));
	am_shift #(.W(24), .SW(5), .DIR(1)) u_fh_ds (.a(fh_m), .amt(fh0_sh_r), .left(1'b0), .arith(1'b0), .y(fh_ds_w));
	am_shift #(.W(24), .SW(5), .DIR(2)) u_fh_hi (.a({24{1'b1}}), .amt(fh0_sh1_r), .left(1'b1), .arith(1'b0), .y(fh_hi_w));

	reg [7:0]  fh1_ex_r;
	reg [14:0] fh1_ds_r;
	reg [23:0] fh1_hi_r;
	reg        fh1_g_r;
	wire fh2_st_w = (fh_m & ~fh1_hi_r) != 24'd0;
	wire fh2_rnd_w = fh1_g_r && (fh2_st_w || fh1_ds_r[0]);

	reg [14:0] fh2_ds_r;
	reg        fh2_rnd_r, fh2_ovf_r;
	wire [14:0] fh3_dr_w;
	am_add #(.W(15)) u_fh_dr (.a(fh2_ds_r), .b(15'd0), .ci(fh2_rnd_r), .s(fh3_dr_w), .co(fh_co_unused[4]));
	assign fh_co_unused[5] = 1'b0;

	reg [15:0] fh_r;
	wire [15:0] fh_mux_w = fh0_ef_r ? (fh0_mnz_r ? 16'h7E00 : {fh0_sign_r, 15'h7C00}) :
		fh0_ge113_r ? (fh2_ovf_r ? {fh0_sign_r, 15'h7C00} :
			{fh0_sign_r, fh1_ex_r[4:0], fh0_q_r[11:10] != 2'b01 ? 10'd0 : fh0_q_r[9:0]}) :
		fh0_le101_r ? {fh0_sign_r, 15'd0} : {fh0_sign_r, fh3_dr_w};
	wire [15:0] fh = fh_r;
	wire lastj = j == cpp_last_r;
	wire lastp = {1'b0, p} == n_last_r;

	/* The sRGB encoder. */
	wire e_ready, e_valid;
	wire [7:0] e_code;
	atg_fmt_srgbenc enc (
		.clk(clk), .rst_n(rst_n), .i_valid(st == ENC_START),
		.i_ready(e_ready), .i_x(r_final), .o_valid(e_valid), .o_ready(st == ENC), .o_code(e_code)
	);

	/* sRGB alpha: byte 3, 2-stage pipelined UNORM8 average of the four samples, ties to even. */
	wire [39:0] ac2_s_w, ac2_c_w;
	wire [31:0] aq_w;	/* pixel q's at bits 8 q */

	genvar q;
	generate
		for (q = 0; q < 4; q = q + 1) begin : g_alpha
			wire [9:0] aw0 = {2'd0, s_in[8 * (4 * (4 * q) + 3) +: 8]};
			wire [9:0] aw1 = {2'd0, s_in[8 * (4 * (4 * q + 1) + 3) +: 8]};
			wire [9:0] aw2 = {2'd0, s_in[8 * (4 * (4 * q + 2) + 3) +: 8]};
			wire [9:0] aw3 = {2'd0, s_in[8 * (4 * (4 * q + 3) + 3) +: 8]};
			wire [9:0] ac1_s = aw0 ^ aw1 ^ aw2;
			wire [9:0] ac1_c = {((aw0[8:0] & aw1[8:0]) | (aw1[8:0] & aw2[8:0]) | (aw0[8:0] & aw2[8:0])), 1'b0};
			assign ac2_s_w[q*10 +: 10] = ac1_s ^ ac1_c ^ aw3;
			assign ac2_c_w[q*10 +: 10] = {((ac1_s[8:0] & ac1_c[8:0]) | (ac1_c[8:0] & aw3[8:0]) | (ac1_s[8:0] & aw3[8:0])), 1'b0};
			wire [9:0] ac2_s = aq_s_r[q*10 +: 10];
			wire [9:0] ac2_c = aq_c_r[q*10 +: 10];
			wire [9:0] asum, asum_p4;
			wire [1:0] a_co_unused;
			am_add #(.W(10)) a_s (.a(ac2_s), .b(ac2_c), .ci(1'b0), .s(asum), .co(a_co_unused[0]));
			wire [7:0] ap4_s = ac2_s[9:2] ^ ac2_c[9:2] ^ 8'd1;
			wire [7:0] ap4_c = {((ac2_s[8:2] & ac2_c[8:2]) | (ac2_s[8:2] & 7'd1) | (ac2_c[8:2] & 7'd1)), 1'b0};
			am_add #(.W(10)) a_sp4 (.a({ap4_s, ac2_s[1:0]}), .b({ap4_c, ac2_c[1:0]}), .ci(1'b0), .s(asum_p4), .co(a_co_unused[1]));
			assign aq_w[8 * q +: 8] = (asum[1] && (asum[0] || asum[2])) ? asum_p4[9:2] : asum[9:2];

			wire unused_ok = &{1'b0, a_co_unused, asum_p4[1:0]};
		end
	endgenerate

	assign i_ready = st == IDLE;
	assign o_valid = st == OUT;

	/* The channel's result goes in, at its place in the output (as the samples' places, sample 0). */
	wire take = (st == COMMIT) || (st == ENC && e_valid);
	wire [2:0] j_inc;
	wire [1:0] p_inc, step_inc;
	wire [2:0] out_co_unused;
	integer k;

	am_add #(.W(3)) a_j (.a(j), .b(3'd0), .ci(1'b1), .s(j_inc), .co(out_co_unused[0]));
	am_add #(.W(2)) a_p (.a(p), .b(2'd0), .ci(1'b1), .s(p_inc), .co(out_co_unused[1]));
	am_add #(.W(2)) a_step (.a(step), .b(2'd0), .ci(1'b1), .s(step_inc), .co(out_co_unused[2]));

	wire [2:0] h_idx = (m_psl == 3'd1) ? {1'b0, p} :
	                   (m_psl == 3'd2) ? {p, j[0]} :
	                   {p[0], j[1:0]};
	wire [1:0] w_idx = (m_psl == 3'd2) ? p :
	                   (m_psl == 3'd3) ? {p[0], j[0]} :
	                   j[1:0];

	/* Datapath registers without rst_n on CE (cuts Check 5 & Rule 2 s_in CE fanout). */
	wire s_in_load = (st == IDLE) && i_valid;
	genvar gsin;
	generate
		for (gsin = 0; gsin < 32; gsin = gsin + 1) begin : g_sin_sl
			(* keep = "true" *) wire s_in_ce = s_in_load;
			always @(posedge clk) begin
				if (s_in_ce)
					s_in[gsin*16 +: 16] <= in_s[gsin*16 +: 16];
			end
		end
	endgenerate

	always @(posedge clk) begin
		r_xa_128 <= raw_xa_128;
		r_xc_128 <= raw_xc_128;
		r_srgb_a_rom <= atg_srgb_dec(r_xa[7:0]);
		r_srgb_c_rom <= atg_srgb_dec(r_xc[7:0]);
		r_srgb_a <= r_srgb_a_rom;
		r_srgb_c <= r_srgb_c_rom;
		aq_s_r <= ac2_s_w;
		aq_c_r <= ac2_c_w;
		aq_r <= aq_w;
		r_ha_exp_sp0 <= (r_xa[14:10] == 5'd31);
		r_ha_zero0   <= (r_xa[14:10] == 5'd0);
		r_ha_sign0   <= r_xa[15];
		r_ha_man0    <= r_xa[9:0];
		r_ha_exp0    <= r_xa[14:10];
		r_ha_lz0     <= ha_lz_w;
		r_hc_exp_sp0 <= (r_xc[14:10] == 5'd31);
		r_hc_zero0   <= (r_xc[14:10] == 5'd0);
		r_hc_sign0   <= r_xc[15];
		r_hc_man0    <= r_xc[9:0];
		r_hc_exp0    <= r_xc[14:10];
		r_hc_lz0     <= hc_lz_w;
		r_dec_a  <= xa;
		r_dec_c  <= xc;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			st <= IDLE;
			m_fmt <= 3'd0;
			m_psl <= 3'd0;
			m_n <= 3'd0;
			cpp_last_r <= 3'd0;
			n_last_r <= 3'd0;
			p <= 2'd0;
			j <= 3'd0;
			step <= 2'd0;
			fma_wait <= 5'd0;
			r01 <= 32'd0;
			r23 <= 32'd0;
			sm <= 32'd0;
			r_final <= 32'd0;
			r_fa <= 32'd0;
			r_fa_nrm <= 1'b0;
			r_fb <= 32'd0;
			r_fc <= 32'd0;
			r_xa <= 32'd0;
			r_xc <= 32'd0;
			ia_s_r <= 6'd0;
			ic_s_r <= 6'd0;
			ia_h_r <= 5'd0;
			ic_h_r <= 5'd0;
			ia_f_r <= 4'd0;
			ic_f_r <= 4'd0;
			ca_sl_rep <= 16'd0;
			cc_sl_rep <= 16'd0;
			fh0_sign_r <= 1'b0;
			fh0_ef_r <= 1'b0;
			fh0_mnz_r <= 1'b0;
			fh0_ge113_r <= 1'b0;
			fh0_gt142_r <= 1'b0;
			fh0_le101_r <= 1'b0;
			fh0_q_r <= 12'd0;
			fh0_sh_r <= 5'd0;
			fh0_sh1_r <= 5'd0;
			fh1_ex_r <= 8'd0;
			fh1_ds_r <= 15'd0;
			fh1_hi_r <= 24'd0;
			fh1_g_r <= 1'b0;
			fh2_ds_r <= 15'd0;
			fh2_rnd_r <= 1'b0;
			fh2_ovf_r <= 1'b0;
			fh_r <= 16'd0;
			out_p <= 128'd0;
		end else begin
			case (st)
			IDLE: if (i_valid) begin
				m_fmt <= fmt;
				m_psl <= psl;
				m_n <= npx;
				cpp_last_r <= cpp_last_in;
				n_last_r <= n_last_in;
				p <= 2'd0;
				j <= 3'd0;
				step <= 2'd0;
				fma_wait <= 5'd0;
				st <= RUN;
			end
			RUN: begin
				case (fma_wait)
				5'd0: begin
					ia_s_r <= ia_s;
					ic_s_r <= ic_s;
					ia_h_r <= ia_h;
					ic_h_r <= ic_h;
					ia_f_r <= ia_f;
					ic_f_r <= ic_f;
					ca_sl_rep <= {8{chk_a_n}};
					cc_sl_rep <= {8{chk_c_n}};
					fma_wait <= 5'd1;
				end
				5'd1: begin
					/* r_xa_128 / r_xc_128 capture 128b chunk this edge */
					fma_wait <= 5'd2;
				end
				5'd2: begin
					r_xa <= raw_xa;
					r_xc <= raw_xc;
					fma_wait <= 5'd3;
				end
				5'd3: begin
					/* r_srgb_*_rom and r_h*_0 capture this edge */
					fma_wait <= 5'd4;
				end
				5'd4: begin
					/* r_srgb_* <= r_srgb_*_rom and ha/hc ready on this edge */
					fma_wait <= 5'd5;
				end
				5'd5: begin
					/* r_dec_a / r_dec_c capture decoded fp32 this edge */
					fma_wait <= 5'd6;
				end
				5'd6: begin : b_load_fma
					reg [31:0] nxt_fa;
					nxt_fa = step[1] ? (step[0] ? sm : r01) : r_dec_a;
					r_fa <= nxt_fa;
					r_fa_nrm <= (nxt_fa[30:23] != 8'd0);
					r_fb <= step == 2'd3 ? QUARTER : ONE;
					r_fc <= step[1] ? (step[0] ? 32'h80000000 : r23) : r_dec_c;
					fma_wait <= 5'd7;
				end
				5'd7:  fma_wait <= 5'd8;
				5'd8:  fma_wait <= 5'd9;
				5'd9:  fma_wait <= 5'd10;
				5'd10: fma_wait <= 5'd11;
				5'd11: fma_wait <= 5'd12;
				5'd12: fma_wait <= 5'd13;
				5'd13: fma_wait <= 5'd14;
				5'd14: fma_wait <= 5'd15;
				5'd15: fma_wait <= 5'd16;
				5'd16: fma_wait <= 5'd17;
				5'd17: fma_wait <= 5'd18;
				5'd18: fma_wait <= 5'd19;
				5'd19: fma_wait <= 5'd20;
				5'd20: fma_wait <= 5'd21;
				5'd21: fma_wait <= 5'd22;
				5'd22: fma_wait <= 5'd23;
				5'd23: begin
					fma_wait <= 5'd0;
					if (step == 2'd0) begin
						r01 <= fr;
						step <= step_inc;
					end else if (step == 2'd1) begin
						r23 <= fr;
						step <= step_inc;
					end else if (step == 2'd2) begin
						sm <= fr;
						step <= step_inc;
					end else begin
						r_final <= fr;
						step <= 2'd0;
						if (m_fmt == 3'd3)
							st <= ENC_START;
						else if (m_fmt == 3'd4)
							st <= FH0;
						else
							st <= COMMIT;
					end
				end
				default: fma_wait <= 5'd0;
				endcase
			end
			FH0: begin
				fh0_sign_r  <= r_final[31];
				fh0_ef_r    <= (r_final[30:23] == 8'hFF);
				fh0_mnz_r   <= (r_final[22:0] != 23'd0);
				fh0_ge113_r <= (r_final[30:23] >= 8'd113);
				fh0_gt142_r <= (r_final[30:23] > 8'd142);
				fh0_le101_r <= (r_final[30:23] <= 8'd101);
				fh0_q_r     <= fh_q_w;
				fh0_sh_r    <= fh_sh_w;
				fh0_sh1_r   <= fh_sh1_w;
				st <= FH1;
			end
			FH1: begin
				fh1_ex_r <= fh_ex_w;
				fh1_ds_r <= fh_ds_w[14:0];
				fh1_hi_r <= fh_hi_w;
				fh1_g_r  <= fh_m[fh0_sh1_r];
				st <= FH2;
			end
			FH2: begin
				fh2_ds_r  <= fh1_ds_r;
				fh2_rnd_r <= fh2_rnd_w;
				fh2_ovf_r <= (fh1_ex_r >= 8'd31) || fh0_gt142_r;
				st <= FH3;
			end
			FH3: begin
				fh_r <= fh_mux_w;
				st <= COMMIT;
			end
			ENC_START: begin
				st <= ENC;
			end
			ENC: ;
			COMMIT: ;
			default: if (o_ready) begin
				st <= IDLE;
				out_p <= 128'd0;
			end
			endcase
			if (take) begin
				fma_wait <= 5'd0;
				if (m_fmt == 3'd3)
					out_p[8 * {p, j[1:0]} +: 8] <= e_code;
				else if (m_fmt == 3'd4)
					out_p[16 * h_idx +: 16] <= fh;
				else
					out_p[32 * w_idx +: 32] <= r_final;
				j <= lastj ? 3'd0 : j_inc;
				if (lastj)
					p <= p_inc;
				step <= 2'd0;
				if (lastj && lastp) begin
					st <= OUT;
					if (m_fmt == 3'd3)
						for (k = 0; k < 4; k = k + 1)
							if (k < m_n)
								out_p[8 * (4 * k + 3) +: 8] <= aq_r[8 * k +: 8];
				end else
					st <= RUN;
			end
		end
	end

	wire unused_ok = &{1'b0, e_ready, idx_co_unused, last_co_unused, out_co_unused, fh_co_unused, h2f_co_unused,
		fh_ds_w[23:15], ta_ps[0], tc_ps[0], m_n, ps_in[0]};
endmodule

`default_nettype wire
