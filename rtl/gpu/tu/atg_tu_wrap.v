/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_wrap: an integer texel coordinate into the level: {border texel,
 * index} by the address mode (0 repeat, 1 mirrored repeat, 2 clamp to
 * edge, 3 clamp to border).  The former functions tu_wrap and tu_mod of
 * include/atg_tu_fn.vh (the reference model tumodel wrap_idx); i mod p for i in
 * [−9, p + 7]: one correction for p >= 16, a 5-bit remainder (a table of
 * constants) for p < 16.  Combinational.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_wrap #(
	parameter LAT = 0
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire signed [23:0] i,
	input wire [15:0] n,
	input wire [1:0] mode,
	output wire [16:0] w
);
	wire lt = i[23];

	/* Parallel arithmetic operations directly on inputs i and n (no mode[0] mux before add/sub) */
	wire [16:0] mb0_add, mb0_sub, mb1_add, mb1_sub;
	wire [15:0] mir, n1;
	wire [23:0] in_sum, gen_d, ge2n_d;
	wire [8:0] carry_unused;

	wire [15:0] n_b, n_c;
	wire [3:0]  n_d;
	wire mode_a0_0, mode_a0_1;
	genvar gb;
	generate
`ifdef SYNTH_KINTEX7
	for (gb = 0; gb < 16; gb = gb + 1) begin : g_n_buf
		(* keep = "true", dont_touch = "true" *)
		LUT1 #(.INIT(2'b10)) u_nbuf (.I0(n[gb]), .O(n_b[gb]));
		(* keep = "true", dont_touch = "true" *)
		LUT1 #(.INIT(2'b10)) u_ncbuf (.I0(n[gb]), .O(n_c[gb]));
	end
	for (gb = 0; gb < 4; gb = gb + 1) begin : g_nd_buf
		(* keep = "true", dont_touch = "true" *)
		LUT1 #(.INIT(2'b10)) u_ndbuf (.I0(n[gb]), .O(n_d[gb]));
	end
	(* keep = "true", dont_touch = "true" *)
	LUT1 #(.INIT(2'b10)) u_mbuf0 (.I0(mode[0]), .O(mode_a0_0));
	(* keep = "true", dont_touch = "true" *)
	LUT1 #(.INIT(2'b10)) u_mbuf1 (.I0(mode[0]), .O(mode_a0_1));
`elsif SYNTH_KINTEX_ULTRASCALE
	for (gb = 0; gb < 16; gb = gb + 1) begin : g_n_buf
		assign n_b[gb] = n[gb];
		assign n_c[gb] = n[gb];
	end
	for (gb = 0; gb < 4; gb = gb + 1) begin : g_nd_buf
		assign n_d[gb] = n[gb];
	end
	assign mode_a0_0 = mode[0];
	assign mode_a0_1 = mode[0];
`else
	assign n_b = n;
	assign n_c = n;
	assign n_d = n[3:0];
	assign mode_a0_0 = mode[0];
	assign mode_a0_1 = mode[0];
`endif
	endgenerate
	wire [15:0] n_a = n;
	wire [1:0] mode_b = mode;

	/* lt_n: i + n < 0 */
	am_add #(.W(24)) u_lt_n (.a(i), .b({{8{1'b0}}, n_a}), .ci(1'b0), .s(in_sum), .co(carry_unused[3]));
	wire lt_n_s0 = in_sum[23];

	/* ge_n: i >= n */
	am_sub #(.W(24)) u_ge_n (.a(i), .b({8'd0, n_a}), .bi(1'b0), .d(gen_d), .bo(carry_unused[4]));
	wire ge_n_s0 = !gen_d[23];

	/* ge_2n: i >= 2n (used for mirrored repeat p = 2n) */
	am_sub #(.W(24)) u_ge_2n (.a(i), .b({7'd0, n_a, 1'b0}), .bi(1'b0), .d(ge2n_d), .bo(carry_unused[5]));
	wire ge_2n_s0 = !ge2n_d[23];

	/* u_mb0 / u_mb1: i +/- n and i +/- 2n directly on n (<= 10 cmos3, muxed after Stage 0) */
	am_add #(.W(17)) u_mb0_add (.a(i[16:0]), .b({1'b0, n_b}), .ci(1'b0), .s(mb0_add), .co(carry_unused[0]));
	am_sub #(.W(17)) u_mb0_sub (.a(i[16:0]), .b({1'b0, n_b}), .bi(1'b0), .d(mb0_sub), .bo(carry_unused[6]));
	am_add #(.W(17)) u_mb1_add (.a(i[16:0]), .b({n_b, 1'b0}), .ci(1'b0), .s(mb1_add), .co(carry_unused[7]));
	am_sub #(.W(17)) u_mb1_sub (.a(i[16:0]), .b({n_b, 1'b0}), .bi(1'b0), .d(mb1_sub), .bo(carry_unused[8]));

	/* u_mir: 2n - 1 - i directly in parallel */
	am_sub #(.W(16)) u_mir (.a({n_c[14:0], 1'b0}), .b(i[15:0]), .bi(1'b1), .d(mir), .bo(carry_unused[1]));

	/* u_n1: n - 1 */
	am_sub #(.W(16)) u_n1 (.a(n_c), .b(16'd0), .bi(1'b1), .d(n1), .bo(carry_unused[2]));

	/* Per-p 128-entry constant lookup tables decoupled from p[3:0] + 16:1 mux */
	wire [3:0] m_p_w [0:15];
	wire [6:0] lut_idx0 = {mode_a0_0, i[5:0]};
	wire [6:0] lut_idx1 = {mode_a0_1, i[5:0]};
	genvar gp;
	generate
	for (gp = 0; gp < 16; gp = gp + 1) begin : g_lut_p
		(* rom_style = "distributed" *) reg [3:0] lut_p [0:127];
		integer init_m, init_u, s_val, m_val, w_val, n_val;
		initial begin
			for (init_m = 0; init_m < 2; init_m = init_m + 1) begin
				for (init_u = 0; init_u < 64; init_u = init_u + 1) begin
					s_val = (init_u < 32) ? init_u : (init_u - 64);
					m_val = (gp > 0) ? (s_val % gp) : 0;
					if (m_val < 0)
						m_val = m_val + gp;
					if (init_m == 0) begin
						w_val = m_val;
					end else begin
						n_val = gp >> 1;
						w_val = (m_val < n_val) ? m_val : (gp - 1 - m_val);
					end
					lut_p[{init_m[0], init_u[5:0]}] = w_val[3:0];
				end
			end
		end
		assign m_p_w[gp] = lut_p[(gp < 8) ? lut_idx0 : lut_idx1];
	end
	endgenerate
	wire [3:0] m_small_s0 = m_p_w[mode_b[0] ? {n_d[2:0], 1'b0} : n_d[3:0]];
	wire n_ge16_s0 = (n_c[15:4] != 12'd0);
	wire n_ge8_s0  = (n_c[15:3] != 13'd0);

	/* Stage 0 -> Stage 1 Pipeline Registers (when LAT >= 1) */
	(* keep = "true", equivalent_register_removal = "no", max_fanout = 16, extract_reset = "no" *) reg        r_lt_lo, r_lt_hi, r_lt_n_lo, r_lt_n_hi, r_ge_n_lo, r_ge_n_hi, r_ge_2n_lo, r_ge_2n_hi, r_n_ge16_lo, r_n_ge16_hi, r_n_ge8_lo, r_n_ge8_hi;
	(* max_fanout = 16, extract_reset = "no" *) reg [15:0] r_mb0_add, r_mb0_sub, r_mb1_add, r_mb1_sub, r_mir, r_n1, r_i;
	(* extract_reset = "no", rom_style = "distributed" *) reg [3:0]  r_m_small;
	(* keep = "true", equivalent_register_removal = "no", max_fanout = 16, extract_reset = "no" *) reg [1:0]  r_mode_lo, r_mode_hi;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_lt_lo     <= 1'b0;     r_lt_hi     <= 1'b0;
			r_lt_n_lo   <= 1'b0;     r_lt_n_hi   <= 1'b0;
			r_ge_n_lo   <= 1'b0;     r_ge_n_hi   <= 1'b0;
			r_ge_2n_lo  <= 1'b0;     r_ge_2n_hi  <= 1'b0;
			r_n_ge16_lo <= 1'b0;     r_n_ge16_hi <= 1'b0;
			r_n_ge8_lo  <= 1'b0;     r_n_ge8_hi  <= 1'b0;
			r_mb0_add   <= 16'd0;
			r_mb0_sub   <= 16'd0;
			r_mb1_add   <= 16'd0;
			r_mb1_sub   <= 16'd0;
			r_mir       <= 16'd0;
			r_n1        <= 16'd0;
			r_i         <= 16'd0;
			r_m_small   <= 4'd0;
			r_mode_lo   <= 2'd0;     r_mode_hi   <= 2'd0;
		end else if (en) begin
			r_lt_lo     <= lt;       r_lt_hi     <= lt;
			r_lt_n_lo   <= lt_n_s0;  r_lt_n_hi   <= lt_n_s0;
			r_ge_n_lo   <= ge_n_s0;  r_ge_n_hi   <= ge_n_s0;
			r_ge_2n_lo  <= ge_2n_s0; r_ge_2n_hi  <= ge_2n_s0;
			r_n_ge16_lo <= n_ge16_s0; r_n_ge16_hi <= n_ge16_s0;
			r_n_ge8_lo  <= n_ge8_s0;  r_n_ge8_hi  <= n_ge8_s0;
			r_mb0_add   <= mb0_add[15:0];
			r_mb0_sub   <= mb0_sub[15:0];
			r_mb1_add   <= mb1_add[15:0];
			r_mb1_sub   <= mb1_sub[15:0];
			r_mir       <= mir;
			r_n1        <= n1;
			r_i         <= i[15:0];
			r_m_small   <= m_small_s0;
			r_mode_lo   <= mode_b;     r_mode_hi   <= mode_b;
		end
	end

	wire        s_lt_lo     = (LAT >= 1) ? r_lt_lo     : lt;
	wire        s_lt_hi     = (LAT >= 1) ? r_lt_hi     : lt;
	wire        s_lt_n_lo   = (LAT >= 1) ? r_lt_n_lo   : lt_n_s0;
	wire        s_lt_n_hi   = (LAT >= 1) ? r_lt_n_hi   : lt_n_s0;
	wire        s_ge_n_lo   = (LAT >= 1) ? r_ge_n_lo   : ge_n_s0;
	wire        s_ge_n_hi   = (LAT >= 1) ? r_ge_n_hi   : ge_n_s0;
	wire        s_ge_2n_lo  = (LAT >= 1) ? r_ge_2n_lo  : ge_2n_s0;
	wire        s_ge_2n_hi  = (LAT >= 1) ? r_ge_2n_hi  : ge_2n_s0;
	wire        s_p_ge16_lo = (LAT >= 1) ? (r_mode_lo[0] ? r_n_ge8_lo : r_n_ge16_lo) : (mode[0] ? n_ge8_s0 : n_ge16_s0);
	wire        s_p_ge16_hi = (LAT >= 1) ? (r_mode_hi[0] ? r_n_ge8_hi : r_n_ge16_hi) : (mode[0] ? n_ge8_s0 : n_ge16_s0);
	wire [15:0] s_mb0_add   = (LAT >= 1) ? r_mb0_add   : mb0_add[15:0];
	wire [15:0] s_mb0_sub   = (LAT >= 1) ? r_mb0_sub   : mb0_sub[15:0];
	wire [15:0] s_mb1_add   = (LAT >= 1) ? r_mb1_add   : mb1_add[15:0];
	wire [15:0] s_mb1_sub   = (LAT >= 1) ? r_mb1_sub   : mb1_sub[15:0];
	wire [15:0] s_mb0       = {s_lt_hi ? s_mb0_add[15:8] : s_mb0_sub[15:8], s_lt_lo ? s_mb0_add[7:0] : s_mb0_sub[7:0]};
	wire [15:0] s_mb1       = {s_lt_hi ? s_mb1_add[15:8] : s_mb1_sub[15:8], s_lt_lo ? s_mb1_add[7:0] : s_mb1_sub[7:0]};
	wire [15:0] s_mir       = (LAT >= 1) ? r_mir       : mir;
	wire [15:0] s_n1        = (LAT >= 1) ? r_n1        : n1;
	wire [15:0] s_i         = (LAT >= 1) ? r_i         : i[15:0];
	wire [3:0]  s_m_small   = (LAT >= 1) ? r_m_small   : m_small_s0;
	wire [1:0]  s_mode_lo   = (LAT >= 1) ? r_mode_lo   : mode;
	wire [1:0]  s_mode_hi   = (LAT >= 1) ? r_mode_hi   : mode;

	wire [15:0] rep_res = {
		s_p_ge16_hi ? ((s_lt_hi || s_ge_n_hi)  ? s_mb0[15:8] : s_i[15:8]) : 8'd0,
		s_p_ge16_lo ? ((s_lt_lo || s_ge_n_lo)  ? s_mb0[7:0]  : s_i[7:0])  : {4'd0, s_m_small}
	};
	wire [15:0] mir_res = {
		s_p_ge16_hi ? ((s_lt_n_hi || s_ge_2n_hi) ? s_mb1[15:8] : s_lt_hi ? ~s_i[15:8] : s_ge_n_hi ? s_mir[15:8] : s_i[15:8]) : 8'd0,
		s_p_ge16_lo ? ((s_lt_n_lo || s_ge_2n_lo) ? s_mb1[7:0]  : s_lt_lo ? ~s_i[7:0]  : s_ge_n_lo ? s_mir[7:0]  : s_i[7:0])  : {4'd0, s_m_small}
	};

	wire [16:0] w_out;
	assign w_out[7:0]  = s_mode_lo == 2'd0 ? rep_res[7:0]  : s_mode_lo == 2'd1 ? mir_res[7:0]  : (s_lt_lo ? 8'd0 : s_ge_n_lo ? s_n1[7:0]  : s_i[7:0]);
	assign w_out[15:8] = s_mode_hi == 2'd0 ? rep_res[15:8] : s_mode_hi == 2'd1 ? mir_res[15:8] : (s_lt_hi ? 8'd0 : s_ge_n_hi ? s_n1[15:8] : s_i[15:8]);
	assign w_out[16]   = (s_mode_hi == 2'd0 || s_mode_hi == 2'd1) ? 1'b0 : (s_mode_hi[0] && (s_lt_hi || s_ge_n_hi));
	assign w = w_out;

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, en} : 1'b0;
	wire unused_ok = &{1'b0, unused_lat0, mb0_add[16], mb0_sub[16], mb1_add[16], mb1_sub[16], gen_d[22:0], ge2n_d[22:0], in_sum[22:0]};
endmodule

`default_nettype wire
