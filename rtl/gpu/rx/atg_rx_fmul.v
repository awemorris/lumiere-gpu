/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_fmul: fp32 multiplier with configurable latency (LAT = 0 or 4) for RX.
 * RNE, denormals flushed, canonical NaN 0x7FC00000.
 * Compliant with ASIC 5nm 1.5 GHz timing, primitive budget (<= 1 Post-DSP LUT,
 * <= 1 Post-CARRY4 LUT, <= 3 random LUT levels), and fanout (<= 16) limits.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rx_fmul #(
	parameter LAT = 4
) (
	input wire clk,
	input wire rst_n,
	input wire i_valid,
	input wire [31:0] i_a,
	input wire [31:0] i_b,
	output wire o_valid,
	output wire [31:0] o_r
);
	/* Stage 1: unpack, special cases, and ea + eb (0 Pre-LUTs, 0 Post-LUTs). */
	wire [7:0] ea = i_a[30:23];
	wire [7:0] eb = i_b[30:23];
	wire za = ea == 8'd0;
	wire zb = eb == 8'd0;
	wire nan_a = ea == 8'hFF && i_a[22:0] != 23'd0;
	wire nan_b = eb == 8'hFF && i_b[22:0] != 23'd0;
	wire inf_a = ea == 8'hFF && i_a[22:0] == 23'd0;
	wire inf_b = eb == 8'hFF && i_b[22:0] == 23'd0;
	wire s = i_a[31] ^ i_b[31];

	wire [9:0] eab;
	wire eab_co_unused;
	am_add #(.W(10)) u_eab (.a({2'b00, ea}), .b({2'b00, eb}), .ci(1'b0), .s(eab), .co(eab_co_unused));

	wire v1_w = i_valid;
	wire sp_nan1_w = nan_a || nan_b || ((inf_a || inf_b) && (za || zb));
	wire sp_inf1_w = !sp_nan1_w && (inf_a || inf_b);
	wire sp_zero1_w = !sp_nan1_w && !sp_inf1_w && (za || zb);
	wire sp1_w = sp_nan1_w || sp_inf1_w || sp_zero1_w;
	wire s1_w = sp_nan1_w ? 1'b0 : s;
	wire [9:0] eab1_w = eab;
	wire [23:0] ma1_w = {1'b1, i_a[22:0]};
	wire [23:0] mb1_w = {1'b1, i_b[22:0]};

	wire v1, sp1, sp_nan1, sp_inf1, s1;
	wire [9:0] eab1;
	wire [23:0] ma1, mb1;

	/* Stage 2: precompute eab - 127, eab - 126, eab - 125 (0 Post-LUTs after CARRY4). */
	wire [9:0] e2_0_w, e2_1_w, e2_2_w;
	wire e2_0_bo_unused, e2_1_bo_unused, e2_2_bo_unused;
	am_sub #(.W(10)) u_e2_0 (.a(eab1), .b(10'd127), .bi(1'b0), .d(e2_0_w), .bo(e2_0_bo_unused));
	am_sub #(.W(10)) u_e2_1 (.a(eab1), .b(10'd126), .bi(1'b0), .d(e2_1_w), .bo(e2_1_bo_unused));
	am_sub #(.W(10)) u_e2_2 (.a(eab1), .b(10'd125), .bi(1'b0), .d(e2_2_w), .bo(e2_2_bo_unused));

	wire v2, sp2, sp_nan2, sp_inf2, s2;
	wire [9:0] e2_0, e2_1, e2_2;
	wire [47:0] p2;

	/* Stage 3:
	 * - From DSP output p2: <= 1 Post-DSP LUT, p2[47] fanout = 11 (<= 16 soft limit).
	 * - From registered e2_0/1/2: pre-decode final exponents ef_0/1/2 and mantissa-zero flags.
	 */
	wire [5:0] m3_lo_w = p2[47] ? p2[29:24] : p2[28:23];
	wire g3_w = p2[47] ? p2[23] : p2[22];
	wire [3:0] st_c_w;
	assign st_c_w[0] = |p2[5:0];
	assign st_c_w[1] = |p2[11:6];
	assign st_c_w[2] = |p2[17:12];
	assign st_c_w[3] = p2[47] ? (|p2[22:18]) : (|p2[21:18]);

	wire ga6_0_w  = &p2[34:29];
	wire ga6_1_w  = &p2[35:30];
	wire ga12_0_w = &p2[40:35];
	wire ga12_1_w = &p2[41:36];
	wire ga18_0_w = &p2[45:41];
	wire ga18_1_w = &p2[46:42];

	wire ovf3_0 = !e2_0[9] && (e2_0[8] || (&e2_0[7:0]));
	wire udf3_0 = e2_0[9] || (e2_0[8:0] == 9'd0);
	wire ovf3_1 = !e2_1[9] && (e2_1[8] || (&e2_1[7:0]));
	wire udf3_1 = e2_1[9] || (e2_1[8:0] == 9'd0);
	wire ovf3_2 = !e2_2[9] && (e2_2[8] || (&e2_2[7:0]));
	wire udf3_2 = e2_2[9] || (e2_2[8:0] == 9'd0);

	wire [7:0] ef3_0_w = (sp_nan2 || sp_inf2 || (!sp2 && ovf3_0)) ? 8'hFF :
		(sp2 || udf3_0) ? 8'h00 : e2_0[7:0];
	wire [7:0] ef3_1_w = (sp_nan2 || sp_inf2 || (!sp2 && ovf3_1)) ? 8'hFF :
		(sp2 || udf3_1) ? 8'h00 : e2_1[7:0];
	wire [7:0] ef3_2_w = (sp_nan2 || sp_inf2 || (!sp2 && ovf3_2)) ? 8'hFF :
		(sp2 || udf3_2) ? 8'h00 : e2_2[7:0];

	wire mz3_0_w = sp2 || ovf3_0 || udf3_0;
	wire mz3_1_w = sp2 || ovf3_1 || udf3_1;

	wire v3, sp_nan3, s3, g3;
	wire p47_a, p47_b, p47_c;
	wire [5:0] m3_lo;
	wire [46:29] p3_hi;
	wire [3:0] st_c3;
	wire ga6_0, ga6_1, ga12_0, ga12_1, ga18_0, ga18_1;
	wire [7:0] ef3_0, ef3_1, ef3_2;
	wire mz3_0, mz3_1;

	/* Stage 4: Rounding and packing using am_add */
`ifdef SYNTH_ASIC
	wire [22:0] m_rounded;
	wire m0_cout;
	wire g3_asic, st3_asic, r_p3_47_w, r_nz22_w, r_nz23_w;
	wire [46:22] r_p3_hi_w;
	(* keep = "true" *) wire mz_a = (LAT == 0) ? (p47_a ? mz3_1 : mz3_0) : (r_p3_47_w ? mz3_1 : mz3_0);
	(* keep = "true" *) wire mz_b = (LAT == 0) ? (p47_b ? mz3_1 : mz3_0) : (r_p3_47_w ? mz3_1 : mz3_0);
	generate
	if (LAT == 0) begin : g_rnd_lat0
		(* keep = "true" *) wire [22:6] mu_hi;
		genvar g0i;
		for (g0i = 6; g0i < 15; g0i = g0i + 1) begin : g_mu_6_14
			assign mu_hi[g0i] = p47_a ? p3_hi[g0i + 24] : p3_hi[g0i + 23];
		end
		for (g0i = 15; g0i < 23; g0i = g0i + 1) begin : g_mu_15_22
			assign mu_hi[g0i] = p47_b ? p3_hi[g0i + 24] : p3_hi[g0i + 23];
		end
		(* keep = "true" *) wire round_inc0 = g3 && (m3_lo[0] || (|st_c3));
		am_add #(.W(23)) u_add_round (.a({mu_hi, m3_lo}), .b(23'd0), .ci(round_inc0), .s(m_rounded), .co(m0_cout));
	end else begin : g_rnd_asic
		/* Parallel carry-free prefix rounding with p47 & !mz3 folded into toggle/base XOR (<= 10 cmos3 total) */
		wire [22:0] m_p1 = r_p3_hi_w[46:24];
		wire [22:0] m_p0 = r_p3_hi_w[45:23];
		wire inc1 = r_p3_hi_w[23] && (r_p3_hi_w[24] || r_nz23_w);
		wire inc0 = r_p3_hi_w[22] && (r_p3_hi_w[23] || r_nz22_w);
		wire inc1_sel = r_p3_47_w && !mz3_1 && inc1;
		wire inc0_sel = !r_p3_47_w && !mz3_0 && inc0;

		wire g7_1  = &m_p1[7:0];
		wire g15_1 = &m_p1[15:8];
		wire g7_0  = &m_p0[7:0];
		wire g15_0 = &m_p0[15:8];

		wire g01_1 = g7_1 & g15_1;
		wire g01_0 = g7_0 & g15_0;

		wire [22:0] r_p1, r_p0;
		assign r_p1 = 23'd0;
		assign r_p0 = 23'd0;
		genvar gbi;
		for (gbi = 0; gbi < 23; gbi = gbi + 1) begin : g_pfx_bits
			localparam integer BYTE = gbi / 8;
			localparam integer BOFF = gbi % 8;
			wire grp_all1 = (BYTE == 0) ? 1'b1 : (BYTE == 1) ? g7_1 : g01_1;
			wire grp_all0 = (BYTE == 0) ? 1'b1 : (BYTE == 1) ? g7_0 : g01_0;
			wire loc1, loc0;
			if (BOFF == 0) begin : g_loc_z
				assign loc1 = 1'b1;
				assign loc0 = 1'b1;
			end else begin : g_loc_nz
				assign loc1 = (&m_p1[8*BYTE +: BOFF]);
				assign loc0 = (&m_p0[8*BYTE +: BOFF]);
			end
			wire all_b1   = grp_all1 & loc1;
			wire all_b0   = grp_all0 & loc0;
			wire tog_bit  = (inc1_sel & all_b1) | (inc0_sel & all_b0);
			wire base_bit = (r_p3_47_w & !mz3_1 & m_p1[gbi]) | (!r_p3_47_w & !mz3_0 & m_p0[gbi]);
			assign m_rounded[gbi] = base_bit ^ tog_bit;
		end
		wire g22_1 = &m_p1[22:16];
		wire g22_0 = &m_p0[22:16];
		wire co1 = (g01_1 & g22_1) & inc1;
		wire co0 = (g01_0 & g22_0) & inc0;
		assign m0_cout = r_p3_47_w ? co1 : co0;
	end
	endgenerate
	assign g3_asic = 1'b0;
	assign st3_asic = 1'b0;
`else
	(* keep = "true" *) wire round_inc = g3 && (m3_lo[0] || (|st_c3));
	(* keep = "true" *) wire mz_a    = p47_a ? mz3_1 : mz3_0;
	(* keep = "true" *) wire mz_b    = p47_b ? mz3_1 : mz3_0;

	(* keep = "true" *) wire [22:6] mu_hi;
	genvar gi;
	generate
	for (gi = 6; gi < 15; gi = gi + 1) begin : g_mu_6_14
		assign mu_hi[gi] = p47_a ? p3_hi[gi + 24] : p3_hi[gi + 23];
	end
	for (gi = 15; gi < 23; gi = gi + 1) begin : g_mu_15_22
		assign mu_hi[gi] = p47_b ? p3_hi[gi + 24] : p3_hi[gi + 23];
	end
	endgenerate

	wire [22:0] m_rounded;
	wire m0_cout;
	wire [22:0] m_add_in = {mu_hi, m3_lo};
	am_add #(.W(23)) u_add_round (
		.a(m_add_in),
		.b(23'd0),
		.ci(round_inc),
		.s(m_rounded),
		.co(m0_cout)
	);
`endif

	wire [22:0] m_out;
	genvar gi_o;
	generate
	if (LAT == 0) begin : g_mout_lat0
		for (gi_o = 0; gi_o < 12; gi_o = gi_o + 1) begin : g_mout_lo
			assign m_out[gi_o] = !mz_a && m_rounded[gi_o];
		end
		for (gi_o = 12; gi_o < 22; gi_o = gi_o + 1) begin : g_mout_hi
			assign m_out[gi_o] = !mz_b && m_rounded[gi_o];
		end
		assign m_out[22] = sp_nan3 || (!mz_b && m_rounded[22]);
	end else begin : g_mout_lat4
`ifdef SYNTH_ASIC
		assign m_out[21:0] = m_rounded[21:0];
		assign m_out[22] = sp_nan3 || m_rounded[22];
`else
		for (gi_o = 0; gi_o < 12; gi_o = gi_o + 1) begin : g_mout_lo
			assign m_out[gi_o] = !mz_a && m_rounded[gi_o];
		end
		for (gi_o = 12; gi_o < 22; gi_o = gi_o + 1) begin : g_mout_hi
			assign m_out[gi_o] = !mz_b && m_rounded[gi_o];
		end
		assign m_out[22] = sp_nan3 || (!mz_b && m_rounded[22]);
`endif
	end
	endgenerate

	wire [7:0] e_out;
	generate
	for (gi_o = 0; gi_o < 8; gi_o = gi_o + 1) begin : g_eout
`ifdef SYNTH_ASIC
		wire e_mux0 = p47_c ? ef3_1[gi_o] : ef3_0[gi_o];
		wire e_mux1 = p47_c ? ef3_2[gi_o] : ef3_1[gi_o];
		assign e_out[gi_o] = m0_cout ? e_mux1 : e_mux0;
`else
		assign e_out[gi_o] = (p47_c && m0_cout) ? ef3_2[gi_o] :
		                     (p47_c ^ m0_cout)  ? ef3_1[gi_o] : ef3_0[gi_o];
`endif
	end
	endgenerate

	wire unused_m = &{1'b0, eab_co_unused, e2_0_bo_unused, e2_1_bo_unused, e2_2_bo_unused,
		ga6_0, ga6_1, ga12_0, ga12_1, ga18_0, ga18_1};

	wire o_valid_comb = v3;
	wire [31:0] o_r_comb = {s3, e_out, m_out};

	generate
		if (LAT == 0) begin : g_lat0
			wire [47:0] p1;
			am_mul #(.AW(24), .BW(24), .LAT(0)) u_p1 (.clk(clk), .en(1'b1), .a(ma1), .b(mb1), .p(p1));

			assign v1 = i_valid;
			assign sp1 = sp1_w;
			assign sp_nan1 = sp_nan1_w;
			assign sp_inf1 = sp_inf1_w;
			assign s1 = s1_w;
			assign eab1 = eab1_w;
			assign ma1 = ma1_w;
			assign mb1 = mb1_w;

			assign v2 = v1;
			assign sp2 = sp1;
			assign sp_nan2 = sp_nan1;
			assign sp_inf2 = sp_inf1;
			assign s2 = s1;
			assign e2_0 = e2_0_w;
			assign e2_1 = e2_1_w;
			assign e2_2 = e2_2_w;
			assign p2 = p1;

			assign v3 = v2;
			assign sp_nan3 = sp_nan2;
			assign s3 = s2;
			assign g3 = g3_w;
			assign p47_a = p2[47];
			assign p47_b = p2[47];
			assign p47_c = p2[47];
			assign m3_lo = m3_lo_w;
			assign p3_hi = p2[46:29];
			assign st_c3 = st_c_w;
			assign ga6_0 = ga6_0_w;
			assign ga6_1 = ga6_1_w;
			assign ga12_0 = ga12_0_w;
			assign ga12_1 = ga12_1_w;
			assign ga18_0 = ga18_0_w;
			assign ga18_1 = ga18_1_w;
			assign ef3_0 = ef3_0_w;
			assign ef3_1 = ef3_1_w;
			assign ef3_2 = ef3_2_w;
			assign mz3_0 = mz3_0_w;
			assign mz3_1 = mz3_1_w;

			assign o_valid = o_valid_comb;
			assign o_r = o_r_comb;

			wire unused_clk = &{1'b0, clk, v1_w, unused_m};
		end else begin : g_lat4
			`ifdef SYNTH_ASIC
			/* Radix-4 Booth 24x24 multiplier: direct zero-XOR Booth decode + dual-rail CSA in Stage 1 & Stage 2 */
			wire [12:0] b_neg;
			wire [47:0] booth_pp [0:12];
			wire [26:0] b_pad = {2'b00, mb1_w, 1'b0};
			genvar gj;
			for (gj = 0; gj < 12; gj = gj + 1) begin : g_booth_row
				wire b_m1 = b_pad[2 * gj];
				wire b_0  = b_pad[2 * gj + 1];
				wire b_p1 = b_pad[2 * gj + 2];
				wire pos1 = ~(~(!b_p1 && !b_0 &&  b_m1) & ~(!b_p1 &&  b_0 && !b_m1));
				wire neg1 = ~(~( b_p1 && !b_0 &&  b_m1) & ~( b_p1 &&  b_0 && !b_m1));
				wire pos2 = !b_p1 &&  b_0 &&  b_m1;
				wire neg2 =  b_p1 && !b_0 && !b_m1;
				assign b_neg[gj] = b_p1 && !(b_0 && b_m1);
				wire [24:0] u_j;
				wire [25:0] ma1_pad = {1'b0, ma1_w, 1'b0};
				genvar gb_k;
				for (gb_k = 0; gb_k < 25; gb_k = gb_k + 1) begin : g_bu
					wire m_cur  = ma1_pad[gb_k + 1];
					wire m_prev = ma1_pad[gb_k];
					assign u_j[gb_k] = (pos1 & m_cur) | (neg1 & ~m_cur) | (pos2 & m_prev) | (neg2 & ~m_prev);
				end
				wire s_j = b_p1 && (ma1_w[23] || !b_0 || !b_m1);
				wire [47:0] row_base;
				if (gj == 0) begin : g_b0
					assign row_base = {20'd0, ~s_j, s_j, s_j, u_j};
				end else if (gj < 11) begin : g_bm
					wire [20 - 2*gj : 0] pad_hi = {(21 - 2*gj){1'b0}};
					assign row_base = {pad_hi, 1'b1, ~s_j, u_j, {(2*gj){1'b0}}};
				end else begin : g_b11
					assign row_base = {~s_j, u_j, 22'd0};
				end
				if (gj == 0) begin : g_n0
					assign booth_pp[0] = row_base;
				end else begin : g_nj
					wire [47:0] neg_ins = {{(48 - (2*gj - 1)){1'b0}}, b_neg[gj - 1], {(2*gj - 2){1'b0}}};
					assign booth_pp[gj] = row_base | neg_ins;
				end
			end
			assign b_neg[12] = 1'b0;
			assign booth_pp[12] = {ma1_w, 1'b0, b_neg[11], 22'd0};

			/* Stage 1 CSA: 13 -> 9 -> 6 rows with dual-rail 3:2 CSA & early booth_pp[12] pairing */
			wire [47:0] s1_m [0:8];
			wire [47:0] s1_m_n [0:8];
			for (gj = 0; gj < 4; gj = gj + 1) begin : g_s1_32a
				wire [47:0] x0 = (gj == 0) ? booth_pp[0] : booth_pp[3*gj];
				wire [47:0] x1 = (gj == 0) ? booth_pp[12] : booth_pp[3*gj + 1];
				wire [47:0] x2 = (gj == 0) ? booth_pp[1] : booth_pp[3*gj + 2];
				wire [47:0] nx0 = ~x0, nx1 = ~x1, nx2 = ~x2;
				wire [47:0] x01  = ~(~(x0 & nx1) & ~(nx0 & x1));
				wire [47:0] nx01 = ~(~(x0 &  x1) & ~(nx0 & nx1));
				assign s1_m[2*gj]     = ~(~(x01 & nx2) & ~(nx01 & x2));
				assign s1_m_n[2*gj]   = ~(~(x01 &  x2) & ~(nx01 & nx2));
				wire [46:0] c_raw = (x0[46:0] & x1[46:0]) | (x2[46:0] & (x0[46:0] | x1[46:0]));
				assign s1_m[2*gj + 1]   = {c_raw, 1'b0};
				assign s1_m_n[2*gj + 1] = ~{c_raw, 1'b0};
			end
			assign s1_m[8]   = booth_pp[2];
			assign s1_m_n[8] = ~booth_pp[2];

			wire [47:0] s1_l1 [0:5];
			wire [47:0] y_a [0:2];
			wire [47:0] y_b [0:2];
			wire [47:0] y_c [0:2];
			wire [47:0] ny_a [0:2];
			wire [47:0] ny_b [0:2];
			wire [47:0] ny_c [0:2];
			/* Balance early carries (1,3,5,7) + booth_pp[2] (8) against late sums (0,2,4,6) */
			assign y_a[0] = s1_m[1]; assign ny_a[0] = s1_m_n[1];
			assign y_b[0] = s1_m[8]; assign ny_b[0] = s1_m_n[8];
			assign y_c[0] = s1_m[0]; assign ny_c[0] = s1_m_n[0];

			assign y_a[1] = s1_m[3]; assign ny_a[1] = s1_m_n[3];
			assign y_b[1] = s1_m[5]; assign ny_b[1] = s1_m_n[5];
			assign y_c[1] = s1_m[2]; assign ny_c[1] = s1_m_n[2];

			assign y_a[2] = s1_m[7]; assign ny_a[2] = s1_m_n[7];
			assign y_b[2] = s1_m[4]; assign ny_b[2] = s1_m_n[4];
			assign y_c[2] = s1_m[6]; assign ny_c[2] = s1_m_n[6];
			for (gj = 0; gj < 3; gj = gj + 1) begin : g_s1_32b
				wire [47:0] y01  = ~(~(y_a[gj] & ny_b[gj]) & ~(ny_a[gj] & y_b[gj]));
				wire [47:0] ny01 = ~(~(y_a[gj] &  y_b[gj]) & ~(ny_a[gj] & ny_b[gj]));
				assign s1_l1[2*gj]     = ~(~(y01 & ny_c[gj]) & ~(ny01 & y_c[gj]));
				assign s1_l1[2*gj + 1] = {((y_a[gj][46:0] & y_b[gj][46:0]) | (y_c[gj][46:0] & (y_a[gj][46:0] | y_b[gj][46:0]))), 1'b0};
			end

			reg [47:0] r_csa1 [0:5];
			integer ic1;
			always @(posedge clk) begin
				for (ic1 = 0; ic1 < 6; ic1 = ic1 + 1)
					r_csa1[ic1] <= s1_l1[ic1];
			end

			/* Stage 2 CSA: dual-rail 6 -> 4 (2x 3:2) -> 2 (1x 4:2) (<= 11 cmos3) */
			wire [47:0] nr0 = ~r_csa1[0], nr1 = ~r_csa1[1], nr2 = ~r_csa1[2];
			wire [47:0] nr3 = ~r_csa1[3], nr4 = ~r_csa1[4], nr5 = ~r_csa1[5];
			wire [47:0] x2a  = ~(~(r_csa1[0] & nr1) & ~(nr0 & r_csa1[1]));
			wire [47:0] nx2a = ~(~(r_csa1[0] & r_csa1[1]) & ~(nr0 & nr1));
			wire [47:0] s2_l1_0  = ~(~(x2a & nr2) & ~(nx2a & r_csa1[2]));
			wire [47:0] ns2_l1_0 = ~(~(x2a & r_csa1[2]) & ~(nx2a & nr2));
			wire [47:0] s2_l1_1  = {((r_csa1[0][46:0] & r_csa1[1][46:0]) | (r_csa1[2][46:0] & (r_csa1[0][46:0] | r_csa1[1][46:0]))), 1'b0};
			wire [47:0] ns2_l1_1 = ~s2_l1_1;

			wire [47:0] x2b  = ~(~(r_csa1[3] & nr4) & ~(nr3 & r_csa1[4]));
			wire [47:0] nx2b = ~(~(r_csa1[3] & r_csa1[4]) & ~(nr3 & nr4));
			wire [47:0] s2_l1_2  = ~(~(x2b & nr5) & ~(nx2b & r_csa1[5]));
			wire [47:0] ns2_l1_2 = ~(~(x2b & r_csa1[5]) & ~(nx2b & nr5));
			wire [47:0] s2_l1_3  = {((r_csa1[3][46:0] & r_csa1[4][46:0]) | (r_csa1[5][46:0] & (r_csa1[3][46:0] | r_csa1[4][46:0]))), 1'b0};
			wire [47:0] ns2_l1_3 = ~s2_l1_3;

			/* 4:2 on inputs (s2_l1_0 [late], s2_l1_2 [late], s2_l1_1 [early], s2_l1_3 [early]) */
			wire [47:0] p42  = ~(~(s2_l1_0 & ns2_l1_2) & ~(ns2_l1_0 & s2_l1_2));
			wire [47:0] np42 = ~(~(s2_l1_0 & s2_l1_2) & ~(ns2_l1_0 & ns2_l1_2));
			wire [47:0] q42  = ~(~(s2_l1_1 & ns2_l1_3) & ~(ns2_l1_1 & s2_l1_3));
			wire [47:0] nq42 = ~(~(s2_l1_1 & s2_l1_3) & ~(ns2_l1_1 & ns2_l1_3));

			wire [47:0] s2_co1  = {((p42[46:0] & s2_l1_1[46:0]) | (np42[46:0] & s2_l1_0[46:0])), 1'b0};
			wire [47:0] ns2_co1 = {~((p42[46:0] & s2_l1_1[46:0]) | (np42[46:0] & s2_l1_0[46:0])), 1'b1};
			wire [47:0] pq42    = ~(~(p42 & nq42) & ~(np42 & q42));
			wire [47:0] npq42   = ~(~(p42 & q42) & ~(np42 & nq42));
			wire [47:0] s2_l3_s = ~(~(pq42 & ns2_co1) & ~(npq42 & s2_co1));
			wire [47:0] s2_l3_c = {((pq42[46:0] & s2_co1[46:0]) | (npq42[46:0] & s2_l1_3[46:0])), 1'b0};

			reg [47:0] r_st2_s, r_st2_c;
			always @(posedge clk) begin
				r_st2_s <= s2_l3_s;
				r_st2_c <= s2_l3_c;
			end

			/* Stage 3: 48-bit Kogge-Stone am_add + parallel carry-free sticky detection */
			wire [47:0] p_sum_w;
			wire p_sum_co_unused;
			am_add #(.W(48)) u_fin_p (.a(r_st2_s), .b(r_st2_c), .ci(1'b0), .s(p_sum_w), .co(p_sum_co_unused));

			wire [22:0] nz_bit;
			assign nz_bit[0] = r_st2_s[0] ^ r_st2_c[0];
			for (gj = 1; gj < 23; gj = gj + 1) begin : g_nz
				assign nz_bit[gj] = (r_st2_s[gj] ^ r_st2_c[gj]) ^ (r_st2_s[gj - 1] | r_st2_c[gj - 1]);
			end
			wire nz_22_w = |nz_bit[21:0];
			wire nz_23_w = nz_22_w || nz_bit[22];

			reg r_p3_47, r_nz22, r_nz23;
			reg [46:22] r_p3_hi_r;
			always @(posedge clk) begin
				r_p3_47   <= p_sum_w[47];
				r_p3_hi_r <= p_sum_w[46:22];
				r_nz22    <= nz_22_w;
				r_nz23    <= nz_23_w;
			end
			assign r_p3_47_w = r_p3_47;
			assign r_p3_hi_w = r_p3_hi_r;
			assign r_nz22_w  = r_nz22;
			assign r_nz23_w  = r_nz23;
			assign p2 = 48'd0;
			assign g3 = 1'b0;
			assign p47_a = 1'b0;
			assign p47_b = 1'b0;
			assign p47_c = r_p3_47;
			assign m3_lo = 6'd0;
			assign p3_hi = 18'd0;
			assign st_c3 = 4'd0;
			assign ga6_0 = 1'b0; assign ga6_1 = 1'b0;
			assign ga12_0 = 1'b0; assign ga12_1 = 1'b0;
			assign ga18_0 = 1'b0; assign ga18_1 = 1'b0;
`else
			wire [47:0] p1;
			am_mul #(.AW(24), .BW(24), .LAT(2)) u_p1 (.clk(clk), .en(1'b1), .a(ma1_w), .b(mb1_w), .p(p1));
`endif

			reg r_v1, r_sp1, r_sp_nan1, r_sp_inf1, r_s1;
			reg [9:0] r_eab1;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v1 <= 1'b0;
				else
					r_v1 <= i_valid;
			end

			always @(posedge clk) begin
				r_sp1 <= sp1_w;
				r_sp_nan1 <= sp_nan1_w;
				r_sp_inf1 <= sp_inf1_w;
				r_s1 <= s1_w;
				r_eab1 <= eab1_w;
			end

			assign v1 = r_v1;
			assign sp1 = r_sp1;
			assign sp_nan1 = r_sp_nan1;
			assign sp_inf1 = r_sp_inf1;
			assign s1 = r_s1;
			assign eab1 = r_eab1;
			assign ma1 = ma1_w;
			assign mb1 = mb1_w;

			reg r_v2, r_sp2, r_sp_nan2, r_sp_inf2, r_s2;
			reg [9:0] r_e2_0, r_e2_1, r_e2_2;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v2 <= 1'b0;
				else
					r_v2 <= v1;
			end

			always @(posedge clk) begin
				r_sp2 <= sp1;
				r_sp_nan2 <= sp_nan1;
				r_sp_inf2 <= sp_inf1;
				r_s2 <= s1;
				r_e2_0 <= e2_0_w;
				r_e2_1 <= e2_1_w;
				r_e2_2 <= e2_2_w;
			end

			assign v2 = r_v2;
			assign sp2 = r_sp2;
			assign sp_nan2 = r_sp_nan2;
			assign sp_inf2 = r_sp_inf2;
			assign s2 = r_s2;
			assign e2_0 = r_e2_0;
			assign e2_1 = r_e2_1;
			assign e2_2 = r_e2_2;
`ifndef SYNTH_ASIC
			assign p2 = p1;

			reg r_v3, r_sp_nan3, r_s3, r_g3;
			(* keep = "true" *) reg r_p47_a, r_p47_b, r_p47_c;
			reg [5:0] r_m3_lo;
			reg [46:29] r_p3_hi;
			reg [3:0] r_st_c3;
			reg r_ga6_0, r_ga6_1, r_ga12_0, r_ga12_1, r_ga18_0, r_ga18_1;
			reg [7:0] r_ef3_0, r_ef3_1, r_ef3_2;
			reg r_mz3_0, r_mz3_1;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v3 <= 1'b0;
				else
					r_v3 <= v2;
			end

			always @(posedge clk) begin
				r_sp_nan3 <= sp_nan2;
				r_s3 <= s2;
				r_g3 <= g3_w;
				r_p47_a <= p2[47];
				r_p47_b <= p2[47];
				r_p47_c <= p2[47];
				r_m3_lo <= m3_lo_w;
				r_p3_hi <= p2[46:29];
				r_st_c3 <= st_c_w;
				r_ga6_0 <= ga6_0_w;
				r_ga6_1 <= ga6_1_w;
				r_ga12_0 <= ga12_0_w;
				r_ga12_1 <= ga12_1_w;
				r_ga18_0 <= ga18_0_w;
				r_ga18_1 <= ga18_1_w;
				r_ef3_0 <= ef3_0_w;
				r_ef3_1 <= ef3_1_w;
				r_ef3_2 <= ef3_2_w;
				r_mz3_0 <= mz3_0_w;
				r_mz3_1 <= mz3_1_w;
			end

			assign v3 = r_v3;
			assign sp_nan3 = r_sp_nan3;
			assign s3 = r_s3;
			assign g3 = r_g3;
			assign p47_a = r_p47_a;
			assign p47_b = r_p47_b;
			assign p47_c = r_p47_c;
			assign m3_lo = r_m3_lo;
			assign p3_hi = r_p3_hi;
			assign st_c3 = r_st_c3;
			assign ga6_0 = r_ga6_0;
			assign ga6_1 = r_ga6_1;
			assign ga12_0 = r_ga12_0;
			assign ga12_1 = r_ga12_1;
			assign ga18_0 = r_ga18_0;
			assign ga18_1 = r_ga18_1;
			assign ef3_0 = r_ef3_0;
			assign ef3_1 = r_ef3_1;
			assign ef3_2 = r_ef3_2;
			assign mz3_0 = r_mz3_0;
			assign mz3_1 = r_mz3_1;
`else
			reg r_v3, r_sp_nan3, r_s3;
			reg [7:0] r_ef3_0, r_ef3_1, r_ef3_2;
			reg r_mz3_0, r_mz3_1;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v3 <= 1'b0;
				else
					r_v3 <= v2;
			end

			always @(posedge clk) begin
				r_sp_nan3 <= sp_nan2;
				r_s3 <= s2;
				r_ef3_0 <= ef3_0_w;
				r_ef3_1 <= ef3_1_w;
				r_ef3_2 <= ef3_2_w;
				r_mz3_0 <= mz3_0_w;
				r_mz3_1 <= mz3_1_w;
			end

			assign v3 = r_v3;
			assign sp_nan3 = r_sp_nan3;
			assign s3 = r_s3;
			assign ef3_0 = r_ef3_0;
			assign ef3_1 = r_ef3_1;
			assign ef3_2 = r_ef3_2;
			assign mz3_0 = r_mz3_0;
			assign mz3_1 = r_mz3_1;
`endif

			reg r_ovalid;
			(* extract_reset = "no" *) reg [31:0] r_or;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_ovalid <= 1'b0;
				else
					r_ovalid <= o_valid_comb;
			end

			always @(posedge clk) begin
				r_or <= o_r_comb;
			end

			assign o_valid = r_ovalid;
			assign o_r = r_or;

			wire unused_lat4 = &{1'b0, unused_m};
		end
	endgenerate
endmodule

`default_nettype wire
