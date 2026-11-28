/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * am_fp_rint: the integer nearest to |x| (x flushed, exponent below 150)
 * under rm (0 RNE, 1 RTZ, 2 RDN, 3 RUP, 4 RMM, others RNE) and x's sign;
 * n is the magnitude, 25 bits.
 * See spec/library/fp/am_fp_rint.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module am_fp_rint #(
	/* verilator lint_off UNUSEDPARAM */
	parameter LAT = 0
	/* verilator lint_on UNUSEDPARAM */
) (
	input wire [31:0] x,
	input wire [2:0] rm,
	output wire [24:0] n
);
	wire [7:0] e = x[30:23];
	wire e_nz = (e != 8'd0);
	wire x_s = e_nz && x[31];
	wire [23:0] m = {e_nz, x[22:0]};

	wire e_ge150 = e[7] && (|e[6:5] || (e[4] && (e[3] || (e[2] && e[1]))));
	wire e_lt126 = !e[7] && !(&e[6:1]);
	wire e_hi011 = (e[7:5] == 3'b011) && (&e[4:1]);
	wire e_hi100 = (e[7:5] == 3'b100);

	/* Direct per-bit exponent-threshold sticky mask (replaces 24-bit suffix OR + 1-hot e_oh) */
	wire [22:0] st_mask_hi;
	wire [22:0] st_bits;
	genvar gi;
	generate
		for (gi = 0; gi <= 20; gi = gi + 1) begin : g_st_mask
			localparam [4:0] MAX_E5 = 20 - gi;
			assign st_mask_hi[gi] = e_hi011 || (e_hi100 && (e[4:0] <= MAX_E5));
			assign st_bits[gi]    = m[gi] && st_mask_hi[gi];
		end
	endgenerate
	assign st_mask_hi[21] = e_hi011;
	assign st_bits[21]    = m[21] && st_mask_hi[21];
	assign st_mask_hi[22] = e_hi011 && !e[0];
	assign st_bits[22]    = m[22] && st_mask_hi[22];

	wire st_lo = e_lt126 && e_nz && (|m);
	wire st_hi = |st_bits;
	wire st = st_lo || st_hi;

	/* Direct 2-stage right shift aligned to e[7:2] (single LUT6 per hi_oh bit). */
	wire [6:0] hi_oh = {
		(e[7:2] == 6'd31),
		(e[7:2] == 6'd32),
		(e[7:2] == 6'd33),
		(e[7:2] == 6'd34),
		(e[7:2] == 6'd35),
		(e[7:2] == 6'd36),
		(e[7:2] == 6'd37)
	};

	wire [28:0] pre_sh_g = {3'd0, m, 2'd0};
	wire [25:0] s1_sh;
	wire [49:0] s1_sh_p = {24'd0, s1_sh};
	wire [25:0] sh_out;
	generate
		for (gi = 0; gi < 26; gi = gi + 1) begin : g_s1_sh
			assign s1_sh[gi] =
				(e[1:0] == 2'd3) ? pre_sh_g[gi]     :
				(e[1:0] == 2'd2) ? pre_sh_g[gi + 1] :
				(e[1:0] == 2'd1) ? pre_sh_g[gi + 2] :
				                   pre_sh_g[gi + 3];
		end
		for (gi = 0; gi < 26; gi = gi + 1) begin : g_s2_sh
			assign sh_out[gi] =
				(hi_oh[0] & s1_sh_p[gi])      | (hi_oh[1] & s1_sh_p[gi + 4])  |
				(hi_oh[2] & s1_sh_p[gi + 8])  | (hi_oh[3] & s1_sh_p[gi + 12]) |
				(hi_oh[4] & s1_sh_p[gi + 16]) | (hi_oh[5] & s1_sh_p[gi + 20]) |
				(hi_oh[6] & s1_sh_p[gi + 24]);
		end
	endgenerate

	wire [24:0] sh_m = sh_out[25:1];
	wire g = sh_out[0];
	wire q_lsb = sh_m[0];

	reg up;
	always @(*)
		case (rm)
		3'd1: up = 1'b0;
		3'd2: up = x_s && (g || st);
		3'd3: up = !x_s && (g || st);
		3'd4: up = g;
		default: up = g && (st || q_lsb);
		endcase

	/* Carry-propagate vector depends only on sh_m; up gates only at the final XOR. */
	wire [5:0] inc_g4;
	wire [6:0] inc_p4;
	wire [24:0] inc_raw;
	assign inc_p4[0] = 1'b1;
	generate
		for (gi = 0; gi < 6; gi = gi + 1) begin : g_inc_grp
			assign inc_g4[gi] = &sh_m[gi*4 +: 4];
			assign inc_p4[gi + 1] = &inc_g4[gi:0];
			assign inc_raw[gi*4 + 0] = inc_p4[gi];
			assign inc_raw[gi*4 + 1] = inc_p4[gi] & sh_m[gi*4 + 0];
			assign inc_raw[gi*4 + 2] = inc_p4[gi] & (&sh_m[gi*4 + 1 : gi*4 + 0]);
			assign inc_raw[gi*4 + 3] = inc_p4[gi] & (&sh_m[gi*4 + 2 : gi*4 + 0]);
		end
	endgenerate
	assign inc_raw[24] = inc_p4[6];
	assign n = e_ge150 ? {1'b0, m} : (sh_m ^ ({25{up}} & inc_raw));

	wire unused_ok = &{1'b0, pre_sh_g[28:26], s1_sh_p[49:26]};
endmodule

`default_nettype wire
