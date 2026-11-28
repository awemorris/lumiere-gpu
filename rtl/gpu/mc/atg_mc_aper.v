/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_mc_aper: The aperture check of an AMB request: the line is inside
 * [APERTURE_BASE, APERTURE_LIMIT) (line granularity).
 *
 * The behaviour is specified in spec/gpu/mc/atg_mc_aper.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_mc_aper #(
	parameter ADDR_W = 32,	/* Address width */
	parameter LINE_SHIFT = 5	/* log2(LINE_BYTES) */
) (
	input wire [ADDR_W-LINE_SHIFT-1:0] line,
	input wire [ADDR_W-1:0] base,
	input wire [ADDR_W-1:0] limit,
	output wire ok
);
	/* spec/gpu/mc/atg_mc_aper.md */
	localparam AW = ADDR_W - LINE_SHIFT;
	wire [AW-1:0] b = base[ADDR_W-1:LINE_SHIFT];
	wire [AW-1:0] l = limit[ADDR_W-1:LINE_SHIFT];

	/* Parallel-prefix unsigned >= / > comparator (<= 8 cmos3 levels, supports AW <= 64). */
	function fn_cmp;
		input [AW-1:0] x;
		input [AW-1:0] y;
		input ci;
		reg [26:0] xv, yv;
		reg [8:0] g1, p1;
		reg [2:0] g2, p2;
		reg [63:0] g0, t0, g1_w, t1_w, g2_w, t2_w, g3_w, t3_w, g4_w, t4_w, g5_w, t5_w, g6_w;
		integer j, k;
		reg e0, e1, e2, gt;
		begin
`ifdef SYNTH_KINTEX7
			if (AW <= 27) begin
				xv = 27'd0;
				yv = 27'd0;
				xv[AW-1:0] = x;
				yv[AW-1:0] = y;
				for (j = 0; j < 9; j = j + 1) begin
					e0 = ~(xv[3*j+0] ^ yv[3*j+0]);
					e1 = ~(xv[3*j+1] ^ yv[3*j+1]);
					e2 = ~(xv[3*j+2] ^ yv[3*j+2]);
					p1[j] = e2 & e1 & e0;
					gt = (xv[3*j+2] & ~yv[3*j+2]) | (e2 & xv[3*j+1] & ~yv[3*j+1]) | (e2 & e1 & xv[3*j+0] & ~yv[3*j+0]);
					g1[j] = (j == 0 && ci) ? (gt | p1[0]) : gt;
				end
				for (k = 0; k < 3; k = k + 1) begin
					g2[k] = g1[3*k+2] | (p1[3*k+2] & g1[3*k+1]) | (p1[3*k+2] & p1[3*k+1] & g1[3*k]);
					p2[k] = p1[3*k+2] & p1[3*k+1] & p1[3*k];
				end
				fn_cmp = g2[2] | (p2[2] & g2[1]) | (p2[2] & p2[1] & g2[0]);
			end else begin
				g0 = 64'd0;
				t0 = 64'd0;
				g0[AW-1:0] = x & ~y;
				t0[AW-1:0] = x | ~y;
				if (ci)
					g0[0] = t0[0];
				g1_w = g0 | (t0 & {g0[62:0], 1'b0});
				t1_w = t0 & {t0[62:0], 1'b0};
				g2_w = g1_w | (t1_w & {g1_w[61:0], 2'b0});
				t2_w = t1_w & {t1_w[61:0], 2'b0};
				g3_w = g2_w | (t2_w & {g2_w[59:0], 4'b0});
				t3_w = t2_w & {t2_w[59:0], 4'b0};
				g4_w = g3_w | (t3_w & {g3_w[55:0], 8'b0});
				t4_w = t3_w & {t3_w[55:0], 8'b0};
				g5_w = g4_w | (t4_w & {g4_w[47:0], 16'b0});
				t5_w = t4_w & {t4_w[47:0], 16'b0};
				g6_w = g5_w | (t5_w & {g5_w[31:0], 32'b0});
				fn_cmp = g6_w[AW-1];
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (AW <= 27) begin
				xv = 27'd0;
				yv = 27'd0;
/* verilator lint_off SELRANGE */
				xv[AW-1:0] = x;
				yv[AW-1:0] = y;
/* verilator lint_on SELRANGE */
				for (j = 0; j < 9; j = j + 1) begin
					e0 = ~(xv[3*j+0] ^ yv[3*j+0]);
					e1 = ~(xv[3*j+1] ^ yv[3*j+1]);
					e2 = ~(xv[3*j+2] ^ yv[3*j+2]);
					p1[j] = e2 & e1 & e0;
					gt = (xv[3*j+2] & ~yv[3*j+2]) | (e2 & xv[3*j+1] & ~yv[3*j+1]) | (e2 & e1 & xv[3*j+0] & ~yv[3*j+0]);
					g1[j] = (j == 0 && ci) ? (gt | p1[0]) : gt;
				end
				for (k = 0; k < 3; k = k + 1) begin
					g2[k] = g1[3*k+2] | (p1[3*k+2] & g1[3*k+1]) | (p1[3*k+2] & p1[3*k+1] & g1[3*k]);
					p2[k] = p1[3*k+2] & p1[3*k+1] & p1[3*k];
				end
				fn_cmp = g2[2] | (p2[2] & g2[1]) | (p2[2] & p2[1] & g2[0]);
			end else begin
				g0 = 64'd0;
				t0 = 64'd0;
				g0[AW-1:0] = x & ~y;
				t0[AW-1:0] = x | ~y;
				if (ci)
					g0[0] = t0[0];
				g1_w = g0 | (t0 & {g0[62:0], 1'b0});
				t1_w = t0 & {t0[62:0], 1'b0};
				g2_w = g1_w | (t1_w & {g1_w[61:0], 2'b0});
				t2_w = t1_w & {t1_w[61:0], 2'b0};
				g3_w = g2_w | (t2_w & {g2_w[59:0], 4'b0});
				t3_w = t2_w & {t2_w[59:0], 4'b0};
				g4_w = g3_w | (t3_w & {g3_w[55:0], 8'b0});
				t4_w = t3_w & {t3_w[55:0], 8'b0};
				g5_w = g4_w | (t4_w & {g4_w[47:0], 16'b0});
				t5_w = t4_w & {t4_w[47:0], 16'b0};
				g6_w = g5_w | (t5_w & {g5_w[31:0], 32'b0});
				fn_cmp = g6_w[AW-1];
			end
`else
			g0 = 64'd0;
			t0 = 64'd0;
			g0[AW-1:0] = x & ~y;
			t0[AW-1:0] = x | ~y;
			if (ci)
				g0[0] = t0[0];
			g1_w = g0 | (t0 & {g0[62:0], 1'b0});
			t1_w = t0 & {t0[62:0], 1'b0};
			g2_w = g1_w | (t1_w & {g1_w[61:0], 2'b0});
			t2_w = t1_w & {t1_w[61:0], 2'b0};
			g3_w = g2_w | (t2_w & {g2_w[59:0], 4'b0});
			t3_w = t2_w & {t2_w[59:0], 4'b0};
			g4_w = g3_w | (t3_w & {g3_w[55:0], 8'b0});
			t4_w = t3_w & {t3_w[55:0], 8'b0};
			g5_w = g4_w | (t4_w & {g4_w[47:0], 16'b0});
			t5_w = t4_w & {t4_w[47:0], 16'b0};
			g6_w = g5_w | (t5_w & {g5_w[31:0], 32'b0});
			fn_cmp = g6_w[AW-1];
`endif
		end
	endfunction

	assign ok = fn_cmp(line, b, 1'b1) && fn_cmp(l, line, 1'b0);

	wire unused_ok = &{1'b0, base[LINE_SHIFT-1:0], limit[LINE_SHIFT-1:0]};
endmodule

`default_nettype wire
