/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_skid: fanout-buffered register slice / skid buffer for ROP.
 * Compliant with ASIC 5nm 1.5 GHz / Kintex-7 fanout <= 16 limits.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_skid #(
	parameter W = 32
) (
	input wire clk,
	input wire rst_n,

	input wire in_valid,
	output reg in_ready,
	input wire [W-1:0] in_data,

	output reg out_valid,
	input wire out_ready,
	output wire [W-1:0] out_data
);
	reg skid_valid;

	wire rdy_gate_w = skid_valid || in_valid;
	wire bypass_w   = !skid_valid && in_valid && !out_valid;
	wire load_out_raw = !(!(out_ready && rdy_gate_w) && !bypass_w);
	wire load_skid_raw = in_valid && in_ready && !out_ready && out_valid;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			in_ready   <= 1'b1;
			out_valid  <= 1'b0;
			skid_valid <= 1'b0;
		end else begin
			if (skid_valid) begin
				if (out_ready) begin
					out_valid  <= 1'b1;
					skid_valid <= 1'b0;
					in_ready   <= 1'b1;
				end
			end else begin
				if (out_ready || !out_valid) begin
					out_valid <= in_valid;
				end else if (in_valid && in_ready) begin
					skid_valid <= 1'b1;
					in_ready   <= 1'b0;
				end
			end
		end
	end

	localparam NUM_SLICES = (W + 15) / 16;
	localparam NUM_GROUPS = (NUM_SLICES + 15) / 16;

	reg [W-1:0] r_out_data;
	reg [W-1:0] r_skid_data;
	assign out_data = r_out_data;

	wire [NUM_GROUPS-1:0] ld_out_grp;
	wire [NUM_GROUPS-1:0] ld_skid_grp;
	wire [NUM_GROUPS-1:0] sk_val_grp;

	genvar gi, si;
	generate
		for (gi = 0; gi < NUM_GROUPS; gi = gi + 1) begin : g_grp
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) wire ldo_g;
			(* keep = "true", dont_touch = "true" *) wire lds_g;
			(* keep = "true", dont_touch = "true" *) wire skv_g;
			LUT1 #(.INIT(2'b10)) u_g_ldo (.I0(load_out_raw), .O(ldo_g));
			LUT1 #(.INIT(2'b10)) u_g_lds (.I0(load_skid_raw), .O(lds_g));
			LUT1 #(.INIT(2'b10)) u_g_skv (.I0(skid_valid), .O(skv_g));
			assign ld_out_grp[gi] = ldo_g;
			assign ld_skid_grp[gi] = lds_g;
			assign sk_val_grp[gi] = skv_g;
`elsif SYNTH_KINTEX_ULTRASCALE
			(* keep = "true", dont_touch = "true" *) wire ldo_g;
			(* keep = "true", dont_touch = "true" *) wire lds_g;
			(* keep = "true", dont_touch = "true" *) wire skv_g;
			assign ldo_g = load_out_raw;
			assign lds_g = load_skid_raw;
			assign skv_g = skid_valid;
			assign ld_out_grp[gi] = ldo_g;
			assign ld_skid_grp[gi] = lds_g;
			assign sk_val_grp[gi] = skv_g;
`else
			assign ld_out_grp[gi] = load_out_raw;
			assign ld_skid_grp[gi] = load_skid_raw;
			assign sk_val_grp[gi] = skid_valid;
`endif
		end

		for (si = 0; si < NUM_SLICES; si = si + 1) begin : g_slice
			localparam LO = si * 16;
			localparam HI = (si == NUM_SLICES - 1) ? (W - 1) : (si * 16 + 15);
			localparam GRP_IDX = si / 16;

`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) wire ld_out_b;
			(* keep = "true", dont_touch = "true" *) wire ld_skid_b;
			(* keep = "true", dont_touch = "true" *) wire sk_valid_b;
			LUT1 #(.INIT(2'b10)) u_ldo (.I0(ld_out_grp[GRP_IDX]), .O(ld_out_b));
			LUT1 #(.INIT(2'b10)) u_lds (.I0(ld_skid_grp[GRP_IDX]), .O(ld_skid_b));
			LUT1 #(.INIT(2'b10)) u_skv (.I0(sk_val_grp[GRP_IDX]), .O(sk_valid_b));
`elsif SYNTH_KINTEX_ULTRASCALE
			(* keep = "true", dont_touch = "true" *) wire ld_out_b;
			(* keep = "true", dont_touch = "true" *) wire ld_skid_b;
			(* keep = "true", dont_touch = "true" *) wire sk_valid_b;
			assign ld_out_b = ld_out_grp[GRP_IDX];
			assign ld_skid_b = ld_skid_grp[GRP_IDX];
			assign sk_valid_b = sk_val_grp[GRP_IDX];
`else
			wire ld_out_b = ld_out_grp[GRP_IDX];
			wire ld_skid_b = ld_skid_grp[GRP_IDX];
			wire sk_valid_b = sk_val_grp[GRP_IDX];
`endif

			always @(posedge clk) begin
				if (ld_skid_b)
					r_skid_data[HI:LO] <= in_data[HI:LO];
				if (ld_out_b)
					r_out_data[HI:LO] <= sk_valid_b ? r_skid_data[HI:LO] : in_data[HI:LO];
			end
		end
	endgenerate

endmodule

(* keep_hierarchy = "yes" *)
module atg_rop_skid_rep #(
	parameter W = 563,
	parameter R_W = 10
) (
	input wire clk,
	input wire rst_n,

	input wire in_valid,
	output reg in_ready,
	input wire [W-1:0] in_data,

	output reg out_valid,
	input wire [R_W-1:0] out_ready_vec,
	output wire [W-1:0] out_data
);
	localparam SLICE_W = (R_W >= 10) ? (R_W - 1) : R_W;
	localparam CTRL_R  = R_W - 1;
	reg skid_valid;
	localparam NUM_SLICES = (W + 15) / 16;

	reg [W-1:0] r_out_data;
	reg [W-1:0] r_skid_data;
	assign out_data = r_out_data;

	wire [R_W-1:0] in_valid_g;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) reg [NUM_SLICES-1:0] skid_valid_s;
	(* keep = "true", dont_touch = "true" *) reg [SLICE_W-1:0] skid_valid_g, out_valid_g, in_ready_g;
	localparam integer RST_S_N = (NUM_SLICES + 7) / 8;
	wire rst_sync_c;
	wire [SLICE_W-1:0] rst_sync_g;
	wire [RST_S_N-1:0] rst_sync_s;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rstc (.I0(rst_n), .O(rst_sync_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [NUM_SLICES-1:0] skid_valid_s;
	wire [SLICE_W-1:0] skid_valid_g, out_valid_g, in_ready_g;
	localparam integer RST_S_N = (NUM_SLICES + 7) / 8;
	wire rst_sync_c;
	wire [SLICE_W-1:0] rst_sync_g;
	wire [RST_S_N-1:0] rst_sync_s;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rstc (.I0(rst_n), .O(rst_sync_c));
`else
	(* keep = "true", dont_touch = "true" *) reg [NUM_SLICES-1:0] skid_valid_s;
	(* keep = "true", dont_touch = "true" *) reg [SLICE_W-1:0] skid_valid_g, out_valid_g, in_ready_g;
`endif
	genvar gr, si;
	generate
		for (gr = 0; gr < R_W; gr = gr + 1) begin : g_iv
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iv (.I0(in_valid), .O(in_valid_g[gr]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign in_valid_g[gr] = in_valid;
`else
			assign in_valid_g[gr] = in_valid;
`endif
		end
`ifdef SYNTH_KINTEX7
		for (gr = 0; gr < SLICE_W; gr = gr + 1) begin : g_rst_g
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rstg (.I0(rst_n), .O(rst_sync_g[gr]));
		end
		for (gr = 0; gr < RST_S_N; gr = gr + 1) begin : g_rst_s
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rsts (.I0(rst_n), .O(rst_sync_s[gr]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gr = 0; gr < SLICE_W; gr = gr + 1) begin : g_rst_g
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rstg (.I0(rst_n), .O(rst_sync_g[gr]));
		end
		for (gr = 0; gr < RST_S_N; gr = gr + 1) begin : g_rst_s
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_rsts (.I0(rst_n), .O(rst_sync_s[gr]));
		end
`endif
	endgenerate

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge rst_sync_c) begin
		if (rst_sync_c) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge rst_sync_c) begin
		if (rst_sync_c) begin
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
`endif
			in_ready   <= 1'b1;
			out_valid  <= 1'b0;
			skid_valid <= 1'b0;
		end else begin
			if (skid_valid) begin
				if (out_ready_vec[CTRL_R]) begin
					out_valid  <= 1'b1;
					skid_valid <= 1'b0;
					in_ready   <= 1'b1;
				end
			end else begin
				if (out_ready_vec[CTRL_R] || !out_valid) begin
					out_valid <= in_valid_g[CTRL_R];
				end else if (in_valid_g[CTRL_R] && in_ready) begin
					skid_valid <= 1'b1;
					in_ready   <= 1'b0;
				end
			end
		end
	end

	generate
		for (gr = 0; gr < SLICE_W; gr = gr + 1) begin : g_ctrl_g
`ifdef SYNTH_KINTEX7
			always @(posedge clk or posedge rst_sync_g[gr]) begin
				if (rst_sync_g[gr]) begin
					in_ready_g[gr]   <= 1'b1;
					out_valid_g[gr]  <= 1'b0;
					skid_valid_g[gr] <= 1'b0;
				end else begin
					if (skid_valid_g[gr]) begin
						if (out_ready_vec[gr]) begin
							out_valid_g[gr]  <= 1'b1;
							skid_valid_g[gr] <= 1'b0;
							in_ready_g[gr]   <= 1'b1;
						end
					end else begin
						if (out_ready_vec[gr] || !out_valid_g[gr]) begin
							out_valid_g[gr] <= in_valid_g[gr];
						end else if (in_valid_g[gr] && in_ready_g[gr]) begin
							skid_valid_g[gr] <= 1'b1;
							in_ready_g[gr]   <= 1'b0;
						end
					end
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			(* keep = "true", dont_touch = "true" *) reg in_ready_gr, out_valid_gr, skid_valid_gr;
			assign in_ready_g[gr]   = in_ready_gr;
			assign out_valid_g[gr]  = out_valid_gr;
			assign skid_valid_g[gr] = skid_valid_gr;
			always @(posedge clk or posedge rst_sync_g[gr]) begin
				if (rst_sync_g[gr]) begin
					in_ready_gr   <= 1'b1;
					out_valid_gr  <= 1'b0;
					skid_valid_gr <= 1'b0;
				end else begin
					if (skid_valid_gr) begin
						if (out_ready_vec[gr]) begin
							out_valid_gr  <= 1'b1;
							skid_valid_gr <= 1'b0;
							in_ready_gr   <= 1'b1;
						end
					end else begin
						if (out_ready_vec[gr] || !out_valid_gr) begin
							out_valid_gr <= in_valid_g[gr];
						end else if (in_valid_g[gr] && in_ready_gr) begin
							skid_valid_gr <= 1'b1;
							in_ready_gr   <= 1'b0;
						end
					end
				end
			end
`else
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					in_ready_g[gr]   <= 1'b1;
					out_valid_g[gr]  <= 1'b0;
					skid_valid_g[gr] <= 1'b0;
				end else begin
					if (skid_valid_g[gr]) begin
						if (out_ready_vec[gr]) begin
							out_valid_g[gr]  <= 1'b1;
							skid_valid_g[gr] <= 1'b0;
							in_ready_g[gr]   <= 1'b1;
						end
					end else begin
						if (out_ready_vec[gr] || !out_valid_g[gr]) begin
							out_valid_g[gr] <= in_valid_g[gr];
						end else if (in_valid_g[gr] && in_ready_g[gr]) begin
							skid_valid_g[gr] <= 1'b1;
							in_ready_g[gr]   <= 1'b0;
						end
					end
				end
			end
`endif
		end

		for (si = 0; si < NUM_SLICES; si = si + 1) begin : g_slice
			localparam LO = si * 16;
			localparam HI = (si == NUM_SLICES - 1) ? (W - 1) : (si * 16 + 15);
			localparam integer R_IDX = si % SLICE_W;
`ifdef SYNTH_KINTEX7
			always @(posedge clk or posedge rst_sync_s[si / 8]) begin
				if (rst_sync_s[si / 8]) begin
					skid_valid_s[si] <= 1'b0;
				end else if (skid_valid_g[R_IDX]) begin
					if (out_ready_vec[R_IDX])
						skid_valid_s[si] <= 1'b0;
				end else if (!out_ready_vec[R_IDX] && out_valid_g[R_IDX] && in_valid_g[R_IDX] && in_ready_g[R_IDX]) begin
					skid_valid_s[si] <= 1'b1;
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			(* keep = "true", dont_touch = "true" *) reg skid_valid_si;
			assign skid_valid_s[si] = skid_valid_si;
			always @(posedge clk or posedge rst_sync_s[si / 8]) begin
				if (rst_sync_s[si / 8]) begin
					skid_valid_si <= 1'b0;
				end else if (skid_valid_g[R_IDX]) begin
					if (out_ready_vec[R_IDX])
						skid_valid_si <= 1'b0;
				end else if (!out_ready_vec[R_IDX] && out_valid_g[R_IDX] && in_valid_g[R_IDX] && in_ready_g[R_IDX]) begin
					skid_valid_si <= 1'b1;
				end
			end
`else
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					skid_valid_s[si] <= 1'b0;
				end else if (skid_valid_g[R_IDX]) begin
					if (out_ready_vec[R_IDX])
						skid_valid_s[si] <= 1'b0;
				end else if (!out_ready_vec[R_IDX] && out_valid_g[R_IDX] && in_valid_g[R_IDX] && in_ready_g[R_IDX]) begin
					skid_valid_s[si] <= 1'b1;
				end
			end
`endif

`ifdef SYNTH_KINTEX7
			wire ld_out_b, ld_skid_b;
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hA8B8)) u_ldo (
				.I0(out_ready_vec[R_IDX]), .I1(skid_valid_g[R_IDX]),
				.I2(in_valid_g[R_IDX]), .I3(out_valid_g[R_IDX]),
				.O(ld_out_b)
			);
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h4000)) u_lds (
				.I0(out_ready_vec[R_IDX]), .I1(in_valid_g[R_IDX]),
				.I2(in_ready_g[R_IDX]), .I3(out_valid_g[R_IDX]),
				.O(ld_skid_b)
			);
`elsif SYNTH_KINTEX_ULTRASCALE
			wire ld_out_b, ld_skid_b;
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hA8B8)) u_ldo (
				.I0(out_ready_vec[R_IDX]), .I1(skid_valid_g[R_IDX]),
				.I2(in_valid_g[R_IDX]), .I3(out_valid_g[R_IDX]),
				.O(ld_out_b)
			);
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h4000)) u_lds (
				.I0(out_ready_vec[R_IDX]), .I1(in_valid_g[R_IDX]),
				.I2(in_ready_g[R_IDX]), .I3(out_valid_g[R_IDX]),
				.O(ld_skid_b)
			);
`else
			(* keep = "true" *) wire ld_out_b = (out_ready_vec[R_IDX] & (skid_valid_g[R_IDX] | in_valid_g[R_IDX])) |
			                                (~skid_valid_g[R_IDX] & in_valid_g[R_IDX] & ~out_valid_g[R_IDX]);
			(* keep = "true" *) wire ld_skid_b = in_valid_g[R_IDX] & in_ready_g[R_IDX] & ~out_ready_vec[R_IDX] & out_valid_g[R_IDX];
`endif

			always @(posedge clk) begin
				if (ld_skid_b)
					r_skid_data[HI:LO] <= in_data[HI:LO];
				if (ld_out_b)
					r_out_data[HI:LO] <= skid_valid_s[si] ? r_skid_data[HI:LO] : in_data[HI:LO];
			end
		end
	endgenerate

endmodule

`default_nettype wire
