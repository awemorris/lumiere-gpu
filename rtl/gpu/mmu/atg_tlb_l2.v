/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tlb_l2: 256-entry 4-way set-associative L2 TLB (64 sets x 4 ways).
 * Hit latency: 2 clock cycles.
 * Shared across execution units; handles misses from L1 TLBs.
 * See spec/gpu/mmu/atg_mmu.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_ram64x1d_array #(
	parameter WIDTH = 1
) (
	input wire clk,
	input wire we,
	input wire [5:0] waddr,
	input wire [WIDTH-1:0] din,
	input wire [5:0] raddr,
	output wire [WIDTH-1:0] dout
);

	reg [WIDTH-1:0] mem [0:63];
	always @(posedge clk) begin
		if (we) mem[waddr] <= din;
	end
	assign dout = mem[raddr];

endmodule

module atg_tlb_l2 #(
	parameter SETS = 64,
	parameter VA_BITS = 32,
	parameter PA_BITS = 32,
	parameter VPN_BITS = VA_BITS - 12,
	parameter PPN_BITS = PA_BITS - 12,
	parameter SET_W = 6,
	parameter REM_VPN = VPN_BITS - SET_W,
	parameter TAG_W = 4 + REM_VPN		/* {session_id[3:0], vpn[VPN_BITS-1:6]} = 18 bits */
) (
	input wire clk,
	input wire rst_active,

	/* Lookup Request (from L1 miss) */
	input  wire                lookup_valid,
	input  wire [3:0]          lookup_session,
	input  wire [VPN_BITS-1:0] lookup_vpn,
	input  wire [5:0]          lookup_set_w0_vpn,
	input  wire [5:0]          lookup_set_w0_pfn_lo,
	input  wire [5:0]          lookup_set_w0_pfn_hi,
	input  wire [5:0]          lookup_set_w0_flg,
	input  wire [5:0]          lookup_set_w0_sl,
	input  wire [5:0]          lookup_set_w1_vpn,
	input  wire [5:0]          lookup_set_w1_pfn_lo,
	input  wire [5:0]          lookup_set_w1_pfn_hi,
	input  wire [5:0]          lookup_set_w1_flg,
	input  wire [5:0]          lookup_set_w1_sl,
	input  wire [5:0]          lookup_set_w2_vpn,
	input  wire [5:0]          lookup_set_w2_pfn_lo,
	input  wire [5:0]          lookup_set_w2_pfn_hi,
	input  wire [5:0]          lookup_set_w2_flg,
	input  wire [5:0]          lookup_set_w2_sl,
	input  wire [5:0]          lookup_set_w3_vpn,
	input  wire [5:0]          lookup_set_w3_pfn_lo,
	input  wire [5:0]          lookup_set_w3_pfn_hi,
	input  wire [5:0]          lookup_set_w3_flg,
	input  wire [5:0]          lookup_set_w3_sl,
	output reg                 hit,
	output reg  [PPN_BITS-1:0] hit_pfn,
	output reg  [5:0]          hit_flags,

	/* Refill Interface (from Hardware Page Table Walker) */
	input wire                refill_we_w0,
	input wire                refill_we_w1,
	input wire                refill_we_w2,
	input wire                refill_we_w3,
	input wire                refill_we_sl0,
	input wire                refill_we_sl1,
	input wire                refill_we_sl2,
	input wire                refill_we_sl3,
	input wire [3:0]          refill_session,
	input wire [VPN_BITS-1:0] refill_vpn,

	/* Dedicated write addresses per slice (<= 5 RAM cells per address net) */
	input wire [5:0]          refill_set_w0_vpn_a,
	input wire [5:0]          refill_set_w0_vpn_b,
	input wire [5:0]          refill_set_w0_vpn_c,
	input wire [5:0]          refill_set_w0_pfn_a,
	input wire [5:0]          refill_set_w0_pfn_b,
	input wire [5:0]          refill_set_w0_pfn_c,
	input wire [5:0]          refill_set_w0_pfn_d,
	input wire [5:0]          refill_set_w0_flg_a,
	input wire [5:0]          refill_set_w0_flg_b,

	input wire [5:0]          refill_set_w1_vpn_a,
	input wire [5:0]          refill_set_w1_vpn_b,
	input wire [5:0]          refill_set_w1_vpn_c,
	input wire [5:0]          refill_set_w1_pfn_a,
	input wire [5:0]          refill_set_w1_pfn_b,
	input wire [5:0]          refill_set_w1_pfn_c,
	input wire [5:0]          refill_set_w1_pfn_d,
	input wire [5:0]          refill_set_w1_flg_a,
	input wire [5:0]          refill_set_w1_flg_b,

	input wire [5:0]          refill_set_w2_vpn_a,
	input wire [5:0]          refill_set_w2_vpn_b,
	input wire [5:0]          refill_set_w2_vpn_c,
	input wire [5:0]          refill_set_w2_pfn_a,
	input wire [5:0]          refill_set_w2_pfn_b,
	input wire [5:0]          refill_set_w2_pfn_c,
	input wire [5:0]          refill_set_w2_pfn_d,
	input wire [5:0]          refill_set_w2_flg_a,
	input wire [5:0]          refill_set_w2_flg_b,

	input wire [5:0]          refill_set_w3_vpn_a,
	input wire [5:0]          refill_set_w3_vpn_b,
	input wire [5:0]          refill_set_w3_vpn_c,
	input wire [5:0]          refill_set_w3_pfn_a,
	input wire [5:0]          refill_set_w3_pfn_b,
	input wire [5:0]          refill_set_w3_pfn_c,
	input wire [5:0]          refill_set_w3_pfn_d,
	input wire [5:0]          refill_set_w3_flg_a,
	input wire [5:0]          refill_set_w3_flg_b,

	input wire [PPN_BITS-1:0] refill_pfn,
	input wire [5:0]          refill_flags,

	/* Invalidation Interface */
	input wire        flush_trigger,
	input wire        flush_all,
	input wire [3:0]  flush_session
);
	/* Clock and reset naming to prevent anonymous net violations */
	(* keep = "true" *) wire clk_l2 = clk;
	(* keep = "true" *) wire rst_l2_int = rst_active;

	wire [TAG_W-1:0] lookup_tag = {lookup_session, lookup_vpn[VPN_BITS-1:6]};
	wire [5:0]       refill_set_in = refill_vpn[5:0];

	/* Replacement pointer per set distributed into 16 slices */
	wire [31:0] sl_cur_ptr_out;
	(* keep = "true", equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0] r_cur_ptr_g0, r_cur_ptr_g1, r_cur_ptr_g2, r_cur_ptr_g3;
	(* keep = "true", equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0] r_cur_ptr_g4, r_cur_ptr_g5, r_cur_ptr_g6, r_cur_ptr_g7;
	(* keep = "true", equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0] r_cur_ptr_inc;

	/* Input register stage for refill signals to decouple port fanout */
	(* max_fanout = 16 *) reg [3:0]          r_ref_sess;
	(* max_fanout = 16 *) reg [REM_VPN-1:0]  r_ref_vpn;
	(* max_fanout = 16 *) reg [PPN_BITS-1:0] r_ref_pfn;
	(* max_fanout = 16 *) reg [5:0]          r_ref_flags;

	always @(posedge clk_l2 or posedge rst_l2_int) begin
		if (rst_l2_int) begin
			r_cur_ptr_g0  <= 2'd0;
			r_cur_ptr_g1  <= 2'd0;
			r_cur_ptr_g2  <= 2'd0;
			r_cur_ptr_g3  <= 2'd0;
			r_cur_ptr_g4  <= 2'd0;
			r_cur_ptr_g5  <= 2'd0;
			r_cur_ptr_g6  <= 2'd0;
			r_cur_ptr_g7  <= 2'd0;
			r_cur_ptr_inc <= 2'd0;
			r_ref_sess    <= 4'd0;
			r_ref_vpn     <= {REM_VPN{1'b0}};
			r_ref_pfn     <= {PPN_BITS{1'b0}};
			r_ref_flags   <= 6'd0;
		end else begin
			r_cur_ptr_g0  <= cur_ptr_g;
			r_cur_ptr_g1  <= cur_ptr_g;
			r_cur_ptr_g2  <= cur_ptr_g;
			r_cur_ptr_g3  <= cur_ptr_g;
			r_cur_ptr_g4  <= cur_ptr_g;
			r_cur_ptr_g5  <= cur_ptr_g;
			r_cur_ptr_g6  <= cur_ptr_g;
			r_cur_ptr_g7  <= cur_ptr_g;
			r_cur_ptr_inc <= cur_ptr_g;
			r_ref_sess    <= refill_session;
			r_ref_vpn     <= refill_vpn[VPN_BITS-1:6];
			r_ref_pfn     <= refill_pfn;
			r_ref_flags   <= refill_flags;
		end
	end

	wire [1:0] next_ptr;
	wire ptr_co_unused;
	am_add #(.W(2)) u_ptr_inc (
		.a(r_cur_ptr_inc),
		.b(2'd1),
		.ci(1'b0),
		.s(next_ptr),
		.co(ptr_co_unused)
	);

	wire [1:0] next_ptr_g0, next_ptr_g1, next_ptr_g2, next_ptr_g3;

	assign next_ptr_g0 = next_ptr;
	assign next_ptr_g1 = next_ptr;
	assign next_ptr_g2 = next_ptr;
	assign next_ptr_g3 = next_ptr;


	/* Sliced Write Enables per way and field (2-level 4:1 LUT6 tree: 4 LUTs max) */
	wire [1:0] cur_ptr_w0, cur_ptr_w1, cur_ptr_w2, cur_ptr_w3, cur_ptr_g;
`ifdef SYNTH_KINTEX7
	wire [1:0] grp_ptr [0:3];
	genvar gcp, gcpb;
	generate
		for (gcp = 0; gcp < 4; gcp = gcp + 1) begin : g_ptr_grp
			for (gcpb = 0; gcpb < 2; gcpb = gcpb + 1) begin : g_pgb
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_gp (
					.I0(sl_cur_ptr_out[(gcp*4 + 0)*2 + gcpb]),
					.I1(sl_cur_ptr_out[(gcp*4 + 1)*2 + gcpb]),
					.I2(sl_cur_ptr_out[(gcp*4 + 2)*2 + gcpb]),
					.I3(sl_cur_ptr_out[(gcp*4 + 3)*2 + gcpb]),
					.I4(refill_set_in[2]),
					.I5(refill_set_in[3]),
					.O(grp_ptr[gcp][gcpb])
				);
			end
		end
		for (gcpb = 0; gcpb < 2; gcpb = gcpb + 1) begin : g_pf
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw0 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w0[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw1 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w1[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw2 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w2[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw3 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w3[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpg (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_g[gcpb])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] grp_ptr [0:3];
	genvar gcp, gcpb;
	generate
		for (gcp = 0; gcp < 4; gcp = gcp + 1) begin : g_ptr_grp
			for (gcpb = 0; gcpb < 2; gcpb = gcpb + 1) begin : g_pgb
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_gp (
					.I0(sl_cur_ptr_out[(gcp*4 + 0)*2 + gcpb]),
					.I1(sl_cur_ptr_out[(gcp*4 + 1)*2 + gcpb]),
					.I2(sl_cur_ptr_out[(gcp*4 + 2)*2 + gcpb]),
					.I3(sl_cur_ptr_out[(gcp*4 + 3)*2 + gcpb]),
					.I4(refill_set_in[2]),
					.I5(refill_set_in[3]),
					.O(grp_ptr[gcp][gcpb])
				);
			end
		end
		for (gcpb = 0; gcpb < 2; gcpb = gcpb + 1) begin : g_pf
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw0 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w0[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw1 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w1[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw2 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w2[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpw3 (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_w3[gcpb])
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_cpg (
				.I0(grp_ptr[0][gcpb]), .I1(grp_ptr[1][gcpb]),
				.I2(grp_ptr[2][gcpb]), .I3(grp_ptr[3][gcpb]),
				.I4(refill_set_in[4]), .I5(refill_set_in[5]), .O(cur_ptr_g[gcpb])
			);
		end
	endgenerate
`else
	assign cur_ptr_w0 = sl_cur_ptr_out[refill_set_in[5:2]*2 +: 2];
	assign cur_ptr_w1 = sl_cur_ptr_out[refill_set_in[5:2]*2 +: 2];
	assign cur_ptr_w2 = sl_cur_ptr_out[refill_set_in[5:2]*2 +: 2];
	assign cur_ptr_w3 = sl_cur_ptr_out[refill_set_in[5:2]*2 +: 2];
	assign cur_ptr_g  = sl_cur_ptr_out[refill_set_in[5:2]*2 +: 2];
`endif
	(* keep = "true", equivalent_register_removal = "no" *) reg we0_vpn_a, we0_vpn_b, we0_vpn_c;
	(* keep = "true", equivalent_register_removal = "no" *) reg we0_pfn_a, we0_pfn_b, we0_pfn_c, we0_pfn_d;
	(* keep = "true", equivalent_register_removal = "no" *) reg we0_flg_a, we0_flg_b;

	(* keep = "true", equivalent_register_removal = "no" *) reg we1_vpn_a, we1_vpn_b, we1_vpn_c;
	(* keep = "true", equivalent_register_removal = "no" *) reg we1_pfn_a, we1_pfn_b, we1_pfn_c, we1_pfn_d;
	(* keep = "true", equivalent_register_removal = "no" *) reg we1_flg_a, we1_flg_b;

	(* keep = "true", equivalent_register_removal = "no" *) reg we2_vpn_a, we2_vpn_b, we2_vpn_c;
	(* keep = "true", equivalent_register_removal = "no" *) reg we2_pfn_a, we2_pfn_b, we2_pfn_c, we2_pfn_d;
	(* keep = "true", equivalent_register_removal = "no" *) reg we2_flg_a, we2_flg_b;

	(* keep = "true", equivalent_register_removal = "no" *) reg we3_vpn_a, we3_vpn_b, we3_vpn_c;
	(* keep = "true", equivalent_register_removal = "no" *) reg we3_pfn_a, we3_pfn_b, we3_pfn_c, we3_pfn_d;
	(* keep = "true", equivalent_register_removal = "no" *) reg we3_flg_a, we3_flg_b;

	always @(posedge clk_l2 or posedge rst_l2_int) begin
		if (rst_l2_int) begin
			we0_vpn_a <= 1'b0; we0_vpn_b <= 1'b0; we0_vpn_c <= 1'b0;
			we0_pfn_a <= 1'b0; we0_pfn_b <= 1'b0; we0_pfn_c <= 1'b0; we0_pfn_d <= 1'b0;
			we0_flg_a <= 1'b0; we0_flg_b <= 1'b0;

			we1_vpn_a <= 1'b0; we1_vpn_b <= 1'b0; we1_vpn_c <= 1'b0;
			we1_pfn_a <= 1'b0; we1_pfn_b <= 1'b0; we1_pfn_c <= 1'b0; we1_pfn_d <= 1'b0;
			we1_flg_a <= 1'b0; we1_flg_b <= 1'b0;

			we2_vpn_a <= 1'b0; we2_vpn_b <= 1'b0; we2_vpn_c <= 1'b0;
			we2_pfn_a <= 1'b0; we2_pfn_b <= 1'b0; we2_pfn_c <= 1'b0; we2_pfn_d <= 1'b0;
			we2_flg_a <= 1'b0; we2_flg_b <= 1'b0;

			we3_vpn_a <= 1'b0; we3_vpn_b <= 1'b0; we3_vpn_c <= 1'b0;
			we3_pfn_a <= 1'b0; we3_pfn_b <= 1'b0; we3_pfn_c <= 1'b0; we3_pfn_d <= 1'b0;
			we3_flg_a <= 1'b0; we3_flg_b <= 1'b0;
		end else begin
			we0_vpn_a <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_vpn_b <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_vpn_c <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_pfn_a <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_pfn_b <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_pfn_c <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_pfn_d <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_flg_a <= refill_we_w0 & (cur_ptr_w0 == 2'd0);
			we0_flg_b <= refill_we_w0 & (cur_ptr_w0 == 2'd0);

			we1_vpn_a <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_vpn_b <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_vpn_c <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_pfn_a <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_pfn_b <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_pfn_c <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_pfn_d <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_flg_a <= refill_we_w1 & (cur_ptr_w1 == 2'd1);
			we1_flg_b <= refill_we_w1 & (cur_ptr_w1 == 2'd1);

			we2_vpn_a <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_vpn_b <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_vpn_c <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_pfn_a <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_pfn_b <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_pfn_c <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_pfn_d <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_flg_a <= refill_we_w2 & (cur_ptr_w2 == 2'd2);
			we2_flg_b <= refill_we_w2 & (cur_ptr_w2 == 2'd2);

			we3_vpn_a <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_vpn_b <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_vpn_c <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_pfn_a <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_pfn_b <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_pfn_c <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_pfn_d <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_flg_a <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
			we3_flg_b <= refill_we_w3 & (cur_ptr_w3 == 2'd3);
		end
	end

	/* Sliced Way Distributed Dual-Port RAM Arrays using RAM64X1D primitives (WIDTH <= 5) */
	localparam N_VPN_SL = (REM_VPN + 4) / 5;
	localparam N_PFN_SL = (PPN_BITS + 4) / 5;

	wire [REM_VPN-1:0]  dout0_vpn, dout1_vpn, dout2_vpn, dout3_vpn;
	wire [PPN_BITS-1:0] dout0_pfn, dout1_pfn, dout2_pfn, dout3_pfn;

	wire [2:0] dout0_flg_a, dout0_flg_b;
	wire [2:0] dout1_flg_a, dout1_flg_b;
	wire [2:0] dout2_flg_a, dout2_flg_b;
	wire [2:0] dout3_flg_a, dout3_flg_b;

	genvar gvsl, gpsl;
	generate
		for (gvsl = 0; gvsl < N_VPN_SL; gvsl = gvsl + 1) begin : g_vpn_sl
			localparam VB = gvsl * 5;
			localparam VW = (VB + 5 <= REM_VPN) ? 5 : (REM_VPN - VB);
			wire we0_v = (gvsl == 0) ? we0_vpn_a : ((gvsl == 1) ? we0_vpn_b : we0_vpn_c);
			wire we1_v = (gvsl == 0) ? we1_vpn_a : ((gvsl == 1) ? we1_vpn_b : we1_vpn_c);
			wire we2_v = (gvsl == 0) ? we2_vpn_a : ((gvsl == 1) ? we2_vpn_b : we2_vpn_c);
			wire we3_v = (gvsl == 0) ? we3_vpn_a : ((gvsl == 1) ? we3_vpn_b : we3_vpn_c);
			wire [5:0] wa0_v = (gvsl == 0) ? refill_set_w0_vpn_a : ((gvsl == 1) ? refill_set_w0_vpn_b : refill_set_w0_vpn_c);
			wire [5:0] wa1_v = (gvsl == 0) ? refill_set_w1_vpn_a : ((gvsl == 1) ? refill_set_w1_vpn_b : refill_set_w1_vpn_c);
			wire [5:0] wa2_v = (gvsl == 0) ? refill_set_w2_vpn_a : ((gvsl == 1) ? refill_set_w2_vpn_b : refill_set_w2_vpn_c);
			wire [5:0] wa3_v = (gvsl == 0) ? refill_set_w3_vpn_a : ((gvsl == 1) ? refill_set_w3_vpn_b : refill_set_w3_vpn_c);
			atg_ram64x1d_array #(.WIDTH(VW)) u_ram_w0_vpn (.clk(clk_l2), .we(we0_v), .waddr(wa0_v), .din(r_ref_vpn[VB +: VW]), .raddr(lookup_set_w0_vpn), .dout(dout0_vpn[VB +: VW]));
			atg_ram64x1d_array #(.WIDTH(VW)) u_ram_w1_vpn (.clk(clk_l2), .we(we1_v), .waddr(wa1_v), .din(r_ref_vpn[VB +: VW]), .raddr(lookup_set_w1_vpn), .dout(dout1_vpn[VB +: VW]));
			atg_ram64x1d_array #(.WIDTH(VW)) u_ram_w2_vpn (.clk(clk_l2), .we(we2_v), .waddr(wa2_v), .din(r_ref_vpn[VB +: VW]), .raddr(lookup_set_w2_vpn), .dout(dout2_vpn[VB +: VW]));
			atg_ram64x1d_array #(.WIDTH(VW)) u_ram_w3_vpn (.clk(clk_l2), .we(we3_v), .waddr(wa3_v), .din(r_ref_vpn[VB +: VW]), .raddr(lookup_set_w3_vpn), .dout(dout3_vpn[VB +: VW]));
		end

		for (gpsl = 0; gpsl < N_PFN_SL; gpsl = gpsl + 1) begin : g_pfn_sl
			localparam PB = gpsl * 5;
			localparam PW = (PB + 5 <= PPN_BITS) ? 5 : (PPN_BITS - PB);
			localparam IS_HI = (gpsl >= (N_PFN_SL / 2)) ? 1 : 0;
			wire we0_p = (gpsl == 0) ? we0_pfn_a : ((gpsl == 1) ? we0_pfn_b : ((gpsl == 2) ? we0_pfn_c : we0_pfn_d));
			wire we1_p = (gpsl == 0) ? we1_pfn_a : ((gpsl == 1) ? we1_pfn_b : ((gpsl == 2) ? we1_pfn_c : we1_pfn_d));
			wire we2_p = (gpsl == 0) ? we2_pfn_a : ((gpsl == 1) ? we2_pfn_b : ((gpsl == 2) ? we2_pfn_c : we2_pfn_d));
			wire we3_p = (gpsl == 0) ? we3_pfn_a : ((gpsl == 1) ? we3_pfn_b : ((gpsl == 2) ? we3_pfn_c : we3_pfn_d));
			wire [5:0] wa0_p = (gpsl == 0) ? refill_set_w0_pfn_a : ((gpsl == 1) ? refill_set_w0_pfn_b : ((gpsl == 2) ? refill_set_w0_pfn_c : refill_set_w0_pfn_d));
			wire [5:0] wa1_p = (gpsl == 0) ? refill_set_w1_pfn_a : ((gpsl == 1) ? refill_set_w1_pfn_b : ((gpsl == 2) ? refill_set_w1_pfn_c : refill_set_w1_pfn_d));
			wire [5:0] wa2_p = (gpsl == 0) ? refill_set_w2_pfn_a : ((gpsl == 1) ? refill_set_w2_pfn_b : ((gpsl == 2) ? refill_set_w2_pfn_c : refill_set_w2_pfn_d));
			wire [5:0] wa3_p = (gpsl == 0) ? refill_set_w3_pfn_a : ((gpsl == 1) ? refill_set_w3_pfn_b : ((gpsl == 2) ? refill_set_w3_pfn_c : refill_set_w3_pfn_d));
			wire [5:0] ra0_p = IS_HI ? lookup_set_w0_pfn_hi : lookup_set_w0_pfn_lo;
			wire [5:0] ra1_p = IS_HI ? lookup_set_w1_pfn_hi : lookup_set_w1_pfn_lo;
			wire [5:0] ra2_p = IS_HI ? lookup_set_w2_pfn_hi : lookup_set_w2_pfn_lo;
			wire [5:0] ra3_p = IS_HI ? lookup_set_w3_pfn_hi : lookup_set_w3_pfn_lo;
			atg_ram64x1d_array #(.WIDTH(PW)) u_ram_w0_pfn (.clk(clk_l2), .we(we0_p), .waddr(wa0_p), .din(r_ref_pfn[PB +: PW]), .raddr(ra0_p), .dout(dout0_pfn[PB +: PW]));
			atg_ram64x1d_array #(.WIDTH(PW)) u_ram_w1_pfn (.clk(clk_l2), .we(we1_p), .waddr(wa1_p), .din(r_ref_pfn[PB +: PW]), .raddr(ra1_p), .dout(dout1_pfn[PB +: PW]));
			atg_ram64x1d_array #(.WIDTH(PW)) u_ram_w2_pfn (.clk(clk_l2), .we(we2_p), .waddr(wa2_p), .din(r_ref_pfn[PB +: PW]), .raddr(ra2_p), .dout(dout2_pfn[PB +: PW]));
			atg_ram64x1d_array #(.WIDTH(PW)) u_ram_w3_pfn (.clk(clk_l2), .we(we3_p), .waddr(wa3_p), .din(r_ref_pfn[PB +: PW]), .raddr(ra3_p), .dout(dout3_pfn[PB +: PW]));
		end
	endgenerate

	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w0_flg_a (.clk(clk_l2), .we(we0_flg_a), .waddr(refill_set_w0_flg_a), .din(r_ref_flags[2:0]), .raddr(lookup_set_w0_flg),    .dout(dout0_flg_a));
	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w0_flg_b (.clk(clk_l2), .we(we0_flg_b), .waddr(refill_set_w0_flg_b), .din(r_ref_flags[5:3]), .raddr(lookup_set_w0_flg),    .dout(dout0_flg_b));

	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w1_flg_a (.clk(clk_l2), .we(we1_flg_a), .waddr(refill_set_w1_flg_a), .din(r_ref_flags[2:0]), .raddr(lookup_set_w1_flg),    .dout(dout1_flg_a));
	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w1_flg_b (.clk(clk_l2), .we(we1_flg_b), .waddr(refill_set_w1_flg_b), .din(r_ref_flags[5:3]), .raddr(lookup_set_w1_flg),    .dout(dout1_flg_b));

	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w2_flg_a (.clk(clk_l2), .we(we2_flg_a), .waddr(refill_set_w2_flg_a), .din(r_ref_flags[2:0]), .raddr(lookup_set_w2_flg),    .dout(dout2_flg_a));
	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w2_flg_b (.clk(clk_l2), .we(we2_flg_b), .waddr(refill_set_w2_flg_b), .din(r_ref_flags[5:3]), .raddr(lookup_set_w2_flg),    .dout(dout2_flg_b));

	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w3_flg_a (.clk(clk_l2), .we(we3_flg_a), .waddr(refill_set_w3_flg_a), .din(r_ref_flags[2:0]), .raddr(lookup_set_w3_flg),    .dout(dout3_flg_a));
	atg_ram64x1d_array #(.WIDTH(3)) u_ram_w3_flg_b (.clk(clk_l2), .we(we3_flg_b), .waddr(refill_set_w3_flg_b), .din(r_ref_flags[5:3]), .raddr(lookup_set_w3_flg),    .dout(dout3_flg_b));

	/* Replicated buffer trees for slice read selects (<= 10 fanout per buffer) */
	wire [1:0] l2_set_w0_sl_g [0:7];
	wire [1:0] l2_set_w1_sl_g [0:7];
	wire [1:0] l2_set_w2_sl_g [0:7];
	wire [1:0] l2_set_w3_sl_g [0:7];

	/* Buffered upper set bits for 16:1 muxes (<= 12 fanout per buffer) */
	wire [3:0] l2_set_w0_sl_hi_a, l2_set_w0_sl_hi_b;
	wire [3:0] l2_set_w1_sl_hi_a, l2_set_w1_sl_hi_b;
	wire [3:0] l2_set_w2_sl_hi_a, l2_set_w2_sl_hi_b;
	wire [3:0] l2_set_w3_sl_hi_a, l2_set_w3_sl_hi_b;

	/* Buffered refill set bits for 16 slices (<= 16 fanout per buffer) */
	wire [1:0] ref_set_in_g [0:7];
	wire [3:0] ref_set_hi_g [0:7];

	genvar gsb, gsb_i;
	generate
`ifdef SYNTH_KINTEX7
		for (gsb = 0; gsb < 8; gsb = gsb + 1) begin : g_sl_buf8
			for (gsb_i = 0; gsb_i < 2; gsb_i = gsb_i + 1) begin : g_b2
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw0s (.I0(lookup_set_w0_sl[gsb_i]), .O(l2_set_w0_sl_g[gsb][gsb_i]));
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw1s (.I0(lookup_set_w1_sl[gsb_i]), .O(l2_set_w1_sl_g[gsb][gsb_i]));
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw2s (.I0(lookup_set_w2_sl[gsb_i]), .O(l2_set_w2_sl_g[gsb][gsb_i]));
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw3s (.I0(lookup_set_w3_sl[gsb_i]), .O(l2_set_w3_sl_g[gsb][gsb_i]));
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_brs  (.I0(refill_set_in[gsb_i]),    .O(ref_set_in_g[gsb][gsb_i]));
			end
			for (gsb_i = 0; gsb_i < 4; gsb_i = gsb_i + 1) begin : g_b4
				(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_brh  (.I0(refill_set_in[2+gsb_i]),  .O(ref_set_hi_g[gsb][gsb_i]));
			end
		end
		for (gsb_i = 0; gsb_i < 4; gsb_i = gsb_i + 1) begin : g_sl_hibuf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw0ha (.I0(lookup_set_w0_sl[2+gsb_i]), .O(l2_set_w0_sl_hi_a[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw0hb (.I0(lookup_set_w0_sl[2+gsb_i]), .O(l2_set_w0_sl_hi_b[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw1ha (.I0(lookup_set_w1_sl[2+gsb_i]), .O(l2_set_w1_sl_hi_a[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw1hb (.I0(lookup_set_w1_sl[2+gsb_i]), .O(l2_set_w1_sl_hi_b[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw2ha (.I0(lookup_set_w2_sl[2+gsb_i]), .O(l2_set_w2_sl_hi_a[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw2hb (.I0(lookup_set_w2_sl[2+gsb_i]), .O(l2_set_w2_sl_hi_b[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw3ha (.I0(lookup_set_w3_sl[2+gsb_i]), .O(l2_set_w3_sl_hi_a[gsb_i]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_bw3hb (.I0(lookup_set_w3_sl[2+gsb_i]), .O(l2_set_w3_sl_hi_b[gsb_i]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gsb = 0; gsb < 8; gsb = gsb + 1) begin : g_sl_buf8
			for (gsb_i = 0; gsb_i < 2; gsb_i = gsb_i + 1) begin : g_b2
				assign l2_set_w0_sl_g[gsb][gsb_i] = lookup_set_w0_sl[gsb_i];
				assign l2_set_w1_sl_g[gsb][gsb_i] = lookup_set_w1_sl[gsb_i];
				assign l2_set_w2_sl_g[gsb][gsb_i] = lookup_set_w2_sl[gsb_i];
				assign l2_set_w3_sl_g[gsb][gsb_i] = lookup_set_w3_sl[gsb_i];
				assign ref_set_in_g[gsb][gsb_i] = refill_set_in[gsb_i];
			end
			for (gsb_i = 0; gsb_i < 4; gsb_i = gsb_i + 1) begin : g_b4
				assign ref_set_hi_g[gsb][gsb_i] = refill_set_in[2+gsb_i];
			end
		end
		for (gsb_i = 0; gsb_i < 4; gsb_i = gsb_i + 1) begin : g_sl_hibuf
			assign l2_set_w0_sl_hi_a[gsb_i] = lookup_set_w0_sl[2+gsb_i];
			assign l2_set_w0_sl_hi_b[gsb_i] = lookup_set_w0_sl[2+gsb_i];
			assign l2_set_w1_sl_hi_a[gsb_i] = lookup_set_w1_sl[2+gsb_i];
			assign l2_set_w1_sl_hi_b[gsb_i] = lookup_set_w1_sl[2+gsb_i];
			assign l2_set_w2_sl_hi_a[gsb_i] = lookup_set_w2_sl[2+gsb_i];
			assign l2_set_w2_sl_hi_b[gsb_i] = lookup_set_w2_sl[2+gsb_i];
			assign l2_set_w3_sl_hi_a[gsb_i] = lookup_set_w3_sl[2+gsb_i];
			assign l2_set_w3_sl_hi_b[gsb_i] = lookup_set_w3_sl[2+gsb_i];
		end
`else
		for (gsb = 0; gsb < 8; gsb = gsb + 1) begin : g_sl_buf8
			assign l2_set_w0_sl_g[gsb] = lookup_set_w0_sl[1:0];
			assign l2_set_w1_sl_g[gsb] = lookup_set_w1_sl[1:0];
			assign l2_set_w2_sl_g[gsb] = lookup_set_w2_sl[1:0];
			assign l2_set_w3_sl_g[gsb] = lookup_set_w3_sl[1:0];
			assign ref_set_in_g[gsb]   = refill_set_in[1:0];
			assign ref_set_hi_g[gsb]   = refill_set_in[5:2];
		end
		assign l2_set_w0_sl_hi_a = lookup_set_w0_sl[5:2];
		assign l2_set_w0_sl_hi_b = lookup_set_w0_sl[5:2];
		assign l2_set_w1_sl_hi_a = lookup_set_w1_sl[5:2];
		assign l2_set_w1_sl_hi_b = lookup_set_w1_sl[5:2];
		assign l2_set_w2_sl_hi_a = lookup_set_w2_sl[5:2];
		assign l2_set_w2_sl_hi_b = lookup_set_w2_sl[5:2];
		assign l2_set_w3_sl_hi_a = lookup_set_w3_sl[5:2];
		assign l2_set_w3_sl_hi_b = lookup_set_w3_sl[5:2];
`endif
	endgenerate


	/* 16 Slices: Distributed Flush registers, Replacement pointers, and Valid/Session storage */
	wire [15:0] sl_v0_out, sl_v1_out, sl_v2_out, sl_v3_out;
	wire [63:0] sl_sess0_out, sl_sess1_out, sl_sess2_out, sl_sess3_out;

	genvar gsl;
	generate
	for (gsl = 0; gsl < 16; gsl = gsl + 1) begin : g_slice
		wire [3:0] sl_id = gsl[3:0];

		(* max_fanout = 16 *) reg [3:0] r_sl_sess;
		(* max_fanout = 16 *) reg       r_sl_trig;
		(* max_fanout = 16 *) reg       r_sl_all;

		always @(posedge clk_l2 or posedge rst_l2_int) begin
			if (rst_l2_int) begin
				r_sl_sess <= 4'd0;
				r_sl_trig <= 1'b0;
				r_sl_all  <= 1'b0;
			end else begin
				r_sl_sess <= flush_session;
				r_sl_trig <= flush_trigger;
				r_sl_all  <= flush_all;
			end
		end

		/* 4 sets of replacement pointers per slice */
		reg [1:0] sl_alloc_ptr [0:3];

		/* 4 sets of valid bits and session IDs */
		reg [3:0] v0, v1, v2, v3;
		reg [3:0] sess0 [0:3];
		reg [3:0] sess1 [0:3];
		reg [3:0] sess2 [0:3];
		reg [3:0] sess3 [0:3];

		wire [1:0] sl_next_ptr = (gsl < 4)  ? next_ptr_g0 :
		                         (gsl < 8)  ? next_ptr_g1 :
		                         (gsl < 12) ? next_ptr_g2 : next_ptr_g3;

		wire [1:0] sl_cur_ref_lo = ref_set_in_g[gsl >> 1];
		wire [3:0] sl_cur_ref_hi = ref_set_hi_g[gsl >> 1];

		wire [1:0] sl_cur_ptr_local =
			(gsl < 2)  ? r_cur_ptr_g0 :
			(gsl < 4)  ? r_cur_ptr_g1 :
			(gsl < 6)  ? r_cur_ptr_g2 :
			(gsl < 8)  ? r_cur_ptr_g3 :
			(gsl < 10) ? r_cur_ptr_g4 :
			(gsl < 12) ? r_cur_ptr_g5 :
			(gsl < 14) ? r_cur_ptr_g6 : r_cur_ptr_g7;

		wire sl_ref_we_in = (gsl < 4)  ? refill_we_sl0 :
		                    (gsl < 8)  ? refill_we_sl1 :
		                    (gsl < 12) ? refill_we_sl2 : refill_we_sl3;

		/* Local registered match and lower set address to isolate fanout */
		(* keep = "true" *) reg [1:0] sl_cur_set_lo;
		(* keep = "true" *) reg       sl_match_w01;
		(* keep = "true" *) reg       sl_match_w23;
		(* keep = "true" *) reg       sl_match_ptr;

		always @(posedge clk_l2 or posedge rst_l2_int) begin
			if (rst_l2_int) begin
				sl_cur_set_lo <= 2'd0;
				sl_match_w01  <= 1'b0;
				sl_match_w23  <= 1'b0;
				sl_match_ptr  <= 1'b0;
			end else begin
				sl_cur_set_lo <= sl_cur_ref_lo;
				sl_match_w01  <= (sl_cur_ref_hi == sl_id) & sl_ref_we_in;
				sl_match_w23  <= (sl_cur_ref_hi == sl_id) & sl_ref_we_in;
				sl_match_ptr  <= (sl_cur_ref_hi == sl_id) & sl_ref_we_in;
			end
		end

		wire we_sl0 = sl_match_w01 & (sl_cur_ptr_local == 2'd0);
		wire we_sl1 = sl_match_w01 & (sl_cur_ptr_local == 2'd1);
		wire we_sl2 = sl_match_w23 & (sl_cur_ptr_local == 2'd2);
		wire we_sl3 = sl_match_w23 & (sl_cur_ptr_local == 2'd3);

		integer si;
		always @(posedge clk_l2 or posedge rst_l2_int) begin
			if (rst_l2_int) begin
				sl_alloc_ptr[0] <= 2'd0;
				sl_alloc_ptr[1] <= 2'd0;
				sl_alloc_ptr[2] <= 2'd0;
				sl_alloc_ptr[3] <= 2'd0;
				v0 <= 4'd0;
				v1 <= 4'd0;
				v2 <= 4'd0;
				v3 <= 4'd0;
				for (si = 0; si < 4; si = si + 1) begin
					sess0[si] <= 4'd0;
					sess1[si] <= 4'd0;
					sess2[si] <= 4'd0;
					sess3[si] <= 4'd0;
				end
			end else begin
				if (sl_match_ptr) begin
					sl_alloc_ptr[sl_cur_set_lo] <= sl_next_ptr;
				end

				if (r_sl_all) begin
					v0 <= 4'd0;
					v1 <= 4'd0;
					v2 <= 4'd0;
					v3 <= 4'd0;
				end else if (r_sl_trig) begin
					for (si = 0; si < 4; si = si + 1) begin
						if (sess0[si] == r_sl_sess) v0[si] <= 1'b0;
						if (sess1[si] == r_sl_sess) v1[si] <= 1'b0;
						if (sess2[si] == r_sl_sess) v2[si] <= 1'b0;
						if (sess3[si] == r_sl_sess) v3[si] <= 1'b0;
					end
				end

				if (we_sl0) begin
					v0[sl_cur_set_lo]    <= 1'b1;
					sess0[sl_cur_set_lo] <= r_ref_sess;
				end
				if (we_sl1) begin
					v1[sl_cur_set_lo]    <= 1'b1;
					sess1[sl_cur_set_lo] <= r_ref_sess;
				end
				if (we_sl2) begin
					v2[sl_cur_set_lo]    <= 1'b1;
					sess2[sl_cur_set_lo] <= r_ref_sess;
				end
				if (we_sl3) begin
					v3[sl_cur_set_lo]    <= 1'b1;
					sess3[sl_cur_set_lo] <= r_ref_sess;
				end
			end
		end

		wire [1:0] sl_r_set0 = l2_set_w0_sl_g[gsl >> 1];
		wire [1:0] sl_r_set1 = l2_set_w1_sl_g[gsl >> 1];
		wire [1:0] sl_r_set2 = l2_set_w2_sl_g[gsl >> 1];
		wire [1:0] sl_r_set3 = l2_set_w3_sl_g[gsl >> 1];

		/* Read 4:1 multiplexer within slice using buffered set selects */
		assign sl_cur_ptr_out[gsl*2 +: 2] = sl_alloc_ptr[refill_set_in[1:0]];
		assign sl_v0_out[gsl]             = v0[sl_r_set0];
		assign sl_v1_out[gsl]             = v1[sl_r_set1];
		assign sl_v2_out[gsl]             = v2[sl_r_set2];
		assign sl_v3_out[gsl]             = v3[sl_r_set3];

		assign sl_sess0_out[gsl*4 +: 4]   = sess0[sl_r_set0];
		assign sl_sess1_out[gsl*4 +: 4]   = sess1[sl_r_set1];
		assign sl_sess2_out[gsl*4 +: 4]   = sess2[sl_r_set2];
		assign sl_sess3_out[gsl*4 +: 4]   = sess3[sl_r_set3];
	end
	endgenerate

	/* 16:1 multiplexer across slices with split address buffers (<= 12 fanout) */
	wire read_v0 = sl_v0_out[l2_set_w0_sl_hi_a];
	wire read_v1 = sl_v1_out[l2_set_w1_sl_hi_a];
	wire read_v2 = sl_v2_out[l2_set_w2_sl_hi_a];
	wire read_v3 = sl_v3_out[l2_set_w3_sl_hi_a];

	wire [1:0] read_sess0_lo = sl_sess0_out[{l2_set_w0_sl_hi_a, 2'b00} +: 2];
	wire [1:0] read_sess0_hi = sl_sess0_out[{l2_set_w0_sl_hi_b, 2'b10} +: 2];
	wire [3:0] read_sess0    = {read_sess0_hi, read_sess0_lo};

	wire [1:0] read_sess1_lo = sl_sess1_out[{l2_set_w1_sl_hi_a, 2'b00} +: 2];
	wire [1:0] read_sess1_hi = sl_sess1_out[{l2_set_w1_sl_hi_b, 2'b10} +: 2];
	wire [3:0] read_sess1    = {read_sess1_hi, read_sess1_lo};

	wire [1:0] read_sess2_lo = sl_sess2_out[{l2_set_w2_sl_hi_a, 2'b00} +: 2];
	wire [1:0] read_sess2_hi = sl_sess2_out[{l2_set_w2_sl_hi_b, 2'b10} +: 2];
	wire [3:0] read_sess2    = {read_sess2_hi, read_sess2_lo};

	wire [1:0] read_sess3_lo = sl_sess3_out[{l2_set_w3_sl_hi_a, 2'b00} +: 2];
	wire [1:0] read_sess3_hi = sl_sess3_out[{l2_set_w3_sl_hi_b, 2'b10} +: 2];
	wire [3:0] read_sess3    = {read_sess3_hi, read_sess3_lo};

	/* Combined flag outputs from RAM slices */
	wire [5:0] dout0_flags = {dout0_flg_b, dout0_flg_a};
	wire [5:0] dout1_flags = {dout1_flg_b, dout1_flg_a};
	wire [5:0] dout2_flags = {dout2_flg_b, dout2_flg_a};
	wire [5:0] dout3_flags = {dout3_flg_b, dout3_flg_a};

	/* Stage 1: Pipeline registers for Set Read (Clean 1-cycle boundary) */
	(* max_fanout = 16 *) reg                r1_valid;
	(* max_fanout = 16 *) reg [TAG_W-1:0]    r1_lookup_tag;
	(* max_fanout = 16 *) reg                r1_v0, r1_v1, r1_v2, r1_v3;
	(* max_fanout = 16 *) reg [TAG_W-1:0]    r1_tag0, r1_tag1, r1_tag2, r1_tag3;
	(* max_fanout = 16 *) reg [PPN_BITS-1:0] r1_pfn0, r1_pfn1, r1_pfn2, r1_pfn3;
	(* max_fanout = 16 *) reg [5:0]          r1_flags0, r1_flags1, r1_flags2, r1_flags3;

	always @(posedge clk_l2 or posedge rst_l2_int) begin
		if (rst_l2_int) begin
			r1_valid      <= 1'b0;
			r1_lookup_tag <= {TAG_W{1'b0}};
			r1_v0         <= 1'b0;
			r1_v1         <= 1'b0;
			r1_v2         <= 1'b0;
			r1_v3         <= 1'b0;
		end else begin
			r1_valid      <= lookup_valid;
			r1_lookup_tag <= lookup_tag;
			r1_v0         <= read_v0;
			r1_v1         <= read_v1;
			r1_v2         <= read_v2;
			r1_v3         <= read_v3;
		end
	end

	/* Data payload from RAM64X1D primitives (0 RAM output logic levels) */
	always @(posedge clk_l2) begin
		r1_tag0   <= {read_sess0, dout0_vpn};
		r1_tag1   <= {read_sess1, dout1_vpn};
		r1_tag2   <= {read_sess2, dout2_vpn};
		r1_tag3   <= {read_sess3, dout3_vpn};
		r1_pfn0   <= dout0_pfn;
		r1_pfn1   <= dout1_pfn;
		r1_pfn2   <= dout2_pfn;
		r1_pfn3   <= dout3_pfn;
		r1_flags0 <= dout0_flags;
		r1_flags1 <= dout1_flags;
		r1_flags2 <= dout2_flags;
		r1_flags3 <= dout3_flags;
	end

	/* Stage 2: 4-Way Compare (<= 3 LUT levels) */
	localparam N_TEQ    = (TAG_W + 2) / 3;
	localparam N_TEQ_LO = (N_TEQ + 1) / 2;
	wire [N_TEQ-1:0] eq0_c, eq1_c, eq2_c, eq3_c;
	genvar gl2c;
	generate
		for (gl2c = 0; gl2c < N_TEQ; gl2c = gl2c + 1) begin : g_l2eq
			localparam TB = gl2c * 3;
			localparam TW = (TB + 3 <= TAG_W) ? 3 : (TAG_W - TB);
`ifdef SYNTH_KINTEX7
			if (TW == 3) begin : g_eq3
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq0 (
					.I0(r1_tag0[TB]), .I1(r1_tag0[TB+1]), .I2(r1_tag0[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq0_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq1 (
					.I0(r1_tag1[TB]), .I1(r1_tag1[TB+1]), .I2(r1_tag1[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq1_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq2 (
					.I0(r1_tag2[TB]), .I1(r1_tag2[TB+1]), .I2(r1_tag2[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq2_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq3 (
					.I0(r1_tag3[TB]), .I1(r1_tag3[TB+1]), .I2(r1_tag3[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq3_c[gl2c])
				);
			end else begin : g_eqs
				assign eq0_c[gl2c] = (r1_tag0[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq1_c[gl2c] = (r1_tag1[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq2_c[gl2c] = (r1_tag2[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq3_c[gl2c] = (r1_tag3[TB +: TW] == r1_lookup_tag[TB +: TW]);
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (TW == 3) begin : g_eq3
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq0 (
					.I0(r1_tag0[TB]), .I1(r1_tag0[TB+1]), .I2(r1_tag0[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq0_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq1 (
					.I0(r1_tag1[TB]), .I1(r1_tag1[TB+1]), .I2(r1_tag1[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq1_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq2 (
					.I0(r1_tag2[TB]), .I1(r1_tag2[TB+1]), .I2(r1_tag2[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq2_c[gl2c])
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq3 (
					.I0(r1_tag3[TB]), .I1(r1_tag3[TB+1]), .I2(r1_tag3[TB+2]),
					.I3(r1_lookup_tag[TB]), .I4(r1_lookup_tag[TB+1]), .I5(r1_lookup_tag[TB+2]), .O(eq3_c[gl2c])
				);
			end else begin : g_eqs
				assign eq0_c[gl2c] = (r1_tag0[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq1_c[gl2c] = (r1_tag1[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq2_c[gl2c] = (r1_tag2[TB +: TW] == r1_lookup_tag[TB +: TW]);
				assign eq3_c[gl2c] = (r1_tag3[TB +: TW] == r1_lookup_tag[TB +: TW]);
			end
`else
			assign eq0_c[gl2c] = (r1_tag0[TB +: TW] == r1_lookup_tag[TB +: TW]);
			assign eq1_c[gl2c] = (r1_tag1[TB +: TW] == r1_lookup_tag[TB +: TW]);
			assign eq2_c[gl2c] = (r1_tag2[TB +: TW] == r1_lookup_tag[TB +: TW]);
			assign eq3_c[gl2c] = (r1_tag3[TB +: TW] == r1_lookup_tag[TB +: TW]);
`endif
		end
	endgenerate

	wire m0_lo = &eq0_c[N_TEQ_LO-1:0];
	wire m0_hi = (&eq0_c[N_TEQ-1:N_TEQ_LO]) & r1_v0;

	wire m1_lo = &eq1_c[N_TEQ_LO-1:0];
	wire m1_hi = (&eq1_c[N_TEQ-1:N_TEQ_LO]) & r1_v1;

	wire m2_lo = &eq2_c[N_TEQ_LO-1:0];
	wire m2_hi = (&eq2_c[N_TEQ-1:N_TEQ_LO]) & r1_v2;

	wire m3_lo = &eq3_c[N_TEQ_LO-1:0];
	wire m3_hi = (&eq3_c[N_TEQ-1:N_TEQ_LO]) & r1_v3;

	wire m0_hit, m1_hit, m2_hit, m3_hit;
	wire [PPN_BITS-1:0] next_pfn;
	wire [5:0] next_flags;

`ifdef SYNTH_KINTEX7
	wire m0_lo_a, m0_lo_b, m0_hi_a, m0_hi_b;
	wire m1_lo_a, m1_lo_b, m1_hi_a, m1_hi_b;
	wire m2_lo_a, m2_lo_b, m2_hi_a, m2_hi_b;
	wire m3_lo_a, m3_lo_b, m3_hi_a, m3_hi_b;
	generate
	if (N_TEQ == 6) begin : g_m_lut
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m0la (.I0(eq0_c[0]), .I1(eq0_c[1]), .I2(eq0_c[2]), .O(m0_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m0lb (.I0(eq0_c[0]), .I1(eq0_c[1]), .I2(eq0_c[2]), .O(m0_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m0ha (.I0(eq0_c[3]), .I1(eq0_c[4]), .I2(eq0_c[5]), .I3(r1_v0), .O(m0_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m0hb (.I0(eq0_c[3]), .I1(eq0_c[4]), .I2(eq0_c[5]), .I3(r1_v0), .O(m0_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m1la (.I0(eq1_c[0]), .I1(eq1_c[1]), .I2(eq1_c[2]), .O(m1_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m1lb (.I0(eq1_c[0]), .I1(eq1_c[1]), .I2(eq1_c[2]), .O(m1_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m1ha (.I0(eq1_c[3]), .I1(eq1_c[4]), .I2(eq1_c[5]), .I3(r1_v1), .O(m1_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m1hb (.I0(eq1_c[3]), .I1(eq1_c[4]), .I2(eq1_c[5]), .I3(r1_v1), .O(m1_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m2la (.I0(eq2_c[0]), .I1(eq2_c[1]), .I2(eq2_c[2]), .O(m2_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m2lb (.I0(eq2_c[0]), .I1(eq2_c[1]), .I2(eq2_c[2]), .O(m2_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m2ha (.I0(eq2_c[3]), .I1(eq2_c[4]), .I2(eq2_c[5]), .I3(r1_v2), .O(m2_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m2hb (.I0(eq2_c[3]), .I1(eq2_c[4]), .I2(eq2_c[5]), .I3(r1_v2), .O(m2_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m3la (.I0(eq3_c[0]), .I1(eq3_c[1]), .I2(eq3_c[2]), .O(m3_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m3lb (.I0(eq3_c[0]), .I1(eq3_c[1]), .I2(eq3_c[2]), .O(m3_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m3ha (.I0(eq3_c[3]), .I1(eq3_c[4]), .I2(eq3_c[5]), .I3(r1_v3), .O(m3_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m3hb (.I0(eq3_c[3]), .I1(eq3_c[4]), .I2(eq3_c[5]), .I3(r1_v3), .O(m3_hi_b));
	end else begin : g_m_gen
		assign m0_lo_a = m0_lo; assign m0_lo_b = m0_lo; assign m0_hi_a = m0_hi; assign m0_hi_b = m0_hi;
		assign m1_lo_a = m1_lo; assign m1_lo_b = m1_lo; assign m1_hi_a = m1_hi; assign m1_hi_b = m1_hi;
		assign m2_lo_a = m2_lo; assign m2_lo_b = m2_lo; assign m2_hi_a = m2_hi; assign m2_hi_b = m2_hi;
		assign m3_lo_a = m3_lo; assign m3_lo_b = m3_lo; assign m3_hi_a = m3_hi; assign m3_hi_b = m3_hi;
	end
	endgenerate

	assign m0_hit = m0_lo_b & m0_hi_b;
	assign m1_hit = m1_lo_b & m1_hi_b;
	assign m2_hit = m2_lo_b & m2_hi_b;
	assign m3_hit = m3_lo_b & m3_hi_b;

	wire next_hit = r1_valid & (m0_hit | m1_hit | m2_hit | m3_hit);

	genvar gpb, gfb;
	generate
		for (gpb = 0; gpb < PPN_BITS; gpb = gpb + 1) begin : g_l2_pfn_mux
			wire p01, p23;
			wire x0_lo = (gpb < 14) ? m0_lo_a : m0_lo_b;
			wire x0_hi = (gpb < 14) ? m0_hi_a : m0_hi_b;
			wire x1_lo = (gpb < 14) ? m1_lo_a : m1_lo_b;
			wire x1_hi = (gpb < 14) ? m1_hi_a : m1_hi_b;
			wire x2_lo = (gpb < 14) ? m2_lo_a : m2_lo_b;
			wire x2_hi = (gpb < 14) ? m2_hi_a : m2_hi_b;
			wire x3_lo = (gpb < 14) ? m3_lo_a : m3_lo_b;
			wire x3_hi = (gpb < 14) ? m3_hi_a : m3_hi_b;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_p01 (
				.I0(x0_lo), .I1(x0_hi), .I2(r1_pfn0[gpb]),
				.I3(x1_lo), .I4(x1_hi), .I5(r1_pfn1[gpb]),
				.O(p01)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_p23 (
				.I0(x2_lo), .I1(x2_hi), .I2(r1_pfn2[gpb]),
				.I3(x3_lo), .I4(x3_hi), .I5(r1_pfn3[gpb]),
				.O(p23)
			);
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_por (.I0(p01), .I1(p23), .O(next_pfn[gpb]));
		end
		for (gfb = 0; gfb < 6; gfb = gfb + 1) begin : g_l2_flg_mux
			wire f01, f23;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_f01 (
				.I0(m0_lo_b), .I1(m0_hi_b), .I2(r1_flags0[gfb]),
				.I3(m1_lo_b), .I4(m1_hi_b), .I5(r1_flags1[gfb]),
				.O(f01)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_f23 (
				.I0(m2_lo_b), .I1(m2_hi_b), .I2(r1_flags2[gfb]),
				.I3(m3_lo_b), .I4(m3_hi_b), .I5(r1_flags3[gfb]),
				.O(f23)
			);
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_for (.I0(f01), .I1(f23), .O(next_flags[gfb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire m0_lo_a, m0_lo_b, m0_hi_a, m0_hi_b;
	wire m1_lo_a, m1_lo_b, m1_hi_a, m1_hi_b;
	wire m2_lo_a, m2_lo_b, m2_hi_a, m2_hi_b;
	wire m3_lo_a, m3_lo_b, m3_hi_a, m3_hi_b;
	generate
	if (N_TEQ == 6) begin : g_m_lut
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m0la (.I0(eq0_c[0]), .I1(eq0_c[1]), .I2(eq0_c[2]), .O(m0_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m0lb (.I0(eq0_c[0]), .I1(eq0_c[1]), .I2(eq0_c[2]), .O(m0_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m0ha (.I0(eq0_c[3]), .I1(eq0_c[4]), .I2(eq0_c[5]), .I3(r1_v0), .O(m0_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m0hb (.I0(eq0_c[3]), .I1(eq0_c[4]), .I2(eq0_c[5]), .I3(r1_v0), .O(m0_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m1la (.I0(eq1_c[0]), .I1(eq1_c[1]), .I2(eq1_c[2]), .O(m1_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m1lb (.I0(eq1_c[0]), .I1(eq1_c[1]), .I2(eq1_c[2]), .O(m1_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m1ha (.I0(eq1_c[3]), .I1(eq1_c[4]), .I2(eq1_c[5]), .I3(r1_v1), .O(m1_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m1hb (.I0(eq1_c[3]), .I1(eq1_c[4]), .I2(eq1_c[5]), .I3(r1_v1), .O(m1_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m2la (.I0(eq2_c[0]), .I1(eq2_c[1]), .I2(eq2_c[2]), .O(m2_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m2lb (.I0(eq2_c[0]), .I1(eq2_c[1]), .I2(eq2_c[2]), .O(m2_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m2ha (.I0(eq2_c[3]), .I1(eq2_c[4]), .I2(eq2_c[5]), .I3(r1_v2), .O(m2_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m2hb (.I0(eq2_c[3]), .I1(eq2_c[4]), .I2(eq2_c[5]), .I3(r1_v2), .O(m2_hi_b));

		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m3la (.I0(eq3_c[0]), .I1(eq3_c[1]), .I2(eq3_c[2]), .O(m3_lo_a));
		(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_m3lb (.I0(eq3_c[0]), .I1(eq3_c[1]), .I2(eq3_c[2]), .O(m3_lo_b));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m3ha (.I0(eq3_c[3]), .I1(eq3_c[4]), .I2(eq3_c[5]), .I3(r1_v3), .O(m3_hi_a));
		(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8000)) u_m3hb (.I0(eq3_c[3]), .I1(eq3_c[4]), .I2(eq3_c[5]), .I3(r1_v3), .O(m3_hi_b));
	end else begin : g_m_gen
		assign m0_lo_a = m0_lo; assign m0_lo_b = m0_lo; assign m0_hi_a = m0_hi; assign m0_hi_b = m0_hi;
		assign m1_lo_a = m1_lo; assign m1_lo_b = m1_lo; assign m1_hi_a = m1_hi; assign m1_hi_b = m1_hi;
		assign m2_lo_a = m2_lo; assign m2_lo_b = m2_lo; assign m2_hi_a = m2_hi; assign m2_hi_b = m2_hi;
		assign m3_lo_a = m3_lo; assign m3_lo_b = m3_lo; assign m3_hi_a = m3_hi; assign m3_hi_b = m3_hi;
	end
	endgenerate

	assign m0_hit = m0_lo_b & m0_hi_b;
	assign m1_hit = m1_lo_b & m1_hi_b;
	assign m2_hit = m2_lo_b & m2_hi_b;
	assign m3_hit = m3_lo_b & m3_hi_b;

	wire next_hit = r1_valid & (m0_hit | m1_hit | m2_hit | m3_hit);

	genvar gpb, gfb;
	generate
		for (gpb = 0; gpb < PPN_BITS; gpb = gpb + 1) begin : g_l2_pfn_mux
			wire p01, p23;
			wire x0_lo = (gpb < 14) ? m0_lo_a : m0_lo_b;
			wire x0_hi = (gpb < 14) ? m0_hi_a : m0_hi_b;
			wire x1_lo = (gpb < 14) ? m1_lo_a : m1_lo_b;
			wire x1_hi = (gpb < 14) ? m1_hi_a : m1_hi_b;
			wire x2_lo = (gpb < 14) ? m2_lo_a : m2_lo_b;
			wire x2_hi = (gpb < 14) ? m2_hi_a : m2_hi_b;
			wire x3_lo = (gpb < 14) ? m3_lo_a : m3_lo_b;
			wire x3_hi = (gpb < 14) ? m3_hi_a : m3_hi_b;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_p01 (
				.I0(x0_lo), .I1(x0_hi), .I2(r1_pfn0[gpb]),
				.I3(x1_lo), .I4(x1_hi), .I5(r1_pfn1[gpb]),
				.O(p01)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_p23 (
				.I0(x2_lo), .I1(x2_hi), .I2(r1_pfn2[gpb]),
				.I3(x3_lo), .I4(x3_hi), .I5(r1_pfn3[gpb]),
				.O(p23)
			);
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_por (.I0(p01), .I1(p23), .O(next_pfn[gpb]));
		end
		for (gfb = 0; gfb < 6; gfb = gfb + 1) begin : g_l2_flg_mux
			wire f01, f23;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_f01 (
				.I0(m0_lo_b), .I1(m0_hi_b), .I2(r1_flags0[gfb]),
				.I3(m1_lo_b), .I4(m1_hi_b), .I5(r1_flags1[gfb]),
				.O(f01)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF80808080808080)) u_f23 (
				.I0(m2_lo_b), .I1(m2_hi_b), .I2(r1_flags2[gfb]),
				.I3(m3_lo_b), .I4(m3_hi_b), .I5(r1_flags3[gfb]),
				.O(f23)
			);
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_for (.I0(f01), .I1(f23), .O(next_flags[gfb]));
		end
	endgenerate
`else
	assign m0_hit = m0_lo & m0_hi;
	assign m1_hit = m1_lo & m1_hi;
	assign m2_hit = m2_lo & m2_hi;
	assign m3_hit = m3_lo & m3_hi;

	wire next_hit = r1_valid & (m0_hit | m1_hit | m2_hit | m3_hit);

	reg [PPN_BITS-1:0] next_pfn_r;
	always @(*) begin
		if (m0_hit)      next_pfn_r = r1_pfn0;
		else if (m1_hit) next_pfn_r = r1_pfn1;
		else if (m2_hit) next_pfn_r = r1_pfn2;
		else             next_pfn_r = r1_pfn3;
	end
	assign next_pfn = next_pfn_r;

	reg [5:0] next_flags_r;
	always @(*) begin
		if (m0_hit)      next_flags_r = r1_flags0;
		else if (m1_hit) next_flags_r = r1_flags1;
		else if (m2_hit) next_flags_r = r1_flags2;
		else             next_flags_r = r1_flags3;
	end
	assign next_flags = next_flags_r;
`endif

	/* Registered Stage 2 outputs (0 logic levels on MMU hit path) */
	always @(posedge clk_l2 or posedge rst_l2_int) begin
		if (rst_l2_int) begin
			hit       <= 1'b0;
			hit_pfn   <= {PPN_BITS{1'b0}};
			hit_flags <= 6'd0;
		end else begin
			hit       <= next_hit;
			hit_pfn   <= next_pfn;
			hit_flags <= next_flags;
		end
	end

	wire unused_ok = &{1'b0, ptr_co_unused};
endmodule

`default_nettype wire
