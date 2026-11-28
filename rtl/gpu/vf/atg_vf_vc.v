/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_vf_vc: Post-Transform Vertex Cache (PT-VC, Phase 033):
 * 32-entry fully-associative post-transform vertex cache for indexed draws.
 * Caches transformed vertex clip positions and varying attributes to eliminate
 * redundant vertex shader execution for shared vertices in index meshes.
 *
 * Lookup is zero-cycle combinational CAM match.
 * On hit: bypasses SU vertex shader issue and reuses cached vertex data.
 * On miss: vertex proceeds through SU, and result is committed upon shader completion.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_vf_vc #(
	parameter NENTRIES = 32,
	parameter VDATA_W  = 512,	/* Clip position (x, y, z, w) + point size + varyings */
	parameter SLOT_W   = 5
) (
	input  wire clk,
	input  wire rst_n,
	input  wire flush,	/* Cache flush on draw call boundary */

	/* Index lookup port */
	input  wire        lookup_valid,
	input  wire [31:0] lookup_index,
	output wire        hit,
	output wire [VDATA_W-1:0] hit_vdata,
	output wire [SLOT_W-1:0]  hit_slot,

	/* SU shader completion commit port */
	input  wire        commit_valid,
	input  wire [31:0] commit_index,
	input  wire [VDATA_W-1:0] commit_vdata
);
	(* keep = "true" *) wire rst_sync = !rst_n;

	reg [31:0]         entry_idx   [0:NENTRIES-1];
	reg [VDATA_W-1:0]  entry_vdata [0:NENTRIES-1];
	reg [NENTRIES-1:0] entry_valid;
	reg [NENTRIES-1:0] alloc_ptr_oh;

	reg                c1_valid;
	reg [31:0]         c1_idx;
	reg [VDATA_W-1:0]  c1_vdata;
	reg [NENTRIES-1:0] c1_match;

	/* Stage C1 commit resolution (<= 7 cmos3 gates from FFs) */
	wire c1_hit = |c1_match;
	wire [NENTRIES-1:0] c1_alloc_oh = c1_hit ? c1_match : alloc_ptr_oh;

	/* Stage C0 incoming commit match (with 1-cycle C1 forwarding) */
	wire [NENTRIES-1:0] c0_match;
	genvar gi;
	generate
		for (gi = 0; gi < NENTRIES; gi = gi + 1) begin: g_commit_match
			assign c0_match[gi] = entry_valid[gi] && (entry_idx[gi] == commit_index);
		end
	endgenerate
	wire c0_same_c1 = c1_valid && (commit_index == c1_idx);
	wire [NENTRIES-1:0] c0_match_fwd = c0_same_c1 ? c1_alloc_oh :
		((c1_valid && !c1_hit) ? (c0_match & ~alloc_ptr_oh) : c0_match);

	/* CAM Lookup logic (0-cycle, including Stage C1 bypass and miss-evict mask) */
	wire c1_hit_lu = c1_valid && (c1_idx == lookup_index);
	wire [NENTRIES-1:0] c1_mask_oh = c1_valid ? c1_alloc_oh : {NENTRIES{1'b0}};
	wire [NENTRIES-1:0] match_vec;
	generate
		for (gi = 0; gi < NENTRIES; gi = gi + 1) begin: g_match
			assign match_vec[gi] = entry_valid[gi] && !c1_mask_oh[gi] && (entry_idx[gi] == lookup_index);
		end
	endgenerate

	wire cam_hit   = |match_vec;
	assign hit = lookup_valid && (c1_hit_lu || cam_hit);

	/* One-hot to binary slot index (OR-mask tree, 0-cycle) */
	wire [SLOT_W-1:0] cam_slot, c1_slot;
	generate
		for (gi = 0; gi < SLOT_W; gi = gi + 1) begin: g_slot_enc
			wire [NENTRIES-1:0] cam_sbits, c1_sbits;
			genvar sj;
			for (sj = 0; sj < NENTRIES; sj = sj + 1) begin: g_sb
				assign cam_sbits[sj] = ((sj >> gi) & 1) ? match_vec[sj] : 1'b0;
				assign c1_sbits[sj]  = ((sj >> gi) & 1) ? c1_alloc_oh[sj] : 1'b0;
			end
			assign cam_slot[gi] = |cam_sbits;
			assign c1_slot[gi]  = |c1_sbits;
		end
	endgenerate
	assign hit_slot = c1_hit_lu ? c1_slot : cam_slot;

	/* Balanced 33-to-1 DeMorgan NAND/NOR OR-AND tree for hit_vdata (entries 0..31 + C1 bypass at 32) */
	wire [VDATA_W-1:0] vd_n0 [0:32];
	wire [VDATA_W-1:0] vd_p1 [0:10];
	wire [VDATA_W-1:0] vd_n2 [0:3];
	wire [VDATA_W-1:0] vd_p3 [0:1];
	generate
		for (gi = 0; gi < 32; gi = gi + 1) begin: g_vd0
			assign vd_n0[gi] = ~(entry_vdata[gi] & {VDATA_W{match_vec[gi]}});
		end
		assign vd_n0[32] = ~(c1_vdata & {VDATA_W{c1_hit_lu}});
		for (gi = 0; gi < 11; gi = gi + 1) begin: g_vd1
			assign vd_p1[gi] = ~(vd_n0[3*gi] & vd_n0[3*gi+1] & vd_n0[3*gi+2]);
		end
		for (gi = 0; gi < 3; gi = gi + 1) begin: g_vd2
			assign vd_n2[gi] = ~(vd_p1[3*gi] | vd_p1[3*gi+1] | vd_p1[3*gi+2]);
		end
		assign vd_n2[3] = ~(vd_p1[9] | vd_p1[10]);
		assign vd_p3[0] = ~(vd_n2[0] & vd_n2[1]);
		assign vd_p3[1] = ~(vd_n2[2] & vd_n2[3]);
	endgenerate
	assign hit_vdata = vd_p3[0] | vd_p3[1];

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			entry_valid  <= {NENTRIES{1'b0}};
			alloc_ptr_oh <= {{NENTRIES-1{1'b0}}, 1'b1};
			c1_valid     <= 1'b0;
			c1_idx       <= 32'd0;
			c1_vdata     <= {VDATA_W{1'b0}};
			c1_match     <= {NENTRIES{1'b0}};
		end else if (flush) begin
			entry_valid  <= {NENTRIES{1'b0}};
			alloc_ptr_oh <= {{NENTRIES-1{1'b0}}, 1'b1};
			c1_valid     <= 1'b0;
			c1_match     <= {NENTRIES{1'b0}};
		end else begin
			if (c1_valid) begin
				entry_valid <= entry_valid | c1_alloc_oh;
				if (!c1_hit)
					alloc_ptr_oh <= {alloc_ptr_oh[NENTRIES-2:0], alloc_ptr_oh[NENTRIES-1]};
			end
			c1_valid <= commit_valid;
			if (commit_valid) begin
				c1_idx   <= commit_index;
				c1_vdata <= commit_vdata;
				c1_match <= c0_match_fwd;
			end
		end
	end

	generate
		for (gi = 0; gi < NENTRIES; gi = gi + 1) begin: g_entry_wr
			always @(posedge clk) begin
				if (c1_valid && c1_alloc_oh[gi]) begin
					entry_idx[gi]   <= c1_idx;
					entry_vdata[gi] <= c1_vdata;
				end
			end
		end
	endgenerate
endmodule

`default_nettype wire
