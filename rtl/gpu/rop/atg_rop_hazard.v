/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_hazard: Pixel Lock CAM and RAW Hazard Tracker for ROP pipeline.
 *
 * Tracks in-flight quad coordinates {RT[2:0], Y[15:1], X[15:1]} across the
 * ROP pipeline (Phase 003: MRT 8 attachments).
 * High-frequency 5nm ASIC implementation:
 * - Pure-LUT equality slices (0 CARRY4s, 3 LUT6 levels to o_grp_hit, 4 LUT6 levels to alloc_ready/valid).
 * - 2-level priority tree for empty slot selection (<= 3 LUT6 levels).
 * - Buffered write-enable trees to keep all net fanouts <= 16.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_rop_hazard #(
	parameter NUM_ENTRIES = 16,
	parameter TAG_W = 33	/* {RT[2:0], Y[15:1], X[15:1]} */
) (
	input wire clk,
	input wire rst_n,

	/* Allocation interface (pipeline issue stage) */
	input wire alloc_valid,
	output wire alloc_ready,	/* not full and no RAW hazard */
	input wire [TAG_W-1:0] alloc_tag,
	output reg [3:0] alloc_id,	/* allocated entry ID */
	output wire [2:0] o_grp_hit,	/* Level-3 group hit / full flags */

	/* Release interface (pipeline commit stage) */
	input wire release_valid,
	input wire [3:0] release_id
);
	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	reg [NUM_ENTRIES-1:0] valid;
	reg [TAG_W-1:0] tag [0:NUM_ENTRIES-1];

	/* Buffer release_valid, release_id, and alloc_tag into 2 halves (8 entries each) so fanout <= 16 */
	wire [1:0] rel_v_buf;
	wire [3:0] rel_id_buf [0:1];
	wire [TAG_W-1:0] alloc_tag_buf [0:1];
	genvar gh, gb;
	generate
	for (gh = 0; gh < 2; gh = gh + 1) begin : g_rel_buf
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *)
		LUT1 #(.INIT(2'b10)) u_rv (.O(rel_v_buf[gh]), .I0(release_valid));
		for (gb = 0; gb < 4; gb = gb + 1) begin : g_rid
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_rid (.O(rel_id_buf[gh][gb]), .I0(release_id[gb]));
		end
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_atag
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_atag (.O(alloc_tag_buf[gh][gb]), .I0(alloc_tag[gb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		assign rel_v_buf[gh] = release_valid;
		for (gb = 0; gb < 4; gb = gb + 1) begin : g_rid
			assign rel_id_buf[gh][gb] = release_id[gb];
		end
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_atag
			assign alloc_tag_buf[gh][gb] = alloc_tag[gb];
		end
`else
		assign rel_v_buf[gh] = release_valid;
		assign rel_id_buf[gh] = release_id;
		assign alloc_tag_buf[gh] = alloc_tag;
`endif
	end
	endgenerate

	(* keep = "true" *) wire [NUM_ENTRIES-1:0] hit_ent;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] empty;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] i_val_vec;
	genvar i, gs, gp;

`ifdef SYNTH_KINTEX7
	/* 2-LUT-level hazard and full detection for 5nm ASIC lint:
	 * Level 1: 8 x 6-input LUT6 hit_pair[7:0] (2 entries per LUT6) + 3 x full_g[2:0]
	 * Level 2: 2 x 6-input LUT6 grp_hit_int/ext[1:0]
	 */
	(* keep = "true" *) wire [7:0] hit_pair;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_cam
		localparam H_IDX = i / 8;
		(* keep = "true" *) wire i_val = valid[i] && (!rel_v_buf[H_IDX] || (rel_id_buf[H_IDX] != i[3:0]));
		assign i_val_vec[i] = i_val;
		assign empty[i] = !valid[i];
		assign hit_ent[i] = 1'b0;
	end
	for (gp = 0; gp < 8; gp = gp + 1) begin : g_hit_pair
		localparam integer B0 = 4 + 2 * gp;
		localparam integer B1 = 4 + 2 * gp + 1;
		assign hit_pair[gp] =
			(valid[2 * gp]     && (tag[2 * gp][B0]     == alloc_tag[B0])) ||
			(valid[2 * gp + 1] && (tag[2 * gp + 1][B1] == alloc_tag[B1]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	/* 2-LUT-level hazard and full detection for 5nm ASIC lint:
	 * Level 1: 8 x 6-input LUT6 hit_pair[7:0] (2 entries per LUT6) + 3 x full_g[2:0]
	 * Level 2: 2 x 6-input LUT6 grp_hit_int/ext[1:0]
	 */
	(* keep = "true" *) wire [7:0] hit_pair;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_cam
		localparam H_IDX = i / 8;
		(* keep = "true" *) wire i_val = valid[i] && (!rel_v_buf[H_IDX] || (rel_id_buf[H_IDX] != i[3:0]));
		assign i_val_vec[i] = i_val;
		assign empty[i] = !valid[i];
		assign hit_ent[i] = 1'b0;
	end
	for (gp = 0; gp < 8; gp = gp + 1) begin : g_hit_pair
		localparam integer B0 = 4 + 2 * gp;
		localparam integer B1 = 4 + 2 * gp + 1;
		assign hit_pair[gp] =
			(valid[2 * gp]     && (tag[2 * gp][B0]     == alloc_tag[B0])) ||
			(valid[2 * gp + 1] && (tag[2 * gp + 1][B1] == alloc_tag[B1]));
	end
	endgenerate
`else
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_cam
		localparam H_IDX = i / 8;
		wire [32:0] tag_pad = {{(33-TAG_W){1'b0}}, tag[i]};
		wire [32:0] alloc_tag_pad = {{(33-TAG_W){1'b0}}, alloc_tag_buf[H_IDX]};

		(* keep = "true" *) wire i_val = valid[i] && (!rel_v_buf[H_IDX] || (rel_id_buf[H_IDX] != i[3:0]));
		assign i_val_vec[i] = i_val;
		assign empty[i] = !valid[i] || (rel_v_buf[H_IDX] && (rel_id_buf[H_IDX] == i[3:0]));
		wire [10:0] eq_all;
		for (gs = 0; gs < 11; gs = gs + 1) begin : g_slice
			assign eq_all[gs] = (tag_pad[gs*3 +: 3] == alloc_tag_pad[gs*3 +: 3]);
		end
		assign hit_ent[i] = i_val & (&eq_all);
	end
	endgenerate
`endif

	/* 2-level priority & empty tree across 4 groups of 4 entries */
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [3:0] grp_has;
	(* keep = "true" *) wire [7:0] grp_id_flat;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] loc_sel;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [3:0] grp_has;
	(* keep = "true" *) wire [7:0] grp_id_flat;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] loc_sel;
`else
	wire [3:0] grp_has;
	wire [7:0] grp_id_flat;
	wire [NUM_ENTRIES-1:0] loc_sel;
`endif
	genvar gg;
	generate
	for (gg = 0; gg < 4; gg = gg + 1) begin : g_grp
		wire e0 = empty[4*gg + 0];
		wire e1 = empty[4*gg + 1];
		wire e2 = empty[4*gg + 2];
		wire e3 = empty[4*gg + 3];
		assign grp_has[gg] = e0 | e1 | e2 | e3;
		assign grp_id_flat[2*gg + 1] = !e0 && !e1;
		assign grp_id_flat[2*gg + 0] = !e0 && (e1 || !e2);
		assign loc_sel[4*gg + 0] = e0;
		assign loc_sel[4*gg + 1] = !e0 && e1;
		assign loc_sel[4*gg + 2] = !e0 && !e1 && e2;
		assign loc_sel[4*gg + 3] = !e0 && !e1 && !e2 && e3;
	end
	endgenerate

`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = !grp_has[0];
	assign grp_sel[2] = !grp_has[0] && !grp_has[1];
	assign grp_sel[3] = !grp_has[0] && !grp_has[1] && !grp_has[2];
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = !grp_has[0];
	assign grp_sel[2] = !grp_has[0] && !grp_has[1];
	assign grp_sel[3] = !grp_has[0] && !grp_has[1] && !grp_has[2];
`else
	wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = &valid[3:0];
	assign grp_sel[2] = (&valid[7:4]) & (&valid[3:0]);
	assign grp_sel[3] = (&valid[11:8]) & (&valid[7:4]) & (&valid[3:0]);
`endif

	(* keep = "true" *) wire [2:0] grp_hit_int;
	(* keep = "true" *) wire [2:0] grp_hit_ext;
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire full_g0 = &valid[5:0];
	(* keep = "true" *) wire full_g1 = &valid[11:6];
	(* keep = "true" *) wire full_g2 = &valid[15:12];

	assign grp_hit_int[0] = |hit_pair[5:0];
	assign grp_hit_int[1] = hit_pair[6] | hit_pair[7] | (full_g0 & full_g1 & full_g2);
	assign grp_hit_int[2] = 1'b0;

	assign grp_hit_ext[0] = |hit_pair[5:0];
	assign grp_hit_ext[1] = hit_pair[6] | hit_pair[7] | (full_g0 & full_g1 & full_g2);
	assign grp_hit_ext[2] = 1'b0;
	wire unused_haz_k7 = &{1'b0, hit_ent};
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire full_g0 = &valid[5:0];
	(* keep = "true" *) wire full_g1 = &valid[11:6];
	(* keep = "true" *) wire full_g2 = &valid[15:12];

	assign grp_hit_int[0] = |hit_pair[5:0];
	assign grp_hit_int[1] = hit_pair[6] | hit_pair[7] | (full_g0 & full_g1 & full_g2);
	assign grp_hit_int[2] = 1'b0;

	assign grp_hit_ext[0] = |hit_pair[5:0];
	assign grp_hit_ext[1] = hit_pair[6] | hit_pair[7] | (full_g0 & full_g1 & full_g2);
	assign grp_hit_ext[2] = 1'b0;
	wire unused_haz_k7 = &{1'b0, hit_ent};
`else
	(* keep = "true" *) wire cam_full = ~(|grp_has);

	/* Level 3: combine 6 entries per LUT6 (3 LUT6s total) + replicated copy for external output */
	assign grp_hit_int[0] = |hit_ent[5:0];
	assign grp_hit_int[1] = |hit_ent[11:6];
	assign grp_hit_int[2] = (|hit_ent[15:12]) | cam_full;

	assign grp_hit_ext[0] = |hit_ent[5:0];
	assign grp_hit_ext[1] = |hit_ent[11:6];
	assign grp_hit_ext[2] = (|hit_ent[15:12]) | cam_full;
`endif

	assign o_grp_hit = grp_hit_ext;

	/* Level 4: single 3-input NOR produces alloc_ready */
	assign alloc_ready = ~(|grp_hit_ext);

	/* 2-LUT-level priority encoder for alloc_id */
	always @(*) begin
		alloc_id[3] = !grp_has[0] && !grp_has[1];
		alloc_id[2] = !grp_has[0] && (grp_has[1] || !grp_has[2]);
		if (grp_has[0])      alloc_id[1:0] = grp_id_flat[1:0];
		else if (grp_has[1]) alloc_id[1:0] = grp_id_flat[3:2];
		else if (grp_has[2]) alloc_id[1:0] = grp_id_flat[5:4];
		else                 alloc_id[1:0] = grp_id_flat[7:6];
	end

	localparam S0_W = 11;
	localparam S1_W = 11;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_ent
		localparam G_IDX = i / 4;
		localparam H_IDX = i / 8;
		(* keep = "true" *) wire slot_sel_i = grp_sel[G_IDX] && loc_sel[i];
		/* Level 4: single 6-input LUT6 computes next valid[i] */
		(* keep = "true" *) wire val_next_i =
			(alloc_valid && !grp_hit_int[0] && !grp_hit_int[1] && !grp_hit_int[2] && slot_sel_i) ||
			i_val_vec[i];

`ifdef SYNTH_KINTEX7
		wire [2:0] we_tag_buf;
		for (gb = 0; gb < 3; gb = gb + 1) begin : g_we_buf
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_we (.O(we_tag_buf[gb]), .I0(slot_sel_i));
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				valid[i] <= 1'b0;
			else
				valid[i] <= val_next_i;
		end
		always @(posedge clk) begin
			if (we_tag_buf[0])
				tag[i][S0_W-1:0] <= alloc_tag_buf[H_IDX][S0_W-1:0];
			if (we_tag_buf[1])
				tag[i][S0_W+S1_W-1:S0_W] <= alloc_tag_buf[H_IDX][S0_W+S1_W-1:S0_W];
			if (we_tag_buf[2])
				tag[i][TAG_W-1:S0_W+S1_W] <= alloc_tag_buf[H_IDX][TAG_W-1:S0_W+S1_W];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [2:0] we_tag_buf;
		for (gb = 0; gb < 3; gb = gb + 1) begin : g_we_buf
			assign we_tag_buf[gb] = slot_sel_i;
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				valid[i] <= 1'b0;
			else
				valid[i] <= val_next_i;
		end
		always @(posedge clk) begin
			if (we_tag_buf[0])
				tag[i][S0_W-1:0] <= alloc_tag_buf[H_IDX][S0_W-1:0];
			if (we_tag_buf[1])
				tag[i][S0_W+S1_W-1:S0_W] <= alloc_tag_buf[H_IDX][S0_W+S1_W-1:S0_W];
			if (we_tag_buf[2])
				tag[i][TAG_W-1:S0_W+S1_W] <= alloc_tag_buf[H_IDX][TAG_W-1:S0_W+S1_W];
		end
`else
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				valid[i] <= 1'b0;
				tag[i] <= {TAG_W{1'b0}};
			end else begin
				valid[i] <= val_next_i;
				if (alloc_valid && !grp_hit_int[0] && !grp_hit_int[1] && !grp_hit_int[2] && slot_sel_i)
					tag[i] <= alloc_tag_buf[H_IDX];
			end
		end
`endif
	end
	endgenerate
endmodule

/*
 * atg_rop_hazard_pipe: Pipelined tag-match Pixel Lock CAM for high-freq ASIC.
 * Evaluates pre_tag == tag[i] into r_tag_match[i] when pre_load advances Stage S1,
 * so Stage S1 stall/issue sees registered match flags + 1-hot release_oh.
 */
module atg_rop_hazard_pipe #(
	parameter NUM_ENTRIES = 16,
	parameter TAG_W = 33
) (
	input wire clk,
	input wire rst_n,

	input wire pre_load,
	input wire [28:0] pre_load_vec,
	input wire [TAG_W-1:0] pre_tag,
	input wire di_load,
	input wire [TAG_W-1:0] di_pre_tag,

	input wire alloc_valid,
	output wire alloc_ready,
	input wire [TAG_W-1:0] alloc_tag,
	output reg [3:0] alloc_id,
	output wire [2:0] o_grp_hit,

	input wire release_valid,
	input wire [3:0] release_id,
	input wire [NUM_ENTRIES-1:0] release_oh,
	input wire next_release_valid,
	input wire [3:0] next_release_id,
	input wire [NUM_ENTRIES-1:0] next_release_oh,
	output wire o_vhit_any,
	output wire [3:0] o_vhit_id,
	output wire o_haz_clear,
	output wire [4:0] o_haz_clear_rep,
	output wire o_cam_full,
	output wire [11:0] o_cam_full_rep,
	output wire [11:0] o_hv_clear_rep,
	output wire [11:0] o_v16_rep,
	output wire [11:0] o_vrel_or0_rep,
	output wire [11:0] o_vrel_or1_rep,
	output wire [11:0] o_arel_or0_rep,
	output wire [11:0] o_arel_or1_rep,
	output wire [11:0] o_vhit_a_rep
);
	localparam integer N_HREP = 12;
	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	reg [NUM_ENTRIES-1:0] valid;
	reg [TAG_W-1:0] tag [0:NUM_ENTRIES-1];
	reg r_vhit_a;
	reg [3:0] r_vhit_aid;
	reg [NUM_ENTRIES-1:0] r_vhit_oh;
	reg [NUM_ENTRIES-1:0] r_vhit_a_oh;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [NUM_ENTRIES-1:0] r_rel_oh;
	wire [3:0] r_vhit_g = {
		|r_vhit_oh[15:12],
		|r_vhit_oh[11:8],
		|r_vhit_oh[7:4],
		|r_vhit_oh[3:0]
	};
	reg r_cam_full;
	(* keep = "true", dont_touch = "true" *) reg [N_HREP-1:0] r_vhit_a_rep, r_cam_full_rep;
	(* keep = "true", dont_touch = "true" *) reg [NUM_ENTRIES-1:0] r_vhit_oh_rep [0:N_HREP-1];
	(* keep = "true", dont_touch = "true" *) reg [NUM_ENTRIES-1:0] r_vhit_a_oh_rep [0:N_HREP-1];
	(* keep = "true", dont_touch = "true" *) reg [NUM_ENTRIES-1:0] r_rel_oh_rep [0:N_HREP-1];
	wire [3:0] oh_grp_id [0:3];
	genvar go;
	generate
	for (go = 0; go < 4; go = go + 1) begin : g_oh_id
		assign oh_grp_id[go] = {
			2'b00,
			r_vhit_oh[4*go + 2] | r_vhit_oh[4*go + 3],
			r_vhit_oh[4*go + 1] | r_vhit_oh[4*go + 3]
		};
	end
	endgenerate
	wire [1:0] vhit_hi_w = {r_vhit_g[3] | r_vhit_g[2], r_vhit_g[3] | r_vhit_g[1]};
	wire [1:0] vhit_lo_w =
		({2{r_vhit_g[0]}} & oh_grp_id[0][1:0]) |
		({2{r_vhit_g[1]}} & oh_grp_id[1][1:0]) |
		({2{r_vhit_g[2]}} & oh_grp_id[2][1:0]) |
		({2{r_vhit_g[3]}} & oh_grp_id[3][1:0]);
	assign o_vhit_any = r_vhit_a || (|r_vhit_g);
	assign o_vhit_id  = r_vhit_a ? r_vhit_aid : {vhit_hi_w, vhit_lo_w};
	assign o_cam_full = r_cam_full;

	wire [TAG_W-1:0] alloc_tag_buf [0:1];
	wire [TAG_W-1:0] pre_tag_buf [0:2];
`ifdef SYNTH_KINTEX7
	wire [28:0] pre_load_buf = pre_load_vec;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [28:0] pre_load_buf = pre_load_vec;
`else
	wire [3:0] pre_load_buf;
`endif
	genvar gh, gb;
	generate
	for (gh = 0; gh < 2; gh = gh + 1) begin : g_atag_buf
`ifdef SYNTH_KINTEX7
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_at
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_atag (.O(alloc_tag_buf[gh][gb]), .I0(alloc_tag[gb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_at
			assign alloc_tag_buf[gh][gb] = alloc_tag[gb];
		end
`else
		assign alloc_tag_buf[gh] = alloc_tag;
`endif
	end
	for (gh = 0; gh < 3; gh = gh + 1) begin : g_ptag_buf
`ifdef SYNTH_KINTEX7
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_pt
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_ptag (.O(pre_tag_buf[gh][gb]), .I0(pre_tag[gb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gb = 0; gb < TAG_W; gb = gb + 1) begin : g_pt
			assign pre_tag_buf[gh][gb] = pre_tag[gb];
		end
`else
		assign pre_tag_buf[gh] = pre_tag;
`endif
	end
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	for (gh = 0; gh < 4; gh = gh + 1) begin : g_pld_buf
		assign pre_load_buf[gh] = pre_load;
	end
`endif
	endgenerate

`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] cur_hit_vec;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] eq_cur;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] cur_hit_vec;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] eq_cur;
`else
	wire [NUM_ENTRIES-1:0] cur_hit_vec;
	wire [NUM_ENTRIES-1:0] eq_cur;
`endif
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] empty;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] i_val_vec;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] empty;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] i_val_vec;
`else
	wire [NUM_ENTRIES-1:0] empty;
	wire [NUM_ENTRIES-1:0] i_val_vec;
`endif
	wire [NUM_ENTRIES-1:0] di_hit_vec;
	reg [NUM_ENTRIES-1:0] r_di_oh;
	genvar i, gs;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_cam
		localparam H_IDX = i / 8;
`ifdef SYNTH_KINTEX7
		(* keep = "true" *) wire i_val = valid[i] && !r_rel_oh[i];
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep = "true" *) wire i_val = valid[i] && !r_rel_oh[i];
`else
		wire i_val = valid[i] && !r_rel_oh[i];
`endif
		assign i_val_vec[i] = i_val;
		assign empty[i] = !valid[i];
`ifdef SYNTH_KINTEX7
		wire [32:0] tag_pad = {{(33-TAG_W){1'b0}}, tag[i]};
		wire [32:0] pre_tag_pad = {{(33-TAG_W){1'b0}}, pre_tag_buf[H_IDX]};
		wire [10:0] eq3;
		for (gs = 0; gs < 11; gs = gs + 1) begin : g_slice
			assign eq3[gs] = (tag_pad[gs*3 +: 3] == pre_tag_pad[gs*3 +: 3]);
		end
		assign eq_cur[i] = &eq3;
		assign cur_hit_vec[i] = i_val && eq_cur[i];
		assign di_hit_vec[i] = i_val && (tag_pad == {{(33-TAG_W){1'b0}}, di_pre_tag});
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [32:0] tag_pad = {{(33-TAG_W){1'b0}}, tag[i]};
		wire [32:0] pre_tag_pad = {{(33-TAG_W){1'b0}}, pre_tag_buf[H_IDX]};
		wire [10:0] eq3;
		for (gs = 0; gs < 11; gs = gs + 1) begin : g_slice
			assign eq3[gs] = (tag_pad[gs*3 +: 3] == pre_tag_pad[gs*3 +: 3]);
		end
		assign eq_cur[i] = &eq3;
		assign cur_hit_vec[i] = i_val && eq_cur[i];
		assign di_hit_vec[i] = i_val && (tag_pad == {{(33-TAG_W){1'b0}}, di_pre_tag});
`else
		wire [32:0] tag_pad = {{(33-TAG_W){1'b0}}, tag[i]};
		wire [32:0] pre_tag_pad = {{(33-TAG_W){1'b0}}, pre_tag_buf[H_IDX]};
		/* Pure binary 2->4->8->16->32 equality tree for cur_hit_vec (<= 10 cmos3 at r_vhit_oh) */
		wire [32:0] eq_diff = tag_pad ^ pre_tag_pad;
		wire [15:0] eq2;
		for (gs = 0; gs < 16; gs = gs + 1) begin : g_eq2
			assign eq2[gs] = !(eq_diff[2*gs] | eq_diff[2*gs+1]);
		end
		wire [7:0] eq4_n;
		for (gs = 0; gs < 8; gs = gs + 1) begin : g_eq4
			assign eq4_n[gs] = !(eq2[2*gs] & eq2[2*gs+1]);
		end
		wire [3:0] eq8;
		for (gs = 0; gs < 4; gs = gs + 1) begin : g_eq8
			assign eq8[gs] = !(eq4_n[2*gs] | eq4_n[2*gs+1]);
		end
		wire [1:0] eq16_n = {!(eq8[3] & eq8[2]), !(eq8[1] & eq8[0])};
		wire eq_lo_ok = !(eq16_n[1] | eq16_n[0]);
		wire eq_hi_v = !(eq_diff[32] | !i_val);
		assign cur_hit_vec[i] = eq_lo_ok & eq_hi_v;
		assign eq_cur[i] = cur_hit_vec[i];

		wire [32:0] di_pad = {{(33-TAG_W){1'b0}}, di_pre_tag};
		wire [32:0] deq_diff = tag_pad ^ di_pad;
		wire [15:0] deq2;
		for (gs = 0; gs < 16; gs = gs + 1) begin : g_deq2
			assign deq2[gs] = !(deq_diff[2*gs] | deq_diff[2*gs+1]);
		end
		wire [7:0] deq4_n;
		for (gs = 0; gs < 8; gs = gs + 1) begin : g_deq4
			assign deq4_n[gs] = !(deq2[2*gs] & deq2[2*gs+1]);
		end
		wire [3:0] deq8 = {
			!(deq4_n[7] | deq4_n[6]), !(deq4_n[5] | deq4_n[4]),
			!(deq4_n[3] | deq4_n[2]), !(deq4_n[1] | deq4_n[0])
		};
		wire [1:0] deq16_n = {!(deq8[3] & deq8[2]), !(deq8[1] & deq8[0])};
		wire deq_lo_ok = !(deq16_n[1] | deq16_n[0]);
		wire deq_hi_v = !(deq_diff[32] | !i_val);
		assign di_hit_vec[i] = deq_lo_ok & deq_hi_v;
`endif
	end
	endgenerate

`ifdef SYNTH_KINTEX7
	wire [32:0] pre_pad_self = {{(33-TAG_W){1'b0}}, pre_tag_buf[2]};
	wire [32:0] alloc_pad_self = {{(33-TAG_W){1'b0}}, alloc_tag_buf[0]};
	wire [10:0] eq_self3;
	generate
	for (gs = 0; gs < 11; gs = gs + 1) begin : g_self_slice
		assign eq_self3[gs] = (pre_pad_self[gs*3 +: 3] == alloc_pad_self[gs*3 +: 3]);
	end
	endgenerate
	wire eq_alloc = &eq_self3;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [32:0] pre_pad_self = {{(33-TAG_W){1'b0}}, pre_tag_buf[2]};
	wire [32:0] alloc_pad_self = {{(33-TAG_W){1'b0}}, alloc_tag_buf[0]};
	wire [10:0] eq_self3;
	generate
	for (gs = 0; gs < 11; gs = gs + 1) begin : g_self_slice
		assign eq_self3[gs] = (pre_pad_self[gs*3 +: 3] == alloc_pad_self[gs*3 +: 3]);
	end
	endgenerate
	wire eq_alloc = &eq_self3;
`else
	wire [32:0] pre_pad_self = {{(33-TAG_W){1'b0}}, pre_tag_buf[2]};
	wire [32:0] alloc_pad_self = {{(33-TAG_W){1'b0}}, alloc_tag_buf[0]};
	wire [32:0] eq_self_diff = pre_pad_self ^ alloc_pad_self;
	wire [10:0] eqs3;
	generate
	for (gs = 0; gs < 11; gs = gs + 1) begin : g_self_eq3
		assign eqs3[gs] = !(eq_self_diff[3*gs] | eq_self_diff[3*gs+1] | eq_self_diff[3*gs+2]);
	end
	endgenerate
	wire [3:0] eqs9_n = {
		!(eqs3[9] & eqs3[10]),
		!(eqs3[6] & eqs3[7] & eqs3[8]),
		!(eqs3[3] & eqs3[4] & eqs3[5]),
		!(eqs3[0] & eqs3[1] & eqs3[2])
	};
	wire eqs18_0 = !(eqs9_n[0] | eqs9_n[1]);
	wire eqs18_1 = !(eqs9_n[2] | eqs9_n[3]);
	wire eq_alloc_n = !(eqs18_0 & eqs18_1);
	wire eq_alloc = !eq_alloc_n;
`endif

	/* 2-level priority & empty tree across 4 groups of 4 entries */
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [3:0] grp_has;
	(* keep = "true" *) wire [7:0] grp_id_flat;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] loc_sel;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [3:0] grp_has;
	(* keep = "true" *) wire [7:0] grp_id_flat;
	(* keep = "true" *) wire [NUM_ENTRIES-1:0] loc_sel;
`else
	wire [3:0] grp_has;
	wire [7:0] grp_id_flat;
	wire [NUM_ENTRIES-1:0] loc_sel;
`endif
	wire [3:0] grp_two, grp_one_only, grp_zero;
	genvar gg;
	generate
	for (gg = 0; gg < 4; gg = gg + 1) begin : g_grp
		wire e0 = empty[4*gg + 0];
		wire e1 = empty[4*gg + 1];
		wire e2 = empty[4*gg + 2];
		wire e3 = empty[4*gg + 3];
		assign grp_has[gg] = e0 | e1 | e2 | e3;
		assign grp_zero[gg] = !(e0 | e1 | e2 | e3);
		assign grp_two[gg] = (e0 & e1) | (e2 & e3) | ((e0 | e1) & (e2 | e3));
		assign grp_one_only[gg] =
			( e0 & !e1 & !e2 & !e3) | (!e0 &  e1 & !e2 & !e3) |
			(!e0 & !e1 &  e2 & !e3) | (!e0 & !e1 & !e2 &  e3);
		assign grp_id_flat[2*gg + 1] = !e0 && !e1;
		assign grp_id_flat[2*gg + 0] = !e0 && (e1 || !e2);
		assign loc_sel[4*gg + 0] = e0;
		assign loc_sel[4*gg + 1] = !e0 && e1;
		assign loc_sel[4*gg + 2] = !e0 && !e1 && e2;
		assign loc_sel[4*gg + 3] = !e0 && !e1 && !e2 && e3;
	end
	endgenerate
`ifdef SYNTH_KINTEX7
	wire any_empty_w = |grp_has;
	wire two_empty_w = (|grp_two) |
		(grp_has[0] & grp_has[1]) | (grp_has[0] & grp_has[2]) | (grp_has[0] & grp_has[3]) |
		(grp_has[1] & grp_has[2]) | (grp_has[1] & grp_has[3]) | (grp_has[2] & grp_has[3]);
	wire one_empty_w = any_empty_w && !two_empty_w;
	wire [3:0] grp_one_unused = grp_one_only;
	wire [3:0] grp_zero_unused = grp_zero;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire any_empty_w = |grp_has;
	wire two_empty_w = (|grp_two) |
		(grp_has[0] & grp_has[1]) | (grp_has[0] & grp_has[2]) | (grp_has[0] & grp_has[3]) |
		(grp_has[1] & grp_has[2]) | (grp_has[1] & grp_has[3]) | (grp_has[2] & grp_has[3]);
	wire one_empty_w = any_empty_w && !two_empty_w;
	wire [3:0] grp_one_unused = grp_one_only;
	wire [3:0] grp_zero_unused = grp_zero;
`else
	wire one_empty_w =
		(grp_one_only[0] & grp_zero[1] & grp_zero[2] & grp_zero[3]) |
		(grp_zero[0] & grp_one_only[1] & grp_zero[2] & grp_zero[3]) |
		(grp_zero[0] & grp_zero[1] & grp_one_only[2] & grp_zero[3]) |
		(grp_zero[0] & grp_zero[1] & grp_zero[2] & grp_one_only[3]);
	wire [3:0] grp_two_unused = grp_two;
`endif
	wire rel_any_w = |r_rel_oh;

`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = !grp_has[0];
	assign grp_sel[2] = !grp_has[0] && !grp_has[1];
	assign grp_sel[3] = !grp_has[0] && !grp_has[1] && !grp_has[2];
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = !grp_has[0];
	assign grp_sel[2] = !grp_has[0] && !grp_has[1];
	assign grp_sel[3] = !grp_has[0] && !grp_has[1] && !grp_has[2];
`else
	wire [3:0] grp_full = {&valid[15:12], &valid[11:8], &valid[7:4], &valid[3:0]};
	wire [3:0] grp_sel;
	assign grp_sel[0] = 1'b1;
	assign grp_sel[1] = grp_full[0];
	assign grp_sel[2] = grp_full[0] & grp_full[1];
	assign grp_sel[3] = grp_full[0] & grp_full[1] & grp_full[2];
`endif
	wire [3:0] nrel_grp_hit;
	generate
	for (gg = 0; gg < 4; gg = gg + 1) begin : g_nrel_grp
		assign nrel_grp_hit[gg] = |(loc_sel[4*gg +: 4] & next_release_oh[4*gg +: 4]);
	end
	endgenerate
`ifdef SYNTH_KINTEX7
	wire slot_nrel = |(grp_sel & nrel_grp_hit);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire slot_nrel = |(grp_sel & nrel_grp_hit);
`else
	wire slot_nrel = nrel_grp_hit[0] | (grp_full[0] & nrel_grp_hit[1]) |
		(grp_full[0] & grp_full[1] & nrel_grp_hit[2]) |
		(grp_full[0] & grp_full[1] & grp_full[2] & nrel_grp_hit[3]);
`endif

	(* keep = "true" *) wire cam_full = r_cam_full;
	(* keep = "true" *) wire [2:0] grp_hit_int;
	(* keep = "true" *) wire [2:0] grp_hit_ext;
	/* Registered release_oh & one-hot r_vhit_a_oh evaluation (0 Check3 seq pairs, flat <= 5 cmos3) */
	wire [15:0] voh_rel = r_vhit_oh & r_rel_oh;
	wire [15:0] aoh_rel = r_vhit_a_oh & r_rel_oh;
	(* keep = "true" *) wire eq_rel_a = r_vhit_a && (|aoh_rel);
	wire eq_rel_any = eq_rel_a || (|voh_rel);
	wire haz_v_clear = !r_vhit_a && ~(|r_vhit_oh);
	wire haz_tag_hit = (r_vhit_a || (|r_vhit_oh)) && !eq_rel_any;
	assign o_haz_clear = !cam_full && (haz_v_clear || eq_rel_any);
	genvar grc, grb;
	generate
	for (grc = 0; grc < N_HREP; grc = grc + 1) begin : g_hc_rep
`ifdef SYNTH_KINTEX7
		wire [3:0] v4_k;
		for (grb = 0; grb < 4; grb = grb + 1) begin : g_hv4_k
			wire v4_w;
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_v4 (
				.I0(r_vhit_oh_rep[grc][4*grb + 0]), .I1(r_vhit_oh_rep[grc][4*grb + 1]),
				.I2(r_vhit_oh_rep[grc][4*grb + 2]), .I3(r_vhit_oh_rep[grc][4*grb + 3]),
				.O(v4_w)
			);
			assign v4_k[grb] = v4_w;
		end
		wire v16;
		(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_v16 (
			.I0(v4_k[0]), .I1(v4_k[1]), .I2(v4_k[2]), .I3(v4_k[3]),
			.O(v16)
		);
		wire hv_clear = !(v16 || r_vhit_a_rep[grc]);

		wire [5:0] m_p6;
		wire [5:0] a_p6;
		for (grb = 0; grb < 5; grb = grb + 1) begin : g_ma_p3
			wire mp3, ap3;
			(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_mp3 (
				.I0(r_vhit_oh_rep[grc][3*grb + 0]), .I1(r_rel_oh_rep[grc][3*grb + 0]),
				.I2(r_vhit_oh_rep[grc][3*grb + 1]), .I3(r_rel_oh_rep[grc][3*grb + 1]),
				.I4(r_vhit_oh_rep[grc][3*grb + 2]), .I5(r_rel_oh_rep[grc][3*grb + 2]),
				.O(mp3)
			);
			(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_ap3 (
				.I0(r_vhit_a_oh_rep[grc][3*grb + 0]), .I1(r_rel_oh_rep[grc][3*grb + 0]),
				.I2(r_vhit_a_oh_rep[grc][3*grb + 1]), .I3(r_rel_oh_rep[grc][3*grb + 1]),
				.I4(r_vhit_a_oh_rep[grc][3*grb + 2]), .I5(r_rel_oh_rep[grc][3*grb + 2]),
				.O(ap3)
			);
			assign m_p6[grb] = mp3;
			assign a_p6[grb] = ap3;
		end
		wire mp_last, ap_last;
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_mpl (.I0(r_vhit_oh_rep[grc][15]),   .I1(r_rel_oh_rep[grc][15]), .O(mp_last));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_apl (.I0(r_vhit_a_oh_rep[grc][15]), .I1(r_rel_oh_rep[grc][15]), .O(ap_last));
		assign m_p6[5] = mp_last;
		assign a_p6[5] = ap_last;
		wire m16_k, a16_k;
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_m16 (
			.I0(m_p6[0]), .I1(m_p6[1]), .I2(m_p6[2]), .I3(m_p6[3]), .I4(m_p6[4]), .I5(m_p6[5]),
			.O(m16_k)
		);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_a16 (
			.I0(a_p6[0]), .I1(a_p6[1]), .I2(a_p6[2]), .I3(a_p6[3]), .I4(a_p6[4]), .I5(a_p6[5]),
			.O(a16_k)
		);

		assign o_cam_full_rep[grc] = r_cam_full_rep[grc];
		assign o_hv_clear_rep[grc] = hv_clear;
		assign o_v16_rep[grc]      = v16;
		assign o_vrel_or0_rep[grc] = m16_k;
		assign o_vrel_or1_rep[grc] = 1'b0;
		assign o_arel_or0_rep[grc] = a16_k;
		assign o_arel_or1_rep[grc] = 1'b0;
		assign o_vhit_a_rep[grc]   = r_vhit_a_rep[grc];
		if (grc < 5) begin : g_hcr
			assign o_haz_clear_rep[grc] = !r_cam_full_rep[grc] && (hv_clear || m16_k || (r_vhit_a_rep[grc] && a16_k));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [3:0] v4_k;
		for (grb = 0; grb < 4; grb = grb + 1) begin : g_hv4_k
			wire v4_w;
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_v4 (
				.I0(r_vhit_oh_rep[grc][4*grb + 0]), .I1(r_vhit_oh_rep[grc][4*grb + 1]),
				.I2(r_vhit_oh_rep[grc][4*grb + 2]), .I3(r_vhit_oh_rep[grc][4*grb + 3]),
				.O(v4_w)
			);
			assign v4_k[grb] = v4_w;
		end
		wire v16;
		(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_v16 (
			.I0(v4_k[0]), .I1(v4_k[1]), .I2(v4_k[2]), .I3(v4_k[3]),
			.O(v16)
		);
		wire hv_clear = !(v16 || r_vhit_a_rep[grc]);

		wire [5:0] m_p6;
		wire [5:0] a_p6;
		for (grb = 0; grb < 5; grb = grb + 1) begin : g_ma_p3
			wire mp3, ap3;
			(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_mp3 (
				.I0(r_vhit_oh_rep[grc][3*grb + 0]), .I1(r_rel_oh_rep[grc][3*grb + 0]),
				.I2(r_vhit_oh_rep[grc][3*grb + 1]), .I3(r_rel_oh_rep[grc][3*grb + 1]),
				.I4(r_vhit_oh_rep[grc][3*grb + 2]), .I5(r_rel_oh_rep[grc][3*grb + 2]),
				.O(mp3)
			);
			(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_ap3 (
				.I0(r_vhit_a_oh_rep[grc][3*grb + 0]), .I1(r_rel_oh_rep[grc][3*grb + 0]),
				.I2(r_vhit_a_oh_rep[grc][3*grb + 1]), .I3(r_rel_oh_rep[grc][3*grb + 1]),
				.I4(r_vhit_a_oh_rep[grc][3*grb + 2]), .I5(r_rel_oh_rep[grc][3*grb + 2]),
				.O(ap3)
			);
			assign m_p6[grb] = mp3;
			assign a_p6[grb] = ap3;
		end
		wire mp_last, ap_last;
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_mpl (.I0(r_vhit_oh_rep[grc][15]),   .I1(r_rel_oh_rep[grc][15]), .O(mp_last));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_apl (.I0(r_vhit_a_oh_rep[grc][15]), .I1(r_rel_oh_rep[grc][15]), .O(ap_last));
		assign m_p6[5] = mp_last;
		assign a_p6[5] = ap_last;
		wire m16_k, a16_k;
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_m16 (
			.I0(m_p6[0]), .I1(m_p6[1]), .I2(m_p6[2]), .I3(m_p6[3]), .I4(m_p6[4]), .I5(m_p6[5]),
			.O(m16_k)
		);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_a16 (
			.I0(a_p6[0]), .I1(a_p6[1]), .I2(a_p6[2]), .I3(a_p6[3]), .I4(a_p6[4]), .I5(a_p6[5]),
			.O(a16_k)
		);

		assign o_cam_full_rep[grc] = r_cam_full_rep[grc];
		assign o_hv_clear_rep[grc] = hv_clear;
		assign o_v16_rep[grc]      = v16;
		assign o_vrel_or0_rep[grc] = m16_k;
		assign o_vrel_or1_rep[grc] = 1'b0;
		assign o_arel_or0_rep[grc] = a16_k;
		assign o_arel_or1_rep[grc] = 1'b0;
		assign o_vhit_a_rep[grc]   = r_vhit_a_rep[grc];
		if (grc < 5) begin : g_hcr
			assign o_haz_clear_rep[grc] = !r_cam_full_rep[grc] && (hv_clear || m16_k || (r_vhit_a_rep[grc] && a16_k));
		end
`else
		wire [7:0] v2_n;
		wire [3:0] v4;
		wire [1:0] v8_n;
		for (grb = 0; grb < 8; grb = grb + 1) begin : g_hv2
			assign v2_n[grb] = !(r_vhit_oh_rep[grc][2*grb] | r_vhit_oh_rep[grc][2*grb+1]);
		end
		for (grb = 0; grb < 4; grb = grb + 1) begin : g_hv4
			assign v4[grb] = !(v2_n[2*grb] & v2_n[2*grb+1]);
		end
		for (grb = 0; grb < 2; grb = grb + 1) begin : g_hv8
			assign v8_n[grb] = !(v4[2*grb] | v4[2*grb+1]);
		end
		wire v16 = !(v8_n[0] & v8_n[1]);
		wire hv_clear = !(v16 | r_vhit_a_rep[grc]);

		wire [15:0] vm_n;
		wire [7:0]  m2;
		wire [3:0]  m4_n;
		wire [1:0]  m8;
		wire [15:0] va_n;
		wire [7:0]  a2;
		wire [3:0]  a4_n;
		wire [1:0]  a8;
		for (grb = 0; grb < 16; grb = grb + 1) begin : g_rm1
			assign vm_n[grb] = !(r_vhit_oh_rep[grc][grb] & r_rel_oh_rep[grc][grb]);
			assign va_n[grb] = !(r_vhit_a_oh_rep[grc][grb] & r_rel_oh_rep[grc][grb]);
		end
		for (grb = 0; grb < 8; grb = grb + 1) begin : g_rm2
			assign m2[grb] = !(vm_n[2*grb] & vm_n[2*grb+1]);
			assign a2[grb] = !(va_n[2*grb] & va_n[2*grb+1]);
		end
		for (grb = 0; grb < 4; grb = grb + 1) begin : g_rm4
			assign m4_n[grb] = !(m2[2*grb] | m2[2*grb+1]);
			assign a4_n[grb] = !(a2[2*grb] | a2[2*grb+1]);
		end
		for (grb = 0; grb < 2; grb = grb + 1) begin : g_rm8
			assign m8[grb] = !(m4_n[2*grb] & m4_n[2*grb+1]);
			assign a8[grb] = !(a4_n[2*grb] & a4_n[2*grb+1]);
		end

		assign o_cam_full_rep[grc] = r_cam_full_rep[grc];
		assign o_hv_clear_rep[grc] = hv_clear;
		assign o_v16_rep[grc]      = v16;
		assign o_vrel_or0_rep[grc] = m8[0];
		assign o_vrel_or1_rep[grc] = m8[1];
		assign o_arel_or0_rep[grc] = a8[0];
		assign o_arel_or1_rep[grc] = a8[1];
		assign o_vhit_a_rep[grc]   = r_vhit_a_rep[grc];
		if (grc < 5) begin : g_hcr
			assign o_haz_clear_rep[grc] = !r_cam_full_rep[grc] && (hv_clear || m8[0] || m8[1] || (r_vhit_a_rep[grc] && (a8[0] || a8[1])));
		end
`endif
	end
	endgenerate
	assign grp_hit_int[0] = haz_tag_hit;
	assign grp_hit_int[1] = 1'b0;
	assign grp_hit_int[2] = cam_full;

	assign grp_hit_ext[0] = grp_hit_int[0];
	assign grp_hit_ext[1] = grp_hit_int[1];
	assign grp_hit_ext[2] = grp_hit_int[2];

	assign o_grp_hit = grp_hit_ext;
	assign alloc_ready = !((|r_di_oh) || cam_full);

	always @(*) begin
		alloc_id[3] = !grp_has[0] && !grp_has[1];
		alloc_id[2] = !grp_has[0] && (grp_has[1] || !grp_has[2]);
		if (grp_has[0])      alloc_id[1:0] = grp_id_flat[1:0];
		else if (grp_has[1]) alloc_id[1:0] = grp_id_flat[3:2];
		else if (grp_has[2]) alloc_id[1:0] = grp_id_flat[5:4];
		else                 alloc_id[1:0] = grp_id_flat[7:6];
	end

	wire alloc_go = alloc_valid && !haz_tag_hit && !cam_full;
	wire [1:0] alloc_go_buf;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ag0 (.O(alloc_go_buf[0]), .I0(alloc_go));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ag1 (.O(alloc_go_buf[1]), .I0(alloc_go));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign alloc_go_buf[0] = alloc_go;
	assign alloc_go_buf[1] = alloc_go;
`else
	assign alloc_go_buf = {2{alloc_go}};
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_cam_full <= 1'b0;
			r_cam_full_rep <= {N_HREP{1'b0}};
		end else begin
			r_cam_full <= !rel_any_w && (r_cam_full || (one_empty_w && alloc_go));
			r_cam_full_rep <= {N_HREP{!rel_any_w && (r_cam_full || (one_empty_w && alloc_go))}};
		end
	end

	wire [NUM_ENTRIES-1:0] slot_sel_vec;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_ssv
		assign slot_sel_vec[i] = grp_sel[i / 4] && loc_sel[i];
	end
	endgenerate

	(* keep = "true" *) wire alloc_hit_w = alloc_go && eq_alloc;
	wire unused_rel_ports = &{1'b0, nrel_grp_hit, slot_nrel, eq_cur, release_valid, release_id, release_oh, next_release_valid, next_release_id, pre_load};
	integer k_vh, k_rp;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_di_oh      <= {NUM_ENTRIES{1'b0}};
			r_vhit_a     <= 1'b0;
			r_vhit_aid   <= 4'd0;
			r_vhit_oh    <= {NUM_ENTRIES{1'b0}};
			r_vhit_a_oh  <= {NUM_ENTRIES{1'b0}};
			r_rel_oh     <= {NUM_ENTRIES{1'b0}};
			r_vhit_a_rep <= {N_HREP{1'b0}};
			for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1) begin
				r_vhit_oh_rep[k_rp]   <= {NUM_ENTRIES{1'b0}};
				r_vhit_a_oh_rep[k_rp] <= {NUM_ENTRIES{1'b0}};
				r_rel_oh_rep[k_rp]    <= {NUM_ENTRIES{1'b0}};
			end
		end else begin
			r_rel_oh <= next_release_oh;
			for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1)
				r_rel_oh_rep[k_rp] <= next_release_oh;
			if (di_load)
				r_di_oh <= di_hit_vec;
			else
				r_di_oh <= r_di_oh & ~r_rel_oh;
`ifdef SYNTH_KINTEX7
			if (pre_load_buf[24]) begin
				r_vhit_a     <= alloc_hit_w;
				r_vhit_aid   <= alloc_id;
			end else if (eq_rel_a) begin
				r_vhit_a <= 1'b0;
			end
			if (pre_load_buf[27]) begin
				r_vhit_a_rep[5:0] <= {6{alloc_hit_w}};
			end else if (eq_rel_a) begin
				for (k_rp = 0; k_rp < 6; k_rp = k_rp + 1)
					r_vhit_a_rep[k_rp] <= 1'b0;
			end
			if (pre_load_buf[28]) begin
				r_vhit_a_rep[11:6] <= {6{alloc_hit_w}};
			end else if (eq_rel_a) begin
				for (k_rp = 6; k_rp < N_HREP; k_rp = k_rp + 1)
					r_vhit_a_rep[k_rp] <= 1'b0;
			end
			if (pre_load_buf[25])
				r_vhit_oh <= cur_hit_vec;
			else begin
				for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
					if (r_rel_oh[k_vh])
						r_vhit_oh[k_vh] <= 1'b0;
				end
			end
			if (pre_load_buf[26])
				r_vhit_a_oh <= slot_sel_vec;
			for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1) begin
				if (pre_load_buf[k_rp])
					r_vhit_oh_rep[k_rp] <= cur_hit_vec;
				else begin
					for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
						if (r_rel_oh[k_vh])
							r_vhit_oh_rep[k_rp][k_vh] <= 1'b0;
					end
				end
				if (pre_load_buf[12 + k_rp])
					r_vhit_a_oh_rep[k_rp] <= slot_sel_vec;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (pre_load_buf[24]) begin
				r_vhit_a     <= alloc_hit_w;
				r_vhit_aid   <= alloc_id;
			end else if (eq_rel_a) begin
				r_vhit_a <= 1'b0;
			end
			if (pre_load_buf[27]) begin
				r_vhit_a_rep[5:0] <= {6{alloc_hit_w}};
			end else if (eq_rel_a) begin
				for (k_rp = 0; k_rp < 6; k_rp = k_rp + 1)
					r_vhit_a_rep[k_rp] <= 1'b0;
			end
			if (pre_load_buf[28]) begin
				r_vhit_a_rep[11:6] <= {6{alloc_hit_w}};
			end else if (eq_rel_a) begin
				for (k_rp = 6; k_rp < N_HREP; k_rp = k_rp + 1)
					r_vhit_a_rep[k_rp] <= 1'b0;
			end
			if (pre_load_buf[25])
				r_vhit_oh <= cur_hit_vec;
			else begin
				for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
					if (r_rel_oh[k_vh])
						r_vhit_oh[k_vh] <= 1'b0;
				end
			end
			if (pre_load_buf[26])
				r_vhit_a_oh <= slot_sel_vec;
			for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1) begin
				if (pre_load_buf[k_rp])
					r_vhit_oh_rep[k_rp] <= cur_hit_vec;
				else begin
					for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
						if (r_rel_oh[k_vh])
							r_vhit_oh_rep[k_rp][k_vh] <= 1'b0;
					end
				end
				if (pre_load_buf[12 + k_rp])
					r_vhit_a_oh_rep[k_rp] <= slot_sel_vec;
			end
`else
			if (pre_load_buf[0]) begin
				r_vhit_a     <= alloc_hit_w;
				r_vhit_aid   <= alloc_id;
				r_vhit_oh    <= cur_hit_vec;
				r_vhit_a_oh  <= slot_sel_vec;
				r_vhit_a_rep <= {N_HREP{alloc_hit_w}};
				for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1) begin
					r_vhit_oh_rep[k_rp]   <= cur_hit_vec;
					r_vhit_a_oh_rep[k_rp] <= slot_sel_vec;
				end
			end else begin
				if (eq_rel_a)
					r_vhit_a <= 1'b0;
				for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
					if (r_rel_oh[k_vh])
						r_vhit_oh[k_vh] <= 1'b0;
				end
				for (k_rp = 0; k_rp < N_HREP; k_rp = k_rp + 1) begin
					if (eq_rel_a)
						r_vhit_a_rep[k_rp] <= 1'b0;
					for (k_vh = 0; k_vh < NUM_ENTRIES; k_vh = k_vh + 1) begin
						if (r_rel_oh[k_vh])
							r_vhit_oh_rep[k_rp][k_vh] <= 1'b0;
					end
				end
			end
`endif
		end
	end

	localparam S0_W = 11;
	localparam S1_W = 11;
	generate
	for (i = 0; i < NUM_ENTRIES; i = i + 1) begin : g_ent
		localparam G_IDX = i / 4;
		localparam H_IDX = i / 8;
`ifdef SYNTH_KINTEX7
		(* keep = "true" *) wire slot_sel_i = grp_sel[G_IDX] && loc_sel[i];
		(* keep = "true" *) wire alloc_slot_i = alloc_go_buf[H_IDX] && slot_sel_i;
		(* keep = "true" *) wire val_next_i = alloc_slot_i || i_val_vec[i];
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep = "true" *) wire slot_sel_i = grp_sel[G_IDX] && loc_sel[i];
		(* keep = "true" *) wire alloc_slot_i = alloc_go_buf[H_IDX] && slot_sel_i;
		(* keep = "true" *) wire val_next_i = alloc_slot_i || i_val_vec[i];
`else
		wire slot_sel_i = grp_sel[G_IDX] && loc_sel[i];
		wire alloc_slot_i = alloc_go_buf[H_IDX] && slot_sel_i;
		wire val_next_i = alloc_slot_i || i_val_vec[i];
`endif

		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				valid[i] <= 1'b0;
			else
				valid[i] <= val_next_i;
		end

`ifdef SYNTH_KINTEX7
		wire [2:0] we_tag_buf;
		for (gb = 0; gb < 3; gb = gb + 1) begin : g_we_buf
			(* keep = "true", dont_touch = "true" *)
			LUT1 #(.INIT(2'b10)) u_we (.O(we_tag_buf[gb]), .I0(slot_sel_i));
		end
		always @(posedge clk) begin
			if (we_tag_buf[0])
				tag[i][S0_W-1:0] <= alloc_tag_buf[H_IDX][S0_W-1:0];
			if (we_tag_buf[1])
				tag[i][S0_W+S1_W-1:S0_W] <= alloc_tag_buf[H_IDX][S0_W+S1_W-1:S0_W];
			if (we_tag_buf[2])
				tag[i][TAG_W-1:S0_W+S1_W] <= alloc_tag_buf[H_IDX][TAG_W-1:S0_W+S1_W];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [2:0] we_tag_buf;
		for (gb = 0; gb < 3; gb = gb + 1) begin : g_we_buf
			assign we_tag_buf[gb] = slot_sel_i;
		end
		always @(posedge clk) begin
			if (we_tag_buf[0])
				tag[i][S0_W-1:0] <= alloc_tag_buf[H_IDX][S0_W-1:0];
			if (we_tag_buf[1])
				tag[i][S0_W+S1_W-1:S0_W] <= alloc_tag_buf[H_IDX][S0_W+S1_W-1:S0_W];
			if (we_tag_buf[2])
				tag[i][TAG_W-1:S0_W+S1_W] <= alloc_tag_buf[H_IDX][TAG_W-1:S0_W+S1_W];
		end
`else
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				tag[i] <= {TAG_W{1'b0}};
			else if (alloc_go_buf[H_IDX] && slot_sel_i)
				tag[i] <= alloc_tag_buf[H_IDX];
		end
`endif
	end
	endgenerate
	wire unused_rel_v = release_valid;
endmodule

`default_nettype wire
