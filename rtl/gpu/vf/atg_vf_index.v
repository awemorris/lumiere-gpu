/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_vf_index: VF index stage: walks FIRST..FIRST+COUNT (non-indexed) or
 * reads the index buffer (u16/u32), detects primitive restart, and repeats
 * for INSTANCE_COUNT instances.
 *
 * On kick the walk starts from the STATE words of the VF range (the CP does
 * not change them under a draw; a kick while busy is ignored).
 * Non-indexed: iv_index = FIRST + n, no memory access.  Indexed: the lines
 * of INDEX_BASE + (FIRST + n) * size are read in order (one line request
 * ahead, two line buffers, the tag names the buffer) and each line's
 * indices go out one per cycle; INDEX_BASE is aligned to the index size,
 * so an index never crosses a line.  With PRIM_RESTART[0], 0xFFFF (u16) or
 * 0xFFFFFFFF (u32) goes out as iv_restart.  The whole walk repeats for each
 * instance (iv_inst = FIRST_INSTANCE + i; the lines are read again).
 * iv_last marks the last item of the last instance.  COUNT 0 or
 * INSTANCE_COUNT 0 sends nothing.  rsp_error stops the walk without iv_last
 * (the MC faulted; the CP resets the VF); busy stays up until the requests
 * in flight are answered.  See spec/gpu/vf/atg_vf_index.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_vf_index #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5	/* log2(LINE_BYTES) */
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	input wire kick,
	output wire busy,
	/* Index stream (atg_vf_index -> atg_vf_asm) */
	output wire iv_valid,
	input wire iv_ready,
	output wire [31:0] iv_index,
	output wire [31:0] iv_inst,
	output wire iv_restart,
	output wire iv_last,
	/* AMB client port (design §4.1) */
	output wire req_valid,
	input wire req_ready,
	output wire req_write,
	output wire [ADDR_W-LINE_SHIFT-1:0] req_line,
	output wire [LINE_BYTES-1:0] req_be,
	output wire [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0] req_tag,
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	`include "ghci_regs.vh"
	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam LB = LINE_BYTES * 8;
	localparam NHALVES = LB / 16;

	(* keep = "true" *) wire rst_sync = !rst_n;

	/* Carry-free 32-bit constant comparisons (<= 2 LUT6 levels, 0 CARRY4s) */
	function [5:0] cmp32_const_chunks;
		input [31:0] x;
		input [31:0] c;
		begin
			cmp32_const_chunks[0] = (x[5:0]   == c[5:0]);
			cmp32_const_chunks[1] = (x[11:6]  == c[11:6]);
			cmp32_const_chunks[2] = (x[17:12] == c[17:12]);
			cmp32_const_chunks[3] = (x[23:18] == c[23:18]);
			cmp32_const_chunks[4] = (x[29:24] == c[29:24]);
			cmp32_const_chunks[5] = (x[31:30] == c[31:30]);
		end
	endfunction

	/* STATE indices of the VF range (design §2.3). */
	reg [31:0] s_ibase, s_first, s_count, s_icount, s_finst;
	reg [1:0] s_ifmt;
	reg s_restart;
	reg s_count_nz, s_count_is1, s_icount_nz, s_icount_is1;

	wire we_ibase  = st_we && (st_idx == GHCI_S_INDEX_BASE);
	wire we_ifmt   = st_we && (st_idx == GHCI_S_INDEX_FORMAT);
	wire we_first  = st_we && (st_idx == GHCI_S_FIRST);
	wire we_count  = st_we && (st_idx == GHCI_S_COUNT);
	wire we_rst    = st_we && (st_idx == GHCI_S_PRIM_RESTART);
	wire we_icount = st_we && (st_idx == GHCI_S_INSTANCE_COUNT);
	wire we_finst  = st_we && (st_idx == GHCI_S_FIRST_INSTANCE);

	wire [1:0] we_ibase_b, we_first_b, we_count_b, we_icount_b, we_finst_b;
	genvar gb;
	generate
		for (gb = 0; gb < 2; gb = gb + 1) begin : g_st_we_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ib (.O(we_ibase_b[gb]),  .I0(we_ibase));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_fi (.O(we_first_b[gb]),  .I0(we_first));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_co (.O(we_count_b[gb]),  .I0(we_count));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ic (.O(we_icount_b[gb]), .I0(we_icount));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_fn (.O(we_finst_b[gb]),  .I0(we_finst));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign we_ibase_b[gb] = we_ibase;
			assign we_first_b[gb] = we_first;
			assign we_count_b[gb] = we_count;
			assign we_icount_b[gb] = we_icount;
			assign we_finst_b[gb] = we_finst;
`else
			assign we_ibase_b[gb]  = we_ibase;
			assign we_first_b[gb]  = we_first;
			assign we_count_b[gb]  = we_count;
			assign we_icount_b[gb] = we_icount;
			assign we_finst_b[gb]  = we_finst;
`endif
		end
	endgenerate

	(* keep = "true" *) wire [5:0] wdata_z_c = cmp32_const_chunks(st_wdata, 32'd0);
	(* keep = "true" *) wire [5:0] wdata_1_c = cmp32_const_chunks(st_wdata, 32'd1);
	wire wdata_is0 = &wdata_z_c;
	wire wdata_is1 = &wdata_1_c;

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			s_ibase <= 32'd0;
			s_ifmt <= 2'd0;
			s_first <= 32'd0;
			s_count <= 32'd0;
			s_count_lo_z <= 1'b0;
			s_restart <= 1'b0;
			s_icount <= 32'd0;
			s_finst <= 32'd0;
			s_count_nz <= 1'b0;
			s_count_is1 <= 1'b0;
			s_icount_nz <= 1'b0;
			s_icount_is1 <= 1'b0;
		end else begin
			if (we_ibase_b[0])  s_ibase[15:0]   <= st_wdata[15:0];
			if (we_ibase_b[1])  s_ibase[31:16]  <= st_wdata[31:16];
			if (we_ifmt)        s_ifmt          <= st_wdata[1:0];
			if (we_first_b[0])  s_first[15:0]   <= st_wdata[15:0];
			if (we_first_b[1])  s_first[31:16]  <= st_wdata[31:16];
			if (we_count_b[0])  s_count[15:0]   <= st_wdata[15:0];
			if (we_count_b[1])  s_count[31:16]  <= st_wdata[31:16];
			if (we_count) begin
				s_count_nz   <= !wdata_is0;
				s_count_is1  <= wdata_is1;
				s_count_lo_z <= (st_wdata[15:0] == 16'd0);
			end
			if (we_rst)         s_restart       <= st_wdata[0];
			if (we_icount_b[0]) s_icount[15:0]  <= st_wdata[15:0];
			if (we_icount_b[1]) s_icount[31:16] <= st_wdata[31:16];
			if (we_icount) begin
				s_icount_nz  <= !wdata_is0;
				s_icount_is1 <= wdata_is1;
			end
			if (we_finst_b[0])  s_finst[15:0]   <= st_wdata[15:0];
			if (we_finst_b[1])  s_finst[31:16]  <= st_wdata[31:16];
		end
	end

	/* INDEX_FORMAT: 0 none, 1 u16, 2 (and the reserved 3) u32. */
	wire k_ind = s_ifmt != 2'd0;
	wire [3:0] k_u32_b;
	generate
		for (gb = 0; gb < 4; gb = gb + 1) begin : g_ku32_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ku (.O(k_u32_b[gb]), .I0(s_ifmt[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign k_u32_b[gb] = s_ifmt[1];
`else
			assign k_u32_b[gb] = s_ifmt[1];
`endif
		end
	endgenerate

	wire [31:0] first_sh = {
		k_u32_b[1] ? s_first[29:14] : s_first[30:15],
		k_u32_b[0] ? {s_first[13:0], 2'b00} : {s_first[14:0], 1'b0}
	};
	reg s_count_lo_z;
	wire [31:0] k_a0;
	wire [31:0] k_cm1;
	wire a0_co_unused, cm1_bo_unused;
	am_add #(.W(32)) u_a0 (.a(s_ibase), .b(first_sh), .ci(1'b0), .s(k_a0), .co(a0_co_unused));
	/* Fast 32b nibble-prefix decrementer for k_cm1 = s_count - 1 using pre-registered s_count_lo_z (<= 9 cmos3) */
	wire [7:0] cm1_nib_z;
	wire [7:0] cm1_bin;
	genvar gcm;
	generate
		for (gcm = 0; gcm < 8; gcm = gcm + 1) begin : g_cm1_z
			assign cm1_nib_z[gcm] = ~(|s_count[gcm*4 +: 4]);
		end
		assign cm1_bin[0] = 1'b1;
		assign cm1_bin[1] = cm1_nib_z[0];
		assign cm1_bin[2] = cm1_nib_z[0] && cm1_nib_z[1];
		assign cm1_bin[3] = &cm1_nib_z[2:0];
		assign cm1_bin[4] = s_count_lo_z;
		assign cm1_bin[5] = s_count_lo_z && cm1_nib_z[4];
		assign cm1_bin[6] = s_count_lo_z && cm1_nib_z[4] && cm1_nib_z[5];
		assign cm1_bin[7] = s_count_lo_z && (&cm1_nib_z[6:4]);
		for (gcm = 0; gcm < 8; gcm = gcm + 1) begin : g_cm1_sub
			wire [3:0] sc = s_count[gcm*4 +: 4];
			wire bc = cm1_bin[gcm];
			assign k_cm1[gcm*4 + 0] = sc[0] ^ bc;
			assign k_cm1[gcm*4 + 1] = sc[1] ^ (bc && !sc[0]);
			assign k_cm1[gcm*4 + 2] = sc[2] ^ (bc && !sc[1] && !sc[0]);
			assign k_cm1[gcm*4 + 3] = sc[3] ^ (bc && !sc[2] && !sc[1] && !sc[0]);
		end
	endgenerate
	assign cm1_bo_unused = 1'b0;

	/* Explicit one-hot state flags */
	reg st_is_idle;
	reg st_is_init1;
	reg st_is_init2;
	reg st_is_init3;
	reg st_is_walk;
	reg st_is_drain;

	reg ind, u32;
	wire [3:0] ind_b;
	wire [3:0] u32_b;
	wire [1:0] u32_cm1_b;
	generate
		for (gb = 0; gb < 4; gb = gb + 1) begin : g_iu_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ib (.O(ind_b[gb]), .I0(ind));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ub (.O(u32_b[gb]), .I0(u32));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ind_b[gb] = ind;
			assign u32_b[gb] = u32;
`else
			assign ind_b[gb] = ind;
			assign u32_b[gb] = u32;
`endif
		end
		for (gb = 0; gb < 2; gb = gb + 1) begin : g_ucm_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_uc (.O(u32_cm1_b[gb]), .I0(u32));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign u32_cm1_b[gb] = u32;
`else
			assign u32_cm1_b[gb] = u32;
`endif
		end
	endgenerate

	reg [31:0] k_a0_r;
	reg [31:0] k_cm1_r;
	reg [31:0] a0;
	reg [LA_W-1:0] l0, lend, lend_m1;
	reg l0_eq_lend;

	wire [31:0] k_alast;
	wire al_co_unused;
	am_add #(.W(32)) al (.a(k_a0_r), .b(k_cm1_r), .ci(1'b0), .s(k_alast), .co(al_co_unused));

	wire [LA_W-1:0] lend_m1_w;
	wire lm1_bo_unused;
	am_sub #(.W(LA_W)) u_lend_m1 (.a(lend), .b({{(LA_W-1){1'b0}}, 1'b1}), .bi(1'b0),
		.d(lend_m1_w), .bo(lm1_bo_unused));

	/* Request side: the line sequence l0..lend, once per instance. */
	reg rq_pend, rq_more, rq_slot;
	reg [LA_W-1:0] rq_l;
	reg [31:0] rq_ileft;
	reg rq_ileft_is1;
	reg rq_wrap_r;
	reg [1:0] nalloc;
	reg [1:0] inflight;

	wire [1:0] rq_wrap_b;
	generate
		for (gb = 0; gb < 2; gb = gb + 1) begin : g_rqw_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_rw (.O(rq_wrap_b[gb]), .I0(rq_wrap_r));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign rq_wrap_b[gb] = rq_wrap_r;
`else
			assign rq_wrap_b[gb] = rq_wrap_r;
`endif
		end
	endgenerate

	/* Carry-free LA_W-bit equality checks: l0 == lend and rq_l == lend_m1 */
	localparam integer L_CHUNKS = (LA_W + 2) / 3;
	localparam integer L_CMP_W = L_CHUNKS * 3;
	wire [63:0] l0_pad      = {{(64-LA_W){1'b0}}, l0};
	wire [63:0] lend_pad    = {{(64-LA_W){1'b0}}, lend};
	wire [63:0] rql_pad     = {{(64-LA_W){1'b0}}, rq_l};
	wire [63:0] lend_m1_pad = {{(64-LA_W){1'b0}}, lend_m1};
	wire [L_CMP_W-1:0] l0_ext      = l0_pad[L_CMP_W-1:0];
	wire [L_CMP_W-1:0] lend_ext    = lend_pad[L_CMP_W-1:0];
	wire [L_CMP_W-1:0] rql_ext     = rql_pad[L_CMP_W-1:0];
	wire [L_CMP_W-1:0] lend_m1_ext = lend_m1_pad[L_CMP_W-1:0];
	(* keep = "true" *) wire [L_CHUNKS-1:0] l0_eq_c;
	(* keep = "true" *) wire [L_CHUNKS-1:0] rql_eq_c;
	genvar ge;
	generate
		for (ge = 0; ge < L_CHUNKS; ge = ge + 1) begin : g_eq_chunks
			assign l0_eq_c[ge]  = (l0_ext[3*ge +: 3]   == lend_ext[3*ge +: 3]);
			assign rql_eq_c[ge] = (rql_ext[3*ge +: 3] == lend_m1_ext[3*ge +: 3]);
		end
	endgenerate
	(* keep = "true" *) wire l0_eq_lo  = &l0_eq_c[L_CHUNKS/2-1:0];
	(* keep = "true" *) wire l0_eq_hi  = &l0_eq_c[L_CHUNKS-1:L_CHUNKS/2];
	(* keep = "true" *) wire rql_eq_lo = &rql_eq_c[L_CHUNKS/2-1:0];
	(* keep = "true" *) wire rql_eq_hi = &rql_eq_c[L_CHUNKS-1:L_CHUNKS/2];
	wire l0_eq_lend_w  = l0_eq_lo && l0_eq_hi;
	wire rql_eq_lm1_w  = rql_eq_lo && rql_eq_hi;

	wire [LA_W-1:0] rql_next;
	wire [31:0] rqileft_next;
	wire rql_hi_co_unused, rqi_bo_unused;
	reg rql_lo_all1;
	wire rql_g0  = &rq_l[3:0];
	wire rql_g1  = &rq_l[7:4];
	wire rql_g2  = &rq_l[11:8];
	wire rql_c4  = rql_g0;
	wire rql_c8  = rql_g0 && rql_g1;
	wire rql_c12 = rql_g0 && rql_g1 && rql_g2;
	assign rql_next[0]  = ~rq_l[0];
	assign rql_next[1]  = rq_l[1]  ^ rq_l[0];
	assign rql_next[2]  = rq_l[2]  ^ (&rq_l[1:0]);
	assign rql_next[3]  = rq_l[3]  ^ (&rq_l[2:0]);
	assign rql_next[4]  = rq_l[4]  ^ rql_c4;
	assign rql_next[5]  = rq_l[5]  ^ (rql_c4 && rq_l[4]);
	assign rql_next[6]  = rq_l[6]  ^ (rql_c4 && (&rq_l[5:4]));
	assign rql_next[7]  = rq_l[7]  ^ (rql_c4 && (&rq_l[6:4]));
	assign rql_next[8]  = rq_l[8]  ^ rql_c8;
	assign rql_next[9]  = rq_l[9]  ^ (rql_c8 && rq_l[8]);
	assign rql_next[10] = rq_l[10] ^ (rql_c8 && (&rq_l[9:8]));
	assign rql_next[11] = rq_l[11] ^ (rql_c8 && (&rq_l[10:8]));
	assign rql_next[12] = rq_l[12] ^ rql_c12;
	wire rql_lo_all1_nxt = sel_rql_b[0] ? (&ka0_ext[LINE_SHIFT+12:LINE_SHIFT]) :
	                       (rq_wrap_b[0] ? (&l0[12:0]) : (&rq_l[12:1] && !rq_l[0]));
	/* Fast 4-bit nibble-prefix incrementer for rq_l[LA_W-1:13] (<= 6 cmos3) */
	localparam integer RQL_HI_W = LA_W - 13;
	localparam integer RQL_PAD_W = ((RQL_HI_W + 3) / 4) * 4;
	localparam integer RQL_NIBS = RQL_PAD_W / 4;
	wire [RQL_PAD_W-1:0] rql_hi_pad = {{(RQL_PAD_W-RQL_HI_W){1'b0}}, rq_l[LA_W-1:13]};
	wire [RQL_NIBS-1:0]  rql_hi_nib_all1;
	wire [RQL_NIBS-1:0]  rql_hi_cin;
	wire [RQL_PAD_W-1:0] rql_hi_next_pad;
	genvar grh;
	generate
		for (grh = 0; grh < RQL_NIBS; grh = grh + 1) begin : g_rql_hi_all1
			assign rql_hi_nib_all1[grh] = &rql_hi_pad[grh*4 +: 4];
		end
		assign rql_hi_cin[0] = rql_lo_all1;
		for (grh = 1; grh < RQL_NIBS; grh = grh + 1) begin : g_rql_hi_cin
			assign rql_hi_cin[grh] = rql_lo_all1 && (&rql_hi_nib_all1[grh-1:0]);
		end
		for (grh = 0; grh < RQL_NIBS; grh = grh + 1) begin : g_rql_hi_sum
			wire [3:0] rhn = rql_hi_pad[grh*4 +: 4];
			wire rhc = rql_hi_cin[grh];
			assign rql_hi_next_pad[grh*4 + 0] = rhn[0] ^ rhc;
			assign rql_hi_next_pad[grh*4 + 1] = rhn[1] ^ (rhc && rhn[0]);
			assign rql_hi_next_pad[grh*4 + 2] = rhn[2] ^ (rhc && rhn[1] && rhn[0]);
			assign rql_hi_next_pad[grh*4 + 3] = rhn[3] ^ (rhc && rhn[2] && rhn[1] && rhn[0]);
		end
	endgenerate
	assign rql_next[LA_W-1:13] = rql_hi_next_pad[RQL_HI_W-1:0];
	assign rql_hi_co_unused = 1'b0;
	/* Fast 32b nibble-prefix decrementer for rqileft_next = rq_ileft - 1 (<= 10 cmos3) */
	wire [7:0] rqi_nib_z;
	wire [7:0] rqi_lo3_z;
	wire [3:0] rqi_b8_z;
	wire [7:0] rqi_bin;
	genvar grq;
	generate
		for (grq = 0; grq < 8; grq = grq + 1) begin : g_rqi_z
			assign rqi_nib_z[grq] = ~(|rq_ileft[grq*4 +: 4]);
			assign rqi_lo3_z[grq] = ~(|rq_ileft[grq*4 +: 3]);
		end
		for (grq = 0; grq < 4; grq = grq + 1) begin : g_rqi_b8
			assign rqi_b8_z[grq] = rqi_nib_z[2*grq+1] && rqi_nib_z[2*grq];
		end
		assign rqi_bin[0] = 1'b1;
		assign rqi_bin[1] = rqi_nib_z[0];
		assign rqi_bin[2] = rqi_b8_z[0];
		assign rqi_bin[3] = rqi_b8_z[0] && rqi_nib_z[2];
		assign rqi_bin[4] = rqi_b8_z[0] && rqi_b8_z[1];
		assign rqi_bin[5] = (rqi_b8_z[0] && rqi_b8_z[1]) && rqi_nib_z[4];
		assign rqi_bin[6] = (rqi_b8_z[0] && rqi_b8_z[1]) && rqi_b8_z[2];
		assign rqi_bin[7] = (rqi_b8_z[0] && rqi_b8_z[1]) && (rqi_b8_z[2] && rqi_nib_z[6]);
		for (grq = 0; grq < 8; grq = grq + 1) begin : g_rqi_sub
			wire [3:0] rqi_n = rq_ileft[grq*4 +: 4];
			wire rqi_b = rqi_bin[grq];
			assign rqileft_next[grq*4 + 0] = rqi_n[0] ^ rqi_b;
			assign rqileft_next[grq*4 + 1] = rqi_n[1] ^ (rqi_b && !rqi_n[0]);
			assign rqileft_next[grq*4 + 2] = rqi_n[2] ^ (rqi_b && !rqi_n[1] && !rqi_n[0]);
			assign rqileft_next[grq*4 + 3] = rqi_n[3] ^ (rqi_b && rqi_lo3_z[grq]);
		end
	endgenerate
	assign rqi_bo_unused = 1'b0;

	(* keep = "true" *) wire [5:0] rqi_is2_c = cmp32_const_chunks(rq_ileft, 32'd2);
	wire rqi_is2 = &rqi_is2_c;

	/* Line buffers, filled by tag (with 16 buffered write-enables each). */
	reg [LB-1:0] buf0, buf1;
	reg [NHALVES-1:0] buf0_all1, buf1_all1;
	reg [1:0] bv;

	wire rd_rsp = rsp_valid && !rsp_write;
	wire rd_rsp_ok = rd_rsp && !rsp_error;
	(* keep = "true" *) wire we_buf0_m0 = rd_rsp && !rsp_tag[0];
	(* keep = "true" *) wire we_buf0_m1 = rd_rsp && !rsp_tag[0];
	(* keep = "true" *) wire we_buf0_a  = rd_rsp && !rsp_tag[0];
	(* keep = "true" *) wire we_buf1_m0 = rd_rsp && rsp_tag[0];
	(* keep = "true" *) wire we_buf1_m1 = rd_rsp && rsp_tag[0];
	(* keep = "true" *) wire we_buf1_a  = rd_rsp && rsp_tag[0];
	(* keep = "true" *) reg [15:0] bv_free0_r, bv_free1_r;
	wire [15:0] we_buf0_b, we_buf1_b;
	generate
		for (gb = 0; gb < 16; gb = gb + 1) begin : g_buf_we
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_b0 (.O(we_buf0_b[gb]), .I0(gb < 8 ? we_buf0_m0 : we_buf0_m1));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_b1 (.O(we_buf1_b[gb]), .I0(gb < 8 ? we_buf1_m0 : we_buf1_m1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign we_buf0_b[gb] = gb < 8 ? we_buf0_m0 : we_buf0_m1;
			assign we_buf1_b[gb] = gb < 8 ? we_buf1_m0 : we_buf1_m1;
`else
			assign we_buf0_b[gb] = bv_free0_r[gb] && (gb < 8 ? we_buf0_m0 : we_buf0_m1);
			assign we_buf1_b[gb] = bv_free1_r[gb] && (gb < 8 ? we_buf1_m0 : we_buf1_m1);
`endif
		end
	endgenerate

	wire [NHALVES-1:0] rdata_h_all1;
	generate
		for (gb = 0; gb < NHALVES; gb = gb + 1) begin : g_rh_all1
			(* keep = "true" *) wire [3:0] h_c;
			assign h_c[0] = &rsp_rdata[16*gb + 0  +: 4];
			assign h_c[1] = &rsp_rdata[16*gb + 4  +: 4];
			assign h_c[2] = &rsp_rdata[16*gb + 8  +: 4];
			assign h_c[3] = &rsp_rdata[16*gb + 12 +: 4];
			assign rdata_h_all1[gb] = &h_c;
		end
	endgenerate

	/* Emit side. */
	reg e_slot;
	wire [3:0] e_slot_b;
	reg [31:0] e_a;
	(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *) reg [1:0] e_a_w_rep [0:7];
	(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *) reg [1:0] e_a4_rep;
	(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *) reg [1:0] e_a1_rep;
	reg [31:0] e_v;
	reg [31:0] e_n;
	reg e_n_is1;
	reg e_line_cross;
	reg [31:0] e_inst, e_ileft;
	reg e_ileft_is1;

	generate
		for (gb = 0; gb < 4; gb = gb + 1) begin : g_eslot_buf
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_es (.O(e_slot_b[gb]), .I0(e_slot));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign e_slot_b[gb] = e_slot;
`else
			assign e_slot_b[gb] = e_slot;
`endif
		end
	endgenerate

	/* Registered output stage for iv_* */
	reg iv_valid_r, iv_restart_r, iv_last_r;
	reg [31:0] iv_index_r, iv_inst_r;

	/* Word selection from buf0 / buf1 (2 LUT6 levels, fanout <= 16) */
	wire [31:0] w32;
	generate
		for (gb = 0; gb < 32; gb = gb + 1) begin : g_w32_mux
			localparam integer GRP = gb / 4;
			localparam integer SGRP = gb / 8;
			localparam integer HGRP = gb / 16;
			wire [1:0] wsel = e_a_w_rep[GRP];
			(* keep = "true" *) wire m0_lo = buf0[{1'b0, wsel, gb[4:0]}];
			(* keep = "true" *) wire m0_hi = buf0[{1'b1, wsel, gb[4:0]}];
			(* keep = "true" *) wire m1_lo = buf1[{1'b0, wsel, gb[4:0]}];
			(* keep = "true" *) wire m1_hi = buf1[{1'b1, wsel, gb[4:0]}];
			(* keep = "true" *) wire w_bit = e_slot_b[SGRP] ?
				(e_a4_rep[HGRP] ? m1_hi : m1_lo) :
				(e_a4_rep[HGRP] ? m0_hi : m0_lo);
			assign w32[gb] = w_bit;
		end
	endgenerate

	wire [2:0] w_idx = e_a[4:2];
	wire h0_all1 = e_slot ? buf1_all1[{w_idx, 1'b0}] : buf0_all1[{w_idx, 1'b0}];
	wire h1_all1 = e_slot ? buf1_all1[{w_idx, 1'b1}] : buf0_all1[{w_idx, 1'b1}];
	wire raw_is_restart = ind && s_restart && (u32 ? (h0_all1 && h1_all1) : (e_a[1] ? h1_all1 : h0_all1));

	wire e_have_walk = st_is_walk && (!ind || (e_slot ? bv[1] : bv[0]));
	wire out_free = !iv_valid_r || iv_ready;
	wire take = e_have_walk && out_free;

	assign iv_valid   = iv_valid_r;
	assign iv_index   = iv_index_r;
	assign iv_inst    = iv_inst_r;
	assign iv_restart = iv_restart_r;
	assign iv_last    = iv_last_r;
	assign busy       = !st_is_idle || iv_valid_r;

	wire inst_end = e_n_is1;
	(* keep = "true" *) wire e_have_inst = st_is_walk && inst_end && (!ind || (e_slot ? bv[1] : bv[0]));
	wire [31:0] e_anext;
	wire [31:0] ev_next;
	wire [31:0] einst_next;
	wire [31:0] en_next;
	wire [31:0] eileft_next;
	wire ad_co_unused, ev_co_unused, ei_co_unused, en_bo_unused, eil_bo_unused;

	wire ea_c2 = u32 ? 1'b1 : e_a[1];
	wire ea_c3 = e_a[2] && ea_c2;
	wire ea_c4 = e_a[3] && ea_c3;
	wire ea_ci_hi = u32 ? (&e_a[4:2]) : (&e_a[4:1]);
	assign e_anext[0] = e_a[0];
	assign e_anext[1] = u32 ? e_a[1] : ~e_a[1];
	assign e_anext[2] = e_a[2] ^ ea_c2;
	assign e_anext[3] = e_a[3] ^ ea_c3;
	assign e_anext[4] = e_a[4] ^ ea_c4;
	am_add #(.W(27)) ad (.a(e_a[31:5]), .b(27'd0), .ci(ea_ci_hi), .s(e_anext[31:5]), .co(ad_co_unused));
	am_add #(.W(32)) u_ev_inc (.a(e_v), .b(32'd0), .ci(1'b1), .s(ev_next), .co(ev_co_unused));
	am_add #(.W(32)) u_einst_inc (.a(e_inst), .b(32'd0), .ci(1'b1), .s(einst_next), .co(ei_co_unused));
	/* Fast 32b nibble-prefix decrementers for en_next = e_n - 1 and eileft_next = e_ileft - 1 (<= 9 cmos3) */
	wire [7:0] en_nib_z, en_lo3_z, eil_nib_z, eil_lo3_z;
	wire [3:0] en_b8_z, eil_b8_z;
	wire [7:0] en_bin, eil_bin;
	genvar ged;
	generate
		for (ged = 0; ged < 8; ged = ged + 1) begin : g_ed_z
			assign en_nib_z[ged]  = ~(|e_n[ged*4 +: 4]);
			assign en_lo3_z[ged]  = ~(|e_n[ged*4 +: 3]);
			assign eil_nib_z[ged] = ~(|e_ileft[ged*4 +: 4]);
			assign eil_lo3_z[ged] = ~(|e_ileft[ged*4 +: 3]);
		end
		for (ged = 0; ged < 4; ged = ged + 1) begin : g_ed_b8
			assign en_b8_z[ged]  = en_nib_z[2*ged+1]  && en_nib_z[2*ged];
			assign eil_b8_z[ged] = eil_nib_z[2*ged+1] && eil_nib_z[2*ged];
		end
		assign en_bin[0] = 1'b1;
		assign en_bin[1] = en_nib_z[0];
		assign en_bin[2] = en_b8_z[0];
		assign en_bin[3] = en_b8_z[0] && en_nib_z[2];
		assign en_bin[4] = en_b8_z[0] && en_b8_z[1];
		assign en_bin[5] = (en_b8_z[0] && en_b8_z[1]) && en_nib_z[4];
		assign en_bin[6] = (en_b8_z[0] && en_b8_z[1]) && en_b8_z[2];
		assign en_bin[7] = (en_b8_z[0] && en_b8_z[1]) && (en_b8_z[2] && en_nib_z[6]);
		assign eil_bin[0] = 1'b1;
		assign eil_bin[1] = eil_nib_z[0];
		assign eil_bin[2] = eil_b8_z[0];
		assign eil_bin[3] = eil_b8_z[0] && eil_nib_z[2];
		assign eil_bin[4] = eil_b8_z[0] && eil_b8_z[1];
		assign eil_bin[5] = (eil_b8_z[0] && eil_b8_z[1]) && eil_nib_z[4];
		assign eil_bin[6] = (eil_b8_z[0] && eil_b8_z[1]) && eil_b8_z[2];
		assign eil_bin[7] = (eil_b8_z[0] && eil_b8_z[1]) && (eil_b8_z[2] && eil_nib_z[6]);
		for (ged = 0; ged < 8; ged = ged + 1) begin : g_ed_sub
			wire [3:0] enn  = e_n[ged*4 +: 4];
			wire [3:0] eiln = e_ileft[ged*4 +: 4];
			wire enb  = en_bin[ged];
			wire eilb = eil_bin[ged];
			assign en_next[ged*4 + 0]     = enn[0]  ^ enb;
			assign en_next[ged*4 + 1]     = enn[1]  ^ (enb  && !enn[0]);
			assign en_next[ged*4 + 2]     = enn[2]  ^ (enb  && !enn[1]  && !enn[0]);
			assign en_next[ged*4 + 3]     = enn[3]  ^ (enb  && en_lo3_z[ged]);
			assign eileft_next[ged*4 + 0] = eiln[0] ^ eilb;
			assign eileft_next[ged*4 + 1] = eiln[1] ^ (eilb && !eiln[0]);
			assign eileft_next[ged*4 + 2] = eiln[2] ^ (eilb && !eiln[1] && !eiln[0]);
			assign eileft_next[ged*4 + 3] = eiln[3] ^ (eilb && eil_lo3_z[ged]);
		end
	endgenerate
	assign en_bo_unused  = 1'b0;
	assign eil_bo_unused = 1'b0;

	(* keep = "true" *) wire [5:0] en_is2_c = cmp32_const_chunks(e_n, 32'd2);
	(* keep = "true" *) wire [5:0] eil_is2_c = cmp32_const_chunks(e_ileft, 32'd2);
	wire en_is2 = &en_is2_c;
	wire eil_is2 = &eil_is2_c;

	/* Line boundary crossing pre-registered in e_line_cross */
	wire pop = st_is_walk && ind && (e_slot ? bv[1] : bv[0]) && out_free && (e_n_is1 || e_line_cross);
	wire rq_cand = !rq_pend && st_is_walk && ind && rq_more;
	wire rq_new = rq_cand && (!nalloc[1] || pop);

	assign req_valid = rq_pend;
	assign req_write = 1'b0;
	assign req_line = rq_l;
	assign req_be = {LINE_BYTES{1'b0}};
	assign req_wdata = {LB{1'b0}};
	assign req_tag = {7'd0, rq_slot};
	wire rq_take = rq_pend && req_ready;

	wire [1:0] inflight_next;
	assign inflight_next[1] = (inflight[1] && (rq_take || !rd_rsp)) ||
		(inflight[0] && rq_take && !rd_rsp);
	assign inflight_next[0] = (inflight[1] && !rq_take && rd_rsp) ||
		(inflight[0] && (rq_take == rd_rsp)) ||
		(!inflight[1] && !inflight[0] && rq_take && !rd_rsp);

	wire rq_pend_next     = !rq_take && (rq_pend || rq_new);
	wire rq_wrap_next     = st_is_init2 ? l0_eq_lend_w : (rq_take ? (rq_wrap_r ? l0_eq_lend : rql_eq_lm1_w) : rq_wrap_r);
	wire rq_ileft_is1_nxt = st_is_init1 ? s_icount_is1 : ((rq_take && rq_wrap_r) ? rqi_is2 : rq_ileft_is1);

	/* Buffered clock-enables and mux selects (<= 3 LUT levels, fanout <= 16) */
	wire [1:0] st_init1_b;
`ifdef SYNTH_KINTEX7
	(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
	LUT1 #(.INIT(2'b10)) u_sti1_0 (.O(st_init1_b[0]), .I0(st_is_init1));
	(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
	LUT1 #(.INIT(2'b10)) u_sti1_1 (.O(st_init1_b[1]), .I0(st_is_init1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign st_init1_b[0] = st_is_init1;
	assign st_init1_b[1] = st_is_init1;
`else
	assign st_init1_b[0] = st_is_init1;
	assign st_init1_b[1] = st_is_init1;
`endif

	wire ce_kick      = st_is_idle && kick && s_count_nz && s_icount_nz;
	wire ce_walk_step = st_is_init3 || (e_have_walk && out_free);
	wire ce_inst_step = st_is_init3 || (e_have_inst && out_free);
	wire ce_rql_step  = st_init1_b[0] || rq_take;
	wire ce_rqi_step  = st_init1_b[0] || (rq_take && rq_wrap_r);
	wire ce_iv_out    = take;
	wire sel_walk_rst = st_is_init3 || inst_end;

	wire [1:0] ce_ka0_b, ce_kcm1_b, ce_a0_b, ce_l0_b, ce_lend_b, ce_lm1_b;
	wire [1:0] ce_ea_b, ce_ea_rep_b, ce_ev_b, ce_en_b, ce_einst_b, ce_eil_b, ce_rql_b, ce_rqi_b, ce_ivi_b, ce_ivinst_b;
	wire [1:0] sel_ea_b, sel_ev_b, sel_en_b, sel_ei_b, sel_eil_b, sel_rql_b, sel_rqi_b;
	generate
		for (gb = 0; gb < 2; gb = gb + 1) begin : g_walk_ce
`ifdef SYNTH_KINTEX7
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ka0  (.O(ce_ka0_b[gb]),    .I0(ce_kick));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_kcm  (.O(ce_kcm1_b[gb]),   .I0(ce_kick));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ca0  (.O(ce_a0_b[gb]),     .I0(st_init1_b[0]));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_cl0  (.O(ce_l0_b[gb]),     .I0(st_init1_b[0]));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_cle  (.O(ce_lend_b[gb]),   .I0(st_init1_b[0]));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_clm  (.O(ce_lm1_b[gb]),    .I0(st_is_init2));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ea   (.O(ce_ea_b[gb]),     .I0(ce_walk_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ear  (.O(ce_ea_rep_b[gb]), .I0(ce_walk_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ev   (.O(ce_ev_b[gb]),     .I0(ce_walk_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_en   (.O(ce_en_b[gb]),     .I0(ce_walk_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ei   (.O(ce_einst_b[gb]),  .I0(ce_inst_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_eil  (.O(ce_eil_b[gb]),    .I0(ce_inst_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_rql  (.O(ce_rql_b[gb]),    .I0(ce_rql_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_rqi  (.O(ce_rqi_b[gb]),    .I0(ce_rqi_step));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ivi  (.O(ce_ivi_b[gb]),    .I0(ce_iv_out));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_ivn  (.O(ce_ivinst_b[gb]), .I0(ce_iv_out));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_sea  (.O(sel_ea_b[gb]),    .I0(sel_walk_rst));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_sev  (.O(sel_ev_b[gb]),    .I0(sel_walk_rst));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_sen  (.O(sel_en_b[gb]),    .I0(sel_walk_rst));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_sei  (.O(sel_ei_b[gb]),    .I0(st_is_init3));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_seil (.O(sel_eil_b[gb]),   .I0(st_is_init3));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_srql (.O(sel_rql_b[gb]),   .I0(st_init1_b[1]));
			(* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
			LUT1 #(.INIT(2'b10)) u_srqi (.O(sel_rqi_b[gb]),   .I0(st_init1_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ce_ka0_b[gb] = ce_kick;
			assign ce_kcm1_b[gb] = ce_kick;
			assign ce_a0_b[gb] = st_init1_b[0];
			assign ce_l0_b[gb] = st_init1_b[0];
			assign ce_lend_b[gb] = st_init1_b[0];
			assign ce_lm1_b[gb] = st_is_init2;
			assign ce_ea_b[gb] = ce_walk_step;
			assign ce_ea_rep_b[gb] = ce_walk_step;
			assign ce_ev_b[gb] = ce_walk_step;
			assign ce_en_b[gb] = ce_walk_step;
			assign ce_einst_b[gb] = ce_inst_step;
			assign ce_eil_b[gb] = ce_inst_step;
			assign ce_rql_b[gb] = ce_rql_step;
			assign ce_rqi_b[gb] = ce_rqi_step;
			assign ce_ivi_b[gb] = ce_iv_out;
			assign ce_ivinst_b[gb] = ce_iv_out;
			assign sel_ea_b[gb] = sel_walk_rst;
			assign sel_ev_b[gb] = sel_walk_rst;
			assign sel_en_b[gb] = sel_walk_rst;
			assign sel_ei_b[gb] = st_is_init3;
			assign sel_eil_b[gb] = st_is_init3;
			assign sel_rql_b[gb] = st_init1_b[1];
			assign sel_rqi_b[gb] = st_init1_b[1];
`else
			assign ce_ka0_b[gb]    = ce_kick;
			assign ce_kcm1_b[gb]   = ce_kick;
			assign ce_a0_b[gb]     = st_init1_b[0];
			assign ce_l0_b[gb]     = st_init1_b[0];
			assign ce_lend_b[gb]   = st_init1_b[0];
			assign ce_lm1_b[gb]    = st_is_init2;
			assign ce_ea_b[gb]     = ce_walk_step;
			assign ce_ea_rep_b[gb] = ce_walk_step;
			assign ce_ev_b[gb]     = ce_walk_step;
			assign ce_en_b[gb]     = ce_walk_step;
			assign ce_einst_b[gb]  = ce_inst_step;
			assign ce_eil_b[gb]    = ce_inst_step;
			assign ce_rql_b[gb]    = ce_rql_step;
			assign ce_rqi_b[gb]    = ce_rqi_step;
			assign ce_ivi_b[gb]    = ce_iv_out;
			assign ce_ivinst_b[gb] = ce_iv_out;
			assign sel_ea_b[gb]    = sel_walk_rst;
			assign sel_ev_b[gb]    = sel_walk_rst;
			assign sel_en_b[gb]    = sel_walk_rst;
			assign sel_ei_b[gb]    = st_is_init3;
			assign sel_eil_b[gb]   = st_is_init3;
			assign sel_rql_b[gb]   = st_init1_b[1];
			assign sel_rqi_b[gb]   = st_init1_b[1];
`endif
		end
	endgenerate

	wire [31:0] next_ea_val = {
		sel_ea_b[1] ? a0[31:16] : e_anext[31:16],
		sel_ea_b[0] ? a0[15:0]  : e_anext[15:0]
	};
	wire [31:0] next_ev_val = {
		sel_ev_b[1] ? s_first[31:16] : ev_next[31:16],
		sel_ev_b[0] ? s_first[15:0]  : ev_next[15:0]
	};
	wire [31:0] next_en_val = {
		sel_en_b[1] ? s_count[31:16] : en_next[31:16],
		sel_en_b[0] ? s_count[15:0]  : en_next[15:0]
	};
	wire [31:0] next_ei_val = {
		sel_ei_b[1] ? s_finst[31:16] : einst_next[31:16],
		sel_ei_b[0] ? s_finst[15:0]  : einst_next[15:0]
	};
	wire [31:0] next_eil_val = {
		sel_eil_b[1] ? s_icount[31:16] : eileft_next[31:16],
		sel_eil_b[0] ? s_icount[15:0]  : eileft_next[15:0]
	};
	wire [63:0] ka0_ext = {32'd0, k_a0_r};
	wire [63:0] kalast_ext = {32'd0, k_alast};
	wire [LA_W-1:0] next_rql_val = {
		sel_rql_b[1] ? ka0_ext[ADDR_W-1:LINE_SHIFT+14] : (rq_wrap_b[1] ? l0[LA_W-1:14] : rql_next[LA_W-1:14]),
		sel_rql_b[0] ? ka0_ext[LINE_SHIFT+13:LINE_SHIFT] : (rq_wrap_b[0] ? l0[13:0] : rql_next[13:0])
	};
	wire [31:0] next_rqi_val = {
		sel_rqi_b[1] ? s_icount[31:16] : rqileft_next[31:16],
		sel_rqi_b[0] ? s_icount[15:0]  : rqileft_next[15:0]
	};

	wire [31:0] next_iv_index;
	generate
		for (gb = 0; gb < 16; gb = gb + 1) begin : g_iv_idx_mux
			localparam integer RGRP = gb / 8;
			assign next_iv_index[gb] = ind_b[RGRP] ?
				(u32_b[RGRP] ? w32[gb] : (e_a1_rep[RGRP] ? w32[gb + 16] : w32[gb])) :
				e_v[gb];
			assign next_iv_index[gb + 16] = ind_b[RGRP + 2] ?
				(u32_b[RGRP + 2] ? w32[gb + 16] : 1'b0) :
				e_v[gb + 16];
		end
	endgenerate

	integer bi, wi;
	always @(posedge clk) begin
		for (bi = 0; bi < 16; bi = bi + 1) begin
			if (we_buf0_b[bi])
				buf0[16*bi +: 16] <= rsp_rdata[16*bi +: 16];
			if (we_buf1_b[bi])
				buf1[16*bi +: 16] <= rsp_rdata[16*bi +: 16];
		end
		if (we_buf0_a)
			buf0_all1 <= rdata_h_all1;
		if (we_buf1_a)
			buf1_all1 <= rdata_h_all1;
	end

	/* Flattened next-state logic for bv, bv_free*_r, and nalloc to avoid chained priority trees */
	wire drain_done = st_is_drain && !rq_pend && (rd_rsp ? (inflight == 2'd1) : (inflight == 2'd0));
	wire [1:0] nalloc_next;
	assign nalloc_next[1] = !drain_done && (
		(nalloc[1] && (rq_cand || !pop)) ||
		(nalloc[0] && rq_cand && !pop)
	);
	assign nalloc_next[0] = !drain_done && (
		(nalloc[1] && !rq_cand && pop) ||
		(nalloc[0] && (rq_cand == pop)) ||
		(!nalloc[1] && !nalloc[0] && rq_cand && !pop)
	);
	wire rq_more_next = st_is_init3 ? ind :
		(st_is_drain && drain_done) ? 1'b0 :
		(rq_take && rq_wrap_r && rq_ileft_is1) ? 1'b0 : rq_more;
	wire bv0_next = drain_done ? 1'b0 : (pop && !e_slot) ? 1'b0 : (rd_rsp_ok && !rsp_tag[0]) ? 1'b1 : bv[0];
	wire bv1_next = drain_done ? 1'b0 : (pop &&  e_slot) ? 1'b0 : (rd_rsp_ok &&  rsp_tag[0]) ? 1'b1 : bv[1];
	wire bv0_free_next = drain_done ? 1'b1 : (pop && !e_slot) ? 1'b1 : (rd_rsp_ok && !rsp_tag[0]) ? 1'b0 : bv_free0_r[0];
	wire bv1_free_next = drain_done ? 1'b1 : (pop &&  e_slot) ? 1'b1 : (rd_rsp_ok &&  rsp_tag[0]) ? 1'b0 : bv_free1_r[0];

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			st_is_idle  <= 1'b1;
			st_is_init1 <= 1'b0;
			st_is_init2 <= 1'b0;
			st_is_init3 <= 1'b0;
			st_is_walk  <= 1'b0;
			st_is_drain <= 1'b0;
			ind <= 1'b0;
			u32 <= 1'b0;
			k_a0_r <= 32'd0;
			k_cm1_r <= 32'd0;
			a0 <= 32'd0;
			l0 <= {LA_W{1'b0}};
			lend <= {LA_W{1'b0}};
			lend_m1 <= {LA_W{1'b0}};
			l0_eq_lend <= 1'b0;
			rq_pend <= 1'b0;
			rq_more <= 1'b0;
			rq_slot <= 1'b0;
			rq_l <= {LA_W{1'b0}};
			rql_lo_all1 <= 1'b0;
			rq_ileft <= 32'd0;
			rq_ileft_is1 <= 1'b0;
			rq_wrap_r <= 1'b0;
			nalloc <= 2'd0;
			inflight <= 2'd0;
			bv <= 2'b00;
			bv_free0_r <= 16'hFFFF;
			bv_free1_r <= 16'hFFFF;
			e_slot <= 1'b0;
			e_a <= 32'd0;
			for (wi = 0; wi < 8; wi = wi + 1)
				e_a_w_rep[wi] <= 2'd0;
			e_a4_rep <= 2'd0;
			e_a1_rep <= 2'd0;
			e_v <= 32'd0;
			e_n <= 32'd0;
			e_n_is1 <= 1'b0;
			e_line_cross <= 1'b0;
			e_inst <= 32'd0;
			e_ileft <= 32'd0;
			e_ileft_is1 <= 1'b0;
			iv_valid_r <= 1'b0;
			iv_index_r <= 32'd0;
			iv_inst_r <= 32'd0;
			iv_restart_r <= 1'b0;
			iv_last_r <= 1'b0;
		end else begin
			bv <= {bv1_next, bv0_next};
			bv_free0_r <= {16{bv0_free_next}};
			bv_free1_r <= {16{bv1_free_next}};
			inflight <= inflight_next;

			if (ce_ka0_b[0])  k_a0_r[15:0]     <= k_a0[15:0];
			if (ce_ka0_b[1])  k_a0_r[31:16]    <= k_a0[31:16];
			if (ce_kcm1_b[0]) k_cm1_r[15:0]    <= k_u32_b[2] ? {k_cm1[13:0], 2'b00} : {k_cm1[14:0], 1'b0};
			if (ce_kcm1_b[1]) k_cm1_r[31:16]   <= k_u32_b[3] ? k_cm1[29:14]       : k_cm1[30:15];

			if (ce_a0_b[0])   a0[15:0]         <= k_a0_r[15:0];
			if (ce_a0_b[1])   a0[31:16]        <= k_a0_r[31:16];
			if (ce_l0_b[0])   l0[13:0]         <= ka0_ext[LINE_SHIFT+13:LINE_SHIFT];
			if (ce_l0_b[1])   l0[LA_W-1:14]    <= ka0_ext[ADDR_W-1:LINE_SHIFT+14];
			if (ce_lend_b[0]) lend[13:0]       <= kalast_ext[LINE_SHIFT+13:LINE_SHIFT];
			if (ce_lend_b[1]) lend[LA_W-1:14]  <= kalast_ext[ADDR_W-1:LINE_SHIFT+14];
			if (ce_lm1_b[0])  lend_m1[13:0]    <= lend_m1_w[13:0];
			if (ce_lm1_b[1])  lend_m1[LA_W-1:14] <= lend_m1_w[LA_W-1:14];

			if (ce_rql_b[0]) begin
				rq_l[13:0]   <= next_rql_val[13:0];
				rql_lo_all1  <= rql_lo_all1_nxt;
			end
			if (ce_rql_b[1]) rq_l[LA_W-1:14]   <= next_rql_val[LA_W-1:14];
			if (ce_rqi_b[0]) rq_ileft[15:0]    <= next_rqi_val[15:0];
			if (ce_rqi_b[1]) rq_ileft[31:16]   <= next_rqi_val[31:16];

			rq_pend      <= rq_pend_next;
			rq_wrap_r    <= rq_wrap_next;
			rq_ileft_is1 <= rq_ileft_is1_nxt;
			rq_more      <= rq_more_next;
			if (!rq_take && rq_new)
				rq_slot <= !rq_slot;
			nalloc <= nalloc_next;
			if (pop)
				e_slot <= e_slot; /* bv/bv_free*_r handled above by bv*_next */

			if (ce_ea_b[0])     e_a[15:0]  <= next_ea_val[15:0];
			if (ce_ea_b[1])     e_a[31:16] <= next_ea_val[31:16];
			if (ce_ea_rep_b[0]) begin
				for (wi = 0; wi < 4; wi = wi + 1)
					e_a_w_rep[wi] <= next_ea_val[3:2];
				e_a4_rep <= {2{next_ea_val[4]}};
				e_a1_rep <= {2{next_ea_val[1]}};
				e_line_cross <= u32 ? (&next_ea_val[4:2]) : (&next_ea_val[4:1]);
			end
			if (ce_ea_rep_b[1]) begin
				for (wi = 4; wi < 8; wi = wi + 1)
					e_a_w_rep[wi] <= next_ea_val[3:2];
			end
			if (ce_ev_b[0])    e_v[15:0]       <= next_ev_val[15:0];
			if (ce_ev_b[1])    e_v[31:16]      <= next_ev_val[31:16];
			if (ce_en_b[0])    e_n[15:0]       <= next_en_val[15:0];
			if (ce_en_b[1])    e_n[31:16]      <= next_en_val[31:16];
			if (ce_einst_b[0]) e_inst[15:0]    <= next_ei_val[15:0];
			if (ce_einst_b[1]) e_inst[31:16]   <= next_ei_val[31:16];
			if (ce_eil_b[0])   e_ileft[15:0]   <= next_eil_val[15:0];
			if (ce_eil_b[1])   e_ileft[31:16]  <= next_eil_val[31:16];

			if (ce_ivi_b[0])    iv_index_r[15:0]  <= next_iv_index[15:0];
			if (ce_ivi_b[1])    iv_index_r[31:16] <= next_iv_index[31:16];
			if (ce_ivinst_b[0]) iv_inst_r[15:0]   <= e_inst[15:0];
			if (ce_ivinst_b[1]) iv_inst_r[31:16]  <= e_inst[31:16];
			if (take) begin
				iv_valid_r   <= 1'b1;
				iv_restart_r <= raw_is_restart;
				iv_last_r    <= e_n_is1 && e_ileft_is1;
			end else if (iv_valid_r && iv_ready) begin
				iv_valid_r <= 1'b0;
			end

			if (st_is_idle) begin
				if (kick && s_count_nz && s_icount_nz) begin
					st_is_idle  <= 1'b0;
					st_is_init1 <= 1'b1;
					ind <= k_ind;
					u32 <= s_ifmt[1];
				end
			end else if (st_is_init1) begin
				st_is_init1 <= 1'b0;
				st_is_init2 <= 1'b1;
			end else if (st_is_init2) begin
				st_is_init2 <= 1'b0;
				st_is_init3 <= 1'b1;
				l0_eq_lend <= l0_eq_lend_w;
			end else if (st_is_init3) begin
				st_is_init3 <= 1'b0;
				st_is_walk  <= 1'b1;
				rq_slot <= 1'b1;
				e_slot <= 1'b0;
				e_n_is1 <= s_count_is1;
				e_ileft_is1 <= s_icount_is1;
			end else if (st_is_walk) begin
				if (rd_rsp && rsp_error) begin
					st_is_walk  <= 1'b0;
					st_is_drain <= 1'b1;
					iv_valid_r <= 1'b0;
				end else if (take) begin
					if (pop)
						e_slot <= !e_slot;
					if (inst_end) begin
						e_n_is1 <= s_count_is1;
						e_ileft_is1 <= eil_is2;
						if (e_ileft_is1) begin
							st_is_walk <= 1'b0;
							st_is_idle <= 1'b1;
						end
					end else begin
						e_n_is1 <= en_is2;
					end
				end
			end else if (st_is_drain) begin
				if (drain_done) begin
					st_is_drain <= 1'b0;
					st_is_idle  <= 1'b1;
				end
			end
		end
	end

	wire unused_ok = &{1'b0, rsp_tag[7:1], k_alast[LINE_SHIFT-1:0], e_a[0], e_a[31:5],
		a0_co_unused, cm1_bo_unused, al_co_unused, lm1_bo_unused,
		rql_hi_co_unused, rqi_bo_unused, ad_co_unused, ev_co_unused,
		ei_co_unused, en_bo_unused, eil_bo_unused};
endmodule

`default_nettype wire
