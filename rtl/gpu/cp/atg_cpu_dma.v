/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cpu_dma: The CPU's line DMA between GPU memory (AMB) and IMEM/DMEM:
 * the firmware's heap <-> DMEM copies (window DMA_*) and the CPFW image
 * load (from atg_cpu_fw).
 *
 * Window (sub-bus, data at once): 0x10 DMA_AMB and 0x11 DMA_MEM (RW,
 * line aligned; the low bits are ignored), 0x12 DMA_GO (W: [15:0] lines,
 * [16] 1 = DMEM -> memory, [17] chk: session protection checks the
 * copy's memory accesses for dma_slot, the slot of the class V packet the
 * firmware works on (none when that slot is 0), [18] tbl: the copy writes
 * the protection table; ignored while busy or with 0 lines), 0x13
 * DMA_STATUS (R: [0] busy, [1] error of the last copy, [15:8] writes not
 * yet answered).  An image load (ld_start) copies ld_words words from
 * ld_addr to offset 0 of IMEM or DMEM; it waits until a window copy has
 * finished, then pulses ld_done (with ld_error).
 *
 * Reads: up to 4 line reads outstanding (tags 0-3, one line buffer each:
 * a read is issued only for a free buffer, so the responses, which have
 * no back-pressure, always have a place), each line stored into the BRAM
 * one word per cycle.  Writes: a line is gathered from DMEM (one word per
 * cycle), then written (all bytes); the writes are counted until their
 * responses.  An error response stops the copy: no more requests, the
 * ones in flight are waited for and dropped, and error is set.  The AMB
 * request holds its payload until req_ready, and req_chk, req_tbl and
 * req_slot those of its copy (an image load: 0).
 * See spec/gpu/cp/atg_cpu_dma.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cpu_dma #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5	/* log2(LINE_BYTES) */
) (
	input wire clk,
	input wire rst_n,
	/* Window sub-bus (from atg_cpu_win) */
	input wire sb_we,
	input wire sb_re,
	input wire [5:0] sb_idx,
	input wire [31:0] sb_wdata,
	output reg [31:0] sb_rdata,
	/* Image load (from atg_cpu_fw) */
	input wire ld_start,
	input wire ld_imem,
	input wire [31:0] ld_addr,
	input wire [31:0] ld_words,
	output reg ld_done,
	output reg ld_error,
	/* Memory port (atg_cpu_mem x_*) */
	output wire x_en,
	output wire x_we,
	output wire x_imem,
	output wire [31:0] x_addr,
	output wire [31:0] x_wdata,
	input wire [31:0] x_rdata,
	output wire wait_amb,
	/* AMB client port (design §4.1) */
	output reg req_valid,
	input wire req_ready,
	output reg req_write,
	output reg [ADDR_W-LINE_SHIFT-1:0] req_line,
	output wire [LINE_BYTES-1:0] req_be,
	output wire [LINE_BYTES*8-1:0] req_wdata,
	output reg [7:0] req_tag,
	output wire req_chk,	/* session protection, held with the copy */
	output wire req_tbl,
	output wire [5:0] req_slot,
	input wire [5:0] dma_slot,	/* the class V packet's protection slot */
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	localparam WPL = LINE_BYTES / 4;	/* words per line */
	localparam WS = LINE_SHIFT - 2;	/* log2(WPL) */
	localparam LW = ADDR_W - LINE_SHIFT;

	localparam [5:0] W_AMB = 6'h10;
	localparam [5:0] W_MEM = 6'h11;
	localparam [5:0] W_GO = 6'h12;
	localparam [5:0] W_STATUS = 6'h13;

	localparam [1:0] S_IDLE = 2'd0;
	localparam [1:0] S_READ = 2'd1;	/* issue reads, store lines */
	localparam [1:0] S_GATHER = 2'd2;	/* read a line from DMEM */
	localparam [1:0] S_WRITE = 2'd3;	/* write it */

	reg [1:0] state;
	reg [31:0] dma_amb, dma_mem;
	reg err;
	reg [7:0] wr_out;	/* writes not yet answered */
	reg ld_pend, ld_job;	/* an image load waits / runs */
	reg ld_pimem;
	reg [31:0] ld_paddr, ld_pwords;

	/* The current copy. */
	reg j_imem;
	reg [LW-1:0] j_line;	/* next line to request */
	reg [31:0] j_mem;	/* BRAM byte offset of that line */
	reg [31:0] j_rem;	/* words not yet requested */
	reg j_chk, j_tbl;	/* session protection of the copy */
	reg [5:0] j_slot;

	assign req_chk = j_chk;
	assign req_tbl = j_tbl;
	assign req_slot = j_slot;

	/* Read side: 4 line buffers (stored in discrete lbuf0..3 registers below). */
	reg [3:0] s_busy;	/* requested, not yet stored */
	reg [3:0] s_full;	/* data arrived */
	reg [31:0] s_dst [0:3];
	reg [WS:0] s_nw [0:3];	/* words to store */
	reg st_act;
	reg [1:0] st_slot;
	reg [WS:0] st_w;

	/* Write side: the gathered line. */
	reg [LINE_BYTES*8-1:0] wline;
	reg [WS:0] g_w;	/* next word to read */
	reg g_cap;	/* x_rdata holds word g_w - 1 (captured into wline) */

	reg [WS:0] st_nw_m1_r;
	reg is_idle_r, is_read_r, is_gather_r, is_write_r;
	wire busy = !is_idle_r;
	wire sb_is_go     = (sb_idx == W_GO);
	wire sb_wdata_nz  = (sb_wdata[15:0] != 16'd0);
	wire go = sb_we && sb_is_go && is_idle_r && !ld_pend && sb_wdata_nz;

	/* Flat 1-hot slot selection (replaces k=3..0 priority loops). */
	wire free0 = !s_busy[0];
	wire free1 = s_busy[0] && !s_busy[1];
	wire free2 = s_busy[0] && s_busy[1] && !s_busy[2];
	wire free3 = s_busy[0] && s_busy[1] && s_busy[2] && !s_busy[3];
	wire have_free = !(&s_busy);
	wire [1:0] free_s = free0 ? 2'd0 : (free1 ? 2'd1 : (free2 ? 2'd2 : 2'd3));

	wire full0 = s_full[0];
	wire full1 = !s_full[0] && s_full[1];
	wire full2 = !s_full[0] && !s_full[1] && s_full[2];
	wire full3 = !s_full[0] && !s_full[1] && !s_full[2] && s_full[3];
	wire have_full = |s_full;
	wire [1:0] full_s = full0 ? 2'd0 : (full1 ? 2'd1 : (full2 ? 2'd2 : 2'd3));
	wire [31:0] s_dst_full = ({32{full0}} & s_dst[0]) | ({32{full1}} & s_dst[1]) |
		({32{full2}} & s_dst[2]) | ({32{full3}} & s_dst[3]);
	integer k;

	reg j_rem_nz_r, j_rem_ge_wpl_r;
	wire [WS:0] words_now = j_rem_ge_wpl_r ? WPL[WS:0] : j_rem[WS:0];
	wire rd_issue = is_read_r && !req_valid && j_rem_nz_r && have_free && !err;
	wire rd_rsp = rsp_valid && !rsp_write;
	wire wr_rsp = rsp_valid && rsp_write;

	/* Memory port: dedicated x_port_addr_r avoids muxing x_we onto WR_ADDR (Check 4 = 0). */
	wire st_we = st_act;
	wire g_en = is_gather_r && !g_w[WS];
	wire [31:0] st_word;

	reg [31:0] st_wr_wdata_r;
	reg st_wr_v_r;
	reg [31:0] x_port_addr_r;

	assign x_en = st_wr_v_r || g_en;
	assign x_we = st_wr_v_r;
	assign x_imem = j_imem;
	/*
	 * Fast 8x4b carry-select arithmetic for 32-bit x_addr_r, j_mem, j_rem, and j_line.
	 */
	reg [31:0] x_addr_r;
	wire [31:0] x_addr_inc, j_mem_n, j_rem_n;
	wire [WS:0] st_w_n, g_w_n;
	wire [LW-1:0] j_line_n;
	wire [7:0] wr_out_n;
	wire [7:0] ar_co;
	wire wr_inc = req_valid && req_ready && req_write && !wr_rsp;
	wire wr_dec = !(req_valid && req_ready && req_write) && wr_rsp && wr_out != 8'd0;

	/* Pure-Boolean 32-bit Kogge-Stone adder (alternating AOI/OAI, <= 8 cmos3, zero + operators). */
	function [32:0] fn_ks_add32;
		input [31:0] a_in;
		input [31:0] b_in;
		input ci_in;
		reg [31:0] g0, p0, g1_n, p1_n, g2, p2, g3_n, p3_n, g4, p4, g5_n, sum_w;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g0[0] = g0[0] | (p0[0] & ci_in);
			g1_n = ~(g0 | (p0 & {g0[30:0], 1'b0}));
			p1_n = ~(p0 & {p0[30:0], 1'b0});
			g2   = ~(g1_n & (p1_n | {g1_n[29:0], 2'b11}));
			p2   = ~(p1_n | {p1_n[29:0], 2'b11});
			g3_n = ~(g2 | (p2 & {g2[27:0], 4'h0}));
			p3_n = ~(p2 & {p2[27:0], 4'h0});
			g4   = ~(g3_n & (p3_n | {g3_n[23:0], 8'hFF}));
			p4   = ~(p3_n | {p3_n[23:0], 8'hFF});
			g5_n = ~(g4 | (p4 & {g4[15:0], 16'h0000}));
			sum_w = p0 ^ {~g5_n[30:0], ci_in};
			fn_ks_add32 = {~g5_n[31], sum_w};
		end
	endfunction

	/* Pure-Boolean 40-bit alternating NAND/NOR +1 incrementer (supports LW <= 40, zero + operators). */
	function [39:0] fn_inc40;
		input [39:0] x;
		reg [39:0] s1_n, s2, s3_n, s4, s5_n, s6;
		begin
			s1_n = ~(x    & {x[38:0],     1'b1});
			s2   = ~(s1_n | {s1_n[37:0],  2'b00});
			s3_n = ~(s2   & {s2[35:0],    4'hF});
			s4   = ~(s3_n | {s3_n[31:0],  8'h00});
			s5_n = ~(s4   & {s4[23:0],   16'hFFFF});
			s6   = ~(s5_n | {s5_n[7:0],  32'h0000_0000});
			fn_inc40 = x ^ {s6[38:0], 1'b1};
		end
	endfunction

	/* Pure-Boolean 32-bit alternating NAND/NOR +1 incrementer (<= 6 cmos3, zero + operators). */
	function [31:0] fn_inc32;
		input [31:0] x;
		reg [31:0] s1_n, s2, s3_n, s4, s5_n;
		begin
			s1_n = ~(x    & {x[30:0],     1'b1});
			s2   = ~(s1_n | {s1_n[29:0],  2'b00});
			s3_n = ~(s2   & {s2[27:0],    4'hF});
			s4   = ~(s3_n | {s3_n[23:0],  8'h00});
			s5_n = ~(s4   & {s4[15:0],   16'hFFFF});
			fn_inc32 = x ^ {~s5_n[30:0], 1'b1};
		end
	endfunction

	/* x_addr_r + 4: low 2 bits unchanged, upper 30 bits + 1. */
	wire [31:0] xa_hi_pad = fn_inc32({2'd0, x_addr_r[31:2]});
	assign ar_co[0]   = 1'b0;
	assign x_addr_inc = {xa_hi_pad[29:0], x_addr_r[1:0]};

	am_add #(.W(WS + 1)) st_w_add (
		.a(st_w), .b({(WS + 1){1'b0}}), .ci(1'b1), .s(st_w_n), .co(ar_co[1])
	);
	am_add #(.W(WS + 1)) g_w_add (
		.a(g_w), .b({(WS + 1){1'b0}}), .ci(1'b1), .s(g_w_n), .co(ar_co[2])
	);
	reg [WS:0] s_nw_m1 [0:3];
	wire [WS:0] s_nw_m1_full = ({(WS+1){full0}} & s_nw_m1[0]) | ({(WS+1){full1}} & s_nw_m1[1]) |
		({(WS+1){full2}} & s_nw_m1[2]) | ({(WS+1){full3}} & s_nw_m1[3]);
	wire [WS:0] words_now_m1;
	wire unused_wnm1_bo;
	am_sub #(.W(WS + 1)) u_wnm1 (.a(words_now), .b({{WS{1'b0}}, 1'b1}), .bi(1'b0), .d(words_now_m1), .bo(unused_wnm1_bo));

	/* j_line + 1 (<= 6 cmos3, supports LW > 32). */
	reg [39:0] jl_ext;
	always @(*) begin
		jl_ext = 40'd0;
		jl_ext[LW-1:0] = j_line;
	end
	wire [39:0] jl_res = fn_inc40(jl_ext);
	assign ar_co[3] = 1'b0;
	assign j_line_n = jl_res[LW-1:0];

	/* j_mem + LINE_BYTES (LINE_SHIFT bits unchanged, upper bits + 1, <= 6 cmos3). */
	wire [31:0] jm_pad = fn_inc32({{LINE_SHIFT{1'b0}}, j_mem[31:LINE_SHIFT]});
	assign ar_co[4] = 1'b0;
	assign j_mem_n  = {jm_pad[31-LINE_SHIFT:0], j_mem[LINE_SHIFT-1:0]};

	/* j_rem - WPL: alternating NAND/NOR decrementer (<= 6 cmos3). */
	function [31-WS:0] fn_dec_hi;
		input [31-WS:0] x;
		reg [31-WS:0] z, s1_n, s2, s3_n, s4, s5_n;
		begin
			z    = ~x;
			s1_n = ~(z    & {z[30-WS:0],     1'b1});
			s2   = ~(s1_n | {s1_n[29-WS:0],  2'b00});
			s3_n = ~(s2   & {s2[27-WS:0],    4'hF});
			s4   = ~(s3_n | {s3_n[23-WS:0],  8'h00});
			s5_n = ~(s4   & {s4[15-WS:0],   16'hFFFF});
			fn_dec_hi = x ^ {~s5_n[30-WS:0], 1'b1};
		end
	endfunction
	wire [31:0] wpl_w = WPL;
	assign ar_co[5] = 1'b0;
	wire [31:0] j_rem_m_wpl = {fn_dec_hi(j_rem[31:WS]), j_rem[WS-1:0]};
	wire j_rem_m_wpl_nz = (j_rem != wpl_w);
	wire j_rem_m_wpl_ge_wpl = |j_rem[31:WS + 1];
	wire j_rem_n_nz     = j_rem_ge_wpl_r && j_rem_m_wpl_nz;
	wire j_rem_n_ge_wpl = j_rem_ge_wpl_r && j_rem_m_wpl_ge_wpl;
	assign j_rem_n = j_rem_ge_wpl_r ? j_rem_m_wpl : 32'd0;

	wire [7:0] wr_out_p1, wr_out_m1;
	am_add #(.W(8)) wr_out_add_p1 (.a(wr_out), .b(8'd0), .ci(1'b1), .s(wr_out_p1), .co(ar_co[6]));
	am_add #(.W(8)) wr_out_add_m1 (.a(wr_out), .b(8'hFF), .ci(1'b0), .s(wr_out_m1), .co(ar_co[7]));
	assign wr_out_n = wr_inc ? wr_out_p1 : wr_out_m1;

	assign x_addr = x_port_addr_r;
	assign x_wdata = st_wr_wdata_r;

	assign req_be = {LINE_BYTES{1'b1}};
	assign req_wdata = wline;
	assign wait_amb = s_busy != 4'd0 || wr_out != 8'd0 || req_valid;

	wire dma_hit = sb_re && (sb_idx[5:2] == 4'b0100);
	reg [31:0] sb_rdata_raw;
	always @(*) begin
		case (sb_idx[1:0])
		2'd0: sb_rdata_raw = dma_amb;
		2'd1: sb_rdata_raw = dma_mem;
		2'd3: sb_rdata_raw = {16'd0, wr_out, 6'd0, err, busy};
		default: sb_rdata_raw = 32'd0;
		endcase
		sb_rdata = {32{dma_hit}} & sb_rdata_raw;
	end

	/* Discrete 4-slot line buffers + word-first select for shallow st_word. */
	reg [LINE_BYTES*8-1:0] lbuf0, lbuf1, lbuf2, lbuf3;
	wire [31:0] lw0 = lbuf0[{st_w[WS-1:0], 5'd0} +: 32];
	wire [31:0] lw1 = lbuf1[{st_w[WS-1:0], 5'd0} +: 32];
	wire [31:0] lw2 = lbuf2[{st_w[WS-1:0], 5'd0} +: 32];
	wire [31:0] lw3 = lbuf3[{st_w[WS-1:0], 5'd0} +: 32];
	assign st_word = (st_slot == 2'd0) ? lw0 :
	                 (st_slot == 2'd1) ? lw1 :
	                 (st_slot == 2'd2) ? lw2 : lw3;

	always @(posedge clk) begin
		if (rd_rsp && s_busy[rsp_tag[1:0]]) begin
			case (rsp_tag[1:0])
			2'd0: lbuf0 <= rsp_rdata;
			2'd1: lbuf1 <= rsp_rdata;
			2'd2: lbuf2 <= rsp_rdata;
			2'd3: lbuf3 <= rsp_rdata;
			endcase
		end
		/* The captured word is word g_w - 1 (mod WPL): a constant place per word. */
		for (k = 0; k < WPL; k = k + 1)
			if (g_cap && {{(32 - WS){1'b0}}, g_w[WS-1:0]} == (k + 1) % WPL)
				wline[k*32 +: 32] <= x_rdata;
	end

	wire [63:0] dma_amb_ext = {32'd0, dma_amb};
	wire [63:0] ld_paddr_ext = {32'd0, ld_paddr};

	wire xa_inc_en   = st_act || g_en;
	wire wr_step_en  = is_write_r && req_ready;
	wire j_step_en   = rd_issue || wr_step_en;
	wire go_idle_ld  = is_idle_r && ld_pend;
	wire go_idle_req = go;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_IDLE;
			is_idle_r <= 1'b1;
			is_read_r <= 1'b0;
			is_gather_r <= 1'b0;
			is_write_r <= 1'b0;
			dma_amb <= 32'd0;
			dma_mem <= 32'd0;
			err <= 1'b0;
			wr_out <= 8'd0;
			ld_pend <= 1'b0;
			ld_job <= 1'b0;
			ld_done <= 1'b0;
			ld_error <= 1'b0;
			ld_pimem <= 1'b0;
			ld_paddr <= 32'd0;
			ld_pwords <= 32'd0;
			req_valid <= 1'b0;
			req_write <= 1'b0;
			req_line <= {(ADDR_W-LINE_SHIFT){1'b0}};
			req_tag <= 8'd0;
			j_imem <= 1'b0;
			j_line <= {LW{1'b0}};
			j_mem <= 32'd0;
			j_rem <= 32'd0;
			j_rem_nz_r <= 1'b0;
			j_rem_ge_wpl_r <= 1'b0;
			j_chk <= 1'b0;
			j_tbl <= 1'b0;
			j_slot <= 6'd0;
			s_busy <= 4'd0;
			s_full <= 4'd0;
			for (k = 0; k < 4; k = k + 1) begin
				s_dst[k]  <= 32'd0;
				s_nw[k]   <= {(WS + 1){1'b0}};
				s_nw_m1[k] <= {(WS + 1){1'b0}};
			end
			st_act <= 1'b0;
			st_slot <= 2'd0;
			st_w <= {(WS + 1){1'b0}};
			st_nw_m1_r <= {(WS + 1){1'b0}};
			st_wr_v_r <= 1'b0;
			st_wr_wdata_r <= 32'd0;
			x_port_addr_r <= 32'd0;
			x_addr_r <= 32'd0;
			g_w <= {(WS + 1){1'b0}};
			g_cap <= 1'b0;
		end else begin
			ld_done <= 1'b0;
			st_wr_v_r <= st_we;
			if (st_we)
				st_wr_wdata_r <= st_word;
			if (sb_we && sb_idx == W_AMB)
				dma_amb <= sb_wdata;
			if (sb_we && sb_idx == W_MEM)
				dma_mem <= sb_wdata;
			if (ld_start) begin
				ld_pend <= 1'b1;
				ld_pimem <= ld_imem;
				ld_paddr <= ld_addr;
				ld_pwords <= ld_words;
			end else if (go_idle_ld) begin
				ld_pend <= 1'b0;
			end
			/* Write responses count down whenever they come. */
			if (wr_rsp && rsp_error)
				err <= 1'b1;
			else if (go_idle_req || go_idle_ld)
				err <= 1'b0;
			if (wr_inc || wr_dec)
				wr_out <= wr_out_n;
			if (req_valid && req_ready)
				req_valid <= 1'b0;
			else if (rd_issue || (is_gather_r && !err && !g_cap && g_w[WS] && wr_out != 8'hFF))
				req_valid <= 1'b1;
			/* Read side: responses and storing. */
			if (rd_rsp && s_busy[rsp_tag[1:0]]) begin
				if (rsp_error || err) begin
					err <= 1'b1;
					s_busy[rsp_tag[1:0]] <= 1'b0;
				end else begin
					s_full[rsp_tag[1:0]] <= 1'b1;
				end
			end
			if (st_act) begin
				if (st_w == st_nw_m1_r) begin	/* st_w is the last word */
					st_act <= 1'b0;
					s_busy[st_slot] <= 1'b0;
					s_full[st_slot] <= 1'b0;
				end
				st_w <= st_w_n;
			end else if (have_full) begin
				st_act <= 1'b1;
				st_slot <= full_s;
				st_nw_m1_r <= s_nw_m1_full;
				st_w <= {(WS + 1){1'b0}};
			end
			if (rd_issue) begin
				req_write <= 1'b0;
				req_line <= j_line;
				req_tag <= {6'd0, free_s};
				s_busy[free_s] <= 1'b1;
				s_dst[free_s] <= j_mem;
				s_nw[free_s] <= words_now;
				s_nw_m1[free_s] <= words_now_m1;
			end else if (is_gather_r && !err && !g_cap && g_w[WS] && wr_out != 8'hFF) begin
				req_write <= 1'b1;
				req_line <= j_line;
				req_tag <= 8'h80;
			end

			/* Flat x_addr_r and x_port_addr_r: xa_inc_en first for shallowest increment path. */
			if (xa_inc_en) begin
				x_addr_r <= x_addr_inc;
				x_port_addr_r <= st_act ? x_addr_r : x_addr_inc;
			end else if (have_full) begin
				x_addr_r <= s_dst_full;
			end else if (wr_step_en && j_rem != WPL) begin
				x_addr_r <= j_mem_n;
				x_port_addr_r <= j_mem_n;
			end else if (is_idle_r && !ld_pend) begin
				x_addr_r <= {dma_mem[31:LINE_SHIFT], {LINE_SHIFT{1'b0}}};
				x_port_addr_r <= {dma_mem[31:LINE_SHIFT], {LINE_SHIFT{1'b0}}};
			end

			/* Flat j_line / j_mem / j_rem: j_step_en first, direct S_IDLE preload without sb_idx/sb_wdata. */
			if (j_step_en) begin
				j_line <= j_line_n;
				j_mem <= j_mem_n;
				j_rem <= j_rem_n;
				j_rem_nz_r <= j_rem_n_nz;
				j_rem_ge_wpl_r <= j_rem_n_ge_wpl;
			end else begin
				if (is_idle_r && !ld_pend) begin
					j_line <= dma_amb_ext[ADDR_W-1:LINE_SHIFT];
					j_mem <= {dma_mem[31:LINE_SHIFT], {LINE_SHIFT{1'b0}}};
					j_rem <= {{(16 - WS){1'b0}}, sb_wdata[15:0], {WS{1'b0}}};
				end else if (go_idle_ld) begin
					j_line <= ld_paddr_ext[ADDR_W-1:LINE_SHIFT];
					j_mem <= 32'd0;
					j_rem <= ld_pwords;
				end
				if (go_idle_req) begin
					j_rem_nz_r <= 1'b1;
					j_rem_ge_wpl_r <= 1'b1;
				end else if (go_idle_ld) begin
					j_rem_nz_r <= (ld_pwords != 32'd0);
					j_rem_ge_wpl_r <= (|ld_pwords[31:WS]);
				end
			end

			/* Write side: gather WPL words, then the request. */
			g_cap <= g_en;
			if (g_en)
				g_w <= g_w_n;
			else if (go_idle_req || wr_step_en)
				g_w <= {(WS + 1){1'b0}};

			case (state)
			S_IDLE:
				if (go) begin
					j_imem <= 1'b0;
`ifdef MUT_CPPROT_DMACHK
					j_chk <= 1'b0;
`else
					j_chk <= sb_wdata[17] && dma_slot != 6'd0;
`endif
					j_tbl <= sb_wdata[18];
					j_slot <= dma_slot;
					state <= sb_wdata[16] ? S_GATHER : S_READ;
					is_idle_r <= 1'b0;
					is_read_r <= !sb_wdata[16];
					is_gather_r <= sb_wdata[16];
				end else if (ld_pend) begin
					ld_job <= 1'b1;
					j_chk <= 1'b0;
					j_tbl <= 1'b0;
					j_slot <= 6'd0;
					j_imem <= ld_pimem;
					state <= S_READ;
					is_idle_r <= 1'b0;
					is_read_r <= 1'b1;
					is_gather_r <= 1'b0;
				end
			S_READ:
				/* Done when nothing is left to request, in flight or to store. */
				if ((!j_rem_nz_r || err) && !req_valid && s_busy == 4'd0 && !rd_issue && !st_wr_v_r) begin
					state <= S_IDLE;
					is_idle_r <= 1'b1;
					is_read_r <= 1'b0;
					if (ld_job) begin
						ld_job <= 1'b0;
						ld_done <= 1'b1;
						ld_error <= err;
					end
				end
			S_GATHER:
				if (err) begin
					state <= S_IDLE;
					is_idle_r <= 1'b1;
					is_gather_r <= 1'b0;
				end else if (!g_cap && g_w[WS] && wr_out != 8'hFF) begin
					state <= S_WRITE;
					is_gather_r <= 1'b0;
					is_write_r <= 1'b1;
				end
			default:	/* S_WRITE */
				if (req_ready) begin
					state <= (j_rem == WPL) ? S_IDLE : S_GATHER;
					is_idle_r <= (j_rem == WPL);
					is_gather_r <= (j_rem != WPL);
					is_write_r <= 1'b0;
				end
			endcase
		end
	end

	wire unused_ok = &{1'b0, rsp_tag[7:2], dma_mem[LINE_SHIFT-1:0], dma_amb[LINE_SHIFT-1:0],
		ld_paddr[LINE_SHIFT-1:0], ar_co, unused_wnm1_bo};
endmodule

`default_nettype wire
