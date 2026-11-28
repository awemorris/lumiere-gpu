/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * am_axi4_cdc: Full AXI4 clock domain crossing bridge between s_clk and
 * m_clk.  Supports bursts (AWLEN/AWSIZE/AWBURST/ARLEN/ARSIZE/ARBURST) and
 * caches/prot attributes.  When USE_BRAM=1, the wide W and R data channels
 * use am_afifo_bram (Block RAM + FWFT skid buffer) instead of distributed
 * LUTRAMs, relieving CLBM slice pressure on dense UltraScale+ builds.
 */

`timescale 1ns / 1ps
`default_nettype none

module am_axi4_cdc #(
	parameter A_W        = 32,
	parameter D_W        = 512,
	parameter ID_W       = 4,
	parameter DEPTH_LOG2 = 4,
	parameter USE_BRAM   = 1,
	parameter BRAM_LOG2  = 9
) (
	/* Slave interface (s_clk domain) */
	input  wire             s_clk,
	input  wire             s_rst_n,
	input  wire [ID_W-1:0]  s_awid,
	input  wire [A_W-1:0]   s_awaddr,
	input  wire [7:0]       s_awlen,
	input  wire [2:0]       s_awsize,
	input  wire [1:0]       s_awburst,
	input  wire [3:0]       s_awcache,
	input  wire [2:0]       s_awprot,
	input  wire             s_awvalid,
	output wire             s_awready,
	input  wire [D_W-1:0]   s_wdata,
	input  wire [D_W/8-1:0] s_wstrb,
	input  wire             s_wlast,
	input  wire             s_wvalid,
	output wire             s_wready,
	output wire [ID_W-1:0]  s_bid,
	output wire [1:0]       s_bresp,
	output wire             s_bvalid,
	input  wire             s_bready,
	input  wire [ID_W-1:0]  s_arid,
	input  wire [A_W-1:0]   s_araddr,
	input  wire [7:0]       s_arlen,
	input  wire [2:0]       s_arsize,
	input  wire [1:0]       s_arburst,
	input  wire [3:0]       s_arcache,
	input  wire [2:0]       s_arprot,
	input  wire             s_arvalid,
	output wire             s_arready,
	output wire [ID_W-1:0]  s_rid,
	output wire [D_W-1:0]   s_rdata,
	output wire [1:0]       s_rresp,
	output wire             s_rlast,
	output wire             s_rvalid,
	input  wire             s_rready,

	/* Master interface (m_clk domain) */
	input  wire             m_clk,
	input  wire             m_rst_n,
	output wire [ID_W-1:0]  m_awid,
	output wire [A_W-1:0]   m_awaddr,
	output wire [7:0]       m_awlen,
	output wire [2:0]       m_awsize,
	output wire [1:0]       m_awburst,
	output wire [3:0]       m_awcache,
	output wire [2:0]       m_awprot,
	output wire             m_awvalid,
	input  wire             m_awready,
	output wire [D_W-1:0]   m_wdata,
	output wire [D_W/8-1:0] m_wstrb,
	output wire             m_wlast,
	output wire             m_wvalid,
	input  wire             m_wready,
	input  wire [ID_W-1:0]  m_bid,
	input  wire [1:0]       m_bresp,
	input  wire             m_bvalid,
	output wire             m_bready,
	output wire [ID_W-1:0]  m_arid,
	output wire [A_W-1:0]   m_araddr,
	output wire [7:0]       m_arlen,
	output wire [2:0]       m_arsize,
	output wire [1:0]       m_arburst,
	output wire [3:0]       m_arcache,
	output wire [2:0]       m_arprot,
	output wire             m_arvalid,
	input  wire             m_arready,
	input  wire [ID_W-1:0]  m_rid,
	input  wire [D_W-1:0]   m_rdata,
	input  wire [1:0]       m_rresp,
	input  wire             m_rlast,
	input  wire             m_rvalid,
	output wire             m_rready
);
	localparam AX_W = ID_W + A_W + 20;
	localparam W_W  = D_W + (D_W / 8) + 1;
	localparam R_W  = D_W + ID_W + 3;

	wire m_in_s, s_in_m;
	am_sync s_of_m (.clk(s_clk), .rst_n(s_rst_n), .d(m_rst_n), .q(m_in_s));
	am_sync m_of_s (.clk(m_clk), .rst_n(m_rst_n), .d(s_rst_n), .q(s_in_m));

	wire s_rn = s_rst_n && m_in_s;
	wire m_rn = m_rst_n && s_in_m;

	am_afifo #(.W(AX_W), .DEPTH_LOG2(DEPTH_LOG2)) aw (
		.w_clk(s_clk), .w_rst_n(s_rn), .w_valid(s_awvalid), .w_ready(s_awready),
		.w_data({s_awid, s_awaddr, s_awlen, s_awsize, s_awburst, s_awcache, s_awprot}),
		.r_clk(m_clk), .r_rst_n(m_rn), .r_valid(m_awvalid), .r_ready(m_awready),
		.r_data({m_awid, m_awaddr, m_awlen, m_awsize, m_awburst, m_awcache, m_awprot})
	);

	am_afifo #(.W(ID_W + 2), .DEPTH_LOG2(DEPTH_LOG2)) b (
		.w_clk(m_clk), .w_rst_n(m_rn), .w_valid(m_bvalid), .w_ready(m_bready),
		.w_data({m_bid, m_bresp}),
		.r_clk(s_clk), .r_rst_n(s_rn), .r_valid(s_bvalid), .r_ready(s_bready),
		.r_data({s_bid, s_bresp})
	);

	am_afifo #(.W(AX_W), .DEPTH_LOG2(DEPTH_LOG2)) ar (
		.w_clk(s_clk), .w_rst_n(s_rn), .w_valid(s_arvalid), .w_ready(s_arready),
		.w_data({s_arid, s_araddr, s_arlen, s_arsize, s_arburst, s_arcache, s_arprot}),
		.r_clk(m_clk), .r_rst_n(m_rn), .r_valid(m_arvalid), .r_ready(m_arready),
		.r_data({m_arid, m_araddr, m_arlen, m_arsize, m_arburst, m_arcache, m_arprot})
	);

	generate
		if (USE_BRAM != 0) begin : g_bram
			wire [BRAM_LOG2:0] w_rcnt_unused;
			wire [BRAM_LOG2:0] r_rcnt_unused;

			am_afifo_bram #(.W(W_W), .DEPTH_LOG2(BRAM_LOG2)) w (
				.w_clk(s_clk), .w_rst_n(s_rn), .w_valid(s_wvalid), .w_ready(s_wready),
				.w_data({s_wdata, s_wstrb, s_wlast}),
				.r_clk(m_clk), .r_rst_n(m_rn), .r_valid(m_wvalid), .r_ready(m_wready),
				.r_data({m_wdata, m_wstrb, m_wlast}),
				.r_count(w_rcnt_unused)
			);

			am_afifo_bram #(.W(R_W), .DEPTH_LOG2(BRAM_LOG2)) r (
				.w_clk(m_clk), .w_rst_n(m_rn), .w_valid(m_rvalid), .w_ready(m_rready),
				.w_data({m_rdata, m_rid, m_rresp, m_rlast}),
				.r_clk(s_clk), .r_rst_n(s_rn), .r_valid(s_rvalid), .r_ready(s_rready),
				.r_data({s_rdata, s_rid, s_rresp, s_rlast}),
				.r_count(r_rcnt_unused)
			);
		end else begin : g_dist
			am_afifo #(.W(W_W), .DEPTH_LOG2(DEPTH_LOG2)) w (
				.w_clk(s_clk), .w_rst_n(s_rn), .w_valid(s_wvalid), .w_ready(s_wready),
				.w_data({s_wdata, s_wstrb, s_wlast}),
				.r_clk(m_clk), .r_rst_n(m_rn), .r_valid(m_wvalid), .r_ready(m_wready),
				.r_data({m_wdata, m_wstrb, m_wlast})
			);

			am_afifo #(.W(R_W), .DEPTH_LOG2(DEPTH_LOG2)) r (
				.w_clk(m_clk), .w_rst_n(m_rn), .w_valid(m_rvalid), .w_ready(m_rready),
				.w_data({m_rdata, m_rid, m_rresp, m_rlast}),
				.r_clk(s_clk), .r_rst_n(s_rn), .r_valid(s_rvalid), .r_ready(s_rready),
				.r_data({s_rdata, s_rid, s_rresp, s_rlast})
			);
		end
	endgenerate
endmodule

`default_nettype wire
