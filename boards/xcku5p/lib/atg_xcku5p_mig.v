/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_xcku5p_mig: HamGeek XCKU5P DDR4 memory controller wrapper:
 * 2x MT40A256M16GE-075E (32-bit, 2666 MT/s, 1 GiB, 256-bit AXI4 ui_clk).
 *
 * Bridges:
 *   s2_* (BAR2, 128-bit in s2_clk) -> bar2_cc -> bar2_up (512-bit in clk)
 *   s_* (GPU, 512-bit in clk) + BAR2 -> mem_xbar -> axi_dn (512 -> 256-bit)
 *   -> axi_cc (clk -> ui_clk) -> atg_xcku5p_ddr4
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_xcku5p_mig (
	input wire clk,
	input wire rst_n,
	input wire ref_ibuf,
	input wire ref_clk,
	input wire sys_rst_n,
	output wire mem_ready,
	/* Board registers at 0x1F000 (gclk, xr_* bus) */
	input wire xr_wr,
	input wire [16:0] xr_waddr,
	input wire [31:0] xr_wdata,
	input wire xr_rd,
	input wire [16:0] xr_raddr,
	output reg [31:0] xr_rdata,
	/* AXI4 slave (gclk), atg_top m_* */
	input wire [31:0] s_awaddr,
	input wire [7:0] s_awlen,
	input wire [2:0] s_awsize,
	input wire [1:0] s_awburst,
	input wire [3:0] s_awcache,
	input wire [2:0] s_awprot,
	input wire s_awvalid,
	output wire s_awready,
	input wire [511:0] s_wdata,
	input wire [63:0] s_wstrb,
	input wire s_wlast,
	input wire s_wvalid,
	output wire s_wready,
	output wire [1:0] s_bresp,
	output wire s_bvalid,
	input wire s_bready,
	input wire [31:0] s_araddr,
	input wire [7:0] s_arlen,
	input wire [2:0] s_arsize,
	input wire [1:0] s_arburst,
	input wire [3:0] s_arcache,
	input wire [2:0] s_arprot,
	input wire s_arvalid,
	output wire s_arready,
	output wire [511:0] s_rdata,
	output wire [1:0] s_rresp,
	output wire s_rlast,
	output wire s_rvalid,
	input wire s_rready,
	/* AXI4 slave (s2_clk), PCIe BAR2 window */
	input wire s2_clk,
	input wire s2_rst_n,
	input wire [25:0] s2_awaddr,
	input wire [3:0] s2_awid,
	input wire [7:0] s2_awlen,
	input wire [2:0] s2_awsize,
	input wire [1:0] s2_awburst,
	input wire s2_awvalid,
	output wire s2_awready,
	input wire [127:0] s2_wdata,
	input wire [15:0] s2_wstrb,
	input wire s2_wlast,
	input wire s2_wvalid,
	output wire s2_wready,
	output wire [3:0] s2_bid,
	output wire [1:0] s2_bresp,
	output wire s2_bvalid,
	input wire s2_bready,
	input wire [25:0] s2_araddr,
	input wire [3:0] s2_arid,
	input wire [7:0] s2_arlen,
	input wire [2:0] s2_arsize,
	input wire [1:0] s2_arburst,
	input wire s2_arvalid,
	output wire s2_arready,
	output wire [127:0] s2_rdata,
	output wire [3:0] s2_rid,
	output wire [1:0] s2_rresp,
	output wire s2_rlast,
	output wire s2_rvalid,
	input wire s2_rready,
	/* DDR4 pins (32-bit, 2x MT40A256M16GE-075E) */
	inout wire [31:0] ddr4_dq,
	inout wire [3:0] ddr4_dqs_p,
	inout wire [3:0] ddr4_dqs_n,
	inout wire [3:0] ddr4_dm_n,
	output wire [16:0] ddr4_addr,
	output wire [1:0] ddr4_ba,
	output wire [0:0] ddr4_bg,
	output wire ddr4_act_n,
	output wire ddr4_reset_n,
	output wire [0:0] ddr4_ck_p,
	output wire [0:0] ddr4_ck_n,
	output wire [0:0] ddr4_cke,
	output wire [0:0] ddr4_cs_n,
	output wire [0:0] ddr4_odt
);
	wire ui_clk, ui_rst, calib;
	reg aresetn = 1'b0;

	always @(posedge ui_clk)
		aresetn <= !ui_rst;

	am_sync sync_ready (.clk(clk), .rst_n(rst_n), .d(calib), .q(mem_ready));

	/* Board registers (0x1F000): 0 ID "KU5P", 1 CTRL, 2 STATUS */
	localparam [8:0] XR_PAGE = 9'h1F0;
	reg [31:0] ctrl;
	wire hold_ref, ui_rst_g;

	am_sync sync_hold (.clk(ref_clk), .rst_n(1'b1), .d(ctrl[0]), .q(hold_ref));
	am_sync sync_uirst (.clk(clk), .rst_n(rst_n), .d(ui_rst), .q(ui_rst_g));

	always @(posedge clk) begin
		if (!rst_n) begin
			ctrl <= 32'd0;
			xr_rdata <= 32'd0;
		end else begin
			if (xr_wr && xr_waddr[16:8] == XR_PAGE && xr_waddr[7:2] == 6'd1)
				ctrl <= xr_wdata & 32'h00000001;
			if (xr_rd && xr_raddr[16:8] == XR_PAGE && !xr_raddr[7]) begin
				case (xr_raddr[6:2])
				5'd0: xr_rdata <= 32'h4B553550;	/* "KU5P" */
				5'd1: xr_rdata <= ctrl;
				5'd2: xr_rdata <= {30'd0, ui_rst_g, mem_ready};
				default: xr_rdata <= 32'd0;
				endcase
			end else
				xr_rdata <= 32'd0;
		end
	end

	/* BAR2: 128 -> 512 bit in s2_clk (user_clk), then BRAM CDC s2_clk -> gclk (512 bit) */
	wire [31:0] c_awaddr, c_araddr, e_awaddr, e_araddr;
	wire [7:0] c_awlen, c_arlen, e_awlen, e_arlen;
	wire [2:0] c_awsize, c_arsize, c_awprot, c_arprot, e_awsize, e_arsize, e_awprot, e_arprot;
	wire [1:0] c_awburst, c_arburst, c_bresp, c_rresp, e_awburst, e_arburst, e_bresp, e_rresp;
	wire [3:0] c_awcache, c_arcache, c_awregion, c_arregion, c_awqos, c_arqos;
	wire [3:0] e_awid, e_arid, e_awcache, e_arcache;
	wire c_awlock, c_arlock;
	wire c_awvalid, c_awready, c_wlast, c_wvalid, c_wready, c_bvalid, c_bready;
	wire c_arvalid, c_arready, c_rlast, c_rvalid, c_rready;
	wire e_awvalid, e_awready, e_wlast, e_wvalid, e_wready, e_bvalid, e_bready;
	wire e_arvalid, e_arready, e_rlast, e_rvalid, e_rready;
	wire [511:0] c_wdata, c_rdata, e_wdata, e_rdata;
	wire [63:0] c_wstrb, e_wstrb;

	atg_xcku5p_bar2_up bar2_up (
		.s_axi_aclk(s2_clk), .s_axi_aresetn(s2_rst_n),
		.s_axi_awid(s2_awid), .s_axi_awaddr({6'd0, s2_awaddr}), .s_axi_awlen(s2_awlen),
		.s_axi_awsize(s2_awsize), .s_axi_awburst(s2_awburst), .s_axi_awlock(1'b0),
		.s_axi_awcache(4'd0), .s_axi_awprot(3'd0), .s_axi_awregion(4'd0), .s_axi_awqos(4'd0),
		.s_axi_awvalid(s2_awvalid), .s_axi_awready(s2_awready), .s_axi_wdata(s2_wdata),
		.s_axi_wstrb(s2_wstrb), .s_axi_wlast(s2_wlast), .s_axi_wvalid(s2_wvalid),
		.s_axi_wready(s2_wready), .s_axi_bid(s2_bid), .s_axi_bresp(s2_bresp),
		.s_axi_bvalid(s2_bvalid), .s_axi_bready(s2_bready), .s_axi_arid(s2_arid),
		.s_axi_araddr({6'd0, s2_araddr}), .s_axi_arlen(s2_arlen), .s_axi_arsize(s2_arsize),
		.s_axi_arburst(s2_arburst), .s_axi_arlock(1'b0), .s_axi_arcache(4'd0),
		.s_axi_arprot(3'd0), .s_axi_arregion(4'd0), .s_axi_arqos(4'd0),
		.s_axi_arvalid(s2_arvalid), .s_axi_arready(s2_arready), .s_axi_rid(s2_rid),
		.s_axi_rdata(s2_rdata), .s_axi_rresp(s2_rresp), .s_axi_rlast(s2_rlast),
		.s_axi_rvalid(s2_rvalid), .s_axi_rready(s2_rready),
		.m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen), .m_axi_awsize(c_awsize),
		.m_axi_awburst(c_awburst), .m_axi_awlock(c_awlock), .m_axi_awcache(c_awcache),
		.m_axi_awprot(c_awprot), .m_axi_awregion(c_awregion), .m_axi_awqos(c_awqos),
		.m_axi_awvalid(c_awvalid), .m_axi_awready(c_awready), .m_axi_wdata(c_wdata),
		.m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast), .m_axi_wvalid(c_wvalid),
		.m_axi_wready(c_wready), .m_axi_bresp(c_bresp), .m_axi_bvalid(c_bvalid),
		.m_axi_bready(c_bready), .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen),
		.m_axi_arsize(c_arsize), .m_axi_arburst(c_arburst), .m_axi_arlock(c_arlock),
		.m_axi_arcache(c_arcache), .m_axi_arprot(c_arprot), .m_axi_arregion(c_arregion),
		.m_axi_arqos(c_arqos), .m_axi_arvalid(c_arvalid), .m_axi_arready(c_arready),
		.m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp), .m_axi_rlast(c_rlast),
		.m_axi_rvalid(c_rvalid), .m_axi_rready(c_rready)
	);

	wire [3:0] bar2_cc_s_bid_unused, bar2_cc_s_rid_unused;
	am_axi4_cdc #(
		.A_W(32), .D_W(512), .ID_W(4), .DEPTH_LOG2(3), .USE_BRAM(0)
	) bar2_cc (
		.s_clk(s2_clk), .s_rst_n(s2_rst_n),
		.s_awid(4'd0), .s_awaddr(c_awaddr), .s_awlen(c_awlen),
		.s_awsize(c_awsize), .s_awburst(c_awburst), .s_awcache(c_awcache), .s_awprot(c_awprot),
		.s_awvalid(c_awvalid), .s_awready(c_awready),
		.s_wdata(c_wdata), .s_wstrb(c_wstrb), .s_wlast(c_wlast),
		.s_wvalid(c_wvalid), .s_wready(c_wready),
		.s_bid(bar2_cc_s_bid_unused), .s_bresp(c_bresp), .s_bvalid(c_bvalid), .s_bready(c_bready),
		.s_arid(4'd0), .s_araddr(c_araddr), .s_arlen(c_arlen),
		.s_arsize(c_arsize), .s_arburst(c_arburst), .s_arcache(c_arcache), .s_arprot(c_arprot),
		.s_arvalid(c_arvalid), .s_arready(c_arready),
		.s_rid(bar2_cc_s_rid_unused), .s_rdata(c_rdata), .s_rresp(c_rresp),
		.s_rlast(c_rlast), .s_rvalid(c_rvalid), .s_rready(c_rready),
		.m_clk(clk), .m_rst_n(rst_n),
		.m_awid(e_awid), .m_awaddr(e_awaddr), .m_awlen(e_awlen),
		.m_awsize(e_awsize), .m_awburst(e_awburst), .m_awcache(e_awcache), .m_awprot(e_awprot),
		.m_awvalid(e_awvalid), .m_awready(e_awready),
		.m_wdata(e_wdata), .m_wstrb(e_wstrb), .m_wlast(e_wlast),
		.m_wvalid(e_wvalid), .m_wready(e_wready),
		.m_bid(4'd0), .m_bresp(e_bresp), .m_bvalid(e_bvalid), .m_bready(e_bready),
		.m_arid(e_arid), .m_araddr(e_araddr), .m_arlen(e_arlen),
		.m_arsize(e_arsize), .m_arburst(e_arburst), .m_arcache(e_arcache), .m_arprot(e_arprot),
		.m_arvalid(e_arvalid), .m_arready(e_arready),
		.m_rid(4'd0), .m_rdata(e_rdata), .m_rresp(e_rresp),
		.m_rlast(e_rlast), .m_rvalid(e_rvalid), .m_rready(e_rready)
	);

	/* GPU (S00) + BAR2 (S01) -> 512-bit master in gclk */
	wire [0:0] g_awid, g_bid, g_arid, g_rid;
	wire [31:0] g_awaddr, g_araddr;
	wire [7:0] g_awlen, g_arlen;
	wire [2:0] g_awsize, g_arsize, g_awprot, g_arprot;
	wire [1:0] g_awburst, g_arburst, g_bresp, g_rresp;
	wire [3:0] g_awcache, g_arcache, g_awregion, g_arregion, g_awqos, g_arqos;
	wire g_awlock, g_arlock, g_awvalid, g_awready, g_wlast, g_wvalid, g_wready, g_bvalid;
	wire g_bready, g_arvalid, g_arready, g_rlast, g_rvalid, g_rready;
	wire [511:0] g_wdata, g_rdata;
	wire [63:0] g_wstrb;
	wire [1:0] x_bid, x_rid;
	wire [1023:0] x_rdata;
	wire [3:0] x_bresp, x_rresp;
	wire [1:0] x_awready, x_wready, x_bvalid, x_arready, x_rlast, x_rvalid;

	atg_xcku5p_mem_xbar xbar (
		.aclk(clk), .aresetn(rst_n),
		.s_axi_awid(2'b00), .s_axi_awaddr({e_awaddr, s_awaddr}), .s_axi_awlen({e_awlen, s_awlen}),
		.s_axi_awsize({e_awsize, s_awsize}), .s_axi_awburst({e_awburst, s_awburst}),
		.s_axi_awlock({1'b0, 1'b0}), .s_axi_awcache({e_awcache, s_awcache}),
		.s_axi_awprot({e_awprot, s_awprot}), .s_axi_awqos({4'd0, 4'd0}),
		.s_axi_awvalid({e_awvalid, s_awvalid}), .s_axi_awready(x_awready),
		.s_axi_wdata({e_wdata, s_wdata}), .s_axi_wstrb({e_wstrb, s_wstrb}),
		.s_axi_wlast({e_wlast, s_wlast}), .s_axi_wvalid({e_wvalid, s_wvalid}),
		.s_axi_wready(x_wready), .s_axi_bid(x_bid), .s_axi_bresp(x_bresp),
		.s_axi_bvalid(x_bvalid), .s_axi_bready({e_bready, s_bready}),
		.s_axi_arid(2'b00), .s_axi_araddr({e_araddr, s_araddr}),
		.s_axi_arlen({e_arlen, s_arlen}), .s_axi_arsize({e_arsize, s_arsize}),
		.s_axi_arburst({e_arburst, s_arburst}), .s_axi_arlock({1'b0, 1'b0}),
		.s_axi_arcache({e_arcache, s_arcache}), .s_axi_arprot({e_arprot, s_arprot}),
		.s_axi_arqos({4'd0, 4'd0}), .s_axi_arvalid({e_arvalid, s_arvalid}),
		.s_axi_arready(x_arready), .s_axi_rid(x_rid), .s_axi_rdata(x_rdata),
		.s_axi_rresp(x_rresp), .s_axi_rlast(x_rlast), .s_axi_rvalid(x_rvalid),
		.s_axi_rready({e_rready, s_rready}),
		.m_axi_awid(g_awid), .m_axi_awaddr(g_awaddr), .m_axi_awlen(g_awlen),
		.m_axi_awsize(g_awsize), .m_axi_awburst(g_awburst), .m_axi_awlock(g_awlock),
		.m_axi_awcache(g_awcache), .m_axi_awprot(g_awprot), .m_axi_awregion(g_awregion),
		.m_axi_awqos(g_awqos), .m_axi_awvalid(g_awvalid), .m_axi_awready(g_awready),
		.m_axi_wdata(g_wdata), .m_axi_wstrb(g_wstrb), .m_axi_wlast(g_wlast),
		.m_axi_wvalid(g_wvalid), .m_axi_wready(g_wready), .m_axi_bid(g_bid),
		.m_axi_bresp(g_bresp), .m_axi_bvalid(g_bvalid), .m_axi_bready(g_bready),
		.m_axi_arid(g_arid), .m_axi_araddr(g_araddr), .m_axi_arlen(g_arlen),
		.m_axi_arsize(g_arsize), .m_axi_arburst(g_arburst), .m_axi_arlock(g_arlock),
		.m_axi_arcache(g_arcache), .m_axi_arprot(g_arprot), .m_axi_arregion(g_arregion),
		.m_axi_arqos(g_arqos), .m_axi_arvalid(g_arvalid), .m_axi_arready(g_arready),
		.m_axi_rid(g_rid), .m_axi_rdata(g_rdata), .m_axi_rresp(g_rresp),
		.m_axi_rlast(g_rlast), .m_axi_rvalid(g_rvalid), .m_axi_rready(g_rready)
	);
	assign s_awready = x_awready[0];
	assign s_wready = x_wready[0];
	assign s_bresp = x_bresp[1:0];
	assign s_bvalid = x_bvalid[0];
	assign s_arready = x_arready[0];
	assign s_rdata = x_rdata[511:0];
	assign s_rresp = x_rresp[1:0];
	assign s_rlast = x_rlast[0];
	assign s_rvalid = x_rvalid[0];
	assign e_awready = x_awready[1];
	assign e_wready = x_wready[1];
	assign e_bresp = x_bresp[3:2];
	assign e_bvalid = x_bvalid[1];
	assign e_arready = x_arready[1];
	assign e_rdata = x_rdata[1023:512];
	assign e_rresp = x_rresp[3:2];
	assign e_rlast = x_rlast[1];
	assign e_rvalid = x_rvalid[1];

	/* BRAM CDC clk -> ui_clk (512 bit), then 512 -> 256 bit in ui_clk (333 MHz) */
	wire [0:0] n_awid, n_bid, n_arid, n_rid;
	wire [31:0] n_awaddr, n_araddr;
	wire [7:0] n_awlen, n_arlen;
	wire [2:0] n_awsize, n_arsize, n_awprot, n_arprot;
	wire [1:0] n_awburst, n_arburst, n_bresp, n_rresp;
	wire [3:0] n_awcache, n_arcache;
	wire n_awvalid, n_awready, n_wlast, n_wvalid, n_wready, n_bvalid, n_bready;
	wire n_arvalid, n_arready, n_rlast, n_rvalid, n_rready;
	wire [511:0] n_wdata, n_rdata;
	wire [63:0] n_wstrb;

	am_axi4_cdc #(
		.A_W(32), .D_W(512), .ID_W(1), .DEPTH_LOG2(4), .USE_BRAM(0)
	) cc (
		.s_clk(clk), .s_rst_n(rst_n),
		.s_awid(g_awid), .s_awaddr(g_awaddr), .s_awlen(g_awlen),
		.s_awsize(g_awsize), .s_awburst(g_awburst), .s_awcache(g_awcache), .s_awprot(g_awprot),
		.s_awvalid(g_awvalid), .s_awready(g_awready),
		.s_wdata(g_wdata), .s_wstrb(g_wstrb), .s_wlast(g_wlast),
		.s_wvalid(g_wvalid), .s_wready(g_wready),
		.s_bid(g_bid), .s_bresp(g_bresp), .s_bvalid(g_bvalid), .s_bready(g_bready),
		.s_arid(g_arid), .s_araddr(g_araddr), .s_arlen(g_arlen),
		.s_arsize(g_arsize), .s_arburst(g_arburst), .s_arcache(g_arcache), .s_arprot(g_arprot),
		.s_arvalid(g_arvalid), .s_arready(g_arready),
		.s_rid(g_rid), .s_rdata(g_rdata), .s_rresp(g_rresp),
		.s_rlast(g_rlast), .s_rvalid(g_rvalid), .s_rready(g_rready),
		.m_clk(ui_clk), .m_rst_n(aresetn),
		.m_awid(n_awid), .m_awaddr(n_awaddr), .m_awlen(n_awlen),
		.m_awsize(n_awsize), .m_awburst(n_awburst), .m_awcache(n_awcache), .m_awprot(n_awprot),
		.m_awvalid(n_awvalid), .m_awready(n_awready),
		.m_wdata(n_wdata), .m_wstrb(n_wstrb), .m_wlast(n_wlast),
		.m_wvalid(n_wvalid), .m_wready(n_wready),
		.m_bid(n_bid), .m_bresp(n_bresp), .m_bvalid(n_bvalid), .m_bready(n_bready),
		.m_arid(n_arid), .m_araddr(n_araddr), .m_arlen(n_arlen),
		.m_arsize(n_arsize), .m_arburst(n_arburst), .m_arcache(n_arcache), .m_arprot(n_arprot),
		.m_arvalid(n_arvalid), .m_arready(n_arready),
		.m_rid(n_rid), .m_rdata(n_rdata), .m_rresp(n_rresp),
		.m_rlast(n_rlast), .m_rvalid(n_rvalid), .m_rready(n_rready)
	);

	wire [31:0] u_awaddr, u_araddr;
	wire [7:0] u_awlen, u_arlen;
	wire [2:0] u_awsize, u_arsize, u_awprot, u_arprot;
	wire [1:0] u_awburst, u_arburst, u_bresp, u_rresp;
	wire [3:0] u_awcache, u_arcache, u_awregion, u_arregion, u_awqos, u_arqos, u_bid4, u_rid4;
	wire u_awlock, u_arlock;
	wire u_awvalid, u_awready, u_wlast, u_wvalid, u_wready, u_bvalid, u_bready;
	wire u_arvalid, u_arready, u_rlast, u_rvalid, u_rready;
	wire [255:0] u_wdata, u_rdata;
	wire [31:0] u_wstrb;

	atg_xcku5p_axi_dn axi_dn (
		.s_axi_aclk(ui_clk), .s_axi_aresetn(aresetn),
		.s_axi_awid(n_awid), .s_axi_awaddr(n_awaddr), .s_axi_awlen(n_awlen),
		.s_axi_awsize(n_awsize), .s_axi_awburst(n_awburst), .s_axi_awlock(1'b0),
		.s_axi_awcache(n_awcache), .s_axi_awprot(n_awprot), .s_axi_awregion(4'd0),
		.s_axi_awqos(4'd0), .s_axi_awvalid(n_awvalid), .s_axi_awready(n_awready),
		.s_axi_wdata(n_wdata), .s_axi_wstrb(n_wstrb), .s_axi_wlast(n_wlast),
		.s_axi_wvalid(n_wvalid), .s_axi_wready(n_wready), .s_axi_bid(n_bid),
		.s_axi_bresp(n_bresp), .s_axi_bvalid(n_bvalid), .s_axi_bready(n_bready),
		.s_axi_arid(n_arid), .s_axi_araddr(n_araddr), .s_axi_arlen(n_arlen),
		.s_axi_arsize(n_arsize), .s_axi_arburst(n_arburst), .s_axi_arlock(1'b0),
		.s_axi_arcache(n_arcache), .s_axi_arprot(n_arprot), .s_axi_arregion(4'd0),
		.s_axi_arqos(4'd0), .s_axi_arvalid(n_arvalid), .s_axi_arready(n_arready),
		.s_axi_rid(n_rid), .s_axi_rdata(n_rdata), .s_axi_rresp(n_rresp),
		.s_axi_rlast(n_rlast), .s_axi_rvalid(n_rvalid), .s_axi_rready(n_rready),
		.m_axi_awaddr(u_awaddr), .m_axi_awlen(u_awlen), .m_axi_awsize(u_awsize),
		.m_axi_awburst(u_awburst), .m_axi_awlock(u_awlock), .m_axi_awcache(u_awcache),
		.m_axi_awprot(u_awprot), .m_axi_awregion(u_awregion), .m_axi_awqos(u_awqos),
		.m_axi_awvalid(u_awvalid), .m_axi_awready(u_awready), .m_axi_wdata(u_wdata),
		.m_axi_wstrb(u_wstrb), .m_axi_wlast(u_wlast), .m_axi_wvalid(u_wvalid),
		.m_axi_wready(u_wready), .m_axi_bresp(u_bresp), .m_axi_bvalid(u_bvalid),
		.m_axi_bready(u_bready), .m_axi_araddr(u_araddr), .m_axi_arlen(u_arlen),
		.m_axi_arsize(u_arsize), .m_axi_arburst(u_arburst), .m_axi_arlock(u_arlock),
		.m_axi_arcache(u_arcache), .m_axi_arprot(u_arprot), .m_axi_arregion(u_arregion),
		.m_axi_arqos(u_arqos), .m_axi_arvalid(u_arvalid), .m_axi_arready(u_arready),
		.m_axi_rdata(u_rdata), .m_axi_rresp(u_rresp), .m_axi_rlast(u_rlast),
		.m_axi_rvalid(u_rvalid), .m_axi_rready(u_rready)
	);

	/* Pipeline W-channel between 512->256 downsizer and DDR4 MIG:
	 * breaks the 7-LUT combinational mux path on 333.3 MHz ui_clk with
	 * minimal LUT/FF overhead (only W violated in v9; AW/AR/B/R met easily).
	 */
	wire [29:0] d_awaddr, d_araddr;
	wire [7:0] d_awlen, d_arlen;
	wire [2:0] d_awsize, d_arsize, d_awprot, d_arprot;
	wire [1:0] d_awburst, d_arburst, d_bresp, d_rresp;
	wire [3:0] d_awcache, d_arcache;
	wire d_awvalid, d_awready, d_wlast, d_wvalid, d_wready, d_bvalid, d_bready;
	wire d_arvalid, d_arready, d_rlast, d_rvalid, d_rready;
	wire [255:0] d_wdata, d_rdata;
	wire [31:0] d_wstrb;
	wire [0:0] d_awid_unused, d_bid_unused, d_arid_unused, d_rid_unused;

	am_axi_pipe #(
		.A_W(30), .D_W(256), .ID_W(1),
		.PIPE_AW(0), .PIPE_W(1), .PIPE_B(0), .PIPE_AR(0), .PIPE_R(0)
	) ddr4_pipe (
		.clk(ui_clk), .rst_n(aresetn),
		.s_awid(1'b0), .s_awaddr(u_awaddr[29:0]), .s_awlen(u_awlen),
		.s_awsize(u_awsize), .s_awburst(u_awburst), .s_awcache(u_awcache),
		.s_awprot(u_awprot), .s_awvalid(u_awvalid), .s_awready(u_awready),
		.s_wdata(u_wdata), .s_wstrb(u_wstrb), .s_wlast(u_wlast),
		.s_wvalid(u_wvalid), .s_wready(u_wready),
		.s_bid(d_bid_unused), .s_bresp(u_bresp), .s_bvalid(u_bvalid), .s_bready(u_bready),
		.s_arid(1'b0), .s_araddr(u_araddr[29:0]), .s_arlen(u_arlen),
		.s_arsize(u_arsize), .s_arburst(u_arburst), .s_arcache(u_arcache),
		.s_arprot(u_arprot), .s_arvalid(u_arvalid), .s_arready(u_arready),
		.s_rid(d_rid_unused), .s_rdata(u_rdata), .s_rresp(u_rresp),
		.s_rlast(u_rlast), .s_rvalid(u_rvalid), .s_rready(u_rready),
		.m_awid(d_awid_unused), .m_awaddr(d_awaddr), .m_awlen(d_awlen),
		.m_awsize(d_awsize), .m_awburst(d_awburst), .m_awcache(d_awcache),
		.m_awprot(d_awprot), .m_awvalid(d_awvalid), .m_awready(d_awready),
		.m_wdata(d_wdata), .m_wstrb(d_wstrb), .m_wlast(d_wlast),
		.m_wvalid(d_wvalid), .m_wready(d_wready),
		.m_bid(1'b0), .m_bresp(d_bresp), .m_bvalid(d_bvalid), .m_bready(d_bready),
		.m_arid(d_arid_unused), .m_araddr(d_araddr), .m_arlen(d_arlen),
		.m_arsize(d_arsize), .m_arburst(d_arburst), .m_arcache(d_arcache),
		.m_arprot(d_arprot), .m_arvalid(d_arvalid), .m_arready(d_arready),
		.m_rid(1'b0), .m_rdata(d_rdata), .m_rresp(d_rresp),
		.m_rlast(d_rlast), .m_rvalid(d_rvalid), .m_rready(d_rready)
	);

	atg_xcku5p_ddr4 ddr4 (
		.sys_rst(!sys_rst_n || hold_ref),
		.c0_sys_clk_i(ref_ibuf),
		.c0_init_calib_complete(calib),
		.c0_ddr4_ui_clk(ui_clk),
		.c0_ddr4_ui_clk_sync_rst(ui_rst),
		.c0_ddr4_aresetn(aresetn),
		.c0_ddr4_dq(ddr4_dq),
		.c0_ddr4_dqs_c(ddr4_dqs_n),
		.c0_ddr4_dqs_t(ddr4_dqs_p),
		.c0_ddr4_dm_dbi_n(ddr4_dm_n),
		.c0_ddr4_adr(ddr4_addr),
		.c0_ddr4_ba(ddr4_ba),
		.c0_ddr4_bg(ddr4_bg),
		.c0_ddr4_act_n(ddr4_act_n),
		.c0_ddr4_reset_n(ddr4_reset_n),
		.c0_ddr4_ck_c(ddr4_ck_n),
		.c0_ddr4_ck_t(ddr4_ck_p),
		.c0_ddr4_cke(ddr4_cke),
		.c0_ddr4_cs_n(ddr4_cs_n),
		.c0_ddr4_odt(ddr4_odt),
		.c0_ddr4_s_axi_awid(4'd0),
		.c0_ddr4_s_axi_awaddr(d_awaddr),
		.c0_ddr4_s_axi_awlen(d_awlen),
		.c0_ddr4_s_axi_awsize(d_awsize),
		.c0_ddr4_s_axi_awburst(d_awburst),
		.c0_ddr4_s_axi_awlock(1'b0),
		.c0_ddr4_s_axi_awcache(d_awcache),
		.c0_ddr4_s_axi_awprot(d_awprot),
		.c0_ddr4_s_axi_awqos(4'd0),
		.c0_ddr4_s_axi_awvalid(d_awvalid),
		.c0_ddr4_s_axi_awready(d_awready),
		.c0_ddr4_s_axi_wdata(d_wdata),
		.c0_ddr4_s_axi_wstrb(d_wstrb),
		.c0_ddr4_s_axi_wlast(d_wlast),
		.c0_ddr4_s_axi_wvalid(d_wvalid),
		.c0_ddr4_s_axi_wready(d_wready),
		.c0_ddr4_s_axi_bready(d_bready),
		.c0_ddr4_s_axi_bid(u_bid4),
		.c0_ddr4_s_axi_bresp(d_bresp),
		.c0_ddr4_s_axi_bvalid(d_bvalid),
		.c0_ddr4_s_axi_arid(4'd0),
		.c0_ddr4_s_axi_araddr(d_araddr),
		.c0_ddr4_s_axi_arlen(d_arlen),
		.c0_ddr4_s_axi_arsize(d_arsize),
		.c0_ddr4_s_axi_arburst(d_arburst),
		.c0_ddr4_s_axi_arlock(1'b0),
		.c0_ddr4_s_axi_arcache(d_arcache),
		.c0_ddr4_s_axi_arprot(d_arprot),
		.c0_ddr4_s_axi_arqos(4'd0),
		.c0_ddr4_s_axi_arvalid(d_arvalid),
		.c0_ddr4_s_axi_arready(d_arready),
		.c0_ddr4_s_axi_rready(d_rready),
		.c0_ddr4_s_axi_rid(u_rid4),
		.c0_ddr4_s_axi_rdata(d_rdata),
		.c0_ddr4_s_axi_rresp(d_rresp),
		.c0_ddr4_s_axi_rlast(d_rlast),
		.c0_ddr4_s_axi_rvalid(d_rvalid),
		.dbg_clk(),
		.dbg_bus()
	);

	wire unused_ok = &{1'b0, xr_waddr[1:0], xr_raddr[1:0], u_awaddr[31:30], u_araddr[31:30],
		u_bid4, u_rid4, u_awregion, u_arregion, e_awid, e_arid,
		u_awlock, u_arlock, u_awqos, u_arqos,
		d_awid_unused, d_bid_unused, d_arid_unused, d_rid_unused,
		c_awlock, c_arlock, c_awregion, c_arregion, c_awqos, c_arqos,
		g_awlock, g_arlock, g_awregion, g_arregion, g_awqos, g_arqos,
		bar2_cc_s_bid_unused, bar2_cc_s_rid_unused, x_bid, x_rid};
endmodule

`default_nettype wire
