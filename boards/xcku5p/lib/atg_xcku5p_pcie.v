/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_xcku5p_pcie: HamGeek XCKU5P PCIe Gen3 x4 endpoint (FMC) via xdma in
 * AXI_Bridge mode (128-bit AXI, BAR0 128 KiB, BAR2 64 MiB, 1 MSI vector).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_xcku5p_pcie (
	input wire pcie_refclk_p,
	input wire pcie_refclk_n,
	input wire pcie_perst_n,
	output wire [3:0] pcie_txp,
	output wire [3:0] pcie_txn,
	input wire [3:0] pcie_rxp,
	input wire [3:0] pcie_rxn,
	output wire user_clk,
	output wire user_rst_n,
	output wire link_up,
	/* BAR0: the register window */
	output wire [16:0] m0_awaddr,
	output wire [3:0] m0_awid,
	output wire m0_awvalid,
	input wire m0_awready,
	output wire [63:0] m0_wdata,
	output wire [7:0] m0_wstrb,
	output wire m0_wlast,
	output wire m0_wvalid,
	input wire m0_wready,
	input wire [3:0] m0_bid,
	input wire [1:0] m0_bresp,
	input wire m0_bvalid,
	output wire m0_bready,
	output wire [16:0] m0_araddr,
	output wire [3:0] m0_arid,
	output wire [7:0] m0_arlen,
	output wire m0_arvalid,
	input wire m0_arready,
	input wire [63:0] m0_rdata,
	input wire [3:0] m0_rid,
	input wire [1:0] m0_rresp,
	input wire m0_rlast,
	input wire m0_rvalid,
	output wire m0_rready,
	/* BAR2: the local window */
	output wire [25:0] m2_awaddr,
	output wire [3:0] m2_awid,
	output wire [7:0] m2_awlen,
	output wire [2:0] m2_awsize,
	output wire [1:0] m2_awburst,
	output wire m2_awvalid,
	input wire m2_awready,
	output wire [127:0] m2_wdata,
	output wire [15:0] m2_wstrb,
	output wire m2_wlast,
	output wire m2_wvalid,
	input wire m2_wready,
	input wire [3:0] m2_bid,
	input wire [1:0] m2_bresp,
	input wire m2_bvalid,
	output wire m2_bready,
	output wire [25:0] m2_araddr,
	output wire [3:0] m2_arid,
	output wire [7:0] m2_arlen,
	output wire [2:0] m2_arsize,
	output wire [1:0] m2_arburst,
	output wire m2_arvalid,
	input wire m2_arready,
	input wire [127:0] m2_rdata,
	input wire [3:0] m2_rid,
	input wire [1:0] m2_rresp,
	input wire m2_rlast,
	input wire m2_rvalid,
	output wire m2_rready,
	/* GPU -> host memory (s_clk) */
	input wire s_clk,
	input wire s_rst_n,
	input wire [63:0] s_awaddr,
	input wire [3:0] s_awid,
	input wire [7:0] s_awlen,
	input wire [2:0] s_awsize,
	input wire [1:0] s_awburst,
	input wire s_awvalid,
	output wire s_awready,
	input wire [511:0] s_wdata,
	input wire [63:0] s_wstrb,
	input wire s_wlast,
	input wire s_wvalid,
	output wire s_wready,
	output wire [3:0] s_bid,
	output wire [1:0] s_bresp,
	output wire s_bvalid,
	input wire s_bready,
	input wire [63:0] s_araddr,
	input wire [3:0] s_arid,
	input wire [7:0] s_arlen,
	input wire [2:0] s_arsize,
	input wire [1:0] s_arburst,
	input wire s_arvalid,
	output wire s_arready,
	output wire [511:0] s_rdata,
	output wire [3:0] s_rid,
	output wire [1:0] s_rresp,
	output wire s_rlast,
	output wire s_rvalid,
	input wire s_rready,
	/* MSI */
	input wire msi_req,
	output wire msi_ack
);
	wire refclk, refclk_gt, aclk, axi_aresetn, ctl_aresetn;
	wire irq_ack;

	IBUFDS_GTE4 #(
		.REFCLK_EN_TX_PATH(1'b0),
		.REFCLK_HROW_CK_SEL(2'b00),
		.REFCLK_ICNTL_RX(2'b00)
	) refclk_buf (
		.I(pcie_refclk_p),
		.IB(pcie_refclk_n),
		.CEB(1'b0),
		.O(refclk_gt),
		.ODIV2(refclk)
	);

	wire rst_in = pcie_perst_n && axi_aresetn;
	am_sync s_rst (.clk(aclk), .rst_n(rst_in), .d(1'b1), .q(user_rst_n));
	assign user_clk = aclk;

	/* Bridge master (128-bit) */
	wire [3:0] b_awid, b_bid, b_arid, b_rid;
	wire [63:0] b_awaddr, b_araddr;
	wire [7:0] b_awlen, b_arlen;
	wire [2:0] b_awsize, b_arsize, b_awprot, b_arprot;
	wire [1:0] b_awburst, b_arburst, b_bresp, b_rresp;
	wire [3:0] b_awcache, b_arcache;
	wire b_awlock, b_arlock, b_awvalid, b_awready, b_wlast, b_wvalid, b_wready, b_bvalid;
	wire b_bready, b_arvalid, b_arready, b_rlast, b_rvalid, b_rready;
	wire [127:0] b_wdata, b_rdata;
	wire [15:0] b_wstrb;

	/* MSI handshake */
	reg msi_q = 1'b0;
	reg ack_q = 1'b0;

	/* GPU -> host accesses: BRAM CDC s_clk -> user_clk (512-bit), then 512 -> 128 bit at user_clk */
	wire [63:0] h_awaddr, h_araddr, k_awaddr, k_araddr;
	wire [7:0] h_awlen, h_arlen, k_awlen, k_arlen;
	wire [2:0] h_awsize, h_arsize, h_awprot, h_arprot, k_awsize, k_arsize, k_awprot, k_arprot;
	wire [1:0] h_awburst, h_arburst, h_bresp, h_rresp, k_awburst, k_arburst, k_bresp, k_rresp;
	wire [3:0] h_awcache, h_arcache, h_awregion, h_arregion, h_awqos, h_arqos;
	wire [3:0] k_awcache, k_arcache;
	wire [3:0] k_awid, k_bid, k_arid, k_rid;
	wire h_awlock, h_arlock;
	wire h_awvalid, h_awready, h_wlast, h_wvalid, h_wready, h_bvalid, h_bready;
	wire h_arvalid, h_arready, h_rlast, h_rvalid, h_rready;
	wire k_awvalid, k_awready, k_wlast, k_wvalid, k_wready, k_bvalid, k_bready;
	wire k_arvalid, k_arready, k_rlast, k_rvalid, k_rready;
	wire [127:0] h_wdata, h_rdata;
	wire [511:0] k_wdata, k_rdata;
	wire [15:0] h_wstrb;
	wire [63:0] k_wstrb;

	am_axi4_cdc #(
		.A_W(64), .D_W(512), .ID_W(4), .DEPTH_LOG2(4), .USE_BRAM(0)
	) host_cc (
		.s_clk(s_clk), .s_rst_n(s_rst_n),
		.s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen),
		.s_awsize(s_awsize), .s_awburst(s_awburst), .s_awcache(4'b0011), .s_awprot(3'd0),
		.s_awvalid(s_awvalid), .s_awready(s_awready),
		.s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
		.s_wvalid(s_wvalid), .s_wready(s_wready),
		.s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
		.s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen),
		.s_arsize(s_arsize), .s_arburst(s_arburst), .s_arcache(4'b0011), .s_arprot(3'd0),
		.s_arvalid(s_arvalid), .s_arready(s_arready),
		.s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp),
		.s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),
		.m_clk(aclk), .m_rst_n(user_rst_n),
		.m_awid(k_awid), .m_awaddr(k_awaddr), .m_awlen(k_awlen),
		.m_awsize(k_awsize), .m_awburst(k_awburst), .m_awcache(k_awcache), .m_awprot(k_awprot),
		.m_awvalid(k_awvalid), .m_awready(k_awready),
		.m_wdata(k_wdata), .m_wstrb(k_wstrb), .m_wlast(k_wlast),
		.m_wvalid(k_wvalid), .m_wready(k_wready),
		.m_bid(k_bid), .m_bresp(k_bresp), .m_bvalid(k_bvalid), .m_bready(k_bready),
		.m_arid(k_arid), .m_araddr(k_araddr), .m_arlen(k_arlen),
		.m_arsize(k_arsize), .m_arburst(k_arburst), .m_arcache(k_arcache), .m_arprot(k_arprot),
		.m_arvalid(k_arvalid), .m_arready(k_arready),
		.m_rid(k_rid), .m_rdata(k_rdata), .m_rresp(k_rresp),
		.m_rlast(k_rlast), .m_rvalid(k_rvalid), .m_rready(k_rready)
	);

	atg_xcku5p_host_dn host_dn (
		.s_axi_aclk(aclk), .s_axi_aresetn(user_rst_n),
		.s_axi_awid(k_awid), .s_axi_awaddr(k_awaddr), .s_axi_awlen(k_awlen),
		.s_axi_awsize(k_awsize), .s_axi_awburst(k_awburst), .s_axi_awlock(1'b0),
		.s_axi_awcache(k_awcache), .s_axi_awprot(k_awprot), .s_axi_awregion(4'd0),
		.s_axi_awqos(4'd0), .s_axi_awvalid(k_awvalid), .s_axi_awready(k_awready),
		.s_axi_wdata(k_wdata), .s_axi_wstrb(k_wstrb), .s_axi_wlast(k_wlast),
		.s_axi_wvalid(k_wvalid), .s_axi_wready(k_wready), .s_axi_bid(k_bid),
		.s_axi_bresp(k_bresp), .s_axi_bvalid(k_bvalid), .s_axi_bready(k_bready),
		.s_axi_arid(k_arid), .s_axi_araddr(k_araddr), .s_axi_arlen(k_arlen),
		.s_axi_arsize(k_arsize), .s_axi_arburst(k_arburst), .s_axi_arlock(1'b0),
		.s_axi_arcache(k_arcache), .s_axi_arprot(k_arprot), .s_axi_arregion(4'd0),
		.s_axi_arqos(4'd0), .s_axi_arvalid(k_arvalid), .s_axi_arready(k_arready),
		.s_axi_rid(k_rid), .s_axi_rdata(k_rdata), .s_axi_rresp(k_rresp),
		.s_axi_rlast(k_rlast), .s_axi_rvalid(k_rvalid), .s_axi_rready(k_rready),
		.m_axi_awaddr(h_awaddr), .m_axi_awlen(h_awlen), .m_axi_awsize(h_awsize),
		.m_axi_awburst(h_awburst), .m_axi_awlock(h_awlock), .m_axi_awcache(h_awcache),
		.m_axi_awprot(h_awprot), .m_axi_awregion(h_awregion), .m_axi_awqos(h_awqos),
		.m_axi_awvalid(h_awvalid), .m_axi_awready(h_awready), .m_axi_wdata(h_wdata),
		.m_axi_wstrb(h_wstrb), .m_axi_wlast(h_wlast), .m_axi_wvalid(h_wvalid),
		.m_axi_wready(h_wready), .m_axi_bresp(h_bresp), .m_axi_bvalid(h_bvalid),
		.m_axi_bready(h_bready), .m_axi_araddr(h_araddr), .m_axi_arlen(h_arlen),
		.m_axi_arsize(h_arsize), .m_axi_arburst(h_arburst), .m_axi_arlock(h_arlock),
		.m_axi_arcache(h_arcache), .m_axi_arprot(h_arprot), .m_axi_arregion(h_arregion),
		.m_axi_arqos(h_arqos), .m_axi_arvalid(h_arvalid), .m_axi_arready(h_arready),
		.m_axi_rdata(h_rdata), .m_axi_rresp(h_rresp), .m_axi_rlast(h_rlast),
		.m_axi_rvalid(h_rvalid), .m_axi_rready(h_rready)
	);

	atg_xcku5p_pcie_ip bridge (
		.sys_clk(refclk),
		.sys_clk_gt(refclk_gt),
		.sys_rst_n(pcie_perst_n),
		.cfg_ltssm_state(),
		.user_lnk_up(link_up),
		.pci_exp_txp(pcie_txp), .pci_exp_txn(pcie_txn),
		.pci_exp_rxp(pcie_rxp), .pci_exp_rxn(pcie_rxn),
		.axi_aclk(aclk), .axi_aresetn(axi_aresetn),
		.axi_ctl_aresetn(ctl_aresetn),
		.usr_irq_req(msi_q), .usr_irq_ack(irq_ack),
		.msi_enable(), .msi_vector_width(),
		.s_axib_awid(4'd0), .s_axib_awaddr(h_awaddr), .s_axib_awregion(h_awregion),
		.s_axib_awlen(h_awlen), .s_axib_awsize(h_awsize), .s_axib_awburst(h_awburst),
		.s_axib_awvalid(h_awvalid), .s_axib_awready(h_awready), .s_axib_wdata(h_wdata),
		.s_axib_wstrb(h_wstrb), .s_axib_wlast(h_wlast), .s_axib_wvalid(h_wvalid),
		.s_axib_wready(h_wready), .s_axib_bid(), .s_axib_bresp(h_bresp),
		.s_axib_bvalid(h_bvalid), .s_axib_bready(h_bready), .s_axib_arid(4'd0),
		.s_axib_araddr(h_araddr), .s_axib_arregion(h_arregion), .s_axib_arlen(h_arlen),
		.s_axib_arsize(h_arsize), .s_axib_arburst(h_arburst), .s_axib_arvalid(h_arvalid),
		.s_axib_arready(h_arready), .s_axib_rid(), .s_axib_rdata(h_rdata),
		.s_axib_rresp(h_rresp), .s_axib_rlast(h_rlast), .s_axib_rvalid(h_rvalid),
		.s_axib_rready(h_rready),
		.s_axil_awaddr(32'd0), .s_axil_awprot(3'd0), .s_axil_awvalid(1'b0), .s_axil_awready(),
		.s_axil_wdata(32'd0), .s_axil_wstrb(4'd0), .s_axil_wvalid(1'b0), .s_axil_wready(),
		.s_axil_bvalid(), .s_axil_bresp(), .s_axil_bready(1'b0),
		.s_axil_araddr(32'd0), .s_axil_arprot(3'd0), .s_axil_arvalid(1'b0), .s_axil_arready(),
		.s_axil_rdata(), .s_axil_rresp(), .s_axil_rvalid(), .s_axil_rready(1'b0),
		.interrupt_out(),
		.m_axib_awid(b_awid), .m_axib_awaddr(b_awaddr), .m_axib_awlen(b_awlen),
		.m_axib_awsize(b_awsize), .m_axib_awburst(b_awburst), .m_axib_awprot(b_awprot),
		.m_axib_awvalid(b_awvalid), .m_axib_awready(b_awready), .m_axib_awlock(b_awlock),
		.m_axib_awcache(b_awcache), .m_axib_wdata(b_wdata), .m_axib_wstrb(b_wstrb),
		.m_axib_wlast(b_wlast), .m_axib_wvalid(b_wvalid), .m_axib_wready(b_wready),
		.m_axib_bid(b_bid), .m_axib_bresp(b_bresp), .m_axib_bvalid(b_bvalid),
		.m_axib_bready(b_bready), .m_axib_arid(b_arid), .m_axib_araddr(b_araddr),
		.m_axib_arlen(b_arlen), .m_axib_arsize(b_arsize), .m_axib_arburst(b_arburst),
		.m_axib_arprot(b_arprot), .m_axib_arvalid(b_arvalid), .m_axib_arready(b_arready),
		.m_axib_arlock(b_arlock), .m_axib_arcache(b_arcache), .m_axib_rid(b_rid),
		.m_axib_rdata(b_rdata), .m_axib_rresp(b_rresp), .m_axib_rlast(b_rlast),
		.m_axib_rvalid(b_rvalid), .m_axib_rready(b_rready)
	);

	always @(posedge aclk or negedge user_rst_n) begin
		if (!user_rst_n) begin
			msi_q <= 1'b0;
			ack_q <= 1'b0;
		end else begin
			ack_q <= 1'b0;
			if (msi_q) begin
				if (irq_ack) begin
					msi_q <= 1'b0;
					ack_q <= 1'b1;
				end
			end else if (msi_req && !ack_q)
				msi_q <= 1'b1;
		end
	end
	assign msi_ack = ack_q;

	/* M00 BAR0, M01 BAR2 */
	wire [7:0] x_awid, x_arid, x_bid, x_rid;
	wire [1:0] x_awvalid, x_awready, x_wlast, x_wvalid, x_wready;
	wire [1:0] x_bvalid, x_bready, x_arvalid, x_arready, x_rlast, x_rvalid, x_rready;
	wire [63:0] x_awaddr, x_araddr;
	wire [15:0] x_awlen, x_arlen;
	wire [5:0] x_awsize, x_arsize, x_awprot, x_arprot;
	wire [3:0] x_awburst, x_arburst, x_bresp, x_rresp;
	wire [1:0] x_awlock, x_arlock;
	wire [7:0] x_awcache, x_arcache, x_awregion, x_arregion, x_awqos, x_arqos;
	wire [255:0] x_wdata, x_rdata;
	wire [31:0] x_wstrb;

	atg_xcku5p_bar_xbar split (
		.aclk(aclk), .aresetn(user_rst_n),
		.s_axi_awid(b_awid), .s_axi_awaddr(b_awaddr[31:0]), .s_axi_awlen(b_awlen),
		.s_axi_awsize(b_awsize), .s_axi_awburst(b_awburst), .s_axi_awlock(b_awlock),
		.s_axi_awcache(b_awcache), .s_axi_awprot(b_awprot), .s_axi_awqos(4'd0),
		.s_axi_awvalid(b_awvalid), .s_axi_awready(b_awready), .s_axi_wdata(b_wdata),
		.s_axi_wstrb(b_wstrb), .s_axi_wlast(b_wlast), .s_axi_wvalid(b_wvalid),
		.s_axi_wready(b_wready), .s_axi_bid(b_bid), .s_axi_bresp(b_bresp),
		.s_axi_bvalid(b_bvalid), .s_axi_bready(b_bready), .s_axi_arid(b_arid),
		.s_axi_araddr(b_araddr[31:0]), .s_axi_arlen(b_arlen), .s_axi_arsize(b_arsize),
		.s_axi_arburst(b_arburst), .s_axi_arlock(b_arlock), .s_axi_arcache(b_arcache),
		.s_axi_arprot(b_arprot), .s_axi_arqos(4'd0), .s_axi_arvalid(b_arvalid),
		.s_axi_arready(b_arready), .s_axi_rid(b_rid), .s_axi_rdata(b_rdata),
		.s_axi_rresp(b_rresp), .s_axi_rlast(b_rlast), .s_axi_rvalid(b_rvalid),
		.s_axi_rready(b_rready),
		.m_axi_awid(x_awid), .m_axi_awaddr(x_awaddr), .m_axi_awlen(x_awlen),
		.m_axi_awsize(x_awsize), .m_axi_awburst(x_awburst), .m_axi_awlock(x_awlock),
		.m_axi_awcache(x_awcache), .m_axi_awprot(x_awprot), .m_axi_awregion(x_awregion),
		.m_axi_awqos(x_awqos), .m_axi_awvalid(x_awvalid), .m_axi_awready(x_awready),
		.m_axi_wdata(x_wdata), .m_axi_wstrb(x_wstrb), .m_axi_wlast(x_wlast),
		.m_axi_wvalid(x_wvalid), .m_axi_wready(x_wready), .m_axi_bid(x_bid),
		.m_axi_bresp(x_bresp), .m_axi_bvalid(x_bvalid), .m_axi_bready(x_bready),
		.m_axi_arid(x_arid), .m_axi_araddr(x_araddr), .m_axi_arlen(x_arlen),
		.m_axi_arsize(x_arsize), .m_axi_arburst(x_arburst), .m_axi_arlock(x_arlock),
		.m_axi_arcache(x_arcache), .m_axi_arprot(x_arprot), .m_axi_arregion(x_arregion),
		.m_axi_arqos(x_arqos), .m_axi_arvalid(x_arvalid), .m_axi_arready(x_arready),
		.m_axi_rid(x_rid), .m_axi_rdata(x_rdata), .m_axi_rresp(x_rresp),
		.m_axi_rlast(x_rlast), .m_axi_rvalid(x_rvalid), .m_axi_rready(x_rready)
	);

	/* M00 -> 64 bit: BAR0 */
	wire [31:0] d_awaddr, d_araddr;
	wire [7:0] d_awlen;
	wire [2:0] d_awsize, d_arsize, d_awprot, d_arprot;
	wire [1:0] d_awburst, d_arburst;
	wire [3:0] d_awcache, d_arcache, d_awregion, d_arregion, d_awqos, d_arqos;
	wire d_awlock, d_arlock;

	atg_xcku5p_bar0_dn bar0_dn (
		.s_axi_aclk(aclk), .s_axi_aresetn(user_rst_n),
		.s_axi_awid(x_awid[3:0]), .s_axi_awaddr(x_awaddr[31:0]), .s_axi_awlen(x_awlen[7:0]),
		.s_axi_awsize(x_awsize[2:0]), .s_axi_awburst(x_awburst[1:0]),
		.s_axi_awlock(x_awlock[0]), .s_axi_awcache(x_awcache[3:0]),
		.s_axi_awprot(x_awprot[2:0]), .s_axi_awregion(x_awregion[3:0]),
		.s_axi_awqos(x_awqos[3:0]), .s_axi_awvalid(x_awvalid[0]),
		.s_axi_awready(x_awready[0]), .s_axi_wdata(x_wdata[127:0]),
		.s_axi_wstrb(x_wstrb[15:0]), .s_axi_wlast(x_wlast[0]), .s_axi_wvalid(x_wvalid[0]),
		.s_axi_wready(x_wready[0]), .s_axi_bid(x_bid[3:0]), .s_axi_bresp(x_bresp[1:0]),
		.s_axi_bvalid(x_bvalid[0]), .s_axi_bready(x_bready[0]), .s_axi_arid(x_arid[3:0]),
		.s_axi_araddr(x_araddr[31:0]), .s_axi_arlen(x_arlen[7:0]),
		.s_axi_arsize(x_arsize[2:0]), .s_axi_arburst(x_arburst[1:0]),
		.s_axi_arlock(x_arlock[0]), .s_axi_arcache(x_arcache[3:0]),
		.s_axi_arprot(x_arprot[2:0]), .s_axi_arregion(x_arregion[3:0]),
		.s_axi_arqos(x_arqos[3:0]), .s_axi_arvalid(x_arvalid[0]),
		.s_axi_arready(x_arready[0]), .s_axi_rid(x_rid[3:0]), .s_axi_rdata(x_rdata[127:0]),
		.s_axi_rresp(x_rresp[1:0]), .s_axi_rlast(x_rlast[0]), .s_axi_rvalid(x_rvalid[0]),
		.s_axi_rready(x_rready[0]),
		.m_axi_awaddr(d_awaddr), .m_axi_awlen(d_awlen), .m_axi_awsize(d_awsize),
		.m_axi_awburst(d_awburst), .m_axi_awlock(d_awlock), .m_axi_awcache(d_awcache),
		.m_axi_awprot(d_awprot), .m_axi_awregion(d_awregion), .m_axi_awqos(d_awqos),
		.m_axi_awvalid(m0_awvalid), .m_axi_awready(m0_awready), .m_axi_wdata(m0_wdata),
		.m_axi_wstrb(m0_wstrb), .m_axi_wlast(m0_wlast), .m_axi_wvalid(m0_wvalid),
		.m_axi_wready(m0_wready), .m_axi_bresp(m0_bresp), .m_axi_bvalid(m0_bvalid),
		.m_axi_bready(m0_bready), .m_axi_araddr(d_araddr), .m_axi_arlen(m0_arlen),
		.m_axi_arsize(d_arsize), .m_axi_arburst(d_arburst), .m_axi_arlock(d_arlock),
		.m_axi_arcache(d_arcache), .m_axi_arprot(d_arprot), .m_axi_arregion(d_arregion),
		.m_axi_arqos(d_arqos), .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
		.m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp), .m_axi_rlast(m0_rlast),
		.m_axi_rvalid(m0_rvalid), .m_axi_rready(m0_rready)
	);
	assign m0_awaddr = d_awaddr[16:0];
	assign m0_araddr = d_araddr[16:0];
	assign m0_awid = 4'd0;
	assign m0_arid = 4'd0;

	/* M01: BAR2 */
	assign m2_awaddr = x_awaddr[57:32];
	assign m2_awid = x_awid[7:4];
	assign m2_awlen = x_awlen[15:8];
	assign m2_awsize = x_awsize[5:3];
	assign m2_awburst = x_awburst[3:2];
	assign m2_awvalid = x_awvalid[1];
	assign x_awready[1] = m2_awready;
	assign m2_wdata = x_wdata[255:128];
	assign m2_wstrb = x_wstrb[31:16];
	assign m2_wlast = x_wlast[1];
	assign m2_wvalid = x_wvalid[1];
	assign x_wready[1] = m2_wready;
	assign x_bid[7:4] = m2_bid;
	assign x_bresp[3:2] = m2_bresp;
	assign x_bvalid[1] = m2_bvalid;
	assign m2_bready = x_bready[1];
	assign m2_araddr = x_araddr[57:32];
	assign m2_arid = x_arid[7:4];
	assign m2_arlen = x_arlen[15:8];
	assign m2_arsize = x_arsize[5:3];
	assign m2_arburst = x_arburst[3:2];
	assign m2_arvalid = x_arvalid[1];
	assign x_arready[1] = m2_arready;
	assign x_rid[7:4] = m2_rid;
	assign x_rdata[255:128] = m2_rdata;
	assign x_rresp[3:2] = m2_rresp;
	assign x_rlast[1] = m2_rlast;
	assign x_rvalid[1] = m2_rvalid;
	assign m2_rready = x_rready[1];

	wire unused_ok = &{1'b0, ctl_aresetn, b_awaddr[63:32], b_araddr[63:32],
		d_awaddr[31:17], d_araddr[31:17], d_awlen, d_awsize, d_awburst, d_awlock,
		d_awcache, d_awprot, d_awregion, d_awqos, d_arsize, d_arburst, d_arlock,
		d_arcache, d_arprot, d_arregion, d_arqos, x_awaddr[63:58], x_araddr[63:58],
		x_awaddr[31:0] & 32'd0, m0_bid, m0_rid, x_awlock[1], x_arlock[1],
		x_awcache[7:4], x_arcache[7:4], x_awprot[5:3], x_arprot[5:3],
		x_awregion[7:4], x_arregion[7:4], x_awqos[7:4], x_arqos[7:4],
		h_awlock, h_arlock, h_awcache, h_arcache, h_awprot, h_arprot, h_awqos, h_arqos};
endmodule

`default_nettype wire
