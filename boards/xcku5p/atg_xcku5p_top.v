/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_xcku5p_top: HamGeek XCKU5P board top configuration (xcku5p-ffvb676-2-i):
 *   - 200 MHz oscillator -> MMCME4_BASE -> 400 MHz gclk (default), 25/75 MHz pclk
 *   - 2x MT40A256M16GE-075E DDR4 (32-bit, 2666 MT/s, 1 GiB) via atg_xcku5p_mig
 *   - FMC PCIe 3.0 x4 (xdma AXI Bridge) via atg_xcku5p_pcie
 *   - Parallel 16-bit YCbCr 4:2:2 HDMI + I2C + JTAG register bridge
 *   - Parameterized via include/atg_config.vh
 */

`timescale 1ns / 1ps
`default_nettype none

`include "atg_config.vh"

`ifndef CFG_SU_NPHYS
`define CFG_SU_NPHYS 4
`endif
`ifndef CFG_SU_NWARPS
`define CFG_SU_NWARPS 4
`endif
`ifndef CFG_SU_GRP_W
`define CFG_SU_GRP_W 3
`endif
`ifndef CFG_SU_NREGS
`define CFG_SU_NREGS 32
`endif
`ifndef CFG_SU_LDS_KB
`define CFG_SU_LDS_KB 1
`endif

module atg_xcku5p_top #(
	parameter NSU = 1,
`ifdef CFG_CP_USE_RV32_CORE
	parameter HAS_CPU = (`CFG_CP_USE_RV32_CORE != 0),
`else
	parameter HAS_CPU = 1,
`endif
`ifdef CFG_HAS_BLT
	parameter HAS_BLT = (`CFG_HAS_BLT != 0),
`else
	parameter HAS_BLT = 0,
`endif
`ifdef CFG_MC_L2_CACHE_KB
	parameter HAS_L2 = (`CFG_MC_L2_CACHE_KB != 0 ? 1 : 0),
`else
	parameter HAS_L2 = 0,
`endif
	parameter CPU_IMEM_INIT = "atg1-cp-kc705.imem.hex",
	parameter CPU_DMEM_INIT = "atg1-cp-kc705.dmem.hex"
) (
	input wire sys_clk_p,
	input wire sys_clk_n,
	input wire cpu_reset,
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
	output wire [0:0] ddr4_odt,
	output wire [7:0] led,
	output wire [15:0] hdmi_d,
	output wire hdmi_de,
	output wire hdmi_hs,
	output wire hdmi_vs,
	output wire hdmi_clk,
	inout wire iic_scl,
	inout wire iic_sda,
	output wire iic_mux_reset_n,
	input wire pcie_refclk_p,
	input wire pcie_refclk_n,
	input wire pcie_perst_n,
	output wire [3:0] pcie_txp,
	output wire [3:0] pcie_txn,
	input wire [3:0] pcie_rxp,
	input wire [3:0] pcie_rxn
);
	localparam [31:0] LOCAL_BASE = 32'h80000000;	/* 1 GiB DDR4 at [30:0] -> [29:0] */
	localparam [31:0] LOCAL_BYTES = 32'h40000000;
	localparam [31:0] HOST_LIMIT = 32'h80000000;
	localparam integer GCLK_DIVIDE = 3;		/* 1200 MHz / 3 = 400 MHz */
	localparam [31:0] GCLK_HZ = 32'd1200000000 / GCLK_DIVIDE;

	wire clk, rst_n, ref_ibuf, ref_clk, ref_rst_n, pclk, prst_n, pclk_sel, mem_ready, irq;
	wire [16:0] s_awaddr, s_araddr;
	wire [3:0] s_awid, s_bid, s_arid, s_rid;
	wire [7:0] s_wstrb, s_arlen;
	wire s_awvalid, s_awready, s_wlast, s_wvalid, s_wready, s_bvalid, s_bready;
	wire s_arvalid, s_arready, s_rlast, s_rvalid, s_rready;
	wire [63:0] s_wdata, s_rdata;
	wire [1:0] s_bresp, s_rresp;

	wire m_awid, m_bid, m_arid, m_rid;
	wire [63:0] m_awaddr, m_araddr;
	wire [7:0] m_awlen, m_arlen;
	wire [2:0] m_awsize, m_arsize, m_awprot, m_arprot;
	wire [1:0] m_awburst, m_arburst, m_bresp, m_rresp;
	wire [3:0] m_awcache, m_arcache;
	wire m_awvalid, m_awready, m_wlast, m_wvalid, m_wready, m_bvalid, m_bready;
	wire m_arvalid, m_arready, m_rlast, m_rvalid, m_rready;
	wire [511:0] m_wdata, m_rdata;
	wire [63:0] m_wstrb;

	wire xr_wr, xr_rd, x_req_ready, x_rsp_valid, x_rsp_write, x_rsp_error;
	wire j_wr, j_rd, j_busy;
	wire [16:0] xr_waddr, xr_raddr, j_waddr, j_raddr;
	wire [31:0] xr_wdata, xr_rdata, j_wdata, j_rdata;
	wire [3:0] xr_wstrb, j_wstrb;
	wire [511:0] x_rsp_rdata;
	wire [7:0] x_rsp_tag;
	wire [29:0] tmds;
	wire scl_t, sda_t;
	wire [23:0] px;

	/* PCIe */
	wire user_clk, user_rst_n, link_up;
	wire [16:0] p0_awaddr, p0_araddr;
	wire [3:0] p0_awid, p0_bid, p0_arid, p0_rid;
	wire [7:0] p0_wstrb, p0_arlen;
	wire p0_awvalid, p0_awready, p0_wlast, p0_wvalid, p0_wready, p0_bvalid, p0_bready;
	wire p0_arvalid, p0_arready, p0_rlast, p0_rvalid, p0_rready;
	wire [63:0] p0_wdata, p0_rdata;
	wire [1:0] p0_bresp, p0_rresp;
	wire [25:0] p2_awaddr, p2_araddr;
	wire [3:0] p2_awid, p2_bid, p2_arid, p2_rid;
	wire [7:0] p2_awlen, p2_arlen;
	wire [2:0] p2_awsize, p2_arsize;
	wire [1:0] p2_awburst, p2_arburst, p2_bresp, p2_rresp;
	wire p2_awvalid, p2_awready, p2_wlast, p2_wvalid, p2_wready, p2_bvalid, p2_bready;
	wire p2_arvalid, p2_arready, p2_rlast, p2_rvalid, p2_rready;
	wire [127:0] p2_wdata, p2_rdata;
	wire [15:0] p2_wstrb;

	wire d_awid, d_arid, d_bid_u, d_rid_u, h_awid, h_arid, h_bid, h_rid;
	wire [63:0] d_awaddr, d_araddr, h_awaddr, h_araddr;
	wire [7:0] d_awlen, d_arlen, h_awlen, h_arlen;
	wire [2:0] d_awsize, d_arsize, d_awprot, d_arprot, h_awsize, h_arsize, h_awprot, h_arprot;
	wire [1:0] d_awburst, d_arburst, d_bresp, d_rresp, h_awburst, h_arburst, h_bresp, h_rresp;
	wire [3:0] d_awcache, d_arcache, h_awcache, h_arcache, h_bid4, h_rid4;
	wire d_awvalid, d_awready, d_wlast, d_wvalid, d_wready, d_bvalid, d_bready;
	wire d_arvalid, d_arready, d_rlast, d_rvalid, d_rready;
	wire h_awvalid, h_awready, h_wlast, h_wvalid, h_wready, h_bvalid, h_bready;
	wire h_arvalid, h_arready, h_rlast, h_rvalid, h_rready;
	wire [511:0] d_wdata, d_rdata, h_wdata, h_rdata;
	wire [63:0] d_wstrb, h_wstrb;

	wire dp_awid, dp_arid;
	wire [63:0] dp_awaddr, dp_araddr;
	wire [7:0] dp_awlen, dp_arlen;
	wire [2:0] dp_awsize, dp_arsize, dp_awprot, dp_arprot;
	wire [1:0] dp_awburst, dp_arburst, dp_bresp, dp_rresp;
	wire [3:0] dp_awcache, dp_arcache;
	wire dp_awvalid, dp_awready, dp_wlast, dp_wvalid, dp_wready, dp_bvalid, dp_bready;
	wire dp_arvalid, dp_arready, dp_rlast, dp_rvalid, dp_rready;
	wire [511:0] dp_wdata, dp_rdata;
	wire [63:0] dp_wstrb;

	wire hp_awid, hp_arid, hp_bid, hp_rid;
	wire [63:0] hp_awaddr, hp_araddr;
	wire [7:0] hp_awlen, hp_arlen;
	wire [2:0] hp_awsize, hp_arsize, hp_awprot, hp_arprot;
	wire [1:0] hp_awburst, hp_arburst, hp_bresp, hp_rresp;
	wire [3:0] hp_awcache, hp_arcache;
	wire hp_awvalid, hp_awready, hp_wlast, hp_wvalid, hp_wready, hp_bvalid, hp_bready;
	wire hp_arvalid, hp_arready, hp_rlast, hp_rvalid, hp_rready;
	wire [511:0] hp_wdata, hp_rdata;
	wire [63:0] hp_wstrb;
	wire msi_req, msi_ack;

	wire [25:0] g2_awaddr, g2_araddr;
	wire [3:0] g2_awid, g2_bid, g2_arid, g2_rid;
	wire [7:0] g2_awlen, g2_arlen;
	wire [2:0] g2_awsize, g2_arsize;
	wire [1:0] g2_awburst, g2_arburst, g2_bresp, g2_rresp;
	wire g2_awvalid, g2_awready, g2_wlast, g2_wvalid, g2_wready, g2_bvalid, g2_bready;
	wire g2_arvalid, g2_arready, g2_rlast, g2_rvalid, g2_rready;
	wire [127:0] g2_wdata, g2_rdata;
	wire [15:0] g2_wstrb;
	wire win_en, win_en_u;

	atg_xcku5p_pcie pcie (
		.pcie_refclk_p(pcie_refclk_p), .pcie_refclk_n(pcie_refclk_n),
		.pcie_perst_n(pcie_perst_n), .pcie_txp(pcie_txp), .pcie_txn(pcie_txn),
		.pcie_rxp(pcie_rxp), .pcie_rxn(pcie_rxn),
		.user_clk(user_clk), .user_rst_n(user_rst_n), .link_up(link_up),
		.m0_awaddr(p0_awaddr), .m0_awid(p0_awid), .m0_awvalid(p0_awvalid),
		.m0_awready(p0_awready), .m0_wdata(p0_wdata), .m0_wstrb(p0_wstrb), .m0_wlast(p0_wlast),
		.m0_wvalid(p0_wvalid), .m0_wready(p0_wready), .m0_bid(p0_bid), .m0_bresp(p0_bresp),
		.m0_bvalid(p0_bvalid), .m0_bready(p0_bready), .m0_araddr(p0_araddr),
		.m0_arid(p0_arid), .m0_arlen(p0_arlen), .m0_arvalid(p0_arvalid),
		.m0_arready(p0_arready), .m0_rdata(p0_rdata), .m0_rid(p0_rid), .m0_rresp(p0_rresp),
		.m0_rlast(p0_rlast), .m0_rvalid(p0_rvalid), .m0_rready(p0_rready),
		.m2_awaddr(p2_awaddr), .m2_awid(p2_awid), .m2_awlen(p2_awlen), .m2_awsize(p2_awsize),
		.m2_awburst(p2_awburst), .m2_awvalid(p2_awvalid), .m2_awready(p2_awready),
		.m2_wdata(p2_wdata), .m2_wstrb(p2_wstrb), .m2_wlast(p2_wlast), .m2_wvalid(p2_wvalid),
		.m2_wready(p2_wready), .m2_bid(p2_bid), .m2_bresp(p2_bresp), .m2_bvalid(p2_bvalid),
		.m2_bready(p2_bready), .m2_araddr(p2_araddr), .m2_arid(p2_arid), .m2_arlen(p2_arlen),
		.m2_arsize(p2_arsize), .m2_arburst(p2_arburst), .m2_arvalid(p2_arvalid),
		.m2_arready(p2_arready), .m2_rdata(p2_rdata), .m2_rid(p2_rid), .m2_rresp(p2_rresp),
		.m2_rlast(p2_rlast), .m2_rvalid(p2_rvalid), .m2_rready(p2_rready),
		.s_clk(clk), .s_rst_n(rst_n),
		.s_awaddr(hp_awaddr), .s_awid({3'd0, hp_awid}), .s_awlen(hp_awlen), .s_awsize(hp_awsize),
		.s_awburst(hp_awburst), .s_awvalid(hp_awvalid), .s_awready(hp_awready),
		.s_wdata(hp_wdata), .s_wstrb(hp_wstrb), .s_wlast(hp_wlast), .s_wvalid(hp_wvalid),
		.s_wready(hp_wready), .s_bid(h_bid4), .s_bresp(hp_bresp), .s_bvalid(hp_bvalid),
		.s_bready(hp_bready), .s_araddr(hp_araddr), .s_arid({3'd0, hp_arid}), .s_arlen(hp_arlen),
		.s_arsize(hp_arsize), .s_arburst(hp_arburst), .s_arvalid(hp_arvalid),
		.s_arready(hp_arready), .s_rdata(hp_rdata), .s_rid(h_rid4), .s_rresp(hp_rresp),
		.s_rlast(hp_rlast), .s_rvalid(hp_rvalid), .s_rready(hp_rready),
		.msi_req(msi_req), .msi_ack(msi_ack)
	);

	am_axi_cdc #(.A_W(17), .ID_W(4), .LEN_W(8), .D_W(64), .DEPTH_LOG2(2)) bar0 (
		.s_clk(user_clk), .s_rst_n(user_rst_n),
		.s_awaddr(p0_awaddr), .s_awid(p0_awid), .s_awvalid(p0_awvalid), .s_awready(p0_awready),
		.s_wdata(p0_wdata), .s_wstrb(p0_wstrb), .s_wlast(p0_wlast), .s_wvalid(p0_wvalid),
		.s_wready(p0_wready), .s_bid(p0_bid), .s_bresp(p0_bresp), .s_bvalid(p0_bvalid),
		.s_bready(p0_bready), .s_araddr(p0_araddr), .s_arid(p0_arid), .s_arlen(p0_arlen),
		.s_arvalid(p0_arvalid), .s_arready(p0_arready), .s_rdata(p0_rdata), .s_rid(p0_rid),
		.s_rresp(p0_rresp), .s_rlast(p0_rlast), .s_rvalid(p0_rvalid), .s_rready(p0_rready),
		.m_clk(clk), .m_rst_n(rst_n),
		.m_awaddr(s_awaddr), .m_awid(s_awid), .m_awvalid(s_awvalid), .m_awready(s_awready),
		.m_wdata(s_wdata), .m_wstrb(s_wstrb), .m_wlast(s_wlast), .m_wvalid(s_wvalid),
		.m_wready(s_wready), .m_bid(s_bid), .m_bresp(s_bresp), .m_bvalid(s_bvalid),
		.m_bready(s_bready), .m_araddr(s_araddr), .m_arid(s_arid), .m_arlen(s_arlen),
		.m_arvalid(s_arvalid), .m_arready(s_arready), .m_rdata(s_rdata), .m_rid(s_rid),
		.m_rresp(s_rresp), .m_rlast(s_rlast), .m_rvalid(s_rvalid), .m_rready(s_rready)
	);

	atg_pcie_msi msi (
		.clk(clk), .rst_n(rst_n), .irq(irq), .aw_take(s_awvalid && s_awready),
		.aw_addr(s_awaddr), .b_take(s_bvalid && s_bready), .uclk(user_clk),
		.urst_n(user_rst_n), .msi_req(msi_req), .msi_ack(msi_ack)
	);

	atg_xcku5p_clk #(.GCLK_DIVIDE(GCLK_DIVIDE * 1.0)) clocks (
		.sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .cpu_reset(cpu_reset), .pclk_sel(pclk_sel),
		.gclk(clk), .grst_n(rst_n), .ref_ibuf(ref_ibuf), .ref_clk(ref_clk), .ref_rst_n(ref_rst_n),
		.pclk(pclk), .prst_n(prst_n)
	);

	atg_xcku5p_jtag jtag (
		.clk(clk), .rst_n(rst_n), .j_wr(j_wr), .j_waddr(j_waddr), .j_wdata(j_wdata),
		.j_wstrb(j_wstrb), .j_rd(j_rd), .j_raddr(j_raddr), .j_rdata(j_rdata), .j_busy(j_busy)
	);

	atg_top #(
		.GCLK_HZ(GCLK_HZ),
		.S_ID_W(4),
		.S_LEN_W(8),
		.LINE_BYTES(64),
		.LINE_SHIFT(6),
		.DATA_W(512),
		.LOCAL_BASE(LOCAL_BASE),
		.LOCAL_BYTES(LOCAL_BYTES),
		.WINDOW_BYTES(32'h04000000),
		.HOST_LIMIT(HOST_LIMIT),
		.SUBSYSTEM_VENDOR(16'h10EE),
		.SUBSYSTEM_DEVICE(16'h0007),
		.HAS_CPU(HAS_CPU),
		.CPU_IMEM_INIT(CPU_IMEM_INIT),
		.CPU_DMEM_INIT(CPU_DMEM_INIT),
		.CPU_IMEM_WORDS(32768),
		.CPU_DMEM_WORDS(16384),
		.NSU(`ifdef CFG_NSU `CFG_NSU `else NSU `endif),
		.USE_NOC_FABRIC(`ifdef CFG_USE_NOC_FABRIC (`CFG_USE_NOC_FABRIC != 0) `else 0 `endif),
		.HAS_VF(`ifdef CFG_HAS_VF (`CFG_HAS_VF != 0) `else 1 `endif),
		.HAS_RX(`ifdef CFG_HAS_RX (`CFG_HAS_RX != 0) `else 1 `endif),
		.HAS_TU(`ifdef CFG_HAS_TU (`CFG_HAS_TU != 0) `else 1 `endif),
		.HAS_ROP(`ifdef CFG_HAS_ROP (`CFG_HAS_ROP != 0) `else 1 `endif),
		.HAS_BLT(HAS_BLT),
		.HAS_JTAG(1),
		.HAS_DISP(`ifdef CFG_HAS_DISP (`CFG_HAS_DISP != 0) `else 1 `endif),
		.DC_HAS_I2C(1),
`ifdef CFG_DC_ENABLE_CURSOR
		.DC_HAS_CURSOR(`CFG_DC_ENABLE_CURSOR != 0),
`else
		.DC_HAS_CURSOR(1),
`endif
		.DC_FIFO_LOG2(11),
		.DC_PX_FMT(1),
		.ROP_STENCIL(1),
		.SU_NPHYS(`CFG_SU_NPHYS),
		.SU_NWARPS(`CFG_SU_NWARPS),
		.SU_WARP_W($clog2(`CFG_SU_NWARPS)),
		.SU_GRP_W(`CFG_SU_GRP_W),
		.SU_NREGS(`CFG_SU_NREGS),
		.SU_SB_ENTRIES(4),
		.SU_SBE_W(2),
`ifdef CFG_ENABLE_FP16
		.SU_PACKED(1),
`else
		.SU_PACKED(0),
`endif
`ifdef CFG_ENABLE_SUBGROUP
		.SU_SUBGROUP(1),
`else
		.SU_SUBGROUP(0),
`endif
		.SU_FRAG_RESERVE(1),
		.SU_LDS_BYTES(`CFG_SU_LDS_KB * 1024),
		.SU_VARY_WORDS(16),
`ifdef CFG_MC_ENABLE_PROT
		.HAS_PROT(`CFG_MC_ENABLE_PROT),
`else
		.HAS_PROT(0),
`endif
		.PROT_SLOTS(31),
		.PROT_GRANTS(64),
		.PROT_GIDX_W(6),
		.PROT_PLB(16),
		.PROT_PLB_W(4),
		.HAS_L2(HAS_L2),
`ifdef CFG_MC_L2_WAYS
		.L2_WAYS(`CFG_MC_L2_WAYS),
`else
		.L2_WAYS(2),
`endif
`ifdef CFG_MC_L2_CACHE_KB
		.L2_SET_W(`CFG_MC_L2_CACHE_KB != 0 ? $clog2(((`CFG_MC_L2_CACHE_KB != 0 ? `CFG_MC_L2_CACHE_KB : 16) * 1024) / (64 * `ifdef CFG_MC_L2_WAYS `CFG_MC_L2_WAYS `else 2 `endif)) : 8)
`else
		.L2_SET_W(8)
`endif
	) gpu (
		.clk(clk), .rst_n(rst_n), .irq(irq), .mem_ready(mem_ready), .win_en(win_en),
		.s_awaddr(s_awaddr), .s_awid(s_awid), .s_awvalid(s_awvalid), .s_awready(s_awready), .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast), .s_wvalid(s_wvalid), .s_wready(s_wready), .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid),
		.s_bready(s_bready), .s_araddr(s_araddr), .s_arid(s_arid), .s_arlen(s_arlen), .s_arvalid(s_arvalid), .s_arready(s_arready), .s_rdata(s_rdata), .s_rid(s_rid), .s_rresp(s_rresp), .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),
		.m_awid(m_awid), .m_bid(m_bid), .m_arid(m_arid), .m_rid(m_rid),
		.m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(m_awsize), .m_awburst(m_awburst), .m_awcache(m_awcache), .m_awprot(m_awprot), .m_awvalid(m_awvalid), .m_awready(m_awready), .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready), .m_bresp(m_bresp), .m_bvalid(m_bvalid),
		.m_bready(m_bready), .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arsize(m_arsize), .m_arburst(m_arburst), .m_arcache(m_arcache), .m_arprot(m_arprot), .m_arvalid(m_arvalid), .m_arready(m_arready), .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready),
		.xr_wr(xr_wr), .xr_waddr(xr_waddr), .xr_wdata(xr_wdata), .xr_wstrb(xr_wstrb), .xr_rd(xr_rd),
		.xr_raddr(xr_raddr), .xr_rdata(xr_rdata),
		.x_req_valid(1'b0), .x_req_ready(x_req_ready), .x_req_write(1'b0),
		.x_req_line(26'd0), .x_req_be(64'd0), .x_req_wdata(512'd0), .x_req_tag(8'd0),
		.x_rsp_valid(x_rsp_valid), .x_rsp_write(x_rsp_write), .x_rsp_tag(x_rsp_tag),
		.x_rsp_rdata(x_rsp_rdata), .x_rsp_error(x_rsp_error), .x_vsync(1'b0),
		.j_wr(j_wr), .j_waddr(j_waddr), .j_wdata(j_wdata), .j_wstrb(j_wstrb), .j_rd(j_rd), .j_raddr(j_raddr),
		.j_rdata(j_rdata), .j_busy(j_busy),
		.pclk(pclk), .prst_n(prst_n), .pclk_sel(pclk_sel),
		.tmds(tmds), .px_rgb(px), .px_de(hdmi_de), .px_hs(hdmi_hs), .px_vs(hdmi_vs),
		.scl_t(scl_t), .scl_i(iic_scl), .sda_t(sda_t), .sda_i(iic_sda)
	);

	am_axi_split #(
		.A_W(64), .D_W(512), .ID_W(1), .BASE0({32'd0, LOCAL_BASE}), .SIZE0({32'd0, LOCAL_BYTES}),
		.BASE1(64'h00000000_00000000), .SIZE1({32'd0, HOST_LIMIT}), .CNT_W(3), .WQ_LOG2(2)
	) split (
		.clk(clk), .rst_n(rst_n),
		.s_awid(m_awid), .s_awaddr(m_awaddr), .s_awlen(m_awlen), .s_awsize(m_awsize),
		.s_awburst(m_awburst), .s_awcache(m_awcache), .s_awprot(m_awprot),
		.s_awvalid(m_awvalid), .s_awready(m_awready), .s_wdata(m_wdata), .s_wstrb(m_wstrb),
		.s_wlast(m_wlast), .s_wvalid(m_wvalid), .s_wready(m_wready), .s_bid(m_bid),
		.s_bresp(m_bresp), .s_bvalid(m_bvalid), .s_bready(m_bready), .s_arid(m_arid),
		.s_araddr(m_araddr), .s_arlen(m_arlen), .s_arsize(m_arsize), .s_arburst(m_arburst),
		.s_arcache(m_arcache), .s_arprot(m_arprot), .s_arvalid(m_arvalid),
		.s_arready(m_arready), .s_rid(m_rid), .s_rdata(m_rdata), .s_rresp(m_rresp),
		.s_rlast(m_rlast), .s_rvalid(m_rvalid), .s_rready(m_rready),
		.m0_awid(d_awid), .m0_awaddr(d_awaddr), .m0_awlen(d_awlen), .m0_awsize(d_awsize),
		.m0_awburst(d_awburst), .m0_awcache(d_awcache), .m0_awprot(d_awprot),
		.m0_awvalid(d_awvalid), .m0_awready(d_awready), .m0_wdata(d_wdata),
		.m0_wstrb(d_wstrb), .m0_wlast(d_wlast), .m0_wvalid(d_wvalid), .m0_wready(d_wready),
		.m0_bid(d_bid_u), .m0_bresp(d_bresp), .m0_bvalid(d_bvalid), .m0_bready(d_bready),
		.m0_arid(d_arid), .m0_araddr(d_araddr), .m0_arlen(d_arlen), .m0_arsize(d_arsize),
		.m0_arburst(d_arburst), .m0_arcache(d_arcache), .m0_arprot(d_arprot),
		.m0_arvalid(d_arvalid), .m0_arready(d_arready), .m0_rid(d_rid_u), .m0_rdata(d_rdata),
		.m0_rresp(d_rresp), .m0_rlast(d_rlast), .m0_rvalid(d_rvalid), .m0_rready(d_rready),
		.m1_awid(h_awid), .m1_awaddr(h_awaddr), .m1_awlen(h_awlen), .m1_awsize(h_awsize),
		.m1_awburst(h_awburst), .m1_awcache(h_awcache), .m1_awprot(h_awprot),
		.m1_awvalid(h_awvalid), .m1_awready(h_awready), .m1_wdata(h_wdata),
		.m1_wstrb(h_wstrb), .m1_wlast(h_wlast), .m1_wvalid(h_wvalid), .m1_wready(h_wready),
		.m1_bid(h_bid), .m1_bresp(h_bresp), .m1_bvalid(h_bvalid), .m1_bready(h_bready),
		.m1_arid(h_arid), .m1_araddr(h_araddr), .m1_arlen(h_arlen), .m1_arsize(h_arsize),
		.m1_arburst(h_arburst), .m1_arcache(h_arcache), .m1_arprot(h_arprot),
		.m1_arvalid(h_arvalid), .m1_arready(h_arready), .m1_rid(h_rid), .m1_rdata(h_rdata),
		.m1_rresp(h_rresp), .m1_rlast(h_rlast), .m1_rvalid(h_rvalid), .m1_rready(h_rready)
	);

	am_axi_pipe #(
		.A_W(64), .D_W(512), .ID_W(1),
		.PIPE_AW(1), .PIPE_W(1), .PIPE_B(1), .PIPE_AR(1), .PIPE_R(1)
	) host_pipe (
		.clk(clk), .rst_n(rst_n),
		.s_awid(h_awid), .s_awaddr(h_awaddr), .s_awlen(h_awlen), .s_awsize(h_awsize),
		.s_awburst(h_awburst), .s_awcache(h_awcache), .s_awprot(h_awprot),
		.s_awvalid(h_awvalid), .s_awready(h_awready),
		.s_wdata(h_wdata), .s_wstrb(h_wstrb), .s_wlast(h_wlast), .s_wvalid(h_wvalid), .s_wready(h_wready),
		.s_bid(h_bid), .s_bresp(h_bresp), .s_bvalid(h_bvalid), .s_bready(h_bready),
		.s_arid(h_arid), .s_araddr(h_araddr), .s_arlen(h_arlen), .s_arsize(h_arsize),
		.s_arburst(h_arburst), .s_arcache(h_arcache), .s_arprot(h_arprot),
		.s_arvalid(h_arvalid), .s_arready(h_arready),
		.s_rid(h_rid), .s_rdata(h_rdata), .s_rresp(h_rresp), .s_rlast(h_rlast),
		.s_rvalid(h_rvalid), .s_rready(h_rready),
		.m_awid(hp_awid), .m_awaddr(hp_awaddr), .m_awlen(hp_awlen), .m_awsize(hp_awsize),
		.m_awburst(hp_awburst), .m_awcache(hp_awcache), .m_awprot(hp_awprot),
		.m_awvalid(hp_awvalid), .m_awready(hp_awready),
		.m_wdata(hp_wdata), .m_wstrb(hp_wstrb), .m_wlast(hp_wlast), .m_wvalid(hp_wvalid), .m_wready(hp_wready),
		.m_bid(hp_bid), .m_bresp(hp_bresp), .m_bvalid(hp_bvalid), .m_bready(hp_bready),
		.m_arid(hp_arid), .m_araddr(hp_araddr), .m_arlen(hp_arlen), .m_arsize(hp_arsize),
		.m_arburst(hp_arburst), .m_arcache(hp_arcache), .m_arprot(hp_arprot),
		.m_arvalid(hp_arvalid), .m_arready(hp_arready),
		.m_rid(hp_rid), .m_rdata(hp_rdata), .m_rresp(hp_rresp), .m_rlast(hp_rlast),
		.m_rvalid(hp_rvalid), .m_rready(hp_rready)
	);

	assign hp_bid = h_bid4[0];
	assign hp_rid = h_rid4[0];

	am_sync #(.W(1), .RESET_VALUE(1'b1)) win_sync (
		.clk(user_clk), .rst_n(user_rst_n), .d(win_en), .q(win_en_u)
	);

	am_axi_gate #(.A_W(26), .D_W(128), .ID_W(4), .CNT_W(4)) win_gate (
		.clk(user_clk), .rst_n(user_rst_n), .en(win_en_u),
		.s_awid(p2_awid), .s_awaddr(p2_awaddr), .s_awlen(p2_awlen), .s_awsize(p2_awsize),
		.s_awburst(p2_awburst), .s_awvalid(p2_awvalid), .s_awready(p2_awready),
		.s_wdata(p2_wdata), .s_wstrb(p2_wstrb), .s_wlast(p2_wlast), .s_wvalid(p2_wvalid),
		.s_wready(p2_wready), .s_bid(p2_bid), .s_bresp(p2_bresp), .s_bvalid(p2_bvalid),
		.s_bready(p2_bready), .s_arid(p2_arid), .s_araddr(p2_araddr), .s_arlen(p2_arlen),
		.s_arsize(p2_arsize), .s_arburst(p2_arburst), .s_arvalid(p2_arvalid),
		.s_arready(p2_arready), .s_rid(p2_rid), .s_rdata(p2_rdata), .s_rresp(p2_rresp),
		.s_rlast(p2_rlast), .s_rvalid(p2_rvalid), .s_rready(p2_rready),
		.m_awid(g2_awid), .m_awaddr(g2_awaddr), .m_awlen(g2_awlen), .m_awsize(g2_awsize),
		.m_awburst(g2_awburst), .m_awvalid(g2_awvalid), .m_awready(g2_awready),
		.m_wdata(g2_wdata), .m_wstrb(g2_wstrb), .m_wlast(g2_wlast), .m_wvalid(g2_wvalid),
		.m_wready(g2_wready), .m_bid(g2_bid), .m_bresp(g2_bresp), .m_bvalid(g2_bvalid),
		.m_bready(g2_bready), .m_arid(g2_arid), .m_araddr(g2_araddr), .m_arlen(g2_arlen),
		.m_arsize(g2_arsize), .m_arburst(g2_arburst), .m_arvalid(g2_arvalid),
		.m_arready(g2_arready), .m_rid(g2_rid), .m_rdata(g2_rdata), .m_rresp(g2_rresp),
		.m_rlast(g2_rlast), .m_rvalid(g2_rvalid), .m_rready(g2_rready)
	);

	am_axi_pipe #(
		.A_W(64), .D_W(512), .ID_W(1),
		.PIPE_AW(1), .PIPE_W(1), .PIPE_B(1), .PIPE_AR(1), .PIPE_R(1)
	) dram_pipe (
		.clk(clk), .rst_n(rst_n),
		.s_awid(d_awid), .s_awaddr(d_awaddr), .s_awlen(d_awlen), .s_awsize(d_awsize),
		.s_awburst(d_awburst), .s_awcache(d_awcache), .s_awprot(d_awprot),
		.s_awvalid(d_awvalid), .s_awready(d_awready),
		.s_wdata(d_wdata), .s_wstrb(d_wstrb), .s_wlast(d_wlast), .s_wvalid(d_wvalid), .s_wready(d_wready),
		.s_bid(d_bid_u), .s_bresp(d_bresp), .s_bvalid(d_bvalid), .s_bready(d_bready),
		.s_arid(d_arid), .s_araddr(d_araddr), .s_arlen(d_arlen), .s_arsize(d_arsize),
		.s_arburst(d_arburst), .s_arcache(d_arcache), .s_arprot(d_arprot),
		.s_arvalid(d_arvalid), .s_arready(d_arready),
		.s_rid(d_rid_u), .s_rdata(d_rdata), .s_rresp(d_rresp), .s_rlast(d_rlast),
		.s_rvalid(d_rvalid), .s_rready(d_rready),
		.m_awid(dp_awid), .m_awaddr(dp_awaddr), .m_awlen(dp_awlen), .m_awsize(dp_awsize),
		.m_awburst(dp_awburst), .m_awcache(dp_awcache), .m_awprot(dp_awprot),
		.m_awvalid(dp_awvalid), .m_awready(dp_awready),
		.m_wdata(dp_wdata), .m_wstrb(dp_wstrb), .m_wlast(dp_wlast), .m_wvalid(dp_wvalid), .m_wready(dp_wready),
		.m_bid(1'b0), .m_bresp(dp_bresp), .m_bvalid(dp_bvalid), .m_bready(dp_bready),
		.m_arid(dp_arid), .m_araddr(dp_araddr), .m_arlen(dp_arlen), .m_arsize(dp_arsize),
		.m_arburst(dp_arburst), .m_arcache(dp_arcache), .m_arprot(dp_arprot),
		.m_arvalid(dp_arvalid), .m_arready(dp_arready),
		.m_rid(1'b0), .m_rdata(dp_rdata), .m_rresp(dp_rresp), .m_rlast(dp_rlast),
		.m_rvalid(dp_rvalid), .m_rready(dp_rready)
	);

	atg_xcku5p_mig mig (
		.clk(clk), .rst_n(rst_n), .ref_ibuf(ref_ibuf), .ref_clk(ref_clk), .sys_rst_n(ref_rst_n), .mem_ready(mem_ready),
		.xr_wr(xr_wr), .xr_waddr(xr_waddr), .xr_wdata(xr_wdata), .xr_rd(xr_rd),
		.xr_raddr(xr_raddr), .xr_rdata(xr_rdata),
		.s_awaddr(dp_awaddr[31:0]), .s_awlen(dp_awlen), .s_awsize(dp_awsize), .s_awburst(dp_awburst), .s_awcache(dp_awcache), .s_awprot(dp_awprot), .s_awvalid(dp_awvalid), .s_awready(dp_awready), .s_wdata(dp_wdata), .s_wstrb(dp_wstrb), .s_wlast(dp_wlast), .s_wvalid(dp_wvalid), .s_wready(dp_wready), .s_bresp(dp_bresp), .s_bvalid(dp_bvalid),
		.s_bready(dp_bready), .s_araddr(dp_araddr[31:0]), .s_arlen(dp_arlen), .s_arsize(dp_arsize), .s_arburst(dp_arburst), .s_arcache(dp_arcache), .s_arprot(dp_arprot), .s_arvalid(dp_arvalid), .s_arready(dp_arready), .s_rdata(dp_rdata), .s_rresp(dp_rresp), .s_rlast(dp_rlast), .s_rvalid(dp_rvalid), .s_rready(dp_rready),
		.s2_clk(user_clk), .s2_rst_n(user_rst_n),
		.s2_awaddr(g2_awaddr), .s2_awid(g2_awid), .s2_awlen(g2_awlen), .s2_awsize(g2_awsize),
		.s2_awburst(g2_awburst), .s2_awvalid(g2_awvalid), .s2_awready(g2_awready),
		.s2_wdata(g2_wdata), .s2_wstrb(g2_wstrb), .s2_wlast(g2_wlast), .s2_wvalid(g2_wvalid),
		.s2_wready(g2_wready), .s2_bid(g2_bid), .s2_bresp(g2_bresp), .s2_bvalid(g2_bvalid),
		.s2_bready(g2_bready), .s2_araddr(g2_araddr), .s2_arid(g2_arid), .s2_arlen(g2_arlen),
		.s2_arsize(g2_arsize), .s2_arburst(g2_arburst), .s2_arvalid(g2_arvalid),
		.s2_arready(g2_arready), .s2_rdata(g2_rdata), .s2_rid(g2_rid), .s2_rresp(g2_rresp),
		.s2_rlast(g2_rlast), .s2_rvalid(g2_rvalid), .s2_rready(g2_rready),
		.ddr4_dq(ddr4_dq), .ddr4_dqs_p(ddr4_dqs_p), .ddr4_dqs_n(ddr4_dqs_n),
		.ddr4_dm_n(ddr4_dm_n), .ddr4_addr(ddr4_addr), .ddr4_ba(ddr4_ba), .ddr4_bg(ddr4_bg),
		.ddr4_act_n(ddr4_act_n), .ddr4_reset_n(ddr4_reset_n), .ddr4_ck_p(ddr4_ck_p),
		.ddr4_ck_n(ddr4_ck_n), .ddr4_cke(ddr4_cke), .ddr4_cs_n(ddr4_cs_n), .ddr4_odt(ddr4_odt)
	);

	assign iic_scl = scl_t ? 1'bz : 1'b0;
	assign iic_sda = sda_t ? 1'bz : 1'b0;
	assign iic_mux_reset_n = 1'b1;
	assign hdmi_d = px[15:0];

	ODDRE1 #(
		.IS_C_INVERTED(1'b0),
		.IS_D1_INVERTED(1'b0),
		.IS_D2_INVERTED(1'b0),
		.SRVAL(1'b0)
	) hdmi_clk_out (
		.Q(hdmi_clk), .C(pclk), .D1(1'b0), .D2(1'b1), .SR(1'b0)
	);

	reg [26:0] beat = 27'd0;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			beat <= 27'd0;
		else
			beat <= beat + 27'd1;
	end

	assign led = {3'd0, link_up, mem_ready, 1'b0, irq, beat[26]};

	wire unused_ok = &{1'b0, px[23:16], tmds, dp_awid, dp_arid, dp_awaddr[63:32], dp_araddr[63:32],
		h_awcache, h_arcache, h_awprot, h_arprot, h_bid4[3:1], h_rid4[3:1], xr_wstrb,
		x_req_ready, x_rsp_valid, x_rsp_write, x_rsp_error, x_rsp_rdata, x_rsp_tag};
endmodule

`default_nettype wire
