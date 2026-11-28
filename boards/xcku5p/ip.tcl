# HamGeek XCKU5P IP cores, generated from scratch:
#   vivado -mode batch -source boards/xcku5p/ip.tcl -tclargs OUT [NAME...]
# Target: xcku5p-ffvb676-2-i (requires Kintex UltraScale+ device support in Vivado).
# Copyright (C) 2026 Awe Morris; SPDX-License-Identifier: Zlib

if {[llength $argv] < 1} {
	puts "usage: ip.tcl OUT \[NAME...\]"
	exit 2
}
set out [file normalize [lindex $argv 0]]
set only [lrange $argv 1 end]
file mkdir $out
create_project -in_memory -part xcku5p-ffvb676-2-i
set_property target_language Verilog [current_project]

proc wanted {name} {
	global only
	return [expr {[llength $only] == 0 || [lsearch -exact $only $name] >= 0}]
}

proc finish {name} {
	generate_target {instantiation_template synthesis} [get_ips $name]
	synth_ip [get_ips $name]
	puts "ip.tcl: $name done"
}

proc make {name vlnv props} {
	global out
	if {![wanted $name]} {
		return
	}
	create_ip -vlnv $vlnv -module_name $name -dir $out
	set_property -dict $props [get_ips $name]
	finish $name
}

# PCIe Gen3 x4 AXI Bridge (FMC)
make atg_xcku5p_pcie_ip xilinx.com:ip:xdma:4.2 [list \
	CONFIG.functional_mode AXI_Bridge \
	CONFIG.mode_selection Advanced \
	CONFIG.pl_link_cap_max_link_width X4 \
	CONFIG.pl_link_cap_max_link_speed 8.0_GT/s \
	CONFIG.ref_clk_freq 100_MHz \
	CONFIG.axi_addr_width 64 \
	CONFIG.axi_data_width 128_bit \
	CONFIG.en_axi_master_if true \
	CONFIG.en_axi_slave_if true \
	CONFIG.vendor_id 10EE \
	CONFIG.pf0_device_id 9034 \
	CONFIG.pf0_subsystem_vendor_id 10EE \
	CONFIG.pf0_Use_Class_Code_Lookup_Assistant false \
	CONFIG.pf0_class_code_base 03 \
	CONFIG.pf0_class_code_sub 80 \
	CONFIG.pf0_class_code_interface 00 \
	CONFIG.pf0_bar0_enabled true \
	CONFIG.pf0_bar0_type Memory \
	CONFIG.pf0_bar0_scale Kilobytes \
	CONFIG.pf0_bar0_size 128 \
	CONFIG.pciebar2axibar_0 0x0000000000000000 \
	CONFIG.pf0_bar1_enabled false \
	CONFIG.axibar_num 1 \
	CONFIG.axibar_0 0x0000000000000000 \
	CONFIG.axibar_highaddr_0 0x000000007FFFFFFF \
	CONFIG.axibar2pciebar_0 0x0000000000000000 \
	CONFIG.pf0_interrupt_pin NONE \
	CONFIG.pf0_msi_enabled true \
	CONFIG.pf0_msi_cap_multimsgcap 1_vector]

make atg_xcku5p_bar_xbar xilinx.com:ip:axi_crossbar:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.NUM_SI 1 CONFIG.NUM_MI 2 CONFIG.DATA_WIDTH 128 \
	CONFIG.ADDR_WIDTH 32 CONFIG.ID_WIDTH 4 \
	CONFIG.M00_A00_BASE_ADDR 0x0000000000000000 CONFIG.M00_A00_ADDR_WIDTH 17 \
	CONFIG.M01_A00_BASE_ADDR 0x0000000080000000 CONFIG.M01_A00_ADDR_WIDTH 26]

make atg_xcku5p_bar0_dn xilinx.com:ip:axi_dwidth_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.SI_DATA_WIDTH 128 CONFIG.MI_DATA_WIDTH 64 \
	CONFIG.ADDR_WIDTH 32 CONFIG.SI_ID_WIDTH 4]

make atg_xcku5p_bar2_cc xilinx.com:ip:axi_clock_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.DATA_WIDTH 512 CONFIG.ADDR_WIDTH 32 CONFIG.ID_WIDTH 4 \
	CONFIG.ACLK_ASYNC 1]

make atg_xcku5p_host_cc xilinx.com:ip:axi_clock_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.DATA_WIDTH 512 CONFIG.ADDR_WIDTH 64 CONFIG.ID_WIDTH 4 \
	CONFIG.ACLK_ASYNC 1]

make atg_xcku5p_host_dn xilinx.com:ip:axi_dwidth_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.SI_DATA_WIDTH 512 CONFIG.MI_DATA_WIDTH 128 \
	CONFIG.ADDR_WIDTH 64 CONFIG.SI_ID_WIDTH 4]

make atg_xcku5p_bar2_up xilinx.com:ip:axi_dwidth_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.SI_DATA_WIDTH 128 CONFIG.MI_DATA_WIDTH 512 \
	CONFIG.ADDR_WIDTH 32 CONFIG.SI_ID_WIDTH 4]

make atg_xcku5p_mem_xbar xilinx.com:ip:axi_crossbar:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.NUM_SI 2 CONFIG.NUM_MI 1 CONFIG.DATA_WIDTH 512 \
	CONFIG.ADDR_WIDTH 32 CONFIG.ID_WIDTH 1 CONFIG.M00_A00_BASE_ADDR 0x0000000000000000 \
	CONFIG.M00_A00_ADDR_WIDTH 32]

make atg_xcku5p_axi_dn xilinx.com:ip:axi_dwidth_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.SI_DATA_WIDTH 512 CONFIG.MI_DATA_WIDTH 256 \
	CONFIG.ADDR_WIDTH 32 CONFIG.SI_ID_WIDTH 1]

make atg_xcku5p_axi_cc xilinx.com:ip:axi_clock_converter:2.1 [list \
	CONFIG.PROTOCOL AXI4 CONFIG.DATA_WIDTH 512 CONFIG.ADDR_WIDTH 32 CONFIG.ID_WIDTH 1 \
	CONFIG.ACLK_ASYNC 1]

# DDR4 SDRAM: 2x MT40A256M16GE-075E (32-bit, 2666 MT/s, 1 GiB, 256-bit AXI4)
make atg_xcku5p_ddr4 xilinx.com:ip:ddr4:2.2 [list \
	CONFIG.C0.DDR4_TimePeriod 750 \
	CONFIG.C0.DDR4_InputClockPeriod 5000 \
	CONFIG.C0.DDR4_PhyClockRatio 4:1 \
	CONFIG.C0.DDR4_MemoryType Components \
	CONFIG.C0.DDR4_MemoryPart MT40A256M16GE-075E \
	CONFIG.C0.DDR4_MemoryVoltage 1.2V \
	CONFIG.C0.DDR4_DataWidth 32 \
	CONFIG.C0.DDR4_AxiSelection true \
	CONFIG.C0.DDR4_AxiDataWidth 256 \
	CONFIG.C0.DDR4_AxiAddressWidth 30 \
	CONFIG.C0.DDR4_AxiIDWidth 4 \
	CONFIG.System_Clock No_Buffer \
	CONFIG.Reference_Clock No_Buffer]

puts "ip.tcl: all done"
