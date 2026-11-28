## Amaterrace GPU
## Copyright (C) 2026 Awe Morris
##
## SPDX-License-Identifier: Zlib
##
## atg_zcu106_top on the AMD / Xilinx ZCU106 Evaluation Kit
## (Zynq UltraScale+ XCZU7EV-2FFVC1156-E).
##
## Architectural Clocks:
##  - clk_pl_0 (from atg_zcu106_ps): 100.0 MHz reference clock (10.000 ns)
##  - clk_gpu: 400.0 MHz GPU core logic clock (2.500 ns) via MMCME4_BASE
##  - clk_axi: 200.0 MHz PS-PL AXI interface clock (5.000 ns) via MMCME4_BASE
##  - pclk:     75.0 MHz pixel clock (13.333 ns) via MMCME4_BASE

## Asynchronous Clock Domain Groups
## am_axi_cdc, atg_zcu106_axi_cc, atg_dc_fifo, and am_sync cross all boundaries.
set_clock_groups -asynchronous \
	-group [get_clocks -of_objects [get_pins u_clk/mmcm_gpu/CLKOUT0]] \
	-group [get_clocks -of_objects [get_pins u_clk/mmcm_gpu/CLKOUT1]] \
	-group [get_clocks -of_objects [get_pins u_clk/mmcm_gpu/CLKOUT2]]

## User Diagnostic LEDs (UG1244 Table 3-23, Bank 66/67 1.2V LVCMOS12)
set_property -dict { PACKAGE_PIN AL11 IOSTANDARD LVCMOS12 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN AL13 IOSTANDARD LVCMOS12 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN AK13 IOSTANDARD LVCMOS12 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN AE15 IOSTANDARD LVCMOS12 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN AM8  IOSTANDARD LVCMOS12 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN AM9  IOSTANDARD LVCMOS12 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN AM10 IOSTANDARD LVCMOS12 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN AM11 IOSTANDARD LVCMOS12 } [get_ports {led[7]}]

## I2C (ZCU106 IIC0_MAIN, LVCMOS12)
set_property -dict { PACKAGE_PIN AE19 IOSTANDARD LVCMOS12 DRIVE 8 SLEW SLOW } [get_ports iic_scl]
set_property -dict { PACKAGE_PIN AH23 IOSTANDARD LVCMOS12 DRIVE 8 SLEW SLOW } [get_ports iic_sda]

## Parallel Display Outputs (mapped to ZCU106 FMC_HPC0 LA00..LA09 LVCMOS18)
set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[0]}]
set_property -dict { PACKAGE_PIN F17 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[1]}]
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[2]}]
set_property -dict { PACKAGE_PIN H18 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[3]}]
set_property -dict { PACKAGE_PIN K20 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[4]}]
set_property -dict { PACKAGE_PIN L20 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[5]}]
set_property -dict { PACKAGE_PIN K18 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[6]}]
set_property -dict { PACKAGE_PIN K19 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[7]}]
set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[8]}]
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[9]}]
set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[10]}]
set_property -dict { PACKAGE_PIN K17 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[11]}]
set_property -dict { PACKAGE_PIN G19 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[12]}]
set_property -dict { PACKAGE_PIN H19 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[13]}]
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[14]}]
set_property -dict { PACKAGE_PIN J16 IOSTANDARD LVCMOS18 } [get_ports {hdmi_d[15]}]
set_property -dict { PACKAGE_PIN E17 IOSTANDARD LVCMOS18 } [get_ports hdmi_de]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS18 } [get_ports hdmi_hs]
set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS18 } [get_ports hdmi_vs]
set_property -dict { PACKAGE_PIN H16 IOSTANDARD LVCMOS18 } [get_ports hdmi_clk]

set_false_path -to [get_ports {led[*] iic_scl iic_sda hdmi_d[*] hdmi_de hdmi_hs hdmi_vs hdmi_clk}]
set_false_path -from [get_ports {iic_scl iic_sda}]
