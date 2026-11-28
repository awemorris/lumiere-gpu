# ZCU106 Vivado non-project build wrapper
# Copyright (C) 2026 Awe Morris; SPDX-License-Identifier: Zlib

ROOT         := $(abspath ../..)
TOP          ?= atg_zcu106_top
CABLE        ?= digilent
FTDI_SERIAL  ?=
LOADER        = openFPGALoader -c $(CABLE) $(if $(FTDI_SERIAL),--ftdi-serial $(FTDI_SERIAL))

.PHONY: all ip prog

all:
	$(ROOT)/mk/vivado-build.sh zcu106 $(TOP)

ip:
	$(ROOT)/mk/vivado-cmd.sh -mode batch -nojournal -log $(ROOT)/build/vivado/zcu106_ip/ip.log \
		-source $(ROOT)/boards/zcu106/ip.tcl -tclargs $(ROOT)/build/vivado/zcu106_ip

prog:
	$(LOADER) $(ROOT)/build/vivado/$(TOP)/$(TOP).bit
