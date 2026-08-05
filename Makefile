# fpga/cluster simulation targets (Vivado xsim). See fpga/README.md.
#
# Requires Vivado; VIVADO_SETTINGS points at its settings64.sh so xvlog/xelab/xsim
# land on PATH without needing the user's shell to already have sourced it.

VIVADO_SETTINGS ?= /opt/Xilinx/Vivado/settings64.sh
# glbl.v is Xilinx's standard simulation-only global-net helper, required
# whenever unisim primitives (IBUF/BUFG/OBUF -- see clk_reset.v) are
# elaborated in xsim. Lives as a sibling of the Vivado install dir.
GLBL_V ?= $(dir $(VIVADO_SETTINGS))../data/verilog/src/glbl.v

SHELL := /bin/bash
.ONESHELL:
.SHELLFLAGS := -e -c

RTL := fpga/cluster/rtl
SIM := fpga/cluster/sim
HUB_RTL := fpga/hub/rtl
HUB_SIM := fpga/hub/sim

# $(1) = space-separated source files (relative to $(SIM)), $(2) = top testbench module name
#
# xsim always exits 0 even after a testbench calls $fatal, so success/failure
# is determined by grepping the run log for the "PASS:"/"FAIL:" banner every
# testbench in this project prints (see sim/tb_*.v) -- not by xsim's own exit code.
define XSIM_RUN
	source $(VIVADO_SETTINGS) >/dev/null
	cd $(SIM)
	xvlog --work work $(1)
	xelab work.$(2) -s $(2)_sim
	xsim $(2)_sim -R | tee $(2).run.log
	if ! grep -q '^PASS:' $(2).run.log; then
		echo "*** $(2) did not report PASS -- treating as failed ***"
		exit 1
	fi
endef

# Same as XSIM_RUN, but also compiles/links glbl.v + the unisims_ver library --
# needed by any testbench whose DUT (transitively) instantiates Xilinx unisim
# primitives (clk_reset.v's IBUF/BUFG/OBUF).
define XSIM_RUN_UNISIM
	source $(VIVADO_SETTINGS) >/dev/null
	cd $(SIM)
	xvlog --work work $(1) $(GLBL_V)
	xelab work.$(2) work.glbl -s $(2)_sim -L unisims_ver
	xsim $(2)_sim -R | tee $(2).run.log
	if ! grep -q '^PASS:' $(2).run.log; then
		echo "*** $(2) did not report PASS -- treating as failed ***"
		exit 1
	fi
endef

# Same as XSIM_RUN, but for fpga/hub's own rtl/sim tree ($(1)'s paths are
# relative to $(HUB_SIM)). reset_seq.v has no unisim primitives, so no glbl.
define XSIM_RUN_HUB
	source $(VIVADO_SETTINGS) >/dev/null
	cd $(HUB_SIM)
	xvlog --work work $(1)
	xelab work.$(2) -s $(2)_sim
	xsim $(2)_sim -R | tee $(2).run.log
	if ! grep -q '^PASS:' $(2).run.log; then
		echo "*** $(2) did not report PASS -- treating as failed ***"
		exit 1
	fi
endef

.PHONY: sim-pdm sim-cic sim-fir sim-framer sim-clk sim-top sim-reset-seq sim-all clean-sim golden-test

sim-pdm:
	$(call XSIM_RUN,../rtl/pdm_line_demux.v tb_pdm_line_demux.v,tb_pdm_line_demux)

sim-cic:
	$(call XSIM_RUN,../rtl/cic_decimator.v tb_cic_decimator.v,tb_cic_decimator)

sim-fir:
	$(call XSIM_RUN,../rtl/fir_compensator.v tb_fir_compensator.v,tb_fir_compensator)

sim-framer:
	$(call XSIM_RUN,../rtl/spoke_framer.v tb_spoke_framer.v,tb_spoke_framer)

sim-clk:
	$(call XSIM_RUN_UNISIM,../rtl/clk_reset.v tb_clk_reset.v,tb_clk_reset)

sim-top:
	$(call XSIM_RUN_UNISIM,../rtl/pdm_line_demux.v ../rtl/cic_decimator.v ../rtl/fir_compensator.v ../rtl/spoke_framer.v ../rtl/clk_reset.v ../rtl/cluster_top.v tb_cluster_top.v,tb_cluster_top)

sim-reset-seq:
	$(call XSIM_RUN_HUB,../rtl/reset_seq.v tb_reset_seq.v,tb_reset_seq)

golden-test:
	python3 -m pytest fpga/cluster/golden -q

sim-all: golden-test sim-pdm sim-cic sim-fir sim-framer sim-clk sim-top sim-reset-seq
	@echo "=================================================="
	@echo "ALL FPGA CLUSTER SIMULATIONS PASSED"
	@echo "=================================================="

clean-sim:
	rm -rf $(SIM)/xsim.dir $(SIM)/work $(SIM)/*.jou $(SIM)/*.log $(SIM)/*.pb $(SIM)/*.wdb $(SIM)/.Xil $(SIM)/*.run.log
	rm -rf $(HUB_SIM)/xsim.dir $(HUB_SIM)/work $(HUB_SIM)/*.jou $(HUB_SIM)/*.log $(HUB_SIM)/*.pb $(HUB_SIM)/*.wdb $(HUB_SIM)/.Xil $(HUB_SIM)/*.run.log
