# Cluster FPGA (Digilent Cmod S7-25) pin constraints for cluster_top.v.
#
# Only the reset-handshake pins are populated so far (FPGA_RESET_N / SPOKE_ALIVE
# -- see fpga/hub/rtl/reset_seq.v for the other end). The remaining ~21 ports
# cluster_top.v already needs (SPOKE_CLK, PDM_D00-11, SPOKE_D0-5, SPOKE_STROBE)
# are a pre-existing gap unrelated to this feature -- derive them from
# pcb/multi_fpga/cluster_00.kicad_sch the same coordinate-lookup way when that
# broader XDC-authoring work happens.
#
# Package pins sourced from Cmod-S7-25-Master.xdc's own commented-out
# pio30/pio31 reference lines (Digilent's stock constraint file).

set_property -dict { PACKAGE_PIN M13 IOSTANDARD LVCMOS33 PULLTYPE PULLUP } [get_ports { FPGA_RESET_N }]; #IO_L8P_T1_D11_14 Sch=pio[30]
set_property -dict { PACKAGE_PIN J11 IOSTANDARD LVCMOS33 } [get_ports { SPOKE_ALIVE }]; #IO_0_14 Sch=pio[31]
