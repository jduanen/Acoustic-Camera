set_property -dict {PACKAGE_PIN N3 IOSTANDARD LVCMOS33} [get_ports SPOKE_CLK]
set_property -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33} [get_ports SPOKE_ALIVE]
set_property -dict {PACKAGE_PIN M3 IOSTANDARD LVCMOS33} [get_ports PDM_CLK]
set_property -dict {PACKAGE_PIN L3 IOSTANDARD LVCMOS33} [get_ports PDM_D00]
set_property -dict {PACKAGE_PIN A16 IOSTANDARD LVCMOS33} [get_ports PDM_D01]
set_property -dict {PACKAGE_PIN K3 IOSTANDARD LVCMOS33} [get_ports PDM_D02]
set_property -dict {PACKAGE_PIN C15 IOSTANDARD LVCMOS33} [get_ports PDM_D03]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33} [get_ports PDM_D04]
set_property -dict {PACKAGE_PIN A15 IOSTANDARD LVCMOS33} [get_ports PDM_D05]
set_property -dict {PACKAGE_PIN B15 IOSTANDARD LVCMOS33} [get_ports PDM_D06]
set_property -dict {PACKAGE_PIN A14 IOSTANDARD LVCMOS33} [get_ports PDM_D07]
set_property -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports PDM_D08]
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports PDM_D09]
set_property -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports PDM_D10]
set_property -dict {PACKAGE_PIN L1 IOSTANDARD LVCMOS33} [get_ports PDM_D11]
set_property -dict {PACKAGE_PIN M2 IOSTANDARD LVCMOS33} [get_ports SPOKE_D0]
set_property -dict {PACKAGE_PIN N1 IOSTANDARD LVCMOS33} [get_ports SPOKE_D1]
set_property -dict {PACKAGE_PIN N2 IOSTANDARD LVCMOS33} [get_ports SPOKE_D2]
set_property -dict {PACKAGE_PIN P1 IOSTANDARD LVCMOS33} [get_ports SPOKE_D3]
set_property -dict {PACKAGE_PIN R3 IOSTANDARD LVCMOS33} [get_ports SPOKE_D4]
set_property -dict {PACKAGE_PIN T3 IOSTANDARD LVCMOS33} [get_ports SPOKE_D5]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports SPOKE_STROBE]
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS33} [get_ports led0_g]
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports led0_r]
set_property -dict {PACKAGE_PIN B17 IOSTANDARD LVCMOS33} [get_ports led0_b]
# Cluster FPGA (Digilent Cmod A7-35T, part xc7a35tcpg236-1) pin constraints
# for cluster_top.v.
#
# The cluster FPGA moved from the Cmod S7 (XC7S25) to the Cmod A7-35T
# (XC7A35T) -- same DIP-48 socket footprint as the S7 (see
# pcb/multi_fpga/Readme.txt/SCHEMATIC_NOTES.md), swapped because the S7
# couldn't fit this design (needed ~17,600-19,550 LUTs at place_design time
# vs 14,600 available -- opt_design's own report_utilization undersold this,
# see the LUT-crisis conversation this swap came out of). The two chips
# don't share the same clock-capable-pin layout despite sharing a socket, so
# SPOKE_CLK also moved DIP pins (see below) and PDM_D08-11 were reshuffled
# to fill in behind it -- confirmed via `kicad-cli sch export netlist`
# against pcb/multi_fpga/cluster_00.kicad_sch's current wiring, same
# cross-check method as the S7 XDC used. PIOnn -> silicon PACKAGE_PIN taken
# from fpga/demo/CmodA735tDemo/Src/CmodA735tDemo.xdc (Digilent's own master
# XDC for this module, used identically for the hub board's XDC).
#
# All 26 ports are populated.

# Bank 0 (config) voltage: the Cmod A7-35T's configuration bank runs off the
# module's own 3.3V rail, matching every IOSTANDARD LVCMOS33 used below.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

create_clock -period 162.760 -name SPOKE_CLK [get_ports SPOKE_CLK]

set_property PACKAGE_PIN T1 [get_ports FPGA_RESET_N]
set_property IOSTANDARD LVCMOS33 [get_ports FPGA_RESET_N]
set_property PULLTYPE PULLUP [get_ports FPGA_RESET_N]




# On-board RGB LED (LD0), Bank 16 -- common-anode, active-low. Pins taken
# from fpga/demo/CmodA735tDemo/Src/CmodA735tDemo.xdc (Digilent's own master
# XDC), same as fpga/hub/xdc/hub_top.xdc uses for its identical on-board LED
# -- NOT the demo XDC's separate led[0]/led[1] pins (A17/C16), which are a
# different, non-RGB LED.

# False paths (not just waived TIMING-18 warnings -- these are genuinely
# untimed by design, not "checked and found fine" or "ignored for now"):
#
# FPGA_RESET_N: asynchronous reset from the hub FPGA, run through
# clk_reset.v's own 2-FF synchronizer specifically because it's untimed
# relative to SPOKE_CLK -- there is no meaningful setup/hold relationship to
# constrain here.
set_false_path -from [get_ports FPGA_RESET_N]

# PDM_D00-11: pdm_line_sync.v was deliberately designed with a full extra
# clk cycle of capture margin (see that module's header comment) precisely
# so it doesn't depend on tight datasheet/trace-delay-driven timing closure
# -- a false path states that design intent accurately; a fabricated
# set_input_delay number (without real trace-length/mic-datasheet data to
# back it) would just be guessing.
set_false_path -from [get_ports {PDM_D00 PDM_D01 PDM_D02 PDM_D03 PDM_D04 PDM_D05 PDM_D06 PDM_D07 PDM_D08 PDM_D09 PDM_D10 PDM_D11}]

# PDM_CLK, SPOKE_D0-5, SPOKE_STROBE, SPOKE_ALIVE: outputs to the local mics
# / hub board. Nothing on this FPGA depends on their timing being STA-
# verified, and the hub-side logic that would eventually care (spoke
# deframer) doesn't exist yet -- explicitly out of scope so far (see
# fpga/README.md). Revisit with real set_output_delay numbers once the hub
# side is designed and a real cross-board timing budget exists to constrain
# against.
set_false_path -to [get_ports {PDM_CLK SPOKE_D0 SPOKE_D1 SPOKE_D2 SPOKE_D3 SPOKE_D4 SPOKE_D5 SPOKE_STROBE SPOKE_ALIVE}]

# led0_r: driven from the registered rst signal, just the status LED, not
# timing-critical (same reasoning/precedent as hub_top.xdc's led0_r false
# path). led0_g/led0_b aren't listed -- they're pure constants (clk_reset.v's
# led_g_n/led_b_n assigns), no clocked fan-in to flag.
set_false_path -to [get_ports led0_r]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
