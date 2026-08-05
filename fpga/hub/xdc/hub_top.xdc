# Hub FPGA (Digilent Cmod A7-35T) pin constraints for hub_top.v.
#
# Only the reset-handshake + LED pins are populated so far -- the rest of
# the hub's I/O (spoke data/clk/strobe, USB FIFO bridge) is a pre-existing
# gap unrelated to this feature (fpga/README.md: "no hub-FPGA gateware yet").
#
# Which net rides which pin (TCXO_CLK on PIO47, FPGA_RESET_N on PIO48, which
# JA pin carries which SPOKEn_ALIVE) is read directly off
# pcb/multi_fpga/hub.kicad_sch's current wiring via
# `kicad-cli sch export netlist`, so those assignments are solid. The
# specific silicon PACKAGE_PIN letter/digit codes below are transcribed from
# a scanned datasheet table (cmod_a7_sch_rev_c0.pdf sheet 3 of 7) and were
# NOT independently cross-checked against a text source the way the cluster
# XDC's pins were (Cmod-S7-25-Master.xdc's own commented-out lines) -- ALL
# PACKAGE_PIN values below must be re-verified against that PDF (or Digilent's
# official Cmod A7 master XDC, not present in this repo) before synthesis.

set_property -dict { PACKAGE_PIN <TODO> IOSTANDARD LVCMOS33 } [get_ports { TCXO_CLK }]; # Y1, 12.288 MHz -- Bank 34, PIO47's pin -- TODO: re-read table
create_clock -period 81.380 -name TCXO_CLK [get_ports { TCXO_CLK }]

set_property -dict { PACKAGE_PIN <TODO> IOSTANDARD LVCMOS33 } [get_ports { FPGA_RESET_N }]; # Bank 34, PIO48's pin -- TODO: re-read table
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_ALIVE }]; # JA1, Bank 14 -- VERIFY
set_property -dict { PACKAGE_PIN G19 IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_ALIVE }]; # JA2, Bank 14 -- VERIFY
set_property -dict { PACKAGE_PIN N18 IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_ALIVE }]; # JA3, Bank 14 -- VERIFY
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_ALIVE }]; # JA4, Bank 14 -- VERIFY

# On-board RGB LED (LD0), Bank 16 -- common-anode, active-low
set_property -dict { PACKAGE_PIN C16 IOSTANDARD LVCMOS33 } [get_ports { led0_g }];
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports { led0_r }];
set_property -dict { PACKAGE_PIN B17 IOSTANDARD LVCMOS33 } [get_ports { led0_b }];
