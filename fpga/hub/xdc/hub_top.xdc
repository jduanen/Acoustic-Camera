# Hub FPGA (Digilent Cmod A7-35T) pin constraints for hub_top.v.
#
# All ports hub_top.v currently declares are populated. Remaining gap: spoke
# data/clk/strobe + USB FIFO bridge I/O aren't in hub_top.v yet (fpga/README.md:
# "no hub-FPGA gateware yet" for that part).
#
# Net -> Cmod A7 PIOnn / JAn assignments read directly off
# pcb/multi_fpga/hub.kicad_sch's current wiring via `kicad-cli sch export
# netlist`. PIOnn/JAn -> silicon PACKAGE_PIN taken from
# fpga/demo/CmodA735tDemo/Src/CmodA735tDemo.xdc (Digilent's own master XDC for
# this module, IOGroupA/B = pio[], ja[] = JA Pmod header).
#
# FPGA_RESET_N: A6 (hub FPGA) drives PIO48, schematic net "RESET_N", into J19
# pin 1. J19 is a 3-pin jumper: shorting pins 1-2 (normal operation) ties
# RESET_N to the FPGA_RESET_N net that fans out to all 4 SpokeBus connectors;
# shorting pins 2-3 instead holds the spoke FPGAs in permanent reset for
# debugging. So PIO48/V8 is the correct FPGA-side pin regardless of jumper
# position.
#
# SPOKE0-3_ALIVE: A6 has no PIO pins free (all 48 committed to spokes/USB/
# TCXO/reset) and its schematic symbol has no JA Pmod pins modeled. These
# signals reach the FPGA via an external jumper cable: J18 (PMOD_Signals
# breakout, on the SpokeBus side) <-> J20 (A6_JA, DNP documentation symbol
# for the Cmod A7's on-board Pmod header) -- see hub.kicad_sch. J20 pin
# 1/2/3/4 = physical JA1/JA2/JA3/JA4 in order (confirmed with board owner).

set_property -dict { PACKAGE_PIN U8    IOSTANDARD LVCMOS33 } [get_ports { TCXO_CLK }]; #IO_L14P_T2_SRCC_34 Sch=pio[47], 12.288 MHz HCMOS TCXO (Y1)
create_clock -period 81.380 -name TCXO_CLK [get_ports { TCXO_CLK }]

set_property -dict { PACKAGE_PIN V8    IOSTANDARD LVCMOS33 } [get_ports { FPGA_RESET_N }]; #IO_L14N_T2_SRCC_34 Sch=pio[48], schematic net RESET_N (see J19 jumper note above)

set_property -dict { PACKAGE_PIN G17   IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_ALIVE }]; #IO_L5N_T0_D07_14 Sch=ja[1], via J18<->J20 cable
set_property -dict { PACKAGE_PIN G19   IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_ALIVE }]; #IO_L4N_T0_D05_14 Sch=ja[2], via J18<->J20 cable
set_property -dict { PACKAGE_PIN N18   IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_ALIVE }]; #IO_L9P_T1_DQS_14 Sch=ja[3], via J18<->J20 cable
set_property -dict { PACKAGE_PIN L18   IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_ALIVE }]; #IO_L8P_T1_D11_14 Sch=ja[4], via J18<->J20 cable

# On-board RGB LED (LD0), Bank 16 -- common-anode, active-low
set_property -dict { PACKAGE_PIN C16   IOSTANDARD LVCMOS33 } [get_ports { led0_g }];
set_property -dict { PACKAGE_PIN C17   IOSTANDARD LVCMOS33 } [get_ports { led0_r }];
set_property -dict { PACKAGE_PIN B17   IOSTANDARD LVCMOS33 } [get_ports { led0_b }];
