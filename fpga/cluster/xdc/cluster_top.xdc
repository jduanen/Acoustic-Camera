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
# All 23 ports are populated.

set_property -dict { PACKAGE_PIN N3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_CLK }]; #IO_L12P_T1_MRCC_35 Sch=pio[18], schematic net SPOKE0_CLK -- clock-capable, P-side (MRCC)
create_clock -period 325.521 -name SPOKE_CLK [get_ports { SPOKE_CLK }]

set_property -dict { PACKAGE_PIN T1    IOSTANDARD LVCMOS33 PULLTYPE PULLUP } [get_ports { FPGA_RESET_N }]; #IO_L3P_T0_DQS_34 Sch=pio[29]
set_property -dict { PACKAGE_PIN T2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_ALIVE }]; #IO_L1N_T0_34 Sch=pio[30]

set_property -dict { PACKAGE_PIN M3    IOSTANDARD LVCMOS33 } [get_ports { PDM_CLK }]; #IO_L8N_T1_AD14N_35 Sch=pio[01], schematic net C0_PDM_CLK

set_property -dict { PACKAGE_PIN L3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D00 }]; #IO_L8P_T1_AD14P_35 Sch=pio[02], schematic net DATA_00
set_property -dict { PACKAGE_PIN A16   IOSTANDARD LVCMOS33 } [get_ports { PDM_D01 }]; #IO_L12P_T1_MRCC_16 Sch=pio[03], schematic net DATA_01
set_property -dict { PACKAGE_PIN K3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D02 }]; #IO_L7N_T1_AD6N_35 Sch=pio[04], schematic net DATA_02
set_property -dict { PACKAGE_PIN C15   IOSTANDARD LVCMOS33 } [get_ports { PDM_D03 }]; #IO_L11P_T1_SRCC_16 Sch=pio[05], schematic net DATA_03
set_property -dict { PACKAGE_PIN H1    IOSTANDARD LVCMOS33 } [get_ports { PDM_D04 }]; #IO_L3P_T0_DQS_AD5P_35 Sch=pio[06], schematic net DATA_04
set_property -dict { PACKAGE_PIN A15   IOSTANDARD LVCMOS33 } [get_ports { PDM_D05 }]; #IO_L6N_T0_VREF_16 Sch=pio[07], schematic net DATA_05
set_property -dict { PACKAGE_PIN B15   IOSTANDARD LVCMOS33 } [get_ports { PDM_D06 }]; #IO_L11N_T1_SRCC_16 Sch=pio[08], schematic net DATA_06
set_property -dict { PACKAGE_PIN A14   IOSTANDARD LVCMOS33 } [get_ports { PDM_D07 }]; #IO_L6P_T0_16 Sch=pio[09], schematic net DATA_07
set_property -dict { PACKAGE_PIN J3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D08 }]; #IO_L7P_T1_AD6P_35 Sch=pio[10], schematic net DATA_08
set_property -dict { PACKAGE_PIN J1    IOSTANDARD LVCMOS33 } [get_ports { PDM_D09 }]; #IO_L3N_T0_DQS_AD5N_35 Sch=pio[11], schematic net DATA_09
set_property -dict { PACKAGE_PIN K2    IOSTANDARD LVCMOS33 } [get_ports { PDM_D10 }]; #IO_L5P_T0_AD13P_35 Sch=pio[12], schematic net DATA_10
set_property -dict { PACKAGE_PIN L1    IOSTANDARD LVCMOS33 } [get_ports { PDM_D11 }]; #IO_L6N_T0_VREF_35 Sch=pio[13], schematic net DATA_11

set_property -dict { PACKAGE_PIN M2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D0 }]; #IO_L9P_T1_DQS_AD7P_35 Sch=pio[20], schematic net SPOKE0_D0
set_property -dict { PACKAGE_PIN N1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D1 }]; #IO_L10N_T1_AD15N_35 Sch=pio[21], schematic net SPOKE0_D1
set_property -dict { PACKAGE_PIN N2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D2 }]; #IO_L10P_T1_AD15P_35 Sch=pio[22], schematic net SPOKE0_D2
set_property -dict { PACKAGE_PIN P1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D3 }]; #IO_L19N_T3_VREF_35 Sch=pio[23], schematic net SPOKE0_D3
set_property -dict { PACKAGE_PIN R3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D4 }]; #IO_L2P_T0_34 Sch=pio[26], schematic net SPOKE0_D4
set_property -dict { PACKAGE_PIN T3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D5 }]; #IO_L2N_T0_34 Sch=pio[27], schematic net SPOKE0_D5
set_property -dict { PACKAGE_PIN R2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_STROBE }]; #IO_L1P_T0_34 Sch=pio[28], schematic net SPOKE0_STROBE
