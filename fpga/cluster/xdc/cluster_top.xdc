# Cluster FPGA (Digilent Cmod S7-25) pin constraints for cluster_top.v.
#
# All 23 ports are populated. Net -> Cmod S7 PIOnn assignments read directly
# off pcb/multi_fpga/cluster_00.kicad_sch's current wiring via
# `kicad-cli sch export netlist` (cluster_01/02/03.kicad_sch carry the
# identical PIOnn pattern, confirmed by the same export, so this one XDC
# applies unmodified to all 4 cluster boards -- only the SPOKEn_* prefix on
# the hub side differs, not any pin number here). PIOnn -> silicon
# PACKAGE_PIN taken from Cmod-S7-25-Master.xdc's own commented-out pio1-48
# reference lines (Digilent's stock constraint file, fpga/demo/hw/hw.srcs/
# constrs_1/imports/constraints/Cmod-S7-25-Master.xdc).
#
# FPGA_RESET_N (PIO30/M13) and SPOKE_ALIVE (PIO31/J11) were already present
# and are confirmed correct by this same cross-check.

set_property -dict { PACKAGE_PIN L13   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_CLK }]; #IO_L7P_T1_D09_14 Sch=pio[29], schematic net SPOKE0_CLK
create_clock -period 325.521 -name SPOKE_CLK [get_ports { SPOKE_CLK }]

set_property -dict { PACKAGE_PIN M13   IOSTANDARD LVCMOS33 PULLTYPE PULLUP } [get_ports { FPGA_RESET_N }]; #IO_L8P_T1_D11_14 Sch=pio[30]
set_property -dict { PACKAGE_PIN J11   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_ALIVE }]; #IO_0_14 Sch=pio[31]

set_property -dict { PACKAGE_PIN L1    IOSTANDARD LVCMOS33 } [get_ports { PDM_CLK }]; #IO_L18N_T2_34 Sch=pio[01], schematic net C0_PDM_CLK

set_property -dict { PACKAGE_PIN M4    IOSTANDARD LVCMOS33 } [get_ports { PDM_D00 }]; #IO_L19P_T3_34 Sch=pio[02], schematic net DATA_00
set_property -dict { PACKAGE_PIN M3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D01 }]; #IO_L19N_T3_VREF_34 Sch=pio[03], schematic net DATA_01
set_property -dict { PACKAGE_PIN N2    IOSTANDARD LVCMOS33 } [get_ports { PDM_D02 }]; #IO_L20P_T3_34 Sch=pio[04], schematic net DATA_02
set_property -dict { PACKAGE_PIN M2    IOSTANDARD LVCMOS33 } [get_ports { PDM_D03 }]; #IO_L20N_T3_34 Sch=pio[05], schematic net DATA_03
set_property -dict { PACKAGE_PIN P3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D04 }]; #IO_L21P_T3_DQS_34 Sch=pio[06], schematic net DATA_04
set_property -dict { PACKAGE_PIN N3    IOSTANDARD LVCMOS33 } [get_ports { PDM_D05 }]; #IO_L21N_T3_DQS_34 Sch=pio[07], schematic net DATA_05
set_property -dict { PACKAGE_PIN P1    IOSTANDARD LVCMOS33 } [get_ports { PDM_D06 }]; #IO_L22P_T3_34 Sch=pio[08], schematic net DATA_06
set_property -dict { PACKAGE_PIN N1    IOSTANDARD LVCMOS33 } [get_ports { PDM_D07 }]; #IO_L22N_T3_34 Sch=pio[09], schematic net DATA_07
set_property -dict { PACKAGE_PIN P14   IOSTANDARD LVCMOS33 } [get_ports { PDM_D08 }]; #IO_L11P_T1_SRCC_14 Sch=pio[16], schematic net DATA_08
set_property -dict { PACKAGE_PIN P15   IOSTANDARD LVCMOS33 } [get_ports { PDM_D09 }]; #IO_L11N_T1_SRCC_14 Sch=pio[17], schematic net DATA_09
set_property -dict { PACKAGE_PIN N13   IOSTANDARD LVCMOS33 } [get_ports { PDM_D10 }]; #IO_L8N_T1_D12_14 Sch=pio[18], schematic net DATA_10
set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { PDM_D11 }]; #IO_L10N_T1_D15_14 Sch=pio[19], schematic net DATA_11

set_property -dict { PACKAGE_PIN N14   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D0 }]; #IO_L10P_T1_D14_14 Sch=pio[20], schematic net SPOKE0_D0
set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D1 }]; #IO_L9N_T1_DQS_D13_14 Sch=pio[21], schematic net SPOKE0_D1
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D2 }]; #IO_L9P_T1_DQS_14 Sch=pio[22], schematic net SPOKE0_D2
set_property -dict { PACKAGE_PIN L15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D3 }]; #IO_L4N_T0_D05_14 Sch=pio[23], schematic net SPOKE0_D3
set_property -dict { PACKAGE_PIN L14   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D4 }]; #IO_L7N_T1_D10_14 Sch=pio[26], schematic net SPOKE0_D4
set_property -dict { PACKAGE_PIN K14   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_D5 }]; #IO_L4P_T0_D04_14 Sch=pio[27], schematic net SPOKE0_D5
set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE_STROBE }]; #IO_L5P_T0_D06_14 Sch=pio[28], schematic net SPOKE0_STROBE
