# Hub FPGA (Digilent Cmod A7-35T) pin constraints for hub_top.v.
#
# Remaining gap: hub_top.v's USB_CLKOUT/USB_D0-7/USB_WR_N/USB_TXE_N ports
# (usb_framer.v -- see fpga/USB_FRAMING.md) stay unconstrained until A6's
# footprint swap from the Adafruit FT232H breakout to an FTDI UM232H module
# lands on pcb/multi_fpga/hub.kicad_sch (needed for CLKOUT/sync-mode
# support -- see USB_FRAMING.md). They'll land within A6 DIP pins 35-46,
# same as the old async design's USB_D0-7/RXF_N/TXE_N/RD_N/WR_N did -- this
# design needs one fewer pin (11 vs. 12: no RD_N/RXF_N ports, see
# usb_framer.v's header comment), so no spoke-bus pin needs to move.
# USB_CLKOUT specifically must land on an MRCC/SRCC P-side pin (BUFG source)
# -- pio[36]/W5 (IO_L12P_T1_MRCC_34), already within that same pin block,
# satisfies this directly.
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

# Bank 0 (config) voltage: same as the cluster XDC -- Cmod A7-35T's config
# bank runs off the module's own 3.3V rail, matching every IOSTANDARD
# LVCMOS33 used below.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property -dict { PACKAGE_PIN U8    IOSTANDARD LVCMOS33 } [get_ports { TCXO_CLK }]; #IO_L14P_T2_SRCC_34 Sch=pio[47], 12.288 MHz HCMOS TCXO (Y1)
create_clock -period 81.380 -name TCXO_CLK [get_ports { TCXO_CLK }]

set_property -dict { PACKAGE_PIN V8    IOSTANDARD LVCMOS33 } [get_ports { FPGA_RESET_N }]; #IO_L14N_T2_SRCC_34 Sch=pio[48], schematic net RESET_N (see J19 jumper note above)

set_property -dict { PACKAGE_PIN G17   IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_ALIVE }]; #IO_L5N_T0_D07_14 Sch=ja[1], via J18<->J20 cable
set_property -dict { PACKAGE_PIN G19   IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_ALIVE }]; #IO_L4N_T0_D05_14 Sch=ja[2], via J18<->J20 cable
set_property -dict { PACKAGE_PIN N18   IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_ALIVE }]; #IO_L9P_T1_DQS_14 Sch=ja[3], via J18<->J20 cable
set_property -dict { PACKAGE_PIN L18   IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_ALIVE }]; #IO_L8P_T1_D11_14 Sch=ja[4], via J18<->J20 cable

# SPOKE_CLK + SPOKE0-3_D0-5/STROBE: A6 DIP pins 1-14/17-23/27-34, read off
# hub.kicad_sch via `kicad-cli sch export netlist` the same way as every
# other pin above. SPOKE_CLK (pin 34) is a plain LVCMOS33 output -- not a
# clock-capable (MRCC/SRCC) pin, but that restriction only applies to clock
# *inputs* that need to reach a BUFG; this is a BUFG-sourced internal signal
# driving an ordinary output pin instead, so no placement conflict.
set_property -dict { PACKAGE_PIN M3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D0 }]; #IO_L8N_T1_AD14N_35 Sch=pio[01]
set_property -dict { PACKAGE_PIN L3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D1 }]; #IO_L8P_T1_AD14P_35 Sch=pio[02]
set_property -dict { PACKAGE_PIN A16   IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D2 }]; #IO_L12P_T1_MRCC_16 Sch=pio[03]
set_property -dict { PACKAGE_PIN K3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D3 }]; #IO_L7N_T1_AD6N_35 Sch=pio[04]
set_property -dict { PACKAGE_PIN C15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D4 }]; #IO_L11P_T1_SRCC_16 Sch=pio[05]
set_property -dict { PACKAGE_PIN H1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_D5 }]; #IO_L3P_T0_DQS_AD5P_35 Sch=pio[06]
set_property -dict { PACKAGE_PIN A15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE0_STROBE }]; #IO_L6N_T0_VREF_16 Sch=pio[07]

set_property -dict { PACKAGE_PIN B15   IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D0 }]; #IO_L11N_T1_SRCC_16 Sch=pio[08]
set_property -dict { PACKAGE_PIN A14   IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D1 }]; #IO_L6P_T0_16 Sch=pio[09]
set_property -dict { PACKAGE_PIN J3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D2 }]; #IO_L7P_T1_AD6P_35 Sch=pio[10]
set_property -dict { PACKAGE_PIN J1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D3 }]; #IO_L3N_T0_DQS_AD5N_35 Sch=pio[11]
set_property -dict { PACKAGE_PIN K2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D4 }]; #IO_L5P_T0_AD13P_35 Sch=pio[12]
set_property -dict { PACKAGE_PIN L1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_D5 }]; #IO_L6N_T0_VREF_35 Sch=pio[13]
set_property -dict { PACKAGE_PIN L2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE1_STROBE }]; #IO_L5N_T0_AD13N_35 Sch=pio[14]

# A6 DIP pins 15/16 (AIN15/AIN16) skipped -- dedicated XADC analog inputs,
# not usable as GPIO (confirmed, not wired to anything in hub.kicad_sch).

set_property -dict { PACKAGE_PIN M1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D0 }]; #IO_L9N_T1_DQS_AD7N_35 Sch=pio[17]
set_property -dict { PACKAGE_PIN N3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D1 }]; #IO_L12P_T1_MRCC_35 Sch=pio[18]
set_property -dict { PACKAGE_PIN P3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D2 }]; #IO_L12N_T1_MRCC_35 Sch=pio[19]
set_property -dict { PACKAGE_PIN M2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D3 }]; #IO_L9P_T1_DQS_AD7P_35 Sch=pio[20]
set_property -dict { PACKAGE_PIN N1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D4 }]; #IO_L10N_T1_AD15N_35 Sch=pio[21]
set_property -dict { PACKAGE_PIN N2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_D5 }]; #IO_L10P_T1_AD15P_35 Sch=pio[22]
set_property -dict { PACKAGE_PIN P1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE2_STROBE }]; #IO_L19N_T3_VREF_35 Sch=pio[23]

# A6 DIP pins 24/25 (+5V/GND) skipped -- power, not signal.

set_property -dict { PACKAGE_PIN T3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D0 }]; #IO_L2N_T0_34 Sch=pio[27]
set_property -dict { PACKAGE_PIN R2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D1 }]; #IO_L1P_T0_34 Sch=pio[28]
set_property -dict { PACKAGE_PIN T1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D2 }]; #IO_L3P_T0_DQS_34 Sch=pio[29]
set_property -dict { PACKAGE_PIN T2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D3 }]; #IO_L1N_T0_34 Sch=pio[30]
set_property -dict { PACKAGE_PIN U1    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D4 }]; #IO_L3N_T0_DQS_34 Sch=pio[31]
set_property -dict { PACKAGE_PIN W2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_D5 }]; #IO_L5N_T0_34 Sch=pio[32]
set_property -dict { PACKAGE_PIN V2    IOSTANDARD LVCMOS33 } [get_ports { SPOKE3_STROBE }]; #IO_L5P_T0_34 Sch=pio[33]

set_property -dict { PACKAGE_PIN W3    IOSTANDARD LVCMOS33 } [get_ports { SPOKE_CLK }]; #IO_L6N_T0_VREF_34 Sch=pio[34], shared by all 4 SpokeBus connectors on the PCB (see hub_top.v)
# Generated clock defined on bufg_sclk's output pin (the net that actually
# clocks the 4 spoke_deframer instances internally), not the SPOKE_CLK port
# itself -- the port is a separate OBUF fanout of the same sclk_r register,
# with no internal STA consumers of its own; external timing to the cluster
# boards is covered by the SPOKE_D*/STROBE false paths below, same as
# SPOKE0-3_ALIVE/FPGA_RESET_N already are.
create_generated_clock -name sclk -source [get_pins bufg_inst/O] -divide_by 2 [get_pins bufg_sclk/O]

# On-board RGB LED (LD0), Bank 16 -- common-anode, active-low
set_property -dict { PACKAGE_PIN C16   IOSTANDARD LVCMOS33 } [get_ports { led0_g }];
set_property -dict { PACKAGE_PIN C17   IOSTANDARD LVCMOS33 } [get_ports { led0_r }];
set_property -dict { PACKAGE_PIN B17   IOSTANDARD LVCMOS33 } [get_ports { led0_b }];

# False paths (same reasoning as fpga/cluster/xdc/cluster_top.xdc -- these
# are genuinely untimed by design, not "checked and fine" or "ignored"):
#
# SPOKE0-3_ALIVE: async status from 4 independent cluster boards, each on
# its own clock domain. reset_seq.v already runs them through its own 2-FF
# synchronizer (alive_meta/alive_sync) specifically because they're untimed
# relative to TCXO_CLK.
set_false_path -from [get_ports { SPOKE0_ALIVE SPOKE1_ALIVE SPOKE2_ALIVE SPOKE3_ALIVE }]

# FPGA_RESET_N: each cluster board receives this into its own clk_reset.v,
# which runs it through its own 2-FF synchronizer for the same reason --
# no meaningful setup/hold relationship to constrain from the hub side.
# led0_r: just the status LED, not timing-critical.
set_false_path -to [get_ports { FPGA_RESET_N led0_r }]

# SPOKE_CLK: same reasoning as PDM_CLK/SPOKE_D*/SPOKE_ALIVE in
# fpga/cluster/xdc/cluster_top.xdc -- an output to other boards with no real
# board-trace numbers to write a set_output_delay against yet (report_
# methodology's TIMING-18 flags this as "missing output delay relative to
# TCXO_CLK", same class of gap already fixed with false paths elsewhere in
# this project). Revisit with real set_output_delay numbers once the PCB
# layout gives something to constrain against.
set_false_path -to [get_ports { SPOKE_CLK }]

# SPOKE0-3_D0-5/STROBE: genuinely untimed by design, same reasoning as
# fpga/cluster/xdc/cluster_top.xdc's own PDM_D*/SPOKE_D* false paths -- the
# round trip (hub's SPOKE_CLK out -> cluster's own IBUF/BUFG/spoke_framer ->
# cable back to the hub) has no real trace-delay numbers to constrain
# against yet, and spoke_deframer.v was deliberately designed with a full
# clock-period's margin rather than tight source-synchronous timing (see
# SPOKE_FRAMING.md). Captured into the sclk domain via spoke_deframer.v's own
# rise_r register, same as the cluster side treats its incoming SPOKE_D*.
set_false_path -from [get_ports { \
    SPOKE0_D0 SPOKE0_D1 SPOKE0_D2 SPOKE0_D3 SPOKE0_D4 SPOKE0_D5 SPOKE0_STROBE \
    SPOKE1_D0 SPOKE1_D1 SPOKE1_D2 SPOKE1_D3 SPOKE1_D4 SPOKE1_D5 SPOKE1_STROBE \
    SPOKE2_D0 SPOKE2_D1 SPOKE2_D2 SPOKE2_D3 SPOKE2_D4 SPOKE2_D5 SPOKE2_STROBE \
    SPOKE3_D0 SPOKE3_D1 SPOKE3_D2 SPOKE3_D3 SPOKE3_D4 SPOKE3_D5 SPOKE3_STROBE \
}]
