set_property -dict {PACKAGE_PIN U8 IOSTANDARD LVCMOS33} [get_ports TCXO_CLK]
set_property -dict {PACKAGE_PIN V8 IOSTANDARD LVCMOS33} [get_ports FPGA_RESET_N]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports SPOKES_ALIVE]
set_property -dict {PACKAGE_PIN M3 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D0]
set_property -dict {PACKAGE_PIN L3 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D1]
set_property -dict {PACKAGE_PIN A16 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D2]
set_property -dict {PACKAGE_PIN K3 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D3]
set_property -dict {PACKAGE_PIN C15 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D4]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33} [get_ports SPOKE0_D5]
set_property -dict {PACKAGE_PIN A15 IOSTANDARD LVCMOS33} [get_ports SPOKE0_STROBE]
set_property -dict {PACKAGE_PIN B15 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D0]
set_property -dict {PACKAGE_PIN A14 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D1]
set_property -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D2]
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D3]
set_property -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D4]
set_property -dict {PACKAGE_PIN L1 IOSTANDARD LVCMOS33} [get_ports SPOKE1_D5]
set_property -dict {PACKAGE_PIN L2 IOSTANDARD LVCMOS33} [get_ports SPOKE1_STROBE]
set_property -dict {PACKAGE_PIN M1 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D0]
set_property -dict {PACKAGE_PIN N3 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D1]
set_property -dict {PACKAGE_PIN P3 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D2]
set_property -dict {PACKAGE_PIN M2 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D3]
set_property -dict {PACKAGE_PIN N1 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D4]
set_property -dict {PACKAGE_PIN N2 IOSTANDARD LVCMOS33} [get_ports SPOKE2_D5]
set_property -dict {PACKAGE_PIN P1 IOSTANDARD LVCMOS33} [get_ports SPOKE2_STROBE]
set_property -dict {PACKAGE_PIN T3 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D0]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D1]
set_property -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D2]
set_property -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D3]
set_property -dict {PACKAGE_PIN U1 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D4]
set_property -dict {PACKAGE_PIN W2 IOSTANDARD LVCMOS33} [get_ports SPOKE3_D5]
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports SPOKE3_STROBE]
set_property -dict {PACKAGE_PIN W3 IOSTANDARD LVCMOS33} [get_ports SPOKE_CLK]
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS33} [get_ports led0_g]
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports led0_r]
set_property -dict {PACKAGE_PIN B17 IOSTANDARD LVCMOS33} [get_ports led0_b]
set_property -dict {PACKAGE_PIN W7 IOSTANDARD LVCMOS33} [get_ports USB_CLKOUT]
set_property -dict {PACKAGE_PIN V3 IOSTANDARD LVCMOS33} [get_ports USB_D0]
set_property -dict {PACKAGE_PIN W5 IOSTANDARD LVCMOS33} [get_ports USB_D1]
set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports USB_D2]
set_property -dict {PACKAGE_PIN U4 IOSTANDARD LVCMOS33} [get_ports USB_D3]
set_property -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports USB_D4]
set_property -dict {PACKAGE_PIN W4 IOSTANDARD LVCMOS33} [get_ports USB_D5]
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports USB_D6]
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports USB_D7]
set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports USB_WR_N]
set_property -dict {PACKAGE_PIN U3 IOSTANDARD LVCMOS33} [get_ports USB_TXE_N]
set_property ASYNC_REG true [get_cells -hier -filter { NAME =~ "*/tgl?_meta_reg" || NAME =~ "*/tgl?_sync_reg" || NAME =~ "*rst_usb_meta_reg" || NAME =~ "*rst_usb_reg" }]
# Hub FPGA (Digilent Cmod A7-35T) pin constraints for hub_top.v.
#
# All ports hub_top.v currently declares are populated, including the USB
# FIFO bridge (usb_framer.v -- see fpga/USB_FRAMING.md) to the FTDI UM232H
# module (A5). USB_D0-7/USB_WR_N/USB_TXE_N land on A6 DIP pins 35-44, same
# range the old (removed) async Adafruit FT232H design used -- this design
# needs one fewer pin (11 vs. 12: no RD_N/RXF_N ports, see usb_framer.v's
# header comment for why), so no spoke-bus pin needed to move.
#
# USB_CLKOUT specifically had to land on an MRCC/SRCC P-side pin (it feeds a
# BUFG in hub_top.v) -- pio[45]/U7 (IO_L19P_T3_34, where the schematic
# originally routed it) is NOT clock-capable and fails Vivado's placer with
# a hard rule_gclkio_bufg error ("Poor placement for routing between an IO
# pin and BUFG" / "IO Clock Placer failed"), confirmed by an actual
# synth+place run before the schematic was corrected -- same class of issue
# as the earlier SPOKE_CLK/PDM_D08 pin swap. Moved to pio[46]/W7
# (IO_L13P_T2_MRCC_34, P-side MRCC), which was unused and satisfies the
# requirement directly.
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
# SPOKES_ALIVE: A6 has no PIO pins free (all 48 committed to spokes/USB/
# TCXO/reset) and its schematic symbol has no JA Pmod pins modeled. This
# signal reaches the FPGA via an external jumper cable: J18 (PMOD_Signals
# breakout, on the SpokeBus side) <-> J20 (A6_JA, DNP documentation symbol
# for the Cmod A7's on-board Pmod header) -- see hub.kicad_sch. J20 pin
# 1/2/3/4 = physical JA1/JA2/JA3/JA4 in order (confirmed with board owner).
#
# SPOKES_ALIVE is a wired-AND net now (all 4 clusters' SPOKE_ALIVE pins tied
# together on the PCB with one external pull-up, see clk_reset.v/
# reset_seq.v), collapsed from the previous 4 independent SPOKE0-3_ALIVE
# pins -- PROVISIONAL: still on JA1 (G17), the old SPOKE0_ALIVE pin, pending
# the schematic net consolidation. Re-derive from the netlist once that
# lands (JA2-4/G19,N18,L18 go free for future use in the meantime).

# Bank 0 (config) voltage: same as the cluster XDC -- Cmod A7-35T's config
# bank runs off the module's own 3.3V rail, matching every IOSTANDARD
# LVCMOS33 used below.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

create_clock -period 81.380 -name TCXO_CLK [get_ports TCXO_CLK]



# SPOKE_CLK + SPOKE0-3_D0-5/STROBE: A6 DIP pins 1-14/17-23/27-34, read off
# hub.kicad_sch via `kicad-cli sch export netlist` the same way as every
# other pin above. SPOKE_CLK (pin 34) is a plain LVCMOS33 output -- not a
# clock-capable (MRCC/SRCC) pin, but that restriction only applies to clock
# *inputs* that need to reach a BUFG; this is a BUFG-sourced internal signal
# driving an ordinary output pin instead, so no placement conflict.


# A6 DIP pins 15/16 (AIN15/AIN16) skipped -- dedicated XADC analog inputs,
# not usable as GPIO (confirmed, not wired to anything in hub.kicad_sch).


# A6 DIP pins 24/25 (+5V/GND) skipped -- power, not signal.


# Generated clock defined on bufg_sclk's output pin (the net that actually
# clocks the 4 spoke_deframer instances internally), not the SPOKE_CLK port
# itself -- the port is a separate OBUF fanout of the same sclk_r register,
# with no internal STA consumers of its own; external timing to the cluster
# boards is covered by the SPOKE_D*/STROBE false paths below, same as
# SPOKES_ALIVE/FPGA_RESET_N already are.
create_generated_clock -name sclk -source [get_pins bufg_inst/O] -divide_by 2 [get_pins bufg_sclk/O]

# On-board RGB LED (LD0), Bank 16 -- common-anode, active-low. Pin bug fixed:
# led0_g was on C16, which fpga/demo/CmodA735tDemo/Src/CmodA735tDemo.xdc
# (Digilent's own master XDC) actually assigns to led[1], a different,
# non-RGB LED -- the real led0_g pin is B16. Wouldn't have been caught by
# synth/place/route or report_methodology (C16 is a valid, otherwise-unused
# pin on this design), only by real hardware bring-up (wrong LED lighting
# up). Found while cross-checking pins for the cluster FPGA's identical LED
# addition.

# False paths (same reasoning as fpga/cluster/xdc/cluster_top.xdc -- these
# are genuinely untimed by design, not "checked and fine" or "ignored"):
#
# SPOKES_ALIVE: async wired-AND status from 4 independent cluster boards,
# each on its own clock domain. reset_seq.v already runs it through its own
# 2-FF synchronizer (alive_meta/alive_sync) specifically because it's
# untimed relative to TCXO_CLK.
set_false_path -from [get_ports SPOKES_ALIVE]

# FPGA_RESET_N: each cluster board receives this into its own clk_reset.v,
# which runs it through its own 2-FF synchronizer for the same reason --
# no meaningful setup/hold relationship to constrain from the hub side.
# led0_r: just the status LED, not timing-critical.
set_false_path -to [get_ports {FPGA_RESET_N led0_r}]

# SPOKE_CLK: same reasoning as PDM_CLK/SPOKE_D*/SPOKE_ALIVE in
# fpga/cluster/xdc/cluster_top.xdc -- an output to other boards with no real
# board-trace numbers to write a set_output_delay against yet (report_
# methodology's TIMING-18 flags this as "missing output delay relative to
# TCXO_CLK", same class of gap already fixed with false paths elsewhere in
# this project). Revisit with real set_output_delay numbers once the PCB
# layout gives something to constrain against.
set_false_path -to [get_ports SPOKE_CLK]

# SPOKE0-3_D0-5/STROBE: genuinely untimed by design, same reasoning as
# fpga/cluster/xdc/cluster_top.xdc's own PDM_D*/SPOKE_D* false paths -- the
# round trip (hub's SPOKE_CLK out -> cluster's own IBUF/BUFG/spoke_framer ->
# cable back to the hub) has no real trace-delay numbers to constrain
# against yet, and spoke_deframer.v was deliberately designed with a full
# clock-period's margin rather than tight source-synchronous timing (see
# SPOKE_FRAMING.md). Captured into the sclk domain via spoke_deframer.v's own
# rise_r register, same as the cluster side treats its incoming SPOKE_D*.
set_false_path -from [get_ports {SPOKE0_D0 SPOKE0_D1 SPOKE0_D2 SPOKE0_D3 SPOKE0_D4 SPOKE0_D5 SPOKE0_STROBE SPOKE1_D0 SPOKE1_D1 SPOKE1_D2 SPOKE1_D3 SPOKE1_D4 SPOKE1_D5 SPOKE1_STROBE SPOKE2_D0 SPOKE2_D1 SPOKE2_D2 SPOKE2_D3 SPOKE2_D4 SPOKE2_D5 SPOKE2_STROBE SPOKE3_D0 SPOKE3_D1 SPOKE3_D2 SPOKE3_D3 SPOKE3_D4 SPOKE3_D5 SPOKE3_STROBE}]

# USB FIFO bridge (A5, FTDI UM232H) -- see USB_FRAMING.md. Net -> DIP pin
# read off hub.kicad_sch the same way as every pin above; USB_CLKOUT's pin
# choice is explained in this file's header comment.

# USB_CLKOUT: a real, independent clock (FT232H's own 60MHz oscillator, not
# derived from TCXO_CLK) -- create_clock, not a generated clock.
create_clock -period 16.667 -name usb_clk_pin [get_ports USB_CLKOUT]

# usb_clk has no fixed phase relationship to clk/sclk (genuinely
# asynchronous, unlike sclk which is generated from clk) -- first
# fabric-internal async clock-domain crossing in this project's XDC, so the
# constraint has to name the clock groups directly rather than a port (see
# USB_FRAMING.md's "Clock domain crossing" section for the CDC design this
# covers: usb_framer.v's per-spoke toggle+2FF-sync).
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks TCXO_CLK] -group [get_clocks usb_clk_pin]

# ASYNC_REG on both stages of every toggle-bit synchronizer usb_framer.v
# uses to cross spokeN_valid into the usb_clk domain, plus the rst_usb
# synchronizer hub_top.v generates locally -- report_methodology's TIMING-10
# flags CDC synchronizers missing this property; it's a placement hint
# (keep the 2 FFs physically adjacent for minimum metastability MTBF), not a
# functional requirement, but there's no reason not to have it.

# USB_D0-7/USB_WR_N/USB_TXE_N: same reasoning as SPOKE_CLK/SPOKE_D* above --
# genuinely untimed by design (no real board-trace numbers to constrain
# against yet), and the FT245 sync FIFO protocol's own timing is already
# covered by usb_clk_pin/usb_framer.v treating usb_txe_n as an ordinary
# same-domain synchronous input and driving usb_d/usb_wr_n as ordinary
# same-domain registered outputs -- ISE/Vivado just needs telling there's no
# separate board-trace budget to check here, same class of gap TIMING-18
# flags for SPOKE_CLK.
set_false_path -to [get_ports {USB_D0 USB_D1 USB_D2 USB_D3 USB_D4 USB_D5 USB_D6 USB_D7 USB_WR_N}]
set_false_path -from [get_ports USB_TXE_N]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR NO [current_design]
