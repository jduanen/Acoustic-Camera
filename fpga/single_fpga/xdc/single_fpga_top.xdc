# Single-FPGA (ALINX AC7200, xc7a200tfbg484-2) pin constraints -- clock pins.
#
# Clock scheme (pcb/SCHEMATIC_NOTES.md, "PDM Clock Distribution"): the
# 12.288 MHz TCXO (Y1) sits on the mic_array board and reaches the FPGA as
# TCXO_CLK over J1/J2 pin 41. The FPGA divides it by 4 to a 50 %-duty
# 3.072 MHz PDM_CLK and sends that back over J2/J1 pin 45 to the
# CDCLVC1112/CDCLVC1108 buffer tree that clocks all 96 mics.
#
# Both clocks are on front_end CON1 so CON2/CON4 can stay unpopulated.
# Ball numbers come from the AC7200 user manual's CON1 table, and the
# CON1 pin numbers from `kicad-cli sch export netlist` of
# pcb/single_fpga/front_end.kicad_sch:
#   TCXO_CLK  CON1 pin 21  B13_L5_P  Y13   (bank 13, VCCO 3.3V)
#   PDM_CLK   CON1 pin 25  B13_L7_P  AB11  (bank 13, VCCO 3.3V)

# Configuration bank 0 runs at 3.3V on the AC7200 (user manual, power section).
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property -dict {PACKAGE_PIN Y13 IOSTANDARD LVCMOS33} [get_ports TCXO_CLK]
set_property -dict {PACKAGE_PIN AB11 IOSTANDARD LVCMOS33 DRIVE 12 SLEW SLOW} [get_ports PDM_CLK]

create_clock -period 81.380 -name TCXO_CLK [get_ports TCXO_CLK]

# Y13 is not a clock-capable (MRCC/SRCC) pin -- CON1 has no free 3.3V one --
# so the IBUF can't reach a BUFG over dedicated clock routing. Allow fabric
# routing to the BUFG instead; the added delay/jitter (a few hundred ps) is
# irrelevant at 12.288 MHz in / 3.072 MHz out. Divide in fabric off that
# BUFG (a 2-bit counter); don't feed an MMCM/PLL from this pin.
# -quiet: TCXO_CLK_IBUF doesn't exist yet when synth_design parses this file
# (IBUFs are inserted later); it is applied when the file is re-read for
# implementation. If the net name ever stops matching, place_design fails
# with Place 30-574 rather than passing silently.
set_property -quiet CLOCK_DEDICATED_ROUTE FALSE [get_nets -quiet TCXO_CLK_IBUF]

# PDM_CLK drives ~67 mm of unterminated trace (front_end CON1 -> J2 -> mic_array
# U110 CLKIN) plus TP8, ~15-17 pF in all. U110 (CDCLVC1112) needs >= 1 V/ns input
# slew, which DRIVE 4 couldn't guarantee into that load. DRIVE 12's output
# impedance is close to the ~40-45 ohm trace, so the source is roughly matched;
# SLEW SLOW keeps the edges no faster than they need to be.

# Status RGB LED (front_end D1, pcb/SCHEMATIC_NOTES.md "Status LED"). Each pin
# drives the gate of a 2N7002 that sinks one cathode of the common-anode LED on
# +5V: active high (1 = colour on); 100k gate pull-downs keep it off until the
# FPGA is configured. Gate load only, so minimum drive.
#   LED_R  CON1 pin 32  B13_L6_P  W14   (bank 13, VCCO 3.3V)
#   LED_G  CON1 pin 33  B13_L3_N  AB13  (bank 13, VCCO 3.3V)
#   LED_B  CON1 pin 34  B13_L6_N  Y14   (bank 13, VCCO 3.3V)
set_property -dict {PACKAGE_PIN W14  IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports LED_R]
set_property -dict {PACKAGE_PIN AB13 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports LED_G]
set_property -dict {PACKAGE_PIN Y14  IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports LED_B]
