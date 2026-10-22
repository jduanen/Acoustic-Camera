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
set_property -dict {PACKAGE_PIN AB11 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports PDM_CLK]

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

# PDM_CLK has no series resistor on front_end (only J2 -> CON1, ~24 mm), so
# it uses low drive + slow slew (set above) to soften its edges.
