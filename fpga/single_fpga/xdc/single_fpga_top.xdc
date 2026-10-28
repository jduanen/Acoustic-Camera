# Single-FPGA (ALINX AC7200, xc7a200tfbg484-2) pin constraints -- clocks,
# status LED and mic data lines.
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

# PDM_CLK is a forwarded clock (single_fpga_top.v's pdm_div[1] -> OBUF), not data:
# U110 is a buffer with no setup/hold window, so a set_output_delay on it would be a
# made-up number. Declare it as the TCXO/4 generated clock instead (3.072 MHz,
# 325.52 ns). This also satisfies Vivado's TIMING-18 check on the port, and is the
# reference for the mic DATA input delays below.
create_generated_clock -name PDM_CLK -source [get_ports TCXO_CLK] -divide_by 4 [get_ports PDM_CLK]

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
# Nothing samples the LED pins -- they change at human-visible rates -- so there is no
# timing requirement to meet. Exclude them from timing analysis.
set_false_path -to [get_ports {LED_R LED_G LED_B}]

# Mic data lines DATA_00..DATA_47 (front_end CON3, all bank 15, VCCO 3.3V), one bit per
# net: DATA[n] = DATA_nn. Each line carries two IM72D128 mics, L on the falling PDM_CLK
# edge and R on the rising (pcb/SCHEMATIC_NOTES.md "Data Line Assignment"). CON3 pin,
# bank-pin name and ball are from the AC7200 user manual's CON3 table; the CON3 pin of
# each net is from a kicad-cli netlist of pcb/single_fpga/front_end.kicad_sch, and every
# bank-pin name matches the KiCad CON3 symbol's.
#
#
# Input timing (rtl/pdm_sampler.v). Each line's bit for a channel is requested by a
# PDM_CLK edge and captured on the TCXO_CLK edge two cycles later (the edge that also
# launches the next PDM_CLK edge). Delays are from PDM_CLK at the FPGA pin, which the
# generated clock above already places after Vivado's own clock-to-out:
#   clock out to the mics: PDM_CLK net + U110 + arm buffer + MIC_CLK trace
#       buffers 2 x 0.8-2.0 ns (CDCLVC1112/1108 at 3.3 V)
#       traces  ~80 mm + 195-235 mm = 275-315 mm at 5.5-7.0 ps/mm -> 1.5-2.2 ns
#       => 3.1-6.2 ns
#   mic: data driven tDD 40-80 ns after its edge; released tHZ 5-30 ns after the
#        opposite edge (IM72D128 datasheet, digital interface table)
#   return: DATA_nn <= 270 mm over both boards (longest net, both mic branches)
#       => 0-1.9 ns
# max = 6.2 + 80 + 1.9 = 88.1 -> 89 ns   (new bit valid)
# min = 3.1 + 5  + 0   = 8.1  -> 8 ns    (earliest the old bit can go)
# Both edges request data (L on fall, R on rise), so both are constrained. Against the
# 162.76 ns capture that leaves ~74 ns setup and ~8 ns hold margin before Vivado's own
# input-path and clock-skew numbers, which the timing report adds.
set_input_delay -clock PDM_CLK -max 89.0 [get_ports {DATA[*]}]
set_input_delay -clock PDM_CLK -min 8.0  [get_ports {DATA[*]}]
set_input_delay -clock PDM_CLK -clock_fall -max 89.0 -add_delay [get_ports {DATA[*]}]
set_input_delay -clock PDM_CLK -clock_fall -min 8.0  -add_delay [get_ports {DATA[*]}]
# Capture two TCXO cycles after launch (setup), and the hold check against the edge
# that launches the next PDM_CLK edge, i.e. the launch edge itself (-end: counted in
# TCXO_CLK cycles; the default for -hold would count 325 ns PDM_CLK cycles).
set_multicycle_path -setup -end 2 -from [get_clocks PDM_CLK] -to [get_clocks TCXO_CLK]
set_multicycle_path -hold  -end 1 -from [get_clocks PDM_CLK] -to [get_clocks TCXO_CLK]
# report_methodology's TIMING-18 still asks for DATA input delays relative to TCXO_CLK,
# the capturing clock. That's deliberate: the data is launched by PDM_CLK, and
# referencing it there (not TCXO_CLK) is what puts PDM_CLK's own clock-to-out into the
# analysis and limits launches to the two TCXO edges that actually make a PDM_CLK edge.
create_waiver -type METHODOLOGY -id {TIMING-18} -user single_fpga -objects [get_ports {DATA[*]}] \
    -description {DATA input delays are referenced to PDM_CLK (the forwarded clock that launches them), not TCXO_CLK}
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports {DATA[0]}]  ;# CON3.5  B15_L4_P
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {DATA[1]}]  ;# CON3.7  B15_L4_N
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33} [get_ports {DATA[2]}]  ;# CON3.11 B15_L2_P
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS33} [get_ports {DATA[3]}]  ;# CON3.13 B15_L2_N
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {DATA[4]}]  ;# CON3.1  B15_IO0
set_property -dict {PACKAGE_PIN H19 IOSTANDARD LVCMOS33} [get_ports {DATA[5]}]  ;# CON3.17 B15_L12_N
set_property -dict {PACKAGE_PIN J20 IOSTANDARD LVCMOS33} [get_ports {DATA[6]}]  ;# CON3.21 B15_L11_P
set_property -dict {PACKAGE_PIN J21 IOSTANDARD LVCMOS33} [get_ports {DATA[7]}]  ;# CON3.23 B15_L11_N
set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports {DATA[8]}]  ;# CON3.25 B15_L1_N
set_property -dict {PACKAGE_PIN G20 IOSTANDARD LVCMOS33} [get_ports {DATA[9]}]  ;# CON3.26 B15_L8_N
set_property -dict {PACKAGE_PIN H13 IOSTANDARD LVCMOS33} [get_ports {DATA[10]}] ;# CON3.27 B15_L1_P
set_property -dict {PACKAGE_PIN H20 IOSTANDARD LVCMOS33} [get_ports {DATA[11]}] ;# CON3.28 B15_L8_P
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {DATA[12]}] ;# CON3.31 B15_L5_P
set_property -dict {PACKAGE_PIN H22 IOSTANDARD LVCMOS33} [get_ports {DATA[13]}] ;# CON3.32 B15_L7_N
set_property -dict {PACKAGE_PIN H15 IOSTANDARD LVCMOS33} [get_ports {DATA[14]}] ;# CON3.33 B15_L5_N
set_property -dict {PACKAGE_PIN J22 IOSTANDARD LVCMOS33} [get_ports {DATA[15]}] ;# CON3.34 B15_L7_P
set_property -dict {PACKAGE_PIN H14 IOSTANDARD LVCMOS33} [get_ports {DATA[16]}] ;# CON3.35 B15_L3_N
set_property -dict {PACKAGE_PIN K21 IOSTANDARD LVCMOS33} [get_ports {DATA[17]}] ;# CON3.36 B15_L9_P
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {DATA[18]}] ;# CON3.37 B15_L3_P
set_property -dict {PACKAGE_PIN K22 IOSTANDARD LVCMOS33} [get_ports {DATA[19]}] ;# CON3.38 B15_L9_N
set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports {DATA[20]}] ;# CON3.41 B15_L19_P
set_property -dict {PACKAGE_PIN M22 IOSTANDARD LVCMOS33} [get_ports {DATA[21]}] ;# CON3.42 B15_L15_N
set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports {DATA[22]}] ;# CON3.43 B15_L19_N
set_property -dict {PACKAGE_PIN N22 IOSTANDARD LVCMOS33} [get_ports {DATA[23]}] ;# CON3.44 B15_L15_P
set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {DATA[24]}] ;# CON3.45 B15_L20_P
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS33} [get_ports {DATA[25]}] ;# CON3.46 B15_L6_N
set_property -dict {PACKAGE_PIN L13 IOSTANDARD LVCMOS33} [get_ports {DATA[26]}] ;# CON3.47 B15_L20_N
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {DATA[27]}] ;# CON3.48 B15_L6_P
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports {DATA[28]}] ;# CON3.51 B15_L14_P
set_property -dict {PACKAGE_PIN K19 IOSTANDARD LVCMOS33} [get_ports {DATA[29]}] ;# CON3.52 B15_L13_N
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33} [get_ports {DATA[30]}] ;# CON3.53 B15_L14_N
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS33} [get_ports {DATA[31]}] ;# CON3.54 B15_L13_P
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports {DATA[32]}] ;# CON3.55 B15_L21_P
set_property -dict {PACKAGE_PIN M21 IOSTANDARD LVCMOS33} [get_ports {DATA[33]}] ;# CON3.56 B15_L10_P
set_property -dict {PACKAGE_PIN J17 IOSTANDARD LVCMOS33} [get_ports {DATA[34]}] ;# CON3.57 B15_L21_N
set_property -dict {PACKAGE_PIN L21 IOSTANDARD LVCMOS33} [get_ports {DATA[35]}] ;# CON3.58 B15_L10_N
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {DATA[36]}] ;# CON3.61 B15_L23_P
set_property -dict {PACKAGE_PIN N20 IOSTANDARD LVCMOS33} [get_ports {DATA[37]}] ;# CON3.62 B15_L18_P
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {DATA[38]}] ;# CON3.63 B15_L23_N
set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {DATA[39]}] ;# CON3.64 B15_L18_N
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports {DATA[40]}] ;# CON3.65 B15_L22_P
set_property -dict {PACKAGE_PIN N19 IOSTANDARD LVCMOS33} [get_ports {DATA[41]}] ;# CON3.66 B15_L17_N
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports {DATA[42]}] ;# CON3.67 B15_L22_N
set_property -dict {PACKAGE_PIN N18 IOSTANDARD LVCMOS33} [get_ports {DATA[43]}] ;# CON3.68 B15_L17_P
set_property -dict {PACKAGE_PIN M15 IOSTANDARD LVCMOS33} [get_ports {DATA[44]}] ;# CON3.71 B15_L24_P
set_property -dict {PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports {DATA[45]}] ;# CON3.72 B15_L16_P
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports {DATA[46]}] ;# CON3.73 B15_L24_N
set_property -dict {PACKAGE_PIN L18 IOSTANDARD LVCMOS33} [get_ports {DATA[47]}] ;# CON3.74 B15_L16_N
