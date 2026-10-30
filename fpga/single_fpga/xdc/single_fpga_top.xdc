# Single-FPGA (ALINX AC7200, xc7a200tfbg484-2) pin constraints -- clocks,
# status LED, mic data lines and the GbE PHY.
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

# Mic data lines DATA_00..DATA_47 (front_end CON3, VCCO 3.3V), one bit per
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

# Sampler -> pipeline (rtl/single_fpga_top.v "Pipeline clock"). pipe_clk is TCXO_CLK
# through a BUFGCE, so STA times these as same-clock, single-cycle paths -- and the
# BUFG/BUFGCE skew (both fabric-fed from the non-clock-capable pin) then fails hold by
# ~0.1 ns. The real relationship is fixed by the logic, not by any parameter: pdm_l/pdm_r
# change only on the TCXO edge where pdm_div goes 0->1 (pdm_sampler's div_r == 3 branch;
# reset holds them at their initial 0), as does pipe_rst, the pipeline's reset (its
# pdm_div == 0 enable), and pipe_clk rises only where pdm_div goes 2->3. So
# each new value is captured two TCXO cycles after launch (setup -end 2), and the latest
# capture it could disturb is two cycles before launch (hold -end 3: the hold edge moves
# back from launch + 1 cycle, where setup 2 leaves it, to launch - 2). Cells are named
# explicitly: Vivado's -filter globs don't support [lr]-style classes. While nothing
# consumes the pipeline (no MAC yet) synthesis trims these cells, and -quiet doesn't stop
# synthesis noting "Empty from list" (Synth 8-3321) for the first line; that goes away
# once the pipeline drives the MAC.
set_multicycle_path -quiet -setup -end 2 -from [get_cells -quiet {u_pdm_sampler/pdm_l_reg[*] u_pdm_sampler/pdm_r_reg[*] pipe_rst_reg*}]
set_multicycle_path -quiet -hold  -end 3 -from [get_cells -quiet {u_pdm_sampler/pdm_l_reg[*] u_pdm_sampler/pdm_r_reg[*] pipe_rst_reg*}]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS33} [get_ports {DATA[0]}]  ;# CON3.3  B16_IO0
set_property -dict {PACKAGE_PIN G21 IOSTANDARD LVCMOS33} [get_ports {DATA[1]}]  ;# CON3.22 B16_L24_P
set_property -dict {PACKAGE_PIN M17 IOSTANDARD LVCMOS33} [get_ports {DATA[2]}]  ;# CON3.2  B15_IO25
set_property -dict {PACKAGE_PIN G22 IOSTANDARD LVCMOS33} [get_ports {DATA[3]}]  ;# CON3.24 B16_L24_N
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {DATA[4]}]  ;# CON3.1  B15_IO0
set_property -dict {PACKAGE_PIN D22 IOSTANDARD LVCMOS33} [get_ports {DATA[5]}]  ;# CON3.18 B16_L22_N
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

# ---------------------------------------------------------------------------------------
# GbE: AC7200 200 MHz SYS_CLK -> MMCM -> 125 MHz RGMII clocks, the DP83867 (front_end U3)
# on CON3 RGMII and CON1 PHY_RESET_N. Pins from the AC7200 manual's CON1/CON3 tables and a
# kicad-cli netlist of pcb/single_fpga/front_end.kicad_sch (RGMII RX moved to bank 15 so
# BUFIO/BUFR from RXC reaches it -- see pcb/SCHEMATIC_NOTES.md "GbE PHY, RGMII and Y1").
# ---------------------------------------------------------------------------------------

# SYS_CLK: SiT9102 200 MHz differential, bank 34 (1.5 V, the DDR3 bank).
set_property -dict {PACKAGE_PIN R4 IOSTANDARD DIFF_SSTL15} [get_ports SYS_CLK_P]
set_property -dict {PACKAGE_PIN T4 IOSTANDARD DIFF_SSTL15} [get_ports SYS_CLK_N]
create_clock -period 5.000 -name SYS_CLK [get_ports SYS_CLK_P]

# RGMII TX, bank 16 (3.3 V).
#   RGMII_TXC     CON3.14  B16_L23_N  D21
#   RGMII_TX_CTL  CON3.16  B16_L22_P  E22
#   RGMII_TXD0-3  CON3.12/8/6/4  B16_L23_P/L21_P/L21_N/IO25  E21/B21/A21/F21
# All on CON3's even (west) row, south to north TXD3, TXD2, TXD1, TXD0, TXC, TX_CTL --
# U3's own pin order, so the TX bundle routes without crossings.
set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports RGMII_TXC]
set_property -dict {PACKAGE_PIN E22 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports RGMII_TX_CTL]
set_property -dict {PACKAGE_PIN E21 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports {RGMII_TXD[0]}]
set_property -dict {PACKAGE_PIN B21 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports {RGMII_TXD[1]}]
set_property -dict {PACKAGE_PIN A21 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports {RGMII_TXD[2]}]
set_property -dict {PACKAGE_PIN F21 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports {RGMII_TXD[3]}]

# RGMII RX, bank 15 (3.3 V). RXC is on a clock-capable (MRCC) pin.
#   RGMII_RXC     CON3.15  B15_L12_P  J19
#   RGMII_RX_CTL  CON3.11  B15_L2_P   G15
#   RGMII_RXD0-3  CON3.7/5/13/17  B15_L4_N/L4_P/L2_N/L12_N  G18/G17/G16/H19
# All on CON3's odd (east) row, pins 5-17; RXC has to stay on CON3.15 (clock-capable).
set_property -dict {PACKAGE_PIN J19 IOSTANDARD LVCMOS33} [get_ports RGMII_RXC]
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33} [get_ports RGMII_RX_CTL]
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {RGMII_RXD[0]}]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports {RGMII_RXD[1]}]
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS33} [get_ports {RGMII_RXD[2]}]
set_property -dict {PACKAGE_PIN H19 IOSTANDARD LVCMOS33} [get_ports {RGMII_RXD[3]}]

# PHY_RESET_N  CON1.28  B13_L2_N  AB17. Held low ~10 ms after configuration, then high;
# the board's R9 10k pull-up holds it high while the FPGA is unconfigured.
set_property -dict {PACKAGE_PIN AB17 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports PHY_RESET_N]
set_false_path -to [get_ports PHY_RESET_N]

# MDC  CON1.26  B13_L2_P  AB16;  MDIO  CON1.24  B13_L1_N  AA16 (R8 2.2k pull-up on board).
# rtl/mdio_init.v: MDC = TCXO/8 = 1.536 MHz, MDIO changed on MDC's falling edge, so each
# bit this end drives has ~325 ns of setup and hold at the PHY (it needs 10 ns); in the
# read-back the PHY drives MDIO <= 10 ns after an MDC rise and it's sampled as MDC next
# rises, ~650 ns later. Untimed by design.
set_property -dict {PACKAGE_PIN AB16 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports MDC]
set_property -dict {PACKAGE_PIN AA16 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports MDIO]
set_false_path -to [get_ports {MDC MDIO}]
set_false_path -from [get_ports MDIO]

# RX timing (DP83867 RGMII timing, RX clock skew strap mode 1 = 2 ns internal delay):
# at the PHY's pins data is valid >= 1.2 ns before and after each RXC edge (TsetupT,
# TholdT with internal delay). Board skew within the RX group is a few mm (<0.05 ns).
# Center-aligned DDR input: max = T/2 - setup = 2.8 ns, min = hold = 1.2 ns, both edges.
# The BUFIO capture clock arrives ~1 ns late against this, so the RTL delays each RX input
# by a fixed IDELAYE2 (RX_IDELAY_TAPS = 11, ~0.86 ns), centring the window: ~0.45 ns
# setup and hold.
create_clock -period 8.000 -name RGMII_RXC [get_ports RGMII_RXC]
set_input_delay -clock RGMII_RXC -max 2.8 [get_ports {RGMII_RXD[*] RGMII_RX_CTL}]
set_input_delay -clock RGMII_RXC -min 1.2 [get_ports {RGMII_RXD[*] RGMII_RX_CTL}]
set_input_delay -clock RGMII_RXC -clock_fall -max 2.8 -add_delay [get_ports {RGMII_RXD[*] RGMII_RX_CTL}]
set_input_delay -clock RGMII_RXC -clock_fall -min 1.2 -add_delay [get_ports {RGMII_RXD[*] RGMII_RX_CTL}]

# TX timing (RGMII TX clock skew strap mode 1 = the PHY delays TXC by 2 ns internally):
# TXC leaves the FPGA edge-aligned with its data, both from ODDRs on gtx_clk, and must
# meet the RGMII transmitter spec, data within +-0.5 ns of each TXC edge (TskewT) --
# tighter than the PHY's own +-1.0 ns (TsetupR/TholdR with internal delay). Skew-based
# DDR output: each data edge is checked against the same-edge TXC transition
# (multicycle setup 0 / hold -1), max = -skew_after = -0.5, min = +skew_before = 0.5,
# and the opposite-edge pairs are false paths.
create_generated_clock -name RGMII_TXC -source [get_pins mmcm_gtx/CLKOUT0] -multiply_by 1 [get_ports RGMII_TXC]
set_output_delay -clock RGMII_TXC -max -0.5 [get_ports {RGMII_TXD[*] RGMII_TX_CTL}]
set_output_delay -clock RGMII_TXC -min  0.5 [get_ports {RGMII_TXD[*] RGMII_TX_CTL}]
set_output_delay -clock RGMII_TXC -clock_fall -max -0.5 -add_delay [get_ports {RGMII_TXD[*] RGMII_TX_CTL}]
set_output_delay -clock RGMII_TXC -clock_fall -min  0.5 -add_delay [get_ports {RGMII_TXD[*] RGMII_TX_CTL}]
set_multicycle_path -setup 0 -from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -to [get_clocks RGMII_TXC]
set_multicycle_path -hold -1 -from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -to [get_clocks RGMII_TXC]
set_false_path -setup -rise_from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -fall_to [get_clocks RGMII_TXC]
set_false_path -setup -fall_from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -rise_to [get_clocks RGMII_TXC]
set_false_path -hold  -rise_from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -fall_to [get_clocks RGMII_TXC]
set_false_path -hold  -fall_from [get_clocks -of_objects [get_pins mmcm_gtx/CLKOUT0]] -rise_to [get_clocks RGMII_TXC]

# The three clock sources are independent oscillators. Every crossing between them is
# handled in logic: the packetizer's toggle-synchronized ping-pong handoff (TCXO ->
# 125 MHz), status_led's pkt_toggle synchronizer (125 MHz -> TCXO), and the MAC's own
# RXC-rate speed detector (RXC -> 125 MHz).
# The vendored MAC's clock-crossing synchronizers, marked ASYNC_REG here rather than in
# third_party/ (kept unmodified): the speed detector's prescaler sync (RXC -> gtx_clk),
# the MII-select sync and the RX reset synchronizer (gtx_clk -> RXC).
set_property -quiet ASYNC_REG TRUE [get_cells -quiet {u_mac/rx_prescale_sync_reg[*] u_mac/rx_mii_select_sync_reg[*] u_mac/rgmii_phy_if_inst/rx_rst_reg_reg[*]}]
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks TCXO_CLK] \
    -group [get_clocks -include_generated_clocks SYS_CLK] \
    -group [get_clocks RGMII_RXC]
