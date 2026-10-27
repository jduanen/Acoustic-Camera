# Phase 4 Mic Array — Schematic Notes

Reflects the current `pcb/single_fpga/` design (single-FPGA is the primary
design track; `pcb/multi_fpga/` is kept as the proven alternate). The
project outgrew a single "Nexys Video + FMC" board and is now a **two-board
system**: the FPGA lives on its own board, the mic array is a second,
physically separate board, and the two connect with a single high-density
board-to-board connector.

---

## Architecture — Two Boards

| Board | KiCad project | Carries |
|---|---|---|
| **front_end** | `front_end.kicad_pcb` / `front_end.kicad_sch` | ALINX AC7200 FPGA SOM, DP83867 GbE PHY, +5V input and all regulation (+3V6, +3V3_ETH, +2V5_ETH, +1V0_ETH), inter-board connector |
| **mic_array** | `mic_array.kicad_pcb` / `mic_array.kicad_sch` | 96 mics, master clock (TCXO) + full clock distribution tree, local 3.3V regulation (MIC_3V3, CLK_3V3) from +3V6, inter-board connector |

`top.kicad_sch` is the hierarchical parent that ties "Front End" and
"Microphone Array" together as sub-sheets, for whole-project ERC/BOM
purposes — it is **not** a third physical board (`top.kicad_pcb` has no
routing of its own beyond what's mirrored from the two real boards).

This replaces the earlier plan of driving the mic array from an external
FPGA dev board (Nexys A7-200T / Nexys Video) over an FMC LPC connector.
There is no external dev board and no FMC connector in the current design —
the FPGA (Xilinx XC7A200T-2FBG484I) is the ALINX AC7200 SOM, mounted
directly on the front_end board.

### Hierarchical sheets

```
top.kicad_sch
├── mic_array.kicad_sch        ("Microphone Array")
│   ├── mic_arm_00.kicad_sch   … mic_arm_11.kicad_sch   (8 mics + 1 clock buffer each)
│   └── array_connector.kicad_sch   ("Array Inter-board Connector")
└── front_end.kicad_sch        ("Front End")
    ├── ac7200_conn1.kicad_sch … ac7200_conn4.kicad_sch (CON1–CON4, the AC7200's carrier headers; J4 JTAG on conn3)
    ├── gbe.kicad_sch          ("Ethernet Interface": DP83867 PHY, U6 buck, U4/U5 LDOs)
    └── (sheet-local: +5V input J1, U1 buck, J2)
```

`ac7200_conn1–4` each place one real part: CON1–CON4, the four Panasonic
AXK680337YG headers on the carrier that the AC7200's sockets plug into
(symbols `single_fpga:AC7200_CON1`–`AC7200_CON4`, pins named with the module's
signals, numbered 1–80). The module itself is not in the schematic: on the PCB
it is U2, the board-only mechanical footprint `single_fpga:AC7200_Module`
(outline, courtyard, 4 mounting holes), excluded from the BOM and position files.

---

## Reference Designators

| Range | Board | Parts |
|---|---|---|
| U1–U96 | mic_array | IM72D128 mics, 8 per arm (arm *n* = U(8n+1)–U(8n+8)) |
| U97–U108 | mic_array | CDCLVC1108 stage-2 clock buffers, one per arm (arm *n* = U(97+n)) |
| C1–C96 | mic_array | 100 nF decoupling, one per mic VDD pin |
| C105–C116 | mic_array | 100 nF decoupling, one per arm's CDCLVC1108 |
| C117–C137 | mic_array | 100 nF 0603 GND–MIC_3V3 clock return-path stitching caps on F.Cu (see PDM Clock Distribution) |
| C138–C156 | mic_array | 100 nF 0603 extra CLK_3V3 decoupling for U110 and the arm CDCLVC1108s (in `misc.kicad_sch`; see PDM Clock Distribution) |
| C97 | mic_array | 100 nF CLK_3V3 decoupling (TCXO / CDCLVC1112) |
| C100, C101 | mic_array | 1 µF output caps for VR1 (MIC_3V3) and VR2 (CLK_3V3) |
| R1–R12 | mic_array | 10 kΩ pull-ups on each arm's CDCLVC1108 output-enable pin (arm *n* = R(n+1)) |
| R13 | mic_array | 10 kΩ pull-up on CDCLVC1112's output-enable pin |
| R15 | mic_array | 33 Ω source-series resistor on Y1's output (TCXO_CLK to J1) |
| TP1–TP4 | mic_array | Test points: GND, MIC_3V3, +3V6, CLK_3V3 (TCXO_CLK is probed at front_end TP7) |
| Y1 | mic_array | 12.288 MHz TCXO |
| U110 | mic_array | CDCLVC1112, stage-1 (1:12) PDM clock buffer |
| VR1, VR2 | mic_array | MCP1700T-3302E 3.3V LDOs (+3V6 in) — separate MIC_3V3 and CLK_3V3 rails |
| C98, C99 | mic_array | 1 µF input caps for VR1, VR2 |
| J1 | mic_array | Inter-board connector (Panasonic AXK6S80637YG header) |
| CON1–CON4 | front_end | Panasonic AXK680337YG headers that the AC7200 SOM plugs into |
| U2 | front_end (PCB only) | AC7200 SOM mechanical footprint (not in the schematic or BOM) |
| J3 | front_end | HanRun HR911130A RJ45 magjack (GbE) |
| J4 | front_end | 2×7 2 mm JTAG header |
| Y1 | front_end | 25 MHz crystal for the DP83867, `Device:Crystal_GND24` on a 3225 4-pad footprint (part still to be chosen; see "GbE PHY, RGMII and Y1") — unrelated to mic_array's Y1 |
| C12, C13 | front_end | 18 pF Y1 load caps (C12 on XO, C13 on XI) |
| J1 | front_end | +5V/GND main power input (front_end's J1; unrelated to mic_array's J1) |
| C3 | front_end | 100 µF / 25 V aluminium electrolytic bulk on +5V (CP_Elec_6.3x5.8) |
| U1 | front_end | TLV62569DBV buck, +5V → +3V6 |
| L1, R1, R2 | front_end | U1's 2.2 µH inductor and 499 kΩ / 100 kΩ feedback divider |
| C1, C2 | front_end | U1 input (4.7 µF) and output (10 µF) caps |
| U6 | front_end | TLV62569DBV buck, +5V → +3V3_ETH |
| L2, R13, R14 | front_end | U6's 2.2 µH inductor and 453 kΩ / 100 kΩ feedback divider |
| C30, C31 | front_end | U6 input (4.7 µF) and output (10 µF) caps |
| U4 | front_end | TLV75525PDBV LDO, +3V3_ETH → +2V5_ETH (EN tied to IN) |
| U5 | front_end | TLV75510PDRV LDO (WSON-6), +3V3_ETH → +1V0_ETH (EN tied to IN) |
| C24, C25 | front_end | 10 µF input caps for U4, U5 |
| U3 | front_end | DP83867IRRGZ GbE PHY (loads +3V3_ETH, +2V5_ETH, +1V0_ETH); 4.1 mm exposed pad has 17 GND vias (4×4 at 1.2 mm + centre, 0.6/0.3 mm) |
| TP1–TP6 | front_end | Test points: +5V, GND, +3V6, +3V3_ETH, +2V5_ETH, +1V0_ETH |
| TP7, TP8 | front_end | Clock test points: TCXO_CLK (2.6 mm stub, near CON1.21) and PDM_CLK (3.7 mm stub, near CON1.25) |
| TP9, TP10 | front_end | GND test points; TP9 is the probe ground for TP7/TP8 (5–6 mm away) |
| D1 | front_end | Broadcom ASMB-MTB0-0A3A2 common-anode RGB status LED (PLCC-4), anode on +5V; see "Status LED" |
| Q1–Q3 | front_end | 2N7002 low-side switches for D1's red, green, blue cathodes (gates = LED_R/G/B) |
| R16–R18 | front_end | D1 cathode current limits (0603): 620 Ω (red), 390 Ω (green), 390 Ω (blue), ≈5 mA each |
| R19–R21 | front_end | 100 kΩ gate pull-downs (0603) on Q1–Q3 (LED off until the FPGA drives it) |
| J2 | front_end | Inter-board connector (AXK5S80337YG socket, mates J1) |

Mic numbering is a clean U1–U96 block (mic index + 1); the twelve per-arm
CDCLVC1108 buffers follow as U97–U108. The two boards are annotated
independently, so some references (J1, R1–R13, C1–C31, U1–U6, TP1–TP4, Y1)
exist on both boards as different parts.

---

## FPGA: ALINX AC7200 SOM

Xilinx **XC7A200T-2FBG484I**, on ALINX's AC7200 module. Pinout and
mechanical placement are cross-checked against ALINX's real manual and
schematic (see [`pcb/libraries/AC7200/AC7200_README.md`](libraries/AC7200/AC7200_README.md)
— 320/320 connector pins verified, zero discrepancies). The module mounts
on the front_end board through 4× 80-pin, 0.5mm-pitch board-to-board
connectors: the module carries AXK580137YG sockets, and the carrier has the
mating AXK680337YG headers CON1–CON4 (P5K series, 3.0mm mated height, with
positioning bosses in 0.8mm NPTH holes). Each header's pin 1 is on its inner row,
at the end marked by ALINX's pin-1 dot.

This fully supersedes the old Nexys A7-200T / Nexys Video I/O-budget
discussion below (kept for historical context only — see
"Superseded: External Dev-Board Connector" at the bottom of this file).

---

## Power

```
+5V  (front_end J1 "Main Power Input"; C3 100 µF/25 V bulk)
 ├─ AC7200 SOM VCCIN (CON1 pins 1–8)
 ├─ U1 TLV62569 buck (L1 2.2 µH, R1/R2 = 499k/100k) → +3V6  (≈3.59 V)
 │    └─ J2 pins 1–8 ══► mic_array J1 pins 1–8
 │         ├─ VR1 MCP1700-3302E → MIC_3V3  (96 IM72D128 mics)
 │         └─ VR2 MCP1700-3302E → CLK_3V3  (TCXO Y1, U110 CDCLVC1112,
 │                                          12× CDCLVC1108, OE pull-ups)
 └─ U6 TLV62569 buck (L2 2.2 µH, R13/R14 = 453k/100k) → +3V3_ETH  (≈3.32 V)
      ├─ U3 DP83867 VDDIO (pins 19/30/41), MDIO pull-ups, PHY straps, J4 pin 2
      ├─ U4 TLV75525 LDO → +2V5_ETH  (U3 VDDA2P5)
      └─ U5 TLV75510 LDO → +1V0_ETH  (U3 VDD1P0)
```

Buck setpoints follow the TLV62569's 0.6 V reference: Vout = 0.6 V × (1 + Rtop/Rbottom).

- **+3V6** exists only to feed the mic board's two MCP1700 3.3 V LDOs with headroom
  above their dropout; nothing else uses it. MIC_3V3 and CLK_3V3 are separate LDO
  outputs from it, isolating the mic supply from clock-buffer switching noise.
- **Only +3V6 and GND cross the inter-board connector** (J2/J1 pins 1–8 carry
  +3V6). No +5V or regulated 3.3 V rail is shared between boards.
- **+2V5_ETH** uses a TLV75525 (≈0.2–0.3 V dropout) because an AMS1117-2.5 needs
  ≈1.0–1.1 V of headroom and would sag below the DP83867's VDDA2P5 minimum when
  fed from 3.3 V.
- The DP83867's VDDA1P8 pins (13, 48) are unconnected.
- **PWR_FLAGs:** front_end on +5V, GND, +3V6 (the bucks' power-output pin is SW,
  so their post-inductor rails need a flag) and +3V3_ETH (in `gbe.kicad_sch`);
  mic_array on +3V6 and GND (in `array_connector.kicad_sch`), since they arrive
  through J1. LDO outputs need no flag (their output pins are power outputs). In
  `top.kicad_sch` the two boards' +3V6/GND flags meet, so top-level ERC reports two
  power-output-to-power-output errors; these are expected and can be excluded.
- Power symbols must use a **power input** pin: a power symbol with a power-output
  pin does not join its global net in KiCad 10 (this disconnected +3V3_ETH once).

---

## Data Line Assignment

`test/phase4/data_line_assignment.csv` has the full table; unchanged in
concept from the original plan — 48 DATA lines, 2 mics per line:

| DATA line | Mic L (SEL=GND) | Mic R (SEL=MIC_3V3) | Pair distance |
|---|---|---|---|
| DATA_00 | mic 0, arm 0 | mic 1, arm 0 | ~19 mm |
| DATA_01 | mic 2, arm 0 | mic 3, arm 0 | ~19 mm |
| ... | ... | ... | ... |
| DATA_47 | mic 94, arm 11 | mic 95, arm 11 | ~19 mm |

Rule: within each pair, the mic with the lower index has SEL=GND (L), the
higher index has SEL=MIC_3V3 (R). Within each arm, pairs are (0,1), (2,3),
(4,5), (6,7). Data line index = arm × 4 + pair_within_arm. Confirmed
against the actual schematic wiring (MIC_3V3/GND tie counts per arm match
this pairing exactly).

---

## IM72D128 Pin Assignment — corrected from earlier draft

The pin order below is taken directly from the `multi_fpga:IM72D128`
symbol actually used in `mic_arm_00–11.kicad_sch` (the original draft had
pins 1/2 and 3/4 swapped):

| Pin | Name | Connect to |
|---|---|---|
| 1 | DATA | DATA_nn net |
| 2 | VDD | MIC_3V3 + 100 nF to GND |
| 3 | CLK | MIC_CLK_nn (buffered, one net per mic) |
| 4 | SEL | GND (L mic) or MIC_3V3 (R mic) |
| 5 | GND | GND plane |

---

## PDM Clock Distribution — two-stage buffer tree

The reference oscillator sits **on the mic_array board**, but the FPGA
generates the mic clock: Y1's 12.288 MHz goes across the inter-board
connector to the FPGA (CON1), the FPGA divides it by 4 to a 50 %-duty
3.072 MHz `PDM_CLK`, and `PDM_CLK` comes back across the connector to the
buffer tree. 12.288 MHz must never reach the mics directly — the IM72D128's
maximum clock is 5.04 MHz (2.9–3.3 MHz in its normal mode). Because the FPGA
launches `PDM_CLK` itself, it captures DATA in the same clock domain.

Both clocks use CON1 so CON2 and CON4 can stay unpopulated. CON1 has no free
clock-capable (L11–L14) 3.3 V pin, so Y13 is a general I/O. The XDC therefore needs
`set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets <TCXO_CLK IBUF net>]` to let it
reach a BUFG/MMCM through fabric routing. The extra few hundred picoseconds of delay and
jitter are irrelevant at 12.288 MHz in / 3.072 MHz out. Drive `PDM_CLK` (AB11) at
DRIVE 12, SLEW SLOW: U110 needs >= 1 V/ns input slew into ~15-17 pF of unterminated
line, and the 12 mA output impedance roughly matches the ~40-45 ohm trace.
These constraints are in
[`fpga/single_fpga/xdc/single_fpga_top.xdc`](../fpga/single_fpga/xdc/single_fpga_top.xdc).

```
Y1  ECS-TXO-5032-122.8, 12.288 MHz TCXO, ±2.5 ppm  (mic_array, CLK_3V3)
 │   R15 33 Ω series → TCXO_CLK → J1/J2 pin 41 (probe at front_end TP7)
 │                    → CON1 pin 21, B13_L5_P, ball Y13 (3.3 V input)
 │
 FPGA ÷4 → PDM_CLK 3.072 MHz → CON1 pin 25, B13_L7_P, ball AB11 (3.3 V output; TP8)
 │                    → J2/J1 pin 45 (pin 43 = GND between the two clocks)
 │
 ├─ Stage 1: U110 CDCLVC1112 (1:12 LVCMOS buffer)
 │     PDM_CLK → ARM_00_CLK … ARM_11_CLK   (one output per arm)
 │
 └─ Stage 2: one CDCLVC1108 (1:8 LVCMOS buffer) per arm sheet
       (U97 in mic_arm_00, U98 in mic_arm_01, … U108 in mic_arm_11)
       ARM_nn_CLK → MIC_CLK_(8·nn) … MIC_CLK_(8·nn+7)
```

Each mic gets its own buffered clock net, globally numbered
`MIC_CLK_00`–`MIC_CLK_95` (index = arm × 8 + position-in-arm = mic_idx),
so there's no per-sheet net-name collision risk despite every arm sheet
being a copy of the same template. Both buffer ICs have their
output-enable pin pulled up locally (R13 for U110, R1–R12 for the arms'
CDCLVC1108s) rather than driven from the FPGA.

**Clock trace lengths.** Matching isn't functionally required at 3.072 MHz: the
mics' own DATA-valid delay varies 40–100 ns part to part, the two buffer stages allow
0.5 ns part-to-part skew each, and 1 ns of skew is ≈0.3 µm of acoustic path (0.006° at
20 kHz). The tree was still balanced to tidy the skew. Lengths include vias at 1.545 mm
each (F.Cu↔B.Cu copper-centre span of this stackup):

| U110 → mic path | Before | As routed now |
|---|---|---|
| Range | 110.0–234.9 mm | 195.0–234.9 mm |
| Spread | 124.9 mm (~0.77 ns) | 40.0 mm (~0.25 ns) |
| Std dev | 27.5 mm | 13.1 mm |

How: every ARM_nn_CLK except ARM_08 was lengthened so each arm's longest U110 → mic path
is ~234.9 mm (ARM_08 sets the target), then the 17 shortest MIC_CLK_nn paths were
lengthened to a 195 mm floor. Further reduction costs ~7+ traces and ~100+ mm of meander
per 5 mm of spread, so it stops here. Feed nets (both boards, excluding J1/J2):
TCXO_CLK ≈56 mm, PDM_CLK ≈80 mm.

**Return path at clock layer changes.** The mic_array stackup is F.Cu / In1 GND /
In2 MIC_3V3 / B.Cu, so a clock on B.Cu returns on the MIC_3V3 plane, not GND. At each
F.Cu↔B.Cu clock via the return current has to cross between the two planes, which a GND
via can't provide; a GND–MIC_3V3 capacitor next to the via does. C117–C137 (100 nF 0603,
F.Cu, so the board stays single-sided) are placed 1.0–2.6 mm from the 21 clock vias
that previously had no GND via or GND–MIC_3V3 cap within 10 mm, each pad dropping to its
plane through its own via. Prefer
keeping clocks on F.Cu when re-routing; any new layer change should get a cap like these.


**Buffer decoupling.** Each CDCLVC11xx VDD pin should have a cap within ~1–2 mm, but on
F.Cu the area outside the VDD pins is taken by the fanning-out clock outputs. C138–C156
(19 × 100 nF, F.Cu, in `misc.kicad_sch`) are tied into the CLK_3V3 trace that runs under
each buffer and bring every VDD pin of U110 and of U97–U102, U104 and U106–U108 to within
2.0–5.8 mm of a cap (previously up to 9–13 mm). U103 and U105 had no room and still have
pins 10/14 about 9 mm from C108/C110. Opening space around those two buffers, or adding
caps on B.Cu under them, is the remaining improvement.

---

## GbE PHY, RGMII and Y1 (front_end)

**RGMII pin assignment.** The whole TX group is in FPGA bank 16 on CON3: TXC on CON3.14
(B16_L23_N) and TX_CTL on CON3.12 (B16_L23_P, the other half of TXC's pin pair), with
TXD0–TXD3. TX_CTL was originally on CON1.22 (bank 13); that pin is now unused. RXC is on
CON3.15 (B15_L12_P, MRCC), RX_CTL on CON3.2 (B15_IO25, same bank as RXC), RXD0–RXD3 in
bank 16.

**RGMII lengths** (front_end only; the AC7200 module's internal routing is unknown and
adds to these):

| Group | Clock | Worst data/ctl vs clock |
|---|---|---|
| TX | TXC 54.6 mm | 1.8 mm (~11 ps), TXD0 |
| RX | RXC 37.1 mm | 4.4 mm (~27 ps), RXD1 |

**U3 exposed pad.** 17 GND vias (4×4 at 1.2 mm + centre, 0.6/0.3 mm) are marked
**filled and capped** (IPC-4761 Type VII) in their via properties; every other via uses
the board default (tented both sides). ODB++ export carries this as
`filling_f.cu-b.cu` / `capping_f.cu-b.cu` layers (17 vias each). Gerber + Excellon do
**not** — for a Gerber order, send the fab note that sits on User.Drawings below the
board outline, or put it in the order comments. User.Drawings isn't in the board's saved
Gerber plot layers yet; tick it in File → Plot and save the board to include it.

**Y1 crystal circuit.** Y1 uses `Device:Crystal_GND24` on
`Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm`: crystal on pads **1 (XO) and 3 (XI)**
(diagonal), pads **2 and 4 to GND** (each with a GND via, 1.05 mm and 1.8 mm away). It
originally used the 2-pin `Device:Crystal`, which put XI on pad 2 — a case/ground pad on
standard 3225 crystals — so it would not have oscillated. Y1 sits 5.4–5.8 mm from U3 pins
14/15; XI is 9.35 mm and XO 10.02 mm, F.Cu, no vias; C12 is 1.6 mm and C13 3.3 mm from
their Y1 pads.

**Y1 part requirements** (no part number has been chosen yet; DP83867 datasheet
Table 9-3 / §9.2.1.3):

| Parameter | Requirement |
|---|---|
| Frequency | 25 MHz, fundamental, parallel resonant |
| Frequency tolerance over operating temperature | ±50 ppm max (±20–30 ppm parts recommended for margin) |
| Frequency stability, 1-year aging | ±50 ppm max |
| Package / pinout | 3.2 × 2.5 mm, 4 pads, crystal between pads 1 and 3, pads 2/4 GND or case — confirm against the chosen part's datasheet |
| Load capacitance (CL) | Match to C12/C13: **12 pF** with the fitted 18 pF caps (≈18/2 + ~3 pF stray), or TI's recommended **18 pF** with C12/C13 changed to ~27 pF |
| Temperature range | Cover the board's operating range; the tolerance above must hold over it |

Typical families that fit: Abracon ABM8, NDK NX3225SA, ECS-250-…-CKM. When chosen, add
its manufacturer part number to Y1 in `gbe.kicad_sch` and set C12/C13 to its recommended
load caps.

---

## Status LED (front_end)

D1 is an RGB LED the FPGA uses to show operating state. It sits on F.Cu beside the RJ45
(J3), just inside the bottom edge, with Q1–Q3 and R16–R21 in a row to its right. The
three drive nets come from free bank-13 (3.3 V) pins on CON1:

| Net | CON1 pin | Bank pin | FPGA ball |
|---|---|---|---|
| LED_R | 32 | B13_L6_P | W14 |
| LED_G | 33 | B13_L3_N | AB13 |
| LED_B | 34 | B13_L6_N | Y14 |

- **Drive:** common anode on +5V; each cathode goes through its current-limit resistor
  (R16–R18) to a 2N7002 (Q1–Q3) that the FPGA pin switches. A 3.3 V pin can't fully
  forward-bias the ≈3.0 V green and blue dies, so they are switched from +5V instead;
  +5V never reaches an FPGA pin.
- **Polarity:** active high — driving LED_x to 1 turns that colour on. The 100 kΩ
  pull-downs (R19–R21) keep the LED off while the FPGA is unpowered or unconfigured.
- **Current:** ≈5 mA per colour (red Vf ≈2.0 V → 620 Ω; green/blue Vf ≈3.0 V → 390 Ω).
  Adjust R16–R18 for brightness or colour balance; the FET's few-ohm Rds(on) at 3.3 V
  gate drive is negligible.
- **FPGA constraints:** `fpga/single_fpga/xdc/single_fpga_top.xdc`, LVCMOS33, DRIVE 4,
  SLEW SLOW (gate load only). The AC7200's own user LED (ball W5, bank 34) is separate.

---

## Inter-board Connector

| Side | Board | Part |
|---|---|---|
| J1 | mic_array | Panasonic AXK6S80637YG (P5KS 80-pin header, 0.5mm pitch, 9.0mm mated height, positioning bosses) |
| J2 | front_end | Panasonic AXK5S80337YG (mating P5KS socket, positioning bosses) |

Carries all 48 DATA_nn nets plus +3V6 (pins 1–8), GND (16 pins, plus pin 43), and the two
clocks: TCXO_CLK (pin 41, mic_array → FPGA) and PDM_CLK (pin 45, FPGA → mic_array) — see
"PDM Clock Distribution" above. Pins 42 and 44–48 are unused.

---

## Layer Stack — implemented

Both boards are 4-layer:

```
            mic_array                          front_end
F.Cu    signal: mic pads, DATA,            signal
        per-arm clock fan-out
In1.Cu  GND plane                          GND plane
In2.Cu  MIC_3V3 plane                      +5V plane
B.Cu    signal: longer DATA runs,          signal (J2 is mounted on B.Cu)
        connector fan-out
```

---

## Board Outline

- `mic_array.kicad_pcb`: circular, **Ø320 mm**, with a Ø12 mm centre cut-out.
- `front_end.kicad_pcb`: **120 mm × 120 mm** rectangle.

---

## Status (as of the schematics/layouts on disk)

- **mic_array**: routed, ERC- and DRC-clean (0 violations, 0 unconnected; the only parity
  entries are the 8 board-only mounting holes, excluded). Clock tree balanced as above.
- **front_end**: routed, ERC- and DRC-clean (0 violations, 0 unconnected, 0 parity),
  including the status LED (LED_R/G/B ≈114/99/93 mm from CON1.32–34, two vias each).
- **top.kicad_sch**: only the two expected PWR_FLAG power-output-to-power-output errors.
- **Before fab:**
  - choose Y1's part and add its MPN (requirements above); set C12/C13 to suit its CL;
  - add User.Drawings to the front_end Gerber plot layers (or send ODB++) so the U3
    filled/capped-via note reaches the fab;
  - confirm the J1/J2 boss-hole orientation relative to pin 1 against Panasonic's
    product specification drawing (see `cad/assembly/README.md`).

---

## Superseded: External Dev-Board Connector (historical, no longer used)

The project originally planned to drive the mic array from an external
FPGA dev board rather than a custom FPGA carrier:

- Nexys A7-200T exposes only ~32 user I/O (4× Pmod) — 17 short of the 49
  needed for 96 mics (48 DATA + 1 CLK).
- Nexys Video (same XC7A200T, FMC LPC connector, 68 SE I/O) was the
  recommended fix, connecting via a Samtec/Molex FMC LPC plug.
- A 32-channel bring-up option using 3 Pmods on the Nexys A7-200T was also
  scoped.

None of this applies to the current design — the ALINX AC7200 SOM
provides the FPGA directly on the front_end board, and the array connects
to it via the AXK5S80337YG/AXK6S80637YG board-to-board connector described above.
