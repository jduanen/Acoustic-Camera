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
| U97–U108 | mic_array | 1:8 stage-2 clock buffers, one per arm (arm *n* = U(97+n)): fitted as TI LMK1C1108PWR (LCSC C2868241), the pin-compatible successor to the CDCLVC1108 the symbols are named for (1.71–3.465 V supply; CLK_3V3 is 3.3 V) |
| C1–C96 | mic_array | 100 nF decoupling, one per mic VDD pin |
| C105–C116 | mic_array | 100 nF decoupling, one per arm's CDCLVC1108 |
| C117–C137 | mic_array | 100 nF 0603 GND–MIC_3V3 clock return-path stitching caps on F.Cu (see PDM Clock Distribution) |
| C138–C156 | mic_array | 100 nF 0603 extra CLK_3V3 decoupling for U110 and the arm CDCLVC1108s (in `misc.kicad_sch`; see PDM Clock Distribution) |
| C184, C185 | mic_array | 100 nF 0402 extra CLK_3V3 decoupling for U103 / U105, on each buffer's CLK_3V3 feed (in `mic_array_misc.kicad_sch`; see Buffer decoupling) |
| C157–C183 | mic_array | 100 nF 0603 GND–MIC_3V3 return-path stitching caps on F.Cu, second pass: clock vias >10 mm from a cap and the J1 area (in `mic_array_misc.kicad_sch`; see PDM Clock Distribution) |
| C97 | mic_array | 100 nF CLK_3V3 decoupling (TCXO / CDCLVC1112) |
| C100, C101 | mic_array | 1 µF output caps for VR1 (MIC_3V3) and VR2 (CLK_3V3) |
| R1–R12 | mic_array | 10 kΩ pull-ups on each arm's CDCLVC1108 output-enable pin (arm *n* = R(n+1)) |
| R13 | mic_array | 10 kΩ pull-up on CDCLVC1112's output-enable pin |
| R15 | mic_array | 33 Ω source-series resistor on Y1's output (TCXO_CLK to J1) |
| TP1–TP4 | mic_array | Test points: GND, MIC_3V3, +3V6, CLK_3V3 (TCXO_CLK is probed at front_end TP7) |
| Y1 | mic_array | 12.288 MHz master oscillator: Abracon ASFL1-12.288MHZ-LCS-T (LCSC C276419), 3.3 V HCMOS, ±50 ppm, 5.0 × 3.2 mm; pin 1 tri-state, left open (= running). Replaced the ECS-TXO-5032 ±2.5 ppm TCXO, which LCSC doesn't stock: every mic shares this clock, so ±50 ppm only shifts the absolute sample rate (≈2.4 Hz at 48 kHz) and doesn't affect beamforming |
| U110 | mic_array | CDCLVC1112, stage-1 (1:12) PDM clock buffer: TI CDCLVC1112PWR (LCSC C2876940) |
| VR1, VR2 | mic_array | MCP1700T-3302E 3.3V LDOs (+3V6 in) — separate MIC_3V3 and CLK_3V3 rails |
| C98, C99 | mic_array | 1 µF input caps for VR1, VR2 |
| J1 | mic_array | Inter-board connector (Panasonic AXK6S80637YG header) |
| CON1–CON4 | front_end | Panasonic AXK680337YG headers that the AC7200 SOM plugs into |
| U2 | front_end (PCB only) | AC7200 SOM mechanical footprint (not in the schematic or BOM) |
| J3 | front_end | HanRun HR911130A RJ45 magjack (GbE) |
| J4 | front_end | 2×7 2 mm JTAG header |
| Y1 | front_end | 25 MHz crystal for the DP83867, `Device:Crystal_GND24` on a 3225 4-pad footprint (part still to be chosen; see "GbE PHY, RGMII and Y1") — unrelated to mic_array's Y1 |
| C12, C13 | front_end | 15 pF Y1 load caps (C12 on XO, C13 on XI) |
| J1 | front_end | +5V/GND main power input (front_end's J1; unrelated to mic_array's J1) |
| Q4 | front_end | AO3401A P-MOSFET (SOT-23, LCSC C15127) reverse-polarity protection on the +5V input: drain to J1 pin 1 (net +5V_IN), source to +5V, gate to GND |
| C3 | front_end | 100 µF / 25 V aluminium electrolytic bulk on +5V (CP_Elec_6.3x5.8) |
| U1 | front_end | TLV62569DBV buck, +5V → +3V6 |
| L1, R1, R2 | front_end | U1's 2.2 µH inductor and 499 kΩ / 100 kΩ feedback divider |
| C1, C2 | front_end | U1 input (4.7 µF) and output (10 µF) caps |
| U6 | front_end | TLV62569DBV buck, +5V → +3V3_ETH |
| L2, R13, R14 | front_end | U6's 2.2 µH inductor and 453 kΩ / 100 kΩ feedback divider |
| C30, C31 | front_end | U6 input (4.7 µF) and output (10 µF) caps |
| U4 | front_end | TLV74325PDBV LDO (SOT-23-5, 300 mA, LCSC C485154; same pinout as the earlier TLV75525PDBV), +3V3_ETH → +2V5_ETH (EN tied to IN) |
| U5 | front_end | TLV75710PDRV LDO (WSON-6, 1 A, LCSC C2861391; same pinout as the earlier TLV75510PDRV), +3V3_ETH → +1V0_ETH (EN tied to IN) |
| C24, C25 | front_end | 10 µF input caps for U4, U5 |
| C4–C11, C14–C23 | front_end | U3 decoupling: a 1 µF + 0.1 µF pair per supply pin (TI requirement), the 0.1 µF placed closest to its pin |
| C32 | front_end | 0.1 µF from J3 pin 1 (net ETH_CT, the magjack's common PHY-side centre tap) to GND, per the DP83867 datasheet |
| C33, C34 | front_end | 22 µF 0805 + 0.1 µF local decoupling for the AC7200 +5V input (CON1 pins 1–8), just outside the module outline |
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
independently, so some references (J1, R1–R13, C1–C34, U1–U6, TP1–TP4, Y1)
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
+5V  (front_end J1 "Main Power Input" → Q4 reverse-polarity P-FET; C3 100 µF/25 V bulk)
 ├─ AC7200 SOM VCCIN (CON1 pins 1–8)
 ├─ U1 TLV62569 buck (L1 2.2 µH, R1/R2 = 499k/100k) → +3V6  (≈3.59 V)
 │    └─ J2 pins 1–8 ══► mic_array J1 pins 1–8
 │         ├─ VR1 MCP1700-3302E → MIC_3V3  (96 IM72D128 mics)
 │         └─ VR2 MCP1700-3302E → CLK_3V3  (TCXO Y1, U110 CDCLVC1112,
 │                                          12× CDCLVC1108, OE pull-ups)
 └─ U6 TLV62569 buck (L2 2.2 µH, R13/R14 = 453k/100k) → +3V3_ETH  (≈3.32 V)
      ├─ U3 DP83867 VDDIO (pins 19/30/41), MDIO pull-ups, PHY straps, J4 pin 2
      ├─ U4 TLV74325 LDO → +2V5_ETH  (U3 VDDA2P5)
      └─ U5 TLV75710 LDO → +1V0_ETH  (U3 VDD1P0)
```

**Reverse-polarity protection.** Q4 (AO3401A, P-channel, −30 V/−4 A, ≈60 mΩ at Vgs = −4.5 V)
sits in series between J1 pin 1 (+5V_IN) and +5V, ahead of C3, so a reversed cable can't reach the
bulk cap, the bucks or the AC7200. With correct polarity the body diode conducts first, pulling the
source up; Vgs ≈ −5 V then turns the channel fully on. Reversed, the body diode blocks and Vgs ≈ 0.
Load estimate: the AC7200 ≈ 0.3–0.4 A at 5 V (Vivado report_power on the routed design: 0.45 W
on-chip, 0.19 A Vccint, 0.10 A Vccaux; plus DDR3, flash and converter losses) and the front_end's own
rails ≈ 0.3 A, so ≈ 0.6–0.7 A total, i.e. ≈ 30 mV and ≈ 25 mW in Q4. On the board, Q4 is just left of
J1: a 0.8 mm F.Cu track from J1 pin 1 to the drain, the source dropping into the In2 +5V island
through two vias, the gate to a GND via.

Buck setpoints follow the TLV62569's 0.6 V reference: Vout = 0.6 V × (1 + Rtop/Rbottom).

- **+3V6** exists only to feed the mic board's two MCP1700 3.3 V LDOs with headroom
  above their dropout; nothing else uses it. MIC_3V3 and CLK_3V3 are separate LDO
  outputs from it, isolating the mic supply from clock-buffer switching noise.
- **Only +3V6 and GND cross the inter-board connector** (J2/J1 pins 1–8 carry
  +3V6). No +5V or regulated 3.3 V rail is shared between boards.
- **+2V5_ETH** uses a TLV74325 (≈0.2–0.3 V dropout) because an AMS1117-2.5 needs
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
Y1  ASFL1-12.288MHZ-LCS-T, 12.288 MHz HCMOS oscillator, ±50 ppm  (mic_array, CLK_3V3)
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
mics' own DATA-driven delay (tDD) varies 40–80 ns part to part, the two buffer stages allow
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
plane through its own via. A second pass, C157–C183 (27 more, same style), counts only
GND–MIC_3V3 caps (a GND via is no help here): every clock layer-change via that was more
than 10 mm from such a cap, and the DATA/clock vias around J1, got a cap within 4 mm
(6.5 mm where nothing closer fits), placed clear of courtyards, labels and the camera
cut-out. Clock vias by distance to the nearest cap are now 39 / 35 / 90 / 8 in the
0–3 / 3–5 / 5–10 / 10–20 mm bands (was 25 / 13 / 75 / 59). The rest:
- ~46 vias in J1's own fan-out between its pin rows have no room for a cap; J1's GND pins
  carry their return to the front_end.
- 5 clock vias still have no spot: three at the lower edge of the camera cut-out
  (ARM_00/01/03_CLK) and two next to U107/U108 (MIC_CLK_80, MIC_CLK_92).

Prefer keeping clocks on F.Cu when re-routing; any new layer change should get a cap like
these.


**Buffer decoupling.** Each CDCLVC11xx VDD pin should have a cap within ~1–2 mm, but on
F.Cu the area outside the VDD pins is taken by the fanning-out clock outputs. C138–C156
(19 × 100 nF, F.Cu, in `misc.kicad_sch`) are tied into the CLK_3V3 trace that runs under
each buffer and bring every VDD pin of U110 and of U97–U102, U104 and U106–U108 to within
2.0–5.8 mm of a cap (previously up to 9–13 mm). U103 and U105 had no room at pins 10/14
(about 9 mm from C108/C110). C184/C185 (100 nF 0402, F.Cu) now sit on those pins' CLK_3V3
feeds:
- C185 is tied into U105's F.Cu feed trace.
- C184 connects through a new via onto U103's B.Cu feed, 1.5 mm past the feed's existing
  via.

That brings U103 pins 10/14 to 6.2 / 5.1 mm and U105 to 3.4 / 6.0 mm. Every CDCLVC1108 VDD
pin is now within 6.2 mm of a cap. Getting closer than that would mean opening space
around those two buffers, or a B.Cu cap, which would make the board double-sided for
assembly.

---

## GbE PHY, RGMII and Y1 (front_end)

**RGMII pin assignment.** Each group has its own bank and its own row of CON3. CON3's
odd (east) row is bank 15 for pins 1–23 and its even (west) row bank 16 for pins 4–24
(pin 2 is bank 15).

| Row | Pins, south → north |
|---|---|
| East, bank 15 (RX) | 5 RXD1 (B15_L4_P), 7 RXD0 (L4_N), 11 RX_CTL (L2_P), 13 RXD2 (L2_N), 15 RXC (L12_P, MRCC), 17 RXD3 (L12_N) |
| West, bank 16 (TX) | 4 TXD3 (B16_IO25), 6 TXD2 (L21_N), 8 TXD1 (L21_P), 12 TXD0 (L23_P), 14 TXC (L23_N), 16 TX_CTL (L22_P) |

TX follows U3's own pin order (pins 25–29 TXD3…TXC, then TX_CTL on pin 37). RX can't be
fully ordered: RXC has to stay on CON3.15, the clock-capable pin. The RX group must share a
bank with RXC: RX data is clocked by a BUFIO from the RXC pin, and a BUFIO only reaches its
own bank (Vivado: "A BUFIO can only drive loads in the same IO bank"; through a BUFG
instead, the ~5 ns clock insertion leaves no setup/hold solution in the PHY's 2.4 ns
window). RXD0–RXD3 were originally in bank 16 (CON3.6/8/16/18), RX_CTL on CON3.2 and TXD2
on CON3.3, and TX_CTL originally on CON1.22 (bank 13, now unused). The DATA lines moved out
of the way have tens of ns of margin and don't care which bank they're in: DATA_00 → CON3.3,
DATA_01 → .22, DATA_02 → .2, DATA_03 → .24, DATA_05 → .18.

U3's RGMII pins run down its east edge with TX south of RX, so routed north to CON3 the TX
bundle lands naturally east of RX, the opposite of the banks' rows: the two bundles cross
once. Rotating U3 90° counter-clockwise would remove that crossing.

**RGMII timing set by the PHY straps** (DP83867 Tables 7-6, 7-9, 7-10):
- TX clock skew: R5/R6 are **DNP**, so LED_1 reads mode 1 (internal 9k pull-down); LED_2
  is mode 1 too (only R4 330 Ω into an LED) → TX[2:0] = 000 → **mode 1, 2.0 ns**. The PHY
  delays TXC itself, so the FPGA sends TXC edge-aligned with the data (RGMII TskewT
  ±0.5 ns; the PHY accepts ±1.0 ns). R5 10k / R6 2.49k (LED_1 mode 2) gave TX[2] = 1,
  mode 5, 0 ns: the FPGA then had to shift TXC 90° itself, and that closed timing with
  only ~20 ps either side across process corners, so they were left unfitted.
- RX clock skew: GPIO_0/GPIO_1 unstrapped → **mode 1, 2.0 ns**. The PHY centres RXC in the
  data (≥1.2 ns setup and hold at its pins). The FPGA's BUFIO capture clock lands ~1 ns
  late against that, so it delays each RX input with a fixed IDELAYE2 (11 taps, ~0.86 ns),
  leaving ~0.45 ns setup and hold. The receive path is live: the FPGA answers ARP
  requests for the board's IP.
- RX_CTRL = mode 3 (R11 5.76k / R12 2.49k), as the datasheet requires; autoneg enabled.
  LED_1 mode 1 sets ANEG_SEL = 0 (as mode 2 did): the PHY comes out of reset advertising
  10/100/1000. The FPGA then rewrites that over MDIO (PHY address 0, RX_D0/RX_D2
  unstrapped): ANAR = 0x0001 (no 10/100), BMCR = 0x1340 (restart autoneg), so it links
  at 1000 Mb/s or not at all -- the UDP stream can't be sustained at 10/100. It then
  reads ANAR back; anything but 0x0001 (0xFFFF if the PHY doesn't answer, 0x01E1 if it
  ignored the write) turns the status LED solid red.

**RGMII lengths** (front_end only, pad to pad; each net has 2 vias, counted at 1.545 mm
each; the AC7200 module's internal routing is unknown and adds to these). Re-balanced with
tuning meanders after the row-grouping pin move:

| Group | Clock | Data/ctl | Worst vs clock | Group spread |
|---|---|---|---|---|
| TX | TXC 53.05 mm | 53.22–53.54 mm | +0.49 mm (~3 ps), TXD0 | 0.49 mm |
| RX | RXC 44.11 mm | 43.64–44.33 mm | −0.46 mm (~3 ps), RXD1 | 0.69 mm |

For scale: the PHY accepts TXC within ±1.0 ns of its data and gives ±1.2 ns of RX setup
and hold, so these skews are a few percent of the margin.

**U3 and U5 exposed pads.** U3's 17 GND vias (4×4 at 1.2 mm + centre, 0.6/0.3 mm) and
U5's 2 (0.6/0.3 mm, under its two paste windows) are marked **filled and capped**
(IPC-4761 Type VII) in their via properties, so paste over them can't wick down the
barrels; every other via uses the board default (tented both sides). ODB++ export carries
this as `filling_f.cu-b.cu` / `capping_f.cu-b.cu` layers (19 vias each). Gerber + Excellon do
**not** — for a Gerber order, the fab note on User.Drawings below the board outline
carries it (User.Drawings is in the board's saved plot layers, so it plots as
`front_end-User_Drawings.gbr`); point the fab at it or repeat it in the order comments.
The saved plot format is Gerber.

**Y1 crystal circuit.** Y1 uses `Device:Crystal_GND24` on
`Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm`: crystal on pads **1 (XO) and 3 (XI)**
(diagonal), pads **2 and 4 to GND** (each with a GND via, 1.05 mm and 1.8 mm away). It
originally used the 2-pin `Device:Crystal`, which put XI on pad 2 — a case/ground pad on
standard 3225 crystals — so it would not have oscillated. Y1 sits 5.4–5.8 mm from U3 pins
14/15; XI is 9.35 mm and XO 10.02 mm, F.Cu, no vias; C12 is 1.6 mm and C13 3.3 mm from
their Y1 pads.

**Y1 part requirements** (DP83867 datasheet
Table 9-3 / §9.2.1.3):

| Parameter | Requirement |
|---|---|
| Frequency | 25 MHz, fundamental, parallel resonant |
| Frequency tolerance over operating temperature | ±50 ppm max (±20–30 ppm parts recommended for margin) |
| Frequency stability, 1-year aging | ±50 ppm max |
| Package / pinout | 3.2 × 2.5 mm, 4 pads, crystal between pads 1 and 3, pads 2/4 GND or case — confirm against the chosen part's datasheet |
| Load capacitance (CL) | Match to C12/C13: **12 pF** with 15–18 pF caps (≈C/2 + 3–4 pF stray), or TI's recommended **18 pF** with C12/C13 changed to ~27 pF |
| Temperature range | Cover the board's operating range; the tolerance above must hold over it |

**Chosen:** YXC X322525MOB4SI (LCSC C9006): 25 MHz, CL 12 pF, ±10 ppm tolerance, ±20 ppm
stability, SMD3225-4P. C12/C13 changed from 18 pF to 15 pF (FH 0402CG150J500NT, C1548) to
centre the 12 pF load (15/2 + ~4 pF stray ≈ 11.5 pF). The datasheet land pattern (1.4 × 1.2 mm
pads, 0.8 × 0.5 mm gaps; #1 bottom-left, anticlockwise; GND on #2/#4) matches the footprint.

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
- **Colours** (`fpga/single_fpga/rtl/status_led.v`): YELLOW while in reset, GREEN once
  running, GREEN/BLUE alternating (≈171 ms each) while packets are going out, reverting to
  GREEN ≈0.34 s after they stop. Solid RED is a PHY configuration fault: the MDIO
  read-back of the DP83867's advertisement didn't return 1000 Mb/s only.

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
In2.Cu  MIC_3V3 plane                      GND, with a +5V island (below)
B.Cu    signal: longer DATA runs,          signal (J2 is mounted on B.Cu)
        connector fan-out
```

**front_end In2.** In2 used to be one +5V plane, so every signal on B.Cu returned on +5V
and each F.Cu↔B.Cu signal via (107 of them) needed a +5V–GND cap nearby; the nearest
were 15–56 mm away. In2 is now a GND fill ("Ground Plane In2", priority 0) with a +5V
island ("+5V Island", priority 1) that only covers where +5V goes:
- a strip on the right from J1/C3/TP1 up to CON1 (x 128.5–140, y 77.5–157), widening to
  x 119.3 at CON1's +5V pins (y 77.5–84.5);
- a branch west along y 127–135 to the two bucks' input vias (U1, U6, C1, C30).

D1's anode gets +5V from a 0.5 mm In2 track from the island (94.2, 134) to its via at
(84, 154.2); it crosses only the +2V5_ETH power trace on B.Cu. The only B.Cu traces over
or across the island are LED_R/G/B, which are effectively DC.

With both inner layers GND, a layer change only needs a GND via next to it; 26 were added.
Of the 123 signal layer-change vias outside the island, 69 have a GND via within 2 mm and
99 within 3 mm. The other 24 sit in the dense J2 / CON2 / CON3 fan-outs, where a 0.6 mm
via doesn't fit; those connectors' GND pins are the return there.

The inner layers carry no description in their names (plain "In1.Cu" / "In2.Cu"), so the
plotted Gerbers are `front_end-In1_Cu.gbr` / `front_end-In2_Cu.gbr`.

---

## Board Outline

- `mic_array.kicad_pcb`: circular, **Ø320 mm**, with a Ø12 mm centre cut-out.
- `front_end.kicad_pcb`: **120 mm × 120 mm** rectangle.
- `mic_test.kicad_pcb`: **120 mm × 120 mm** rectangle, with mic_array's Ø12 mm cut-out (see below).

---

## Mic test board (`pcb/mic_test/`)

A cheaper board for proving out the mic side before ordering the full Ø320 mm array. It is the
centre 120 × 120 mm of mic_array, roughly 1/7 of its area, and plugs into a real front_end. The
front_end itself has no cut-down version: the real board is the test vehicle for J4 programming,
GbE, D1 and its rails. The bring-up procedure is in
[`mic_test/TEST_PLAN.md`](mic_test/TEST_PLAN.md).

**How it was made.**
- The project is a copy of mic_array's, renamed `mic_test`. mic_array's files are untouched.
- Every symbol whose footprint lies outside the 120 mm square is marked *not on board / not in
  BOM / DNP*, so all sheets and nets stay as in mic_array. The 175 excluded parts are 68 mics,
  their caps, and the stitching caps.
- The board kept mic_array's footprints and routing inside the square, verbatim and at the same
  absolute coordinates:
  - J1, the HENC1–4 M4 holes, the HCAM1–4 camera holes, the cut-out, Y1, R15, U110, VR1/VR2,
    TP1–TP3;
  - the TCXO_CLK / PDM_CLK feeds, the J1 fan-out, the DATA routing of the fitted mics, and
    MIC_3V3/GND.
- J1 and every hole match mic_array exactly, so the J1/J2 and standoff alignment tested here is
  the real one.
- Same 4-layer stackup, design rules and DRU (In1 GND, In2 MIC_3V3).

**What it carries.**
- The 28 mics whose courtyards lie inside the square, 2–3 per arm, at their real positions: U1,
  U2, U9–U11, U17, U18, U25, U26, U33–U35, U41, U42, U49, U50, U57–U59, U65, U66, U73, U74,
  U81–U83, U89, U90. They use 16 DATA lines.
- The whole clock tree: Y1, U110, and all twelve CDCLVC1108s U97–U108. Each buffer moved in from
  r ≈ 85 mm to r ≈ 53–69 mm along its own arm's direction, together with its OE pull-up and
  decoupling caps.
- Test additions, in `test_extras.kicad_sch`:
  - **R16–R47**: 10 kΩ pull-ups to MIC_3V3 on the 32 DATA lines with no fitted mic. The FPGA reads
    them as a constant 1, which checks every J1/J2 DATA pin.
  - **R48**: 0.1 Ω 1 % 0805 shunt between J1's +3V6 and the regulators' new net **+3V6_A**.
    TP5 "3V6 IN" is on the J1 side and TP3 is on +3V6_A. VR1/VR2, C98/C99 and TP3 are now on
    +3V6_A.
  - **R49**: 51 Ω 2512, DNP. Fit it to load MIC_3V3 with ≈65 mA, about what the 68 missing mics
    would draw.
  - Probe pairs, each with a GND TP 3 mm away:
    - TP6/TP7 on ARM_02_CLK at U99's input;
    - TP8/TP9 on MIC_CLK_72 at U73;
    - TP10/TP11 on DATA_00 near J1.
  - Silkscreen "MIC TEST rev A".

**Routing still to do** (by hand; the placement of the buffers, pull-ups and test points is a
starting point):
- **ARM_nn_CLK**: U110 → each buffer, meandered to its mic_array length so U110's fan-out and the
  line SI match the real board:

  | Arm | Buffer | ARM_nn_CLK on mic_array (mm) | Fitted mics | Their MIC_CLK on mic_array (mm) |
  |---|---|---|---|---|
  | 0 | U97 | 147.5 | U1, U2 | 81, 61 |
  | 1 | U98 | 149.0 | U9, U10, U11 | 68, 47, 51 |
  | 2 | U99 | 156.0 | U17, U18 | 70, 47 |
  | 3 | U100 | 145.7 | U25, U26 | 83, 53 |
  | 4 | U101 | 150.1 | U33, U34, U35 | 76, 51, 40 |
  | 5 | U102 | 148.3 | U41, U42 | 80, 65 |
  | 6 | U103 | 143.9 | U49, U50 | 82, 54 |
  | 7 | U104 | 143.7 | U57, U58, U59 | 85, 67, 50 |
  | 8 | U105 | 136.0 | U65, U66 | 90, 81 |
  | 9 | U106 | 134.7 | U73, U74 | 91, 80 |
  | 10 | U107 | 142.0 | U81, U82, U83 | 84, 61, 44 |
  | 11 | U108 | 147.3 | U89, U90 | 81, 72 |

- **MIC_CLK**: natural length is fine, but meander **MIC_CLK_72** (the probed one) to 91 mm, the
  worst case on the full board. The buffer outputs feeding unfitted mics stay unconnected.
- The CLK_3V3 feed and GND to each moved buffer.
- The pull-ups, R48 (the cut +3V6 feed: J1 fan-out at (259.5, 194.9) → R48 → the +3V6_A stub at
  (259.5, 196.05)), R49, and the test points.
- Any new F.Cu↔B.Cu clock via gets a GND–MIC_3V3 cap within a few mm, as on mic_array. The
  stitching caps that came along with the square have lost the vias they served:
  - C121–C125, C131, C171, C179;
  - reuse them for the new vias, or mark them not-on-board.

After routing, DRC should show only the 8 board-only holes as parity items, the same as
mic_array. Plot output goes to `pcb/mic_test/gerbers/`.

---

## Status (as of the schematics/layouts on disk)

- **mic_array**: routed, ERC- and DRC-clean (0 violations, 0 unconnected; the only parity
  entries are the 8 board-only mounting holes, excluded). Clock tree balanced as above.
- **front_end**: routed, ERC- and DRC-clean (0 violations, 0 unconnected, 0 parity),
  including the row-grouped RGMII pins, R5/R6 DNP and the re-balanced RGMII lengths above,
  In2 converted to GND + a +5V island, the J3 centre-tap cap C32 and the CON1 caps C33/C34.
- **top.kicad_sch**: only the two expected PWR_FLAG power-output-to-power-output errors.
- **Before fab:**
  - ~~choose Y1's part and add its MPN~~ — done: YXC X322525MOB4SI, 12 pF, C12/C13 → 15 pF, footprint checked;
  - ~~add User.Drawings to the front_end Gerber plot layers~~ — done; the U3 + U5
    filled/capped-via note now plots with the Gerbers (or send ODB++);
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
