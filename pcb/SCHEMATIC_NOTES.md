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
| **front_end** | `front_end.kicad_pcb` / `front_end.kicad_sch` | ALINX AC7200 FPGA SOM, +5V→+3V8 power regulation, inter-board connector |
| **mic_array** | `mic_array.kicad_pcb` / `mic_array.kicad_sch` | 96 mics, master clock (TCXO) + full clock distribution tree, local 3.3V regulation, inter-board connector |

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
    ├── ac7200_conn1.kicad_sch … ac7200_conn4.kicad_sch (AC7200 SOM, one symbol split 4 ways)
    └── (sheet-local: power regulation, J2/J3)
```

`ac7200_conn1–4` each place one unit of the same multi-unit `single_fpga:AC7200`
schematic symbol (U109) — the AC7200's 4× 80-pin connectors (CON1–CON4),
split across 4 sheets for readability, not 4 separate parts.

---

## Reference Designators

| Range | Board | Parts |
|---|---|---|
| U1–U8, U10–U17, U19–U26, … (9 per arm) | mic_array | 8× IM72D128 mics + 1× CDCLVC1108 clock buffer, per arm (arm 0 = U1–U9, arm 1 = U10–U18, … arm 11 = U100–U108) |
| C1–C96 | mic_array | 100 nF decoupling, one per mic VDD pin |
| C97–C101 | mic_array | TCXO / CDCLVC1112 decoupling + bulk |
| R1 (one per arm sheet) | mic_array | 10 kΩ pull-up on each arm's CDCLVC1108 output-enable pin |
| R13 | mic_array | 10 kΩ pull-up on CDCLVC1112's output-enable pin |
| Y1 | mic_array | 12.288 MHz TCXO |
| U110 | mic_array | CDCLVC1112, stage-1 (1:12) PDM clock buffer |
| VR1, VR2 | mic_array | MCP1700T-3302E 3.3V LDOs — separate MIC_3V3 and CLK_3V3 rails |
| J1 | mic_array | Inter-board connector (Panasonic AXK6S80347YG header) |
| U109 | front_end | AC7200 FPGA SOM (Xilinx XC7A200T-2FBG484I), one symbol across CON1–CON4 |
| U111 | front_end | TLV62569DBV, +5V→+3V8 synchronous buck |
| L1 | front_end | 2.2 µH inductor for U111 |
| R14, R15 | front_end | 499 kΩ / 100 kΩ feedback divider setting U111's +3V8 output |
| C102–C104 | front_end | U111 input/output/bulk caps |
| J2 | front_end | Inter-board connector (AXK5S80347YG socket, mates J1) |
| J3 | front_end | +5V/GND power input |

Mic numbering (U references) is no longer a clean U1–U96 block — the
per-arm clock buffer is interleaved into the same reference sequence, so
each arm consumes 9 consecutive U numbers (8 mics + 1 buffer), not 8.

---

## FPGA: ALINX AC7200 SOM

Xilinx **XC7A200T-2FBG484I**, on ALINX's AC7200 module. Pinout and
mechanical placement are cross-checked against ALINX's real manual and
schematic (see [`pcb/libraries/AC7200/AC7200_README.md`](libraries/AC7200/AC7200_README.md)
— 320/320 connector pins verified, zero discrepancies). The module mounts
on the front_end board through 4× 80-pin, 0.5mm-pitch board-to-board
connectors (Panasonic AXK580137YG/AXK680337YG family, 3.0mm mated height).

This fully supersedes the old Nexys A7-200T / Nexys Video I/O-budget
discussion below (kept for historical context only — see
"Superseded: External Dev-Board Connector" at the bottom of this file).

---

## Power

```
+5V (J3, front_end) → TLV62569DBV buck (U111, L1, R14/R15 FB divider) → +3V8
   +3V8 → AC7200 SOM (U109, front_end)
   +3V8 → J2/J1 inter-board connector → mic_array board
             +3V8 → VR1 (MCP1700-3302E) → MIC_3V3   (mic analog supply)
             +3V8 → VR2 (MCP1700-3302E) → CLK_3V3   (TCXO + clock buffers)
```

MIC_3V3 and CLK_3V3 are deliberately separate LDO outputs from the same
+3V8 rail, isolating the mic analog supply from clock-buffer switching
noise. Only +3V8 and GND cross the inter-board connector — no +5V or
regulated 3.3V rail is shared directly between boards.

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

The master clock is generated **on the mic_array board** and never
crosses the inter-board connector — a change from the original single
central-buffer plan, and it also drops the earlier ÷4 divider idea (the
TCXO's 12.288 MHz drives the buffer tree directly):

```
Y1  ECS-TXO-5032-122.8, 12.288 MHz TCXO, ±2.5 ppm  (mic_array, CLK_3V3)
 │
 ├─ Stage 1: U110 CDCLVC1112 (1:12 LVCMOS buffer)
 │     TCXO_CLK → ARM_00_CLK … ARM_11_CLK   (one output per arm)
 │
 └─ Stage 2: one CDCLVC1108 (1:8 LVCMOS buffer) per arm sheet
       (U9 in mic_arm_00, U18 in mic_arm_01, … U108 in mic_arm_11)
       ARM_nn_CLK → MIC_CLK_(8·nn) … MIC_CLK_(8·nn+7)
```

Each mic gets its own buffered clock net, globally numbered
`MIC_CLK_00`–`MIC_CLK_95` (index = arm × 8 + position-in-arm = mic_idx),
so there's no per-sheet net-name collision risk despite every arm sheet
being a copy of the same template. Both buffer ICs have their
output-enable pin pulled up locally (R13 for U110, one R1 per arm for
each CDCLVC1108) rather than driven from the FPGA.

**Matched-length traces** from each buffer stage to its loads are still
important above 1 MHz; keep CLK traces within ±5 mm of each other within
a given fan-out stage.

---

## Inter-board Connector

| Side | Board | Part |
|---|---|---|
| J1 | mic_array | Panasonic AXK6S80347YG (80-pin + 4 fixing pads, 0.5mm pitch, 7.0mm mated height, header) |
| J2 | front_end | Panasonic AXK5S80347YG (mating socket, same family) |

Carries all 48 DATA_nn nets plus +3V8 and GND. No clock signal crosses
this connector — see "PDM Clock Distribution" above.

---

## Layer Stack — implemented

`mic_array.kicad_pcb` is routed on the planned 4-layer stack:

```
F.Cu   (signal)  — mic pads, DATA traces, per-arm clock fan-out
In1.Cu (plane)   — solid reference plane
In2.Cu (plane)   — solid reference plane
B.Cu   (signal)  — longer DATA runs, connector fanout
```

---

## Board Outline

`mic_array.kicad_pcb`'s Edge.Cuts geometry measures **320 mm × 320 mm**
(circular, Ø320 mm) — matches the original plan and is now the as-routed
outline, not just a target.

`front_end.kicad_pcb` has no board outline yet — only its 10 footprints
(AC7200 SOM connectors, U111 buck regulator + support parts, J2, J3) are
placed; routing hasn't started.

---

## Status (as of the schematics/layouts on disk)

- **mic_array.kicad_pcb**: actively being routed (most recent PCB commits
  are all on this file — clock-driver grounding/pull-ups done, general
  routing in progress).
- **front_end.kicad_pcb**: components placed only; no outline, no routing
  yet.

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
to it via the AXK5S/6S80347YG board-to-board connector described above.
