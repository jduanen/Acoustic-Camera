# Phase 4 Mic Array — Schematic Notes

## Reference Designators

| Range | Parts |
|---|---|
| U1–U96 | IM72D128V01XTMA1 mics (mic_idx 0–95 = U1–U96) |
| C1–C96 | 100 nF decoupling caps on each VDD pin |
| C97–C100 | Bulk decoupling (10 µF × 4, one per power zone) |
| Y1 | 12.288 MHz TCXO |
| U97 | PDM clock fan-out buffer (see below) |
| J1 | FMC LPC connector to Nexys Video (Samtec ASP-134604-01 or equivalent) |

---

## Schematic Organisation — Hierarchical Sheets

96 mics is unwieldy on a flat schematic.  Use 12 hierarchical sub-sheets,
one per arm.  Each sub-sheet contains 8 mics + 4 decoupling caps + 4 DATA
net labels.

```
top.kicad_sch
├── arm_00.kicad_sch   (U1–U8,   C1–C4,   DATA_00–DATA_03)
├── arm_01.kicad_sch   (U9–U16,  C5–C8,   DATA_04–DATA_07)
├── ...
└── arm_11.kicad_sch   (U89–U96, C89–C92, DATA_44–DATA_47)
```

In KiCad, duplicate `arm_00.kicad_sch` 11 times and update the reference
numbers and DATA net indices using Edit → Change Symbols / global labels.

---

## Data Line Assignment

`test/phase4/data_line_assignment.csv` has the full table.  Summary:

| DATA line | Mic L (SEL=GND) | Mic R (SEL=VDD) | Pair distance |
|---|---|---|---|
| DATA_00 | U1  (mic 0,  arm 0) | U2  (mic 1,  arm 0) | ~19 mm |
| DATA_01 | U3  (mic 2,  arm 0) | U4  (mic 3,  arm 0) | ~19 mm |
| DATA_02 | U5  (mic 4,  arm 0) | U6  (mic 5,  arm 0) | ~19 mm |
| DATA_03 | U7  (mic 6,  arm 0) | U8  (mic 7,  arm 0) | ~19 mm |
| DATA_04 | U9  (mic 8,  arm 1) | U10 (mic 9,  arm 1) | ~19 mm |
| ...     | ...                  | ...                  | ...    |
| DATA_47 | U95 (mic 94, arm11) | U96 (mic 95, arm11) | ~19 mm |

Rule: within each pair, the mic with the lower index has SEL=GND (L),
the higher index has SEL=VDD (R).

Within each arm, pairs are (0,1), (2,3), (4,5), (6,7).
Data line index = arm × 4 + pair_within_arm.

---

## IM72D128 Pin Assignment

From the Infineon datasheet (confirm pin numbering against the KiCad footprint):

| Pin | Name | Connect to |
|---|---|---|
| 1 | VDD | 1.8 V supply + 100 nF to GND |
| 2 | GND | GND plane |
| 3 | DATA | DATA_nn net |
| 4 | CLK | PDM_CLK (shared, 3.072 MHz) |
| 5 | SEL | GND (L mic) or VDD (R mic) |

> Verify pin numbering against the actual datasheet before ordering the PCB.
> Footprint sources: Infineon website, Ultra Librarian, SnapEDA.

---

## PDM Clock Distribution

3.072 MHz at 96 loads is too much for a bare TCXO output.  Use a clock
fan-out buffer between the TCXO and the mics.

**Recommended part**: TI CDCLVC1310 (1-in, 10-out, 3.3 V LVCMOS, ~$0.60)
or SN74AHCT1G08 chain.  For 96 mics you need:

```
TCXO (12.288 MHz) → ÷4 divider → 3.072 MHz → fan-out buffer
```

Alternatively, the Nexys FPGA generates the 3.072 MHz PDM clock on its
GPIO output and drives a fan-out buffer on the mic array PCB.  This is
simpler: one less IC, and the clock is under FPGA control.

For either topology, **matched-length traces** from the fan-out output to
each mic CLK pin are important above 1 MHz.  Target: all CLK traces within
±5 mm of each other (±5 mm / (2 × 10⁸ m/s) ≈ ±25 ps skew — well within
the IM72D128's CLK setup/hold spec).

---

## Connector to FPGA Development Board

### I/O budget

| Signal | Count |
|---|---|
| PDM DATA (mic → FPGA) | 48 |
| PDM_CLK (FPGA → mics) | 1 |
| **Total FPGA I/O needed** | **49** |

### ⚠ Nexys A7-200T I/O constraint

The Nexys A7-200T exposes 32 user I/O pins (4× Pmod, 8 per connector).
**This is 17 pins short of the 49 needed for a 96-mic array.**

### Recommended fix: Nexys Video (~$325)

Digilent's Nexys Video uses the **same XC7A200T FPGA** as the Nexys A7-200T
and adds an **FMC LPC connector** (68 single-ended I/O — more than enough).

| Board | FPGA | Accessible I/O | FMC | Price |
|---|---|---|---|---|
| Nexys A7-200T | XC7A200T | ~32 (4× Pmod) | No | ~$350 |
| **Nexys Video** | **XC7A200T** | **32 + 68 FMC** | **LPC** | **~$325** |

The Nexys Video is the recommended FPGA hub for this project.  The FMC LPC
connector carries the 48 DATA + 1 CLK via a matched-impedance cable.  Pmod
connectors remain available for bring-up debug (ILA probe points, logic
analyser headers, etc.).

If you already have the Nexys A7-200T, see the **32-channel bringup option**
below.

### Connector: FMC LPC (Nexys Video path)

Standard FMC LPC pinout.  Map DATA_00–DATA_47 and PDM_CLK to FMC LA pins
(LA00–LA33, 34 differential pairs; use positive pin of each pair as
single-ended signal and tie negative to GND on the array PCB).

Use the standard FMC LPC plug (Samtec ASP-134604-01 or Molex 71436-1006)
on the mic array PCB.

### Connector: 2× 34-pin IDC (32-channel bringup option)

If using Nexys A7-200T for a first bring-up with only 32 channels (4 arms
× 8 mics = 32 mics, 16 DATA lines + 1 CLK = 17 signals → fits in 3 PMODs):

| PMOD JA | PMOD JB | PMOD JC |
|---|---|---|
| DATA_00–DATA_07 | DATA_08–DATA_15 | PDM_CLK + spare |

Run full 32-mic HDL first, verify pipeline end-to-end, then migrate to
Nexys Video for all 96 channels.

---

## Layer Stack (4-layer minimum)

```
Top    (signal)   — mic pads, DATA traces, CLK fan-out
GND    (plane)    — solid ground reference for PDM signal integrity
PWR    (plane)    — 1.8 V power pour
Bottom (signal)   — longer DATA runs, connector fanout
```

Keep PDM_CLK traces on the top layer with guard traces to GND.
Route DATA traces as 50 Ω impedance-controlled (top-to-GND reference).
Stitch GND vias around the CLK fan-out buffer.

---

## Board Outline

Array coordinates span ±150 mm (300 mm diameter).  Allow ≥5 mm margin on
all sides → **320 mm × 320 mm** board minimum.  A circular board outline
(Ø 320 mm) is conventional for acoustic camera arrays.

Mount holes: four M3 holes at the PCB corners or compass points.
Camera mount hole at centre (Ø 12 mm clearance for M8 or standard ¼-20
thread insert).
