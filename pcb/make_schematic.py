#!/usr/bin/env python3
"""
make_schematic.py  —  generate KiCad 10 schematics for the 96-mic IM72D128 PDM array.

Outputs (in pcb/mic_array/):
  top.kicad_sch          — root sheet: 12 hierarchical arm sub-sheets
  arm_00.kicad_sch  …    — 12 arm sheets: 8 mics + 4 decoupling caps each
  arm_11.kicad_sch
  mic_array.kicad_pro    — KiCad project file

Usage (from project root):
  python pcb/make_schematic.py
"""

import os
import uuid as _uuid_mod

# ── project constants ─────────────────────────────────────────────────────────

N_ARMS    = 12
N_PER_ARM = 8
N_PAIRS   = N_PER_ARM // 2   # 4 pairs per arm, 2 mics per pair

OUTDIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mic_array")
PROJECT = "mic_array"
KI_VER  = "20260306"   # KiCad 10.0 schematic format version

GRID = 1.27   # KiCad's default schematic grid (50 mil). Every placement
              # constant below is an exact multiple of this so every pin,
              # wire endpoint, and label lands on-grid.

# ── arm sheet layout (mm, A3 landscape 420 × 297 mm) ─────────────────────────
#
#  2-column, 2-row arrangement:
#    pairs 0,1 → column 0 (x≈80)
#    pairs 2,3 → column 1 (x≈230)
#  Within each column: pair in row 0 (y_base≈20) or row 1 (y_base≈135).
#
COL_X    = [63 * GRID, 181 * GRID]   # mic centre x per column (≈80, ≈230)
ROW_BASE = [16 * GRID, 106 * GRID]   # y_base per row (≈20, ≈135)
L_DY     = 16 * GRID                 # L-mic y offset from y_base (≈20)
R_DY     = 47 * GRID                 # R-mic y offset from y_base (≈60)
CAP_DX   = 17 * GRID                 # cap centre x-offset from mic centre (≈22)

# IM72D128 pin connection offsets in *screen* coords (y-down), rel. to symbol centre.
# Derived by negating the y component of the symbol-local y-up pin positions:
#   symbol-local (y-up)              screen (y-down)
#   VDD:  (0,+6.35, 270)  →  (0,  -6.35)
#   GND:  (0,-6.35,  90)  →  (0,  +6.35)
#   DATA: (+6.35,+1.27,180)→  (+6.35, -1.27)
#   CLK:  (-6.35,-1.27, 0) →  (-6.35, +1.27)
#   SEL:  (-6.35,+1.27, 0) →  (-6.35, -1.27)
VDD_DY  = -6.35
GND_DY  = +6.35
DATA_DX = +6.35;  DATA_DY = -1.27
CLK_DX  = -6.35;  CLK_DY  = +1.27
SEL_DX  = -6.35;  SEL_DY  = -1.27

# CLK_DX == SEL_DX: both pins stub out on the same left-side column, just
# 2.54mm apart in y. A CLK-to-CLK wire straight down that column crosses
# directly over the R mic's SEL pin (2.54mm above R's CLK, still within the
# L-to-R wire's span) and shorts CLK to SEL/+1V8. CLK_CLEAR jogs the CLK wire
# further left, off the SEL column, before running it vertically.
CLK_CLEAR = 5.08

# Device:C pin offsets (screen y-down, relative to cap centre):
#   pin 1 (top): symbol-local (0,+3.81,270) → screen (0,-3.81)
#   pin 2 (bot): symbol-local (0,-3.81, 90) → screen (0,+3.81)
CP1_DY = -3.81
CP2_DY = +3.81

# ── helpers ───────────────────────────────────────────────────────────────────

_pwr_n = [0]

def _uid():
    return str(_uuid_mod.uuid4())

def _pref():
    _pwr_n[0] += 1
    return f"#PWR{_pwr_n[0]:04d}"

def _f(x):
    """Float → 2-dp string, trailing zeros stripped."""
    s = f"{x:.2f}"
    if '.' in s:
        s = s.rstrip('0').rstrip('.')
    return s

def _pp(name, val, x=0.0, y=0.0, hide=False, justify=""):
    """Return a KiCad (property ...) line."""
    h = " (hide yes)" if hide else ""
    j = f" (justify {justify})" if justify else ""
    return (f'    (property "{name}" "{val}" (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)){j}{h}))\n')

# ── lib_symbols definitions ───────────────────────────────────────────────────

def _lib_gnd():
    return (
        '  (symbol "power:GND" (power)\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0) (hide yes))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference", "#PWR", 0, -6.35, hide=True)
        + _pp("Value",     "GND",  0, -3.81)
        + _pp("Footprint", "",     0,  0,    hide=True)
        + _pp("Datasheet", "",     0,  0,    hide=True)
        + _pp("Description", "",   0,  0,    hide=True)
        + '    (symbol "GND_0_1"\n'
          '      (polyline\n'
          '        (pts (xy 0 0) (xy 0 -1.27) (xy 1.27 -1.27) (xy 0 -2.54) (xy -1.27 -1.27) (xy 0 -1.27))\n'
          '        (stroke (width 0) (type default)) (fill (type none))))\n'
          '    (symbol "GND_1_1"\n'
          '      (pin power_in line (at 0 0 270) (length 0)\n'
          '        (name "~" (effects (font (size 1.27 1.27))))\n'
          '        (number "1" (effects (font (size 1.27 1.27)))))))\n'
    )

def _lib_vdd():
    return (
        '  (symbol "power:+1V8" (power)\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0) (hide yes))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference", "#PWR",  0, -3.81, hide=True)
        + _pp("Value",     "+1V8",  0,  3.556)
        + _pp("Footprint", "",      0,  0,    hide=True)
        + _pp("Datasheet", "",      0,  0,    hide=True)
        + _pp("Description", "",    0,  0,    hide=True)
        + '    (symbol "+1V8_0_1"\n'
          '      (polyline (pts (xy -0.762 1.27) (xy 0 2.54))\n'
          '        (stroke (width 0) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy 0 2.54) (xy 0.762 1.27))\n'
          '        (stroke (width 0) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy 0 0) (xy 0 2.54))\n'
          '        (stroke (width 0) (type default)) (fill (type none))))\n'
          '    (symbol "+1V8_1_1"\n'
          '      (pin power_in line (at 0 0 90) (length 0)\n'
          '        (name "~" (effects (font (size 1.27 1.27))))\n'
          '        (number "1" (effects (font (size 1.27 1.27)))))))\n'
    )

def _lib_pwr_named(name):
    """Same shape as _lib_vdd(), under an arbitrary rail name -- used when make_arm()
    is called with a vdd_label override (see its docstring). KiCad power symbols are
    one global net per distinct name; the multi-FPGA design needs each cluster's own
    mic array on its own independent +1.8V rail (its own local LDO), not the single
    shared "+1V8" _lib_vdd()/the default vdd_label=None path provides."""
    return (
        f'  (symbol "power:{name}" (power)\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0) (hide yes))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference", "#PWR",  0, -3.81, hide=True)
        + _pp("Value",     name,    0,  3.556)
        + _pp("Footprint", "",      0,  0,    hide=True)
        + _pp("Datasheet", "",      0,  0,    hide=True)
        + _pp("Description", "",    0,  0,    hide=True)
        + f'    (symbol "{name}_0_1"\n'
          '      (polyline (pts (xy -0.762 1.27) (xy 0 2.54))\n'
          '        (stroke (width 0) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy 0 2.54) (xy 0.762 1.27))\n'
          '        (stroke (width 0) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy 0 0) (xy 0 2.54))\n'
          '        (stroke (width 0) (type default)) (fill (type none))))\n'
          f'    (symbol "{name}_1_1"\n'
          '      (pin power_in line (at 0 0 90) (length 0)\n'
          '        (name "~" (effects (font (size 1.27 1.27))))\n'
          '        (number "1" (effects (font (size 1.27 1.27)))))))\n'
    )

def _lib_mic():
    return (
        '  (symbol "IM72D128"\n'
        # offset 0 keeps each pin's name right at its outer tip, clear of the
        # body — a nonzero offset drags the name inward toward the body
        # centre, and with 5 pins this close together (CLK/SEL only 2.54mm
        # apart), any offset above ~0 makes adjacent names overlap.
        '    (pin_numbers (hide yes)) (pin_names (offset 0))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference",   "U",                 0,  8.89)
        + _pp("Value",       "IM72D128V01XTMA1",  0, -8.89)
        + _pp("Footprint",   "",                  0,  0,   hide=True)
        + _pp("Datasheet",   "",                  0,  0,   hide=True)
        + _pp("Description",
              "Infineon IM72D128 PDM MEMS mic, 72 dB SNR, IP57", 0, 0, hide=True)
        + '    (symbol "IM72D128_0_1"\n'
          '      (rectangle (start -3.81 -3.81) (end 3.81 3.81)\n'
          '        (stroke (width 0) (type default)) (fill (type background))))\n'
          '    (symbol "IM72D128_1_1"\n'
          # Pin NUMBERS below match the real IM72D128V01 datasheet (Table 9,
          # "IM72D128V01 pin configuration": 1=DATA, 2=VDD, 3=CLOCK,
          # 4=SELECT, 5=GND) and the vendor's own KiCad symbol
          # (pcb/IM72D128/KiCad/IM72D128V.kicad_sym) -- this project's
          # earlier numbering (1=VDD,2=GND,3=DATA,4=CLK,5=SEL) was simply
          # wrong, not a real part's pinout, and would have shorted VDD to
          # GND and driven DC power onto the DATA pin if fabricated. Only
          # the "number" fields changed here; the geometric pin positions
          # (which only affect how the symbol is drawn, not the real
          # electrical connections) are untouched.
          #
          # VDD: top pin in y-up coords → connection at (0,+6.35), angle 270 = pin points down into body
          '      (pin power_in line (at 0 6.35 270) (length 2.54)\n'
          '        (name "VDD"  (effects (font (size 1.27 1.27))))\n'
          '        (number "2"  (effects (font (size 1.27 1.27)))))\n'
          # GND: bottom pin
          '      (pin power_in line (at 0 -6.35 90) (length 2.54)\n'
          '        (name "GND"  (effects (font (size 1.27 1.27))))\n'
          '        (number "5"  (effects (font (size 1.27 1.27)))))\n'
          # DATA: right-side output, tri-stated on the half-cycle this mic isn't
          # selected (SEL) so its L/R partner can drive the shared line — hence
          # "tri_state" electrical type, not "output", to avoid a false-positive
          # ERC pin-to-pin conflict on the two mics' shared DATA net.
          '      (pin tri_state line (at 6.35 1.27 180) (length 2.54)\n'
          '        (name "DATA" (effects (font (size 1.27 1.27))))\n'
          '        (number "1"  (effects (font (size 1.27 1.27)))))\n'
          # CLK: left-side input, lower
          '      (pin input line (at -6.35 -1.27 0) (length 2.54)\n'
          '        (name "CLK"  (effects (font (size 1.27 1.27))))\n'
          '        (number "3"  (effects (font (size 1.27 1.27)))))\n'
          # SEL: left-side input, upper
          '      (pin input line (at -6.35 1.27 0) (length 2.54)\n'
          '        (name "SEL"  (effects (font (size 1.27 1.27))))\n'
          '        (number "4"  (effects (font (size 1.27 1.27)))))))\n'
    )

def _lib_cap():
    return (
        '  (symbol "C"\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0.254))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference",   "C",       0.635,  2.54,  justify="left")
        + _pp("Value",       "100nF",   0.635, -2.54,  justify="left")
        + _pp("Footprint",   "",        0.965, -3.81,  hide=True)
        + _pp("Datasheet",   "",        0,      0,     hide=True)
        + _pp("Description", "100 nF decoupling capacitor, 0402", 0, 0, hide=True)
        + '    (symbol "C_0_1"\n'
          '      (polyline (pts (xy -2.032 0.762) (xy 2.032 0.762))\n'
          '        (stroke (width 0.508) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy -2.032 -0.762) (xy 2.032 -0.762))\n'
          '        (stroke (width 0.508) (type default)) (fill (type none))))\n'
          '    (symbol "C_1_1"\n'
          '      (pin passive line (at 0 3.81 270) (length 2.794)\n'
          '        (name "~" (effects (font (size 1.27 1.27))))\n'
          '        (number "1" (effects (font (size 1.27 1.27)))))\n'
          '      (pin passive line (at 0 -3.81 90) (length 2.794)\n'
          '        (name "~" (effects (font (size 1.27 1.27))))\n'
          '        (number "2" (effects (font (size 1.27 1.27)))))))\n'
    )

# ── schematic element builders ────────────────────────────────────────────────

def _wire(x1, y1, x2, y2):
    return (f'  (wire (pts (xy {_f(x1)} {_f(y1)}) (xy {_f(x2)} {_f(y2)}))\n'
            f'    (stroke (width 0) (type default))\n'
            f'    (uuid "{_uid()}"))\n')

def _glabel(name, x, y, angle=0, shape="bidirectional"):
    # angle 0  → connection at left point, text extends right
    # angle 180→ connection at right point, text extends left
    justify = "right mirror" if angle == 180 else "left"
    return (f'  (global_label "{name}"\n'
            f'    (shape {shape})\n'
            f'    (at {_f(x)} {_f(y)} {angle})\n'
            f'    (effects (font (size 1.27 1.27)) (justify {justify}))\n'
            f'    (uuid "{_uid()}")\n'
            f'    (property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n'
            f'      (at {_f(x)} {_f(y)} {angle})\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes))))\n')

def _pwr(lib_id, val, x, y, sch_uuid, rot=0):
    ref = _pref()
    return (f'  (symbol (lib_id "{lib_id}")\n'
            f'    (at {_f(x)} {_f(y)} {rot})\n'
            f'    (unit 1)\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (uuid "{_uid()}")\n'
            f'    (property "Reference" "{ref}"\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Value" "{val}"\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Footprint" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Datasheet" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Description" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (pin "1" (uuid "{_uid()}"))\n'
            f'    (instances (project "{PROJECT}"\n'
            f'      (path "/{sch_uuid}" (reference "{ref}") (unit 1)))))\n')

def _mic(ref, x, y, sch_uuid):
    return (f'  (symbol (lib_id "IM72D128")\n'
            f'    (at {_f(x)} {_f(y)} 0)\n'
            f'    (unit 1)\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (uuid "{_uid()}")\n'
            f'    (property "Reference" "{ref}"\n'
            f'      (at {_f(x + 5)} {_f(y + 8.89)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left)))\n'
            f'    (property "Value" "IM72D128V01XTMA1"\n'
            f'      (at {_f(x + 5)} {_f(y - 8.89)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left)))\n'
            f'    (property "Footprint" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Datasheet" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Description" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (pin "1" (uuid "{_uid()}"))\n'
            f'    (pin "2" (uuid "{_uid()}"))\n'
            f'    (pin "3" (uuid "{_uid()}"))\n'
            f'    (pin "4" (uuid "{_uid()}"))\n'
            f'    (pin "5" (uuid "{_uid()}"))\n'
            f'    (instances (project "{PROJECT}"\n'
            f'      (path "/{sch_uuid}" (reference "{ref}") (unit 1)))))\n')

def _cap(ref, x, y, sch_uuid):
    return (f'  (symbol (lib_id "C")\n'
            f'    (at {_f(x)} {_f(y)} 0)\n'
            f'    (unit 1)\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (uuid "{_uid()}")\n'
            f'    (property "Reference" "{ref}"\n'
            f'      (at {_f(x + 2)} {_f(y - 1.5)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left)))\n'
            f'    (property "Value" "100nF"\n'
            f'      (at {_f(x + 2)} {_f(y + 1.5)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left)))\n'
            f'    (property "Footprint" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Datasheet" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Description" ""\n'
            f'      (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (pin "1" (uuid "{_uid()}"))\n'
            f'    (pin "2" (uuid "{_uid()}"))\n'
            f'    (instances (project "{PROJECT}"\n'
            f'      (path "/{sch_uuid}" (reference "{ref}") (unit 1)))))\n')

# ── arm sheet generator ───────────────────────────────────────────────────────

def make_arm(arm_idx, clk_label="PDM_CLK", page_num=None, vdd_label=None):
    """Return (sch_content, sch_uuid) for arm_NN.kicad_sch.

    clk_label: override the PDM clock global label (default "PDM_CLK", matching
    this project's single shared master clock). Used by make_schematic_multi_fpga.py
    to reuse this same per-mic wiring with cluster-specific clock net names, since
    each cluster there receives its own forwarded-clock copy, not one shared net.
    page_num: override this sheet's own page number (default arm_idx + 2, matching
    this project's page layout; the multi_fpga project nests arms one level deeper).
    vdd_label: override the mic/cap VDD rail's net name (default "+1V8", the single
    shared power symbol every arm in this project used originally). Used by
    make_schematic_multi_fpga.py to give each cluster's 24 mics their own
    independent +1.8V rail (fed by that cluster's own local LDO) instead of one
    project-wide "+1V8" net, which would short 4 separate LDO outputs together.
    """
    if page_num is None:
        page_num = arm_idx + 2
    vdd_net = vdd_label or "+1V8"
    sch_uuid = _uid()
    buf = []

    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "10.0")\n'
        f'  (uuid "{sch_uuid}")\n'
        f'  (paper "A3")\n'
        f'  (title_block\n'
        f'    (title "96-mic PDM Array — Arm {arm_idx:02d} (mics {arm_idx*8+1}–{arm_idx*8+8})")\n'
        f'    (comment 1 "IM72D128V01XTMA1 PDM MEMS mics")\n'
        f'    (comment 2 "VDD = +1V8; each mic has 100 nF local decoupling")\n'
        f'    (comment 3 "SEL=GND → L channel (CLK falling); SEL=+1V8 → R channel (CLK rising)")\n'
        f'  )\n'
    )

    buf.append(
        f'  (lib_symbols\n'
        + _lib_mic()
        + _lib_cap()
        + _lib_gnd()
        + (_lib_vdd() if vdd_label is None else _lib_pwr_named(vdd_net))
        + '  )\n'
    )

    for pair in range(N_PAIRS):
        col = pair // 2          # column 0 (pairs 0,1) or column 1 (pairs 2,3)
        row = pair % 2           # row 0 or row 1 within the column
        x   = COL_X[col]
        yL  = ROW_BASE[row] + L_DY
        yR  = ROW_BASE[row] + R_DY

        # Global mic / cap indices (0-based)
        gL   = arm_idx * N_PER_ARM + pair * 2
        gR   = gL + 1
        data = arm_idx * N_PAIRS + pair
        dnet = f"DATA_{data:02d}"

        # ── L mic (SEL = GND) ────────────────────────────────────────────────
        buf.append(_mic(f"U{gL+1}", x, yL, sch_uuid))
        buf.append(_pwr(f"power:{vdd_net}", vdd_net, x, yL + VDD_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x,          yL + GND_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x + SEL_DX, yL + SEL_DY,     sch_uuid))  # SEL=GND
        # decoupling cap for L mic: placed so pin 1 aligns with VDD pin
        cap_yL = yL - 2.54   # cap centre → pin 1 at cap_yL + CP1_DY = yL - 6.35 = VDD pin y
        buf.append(_cap(f"C{gL+1}", x + CAP_DX, cap_yL, sch_uuid))
        buf.append(_pwr(f"power:{vdd_net}", vdd_net, x + CAP_DX, cap_yL + CP1_DY, sch_uuid))  # cap pin 1
        buf.append(_pwr("power:GND",  "GND",  x + CAP_DX, cap_yL + CP2_DY, sch_uuid))  # cap pin 2

        # ── R mic (SEL = +1V8) ───────────────────────────────────────────────
        buf.append(_mic(f"U{gR+1}", x, yR, sch_uuid))
        buf.append(_pwr(f"power:{vdd_net}", vdd_net, x, yR + VDD_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x,          yR + GND_DY,     sch_uuid))
        buf.append(_pwr(f"power:{vdd_net}", vdd_net, x + SEL_DX, yR + SEL_DY,     sch_uuid))  # SEL=+1V8
        cap_yR = yR - 2.54
        buf.append(_cap(f"C{gR+1}", x + CAP_DX, cap_yR, sch_uuid))
        buf.append(_pwr(f"power:{vdd_net}", vdd_net, x + CAP_DX, cap_yR + CP1_DY, sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x + CAP_DX, cap_yR + CP2_DY, sch_uuid))

        # ── CLK: jog left clear of the SEL pin column, then vertical + label ──
        clk_x  = x + CLK_DX
        clk_x2 = clk_x - CLK_CLEAR
        buf.append(_wire(clk_x, yL + CLK_DY, clk_x2, yL + CLK_DY))
        buf.append(_wire(clk_x2, yL + CLK_DY, clk_x2, yR + CLK_DY))
        buf.append(_wire(clk_x2, yR + CLK_DY, clk_x, yR + CLK_DY))
        buf.append(_glabel(clk_label, clk_x2, yL + CLK_DY, angle=180, shape="input"))

        # ── DATA: vertical wire + global label at bottom ─────────────────────
        dat_x = x + DATA_DX
        buf.append(_wire(dat_x, yL + DATA_DY, dat_x, yR + DATA_DY))
        buf.append(_glabel(dnet, dat_x, yR + DATA_DY, angle=0, shape="output"))

    buf.append(
        f'  (sheet_instances\n'
        f'    (path "/"\n'
        f'      (page "{page_num}")))\n'
        f')\n'
    )

    return ''.join(buf), sch_uuid

# ── VITA 57.1 FMC LPC signal → physical pin map ───────────────────────────────
# Verified against Analog Devices HDL repository constraint files.
# LA16-LA33 inferred from the confirmed spacing pattern; verify against
# the VITA 57.1 standard before ordering a PCB.

_FMC_SIG_PIN = {
    'CLK0_M2C_P': 'H4',   'CLK0_M2C_N': 'H5',
    'CLK1_M2C_P': 'G2',   'CLK1_M2C_N': 'G3',
    'LA00_CC_P':  'G6',   'LA00_CC_N':  'G7',
    'LA01_CC_P':  'D8',   'LA01_CC_N':  'D9',
    'LA02_P':  'H7',   'LA02_N':  'H8',
    'LA03_P':  'G9',   'LA03_N':  'G10',
    'LA04_P':  'H10',  'LA04_N':  'H11',
    'LA05_P':  'D11',  'LA05_N':  'D12',
    'LA06_P':  'C10',  'LA06_N':  'C11',
    'LA07_P':  'H13',  'LA07_N':  'H14',
    'LA08_P':  'G12',  'LA08_N':  'G13',
    'LA09_P':  'D14',  'LA09_N':  'D15',
    'LA10_P':  'C14',  'LA10_N':  'C15',
    'LA11_P':  'H16',  'LA11_N':  'H17',
    'LA12_P':  'G15',  'LA12_N':  'G16',
    'LA13_P':  'D17',  'LA13_N':  'D18',
    'LA14_P':  'C18',  'LA14_N':  'C19',
    'LA15_P':  'H19',  'LA15_N':  'H20',
    'LA16_P':  'G18',  'LA16_N':  'G19',
    'LA17_CC_P': 'D20', 'LA17_CC_N': 'D21',
    'LA18_CC_P': 'C20', 'LA18_CC_N': 'C21',
    'LA19_P':  'H22',  'LA19_N':  'H23',
    'LA20_P':  'G21',  'LA20_N':  'G22',
    'LA21_P':  'H25',  'LA21_N':  'H26',
    'LA22_P':  'G24',  'LA22_N':  'G25',
    'LA23_P':  'D23',  'LA23_N':  'D24',
    'LA24_P':  'H28',  'LA24_N':  'H29',
    'LA25_P':  'G27',  'LA25_N':  'G28',
    'LA26_P':  'D26',  'LA26_N':  'D27',
    'LA27_P':  'C26',  'LA27_N':  'C27',
    'LA28_P':  'H31',  'LA28_N':  'H32',
    'LA29_P':  'G30',  'LA29_N':  'G31',
    'LA30_P':  'H34',  'LA30_N':  'H35',
    'LA31_P':  'G33',  'LA31_N':  'G34',
    'LA32_P':  'H37',  'LA32_N':  'H38',
    'LA33_P':  'G36',  'LA33_N':  'G37',
}

# LA names in wire-assignment order (DATA_00..DATA_33 → P side)
_LA_ORDER = [
    'LA00_CC', 'LA01_CC', 'LA02',    'LA03',    'LA04',    'LA05',
    'LA06',    'LA07',    'LA08',    'LA09',    'LA10',    'LA11',
    'LA12',    'LA13',    'LA14',    'LA15',    'LA16',    'LA17_CC',
    'LA18_CC', 'LA19',    'LA20',    'LA21',    'LA22',    'LA23',
    'LA24',    'LA25',    'LA26',    'LA27',    'LA28',    'LA29',
    'LA30',    'LA31',    'LA32',    'LA33',
]

def _build_pin_net():
    """Return {physical_pin: net_name} for all 49 active FMC signals."""
    m = {'H4': 'PDM_CLK'}                          # CLK0_M2C_P
    for i, la in enumerate(_LA_ORDER):
        m[_FMC_SIG_PIN[f'{la}_P']] = f'DATA_{i:02d}'    # DATA_00..DATA_33
        if i < 14:                                        # DATA_34..DATA_47
            m[_FMC_SIG_PIN[f'{la}_N']] = f'DATA_{34+i:02d}'
    return m

_PIN_NET = _build_pin_net()

# ── FMC LPC connector helpers ─────────────────────────────────────────────────

def _read_fmc_sym():
    """Read ASP-134604-01 from the project library, return as lib_symbols entry."""
    import re
    sym_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'FMC_LPC', 'KiCADv6', '2026-05-31_21-41-26.kicad_sym')
    txt = open(sym_path).read()
    start = txt.find('  (symbol "ASP-134604-01"')
    if start == -1:
        raise FileNotFoundError("ASP-134604-01 symbol not found in FMC_LPC library")
    # Walk parentheses to find the matching close
    depth, end = 0, start
    for i, ch in enumerate(txt[start:], start):
        if ch == '(':   depth += 1
        elif ch == ')':
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    sym = txt[start:end]
    # Prefix top-level name with library name for embedding in lib_symbols
    sym = sym.replace('(symbol "ASP-134604-01"', '(symbol "FMC_LPC:ASP-134604-01"', 1)
    # Strip old-format (id N) property ids used in KiCad 6 symbols
    sym = re.sub(r'\s*\(id \d+\)', '', sym)
    return sym

def _fmc_unit(unit_n, row, sx, sy, top_uuid):
    """One unit (row) of J1. Pins at (sx, sy + 2.54*(n-1)) in screen coords."""
    pin_lines = '\n'.join(f'    (pin "{row}{n}" (uuid "{_uid()}"))' for n in range(1, 41))
    return (
        f'  (symbol (lib_id "FMC_LPC:ASP-134604-01")\n'
        f'    (at {_f(sx)} {_f(sy)} 0)\n'
        f'    (unit {unit_n})\n'
        f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
        f'    (uuid "{_uid()}")\n'
        f'    (property "Reference" "J1"\n'
        f'      (at {_f(sx + 8)} {_f(sy - 4)} 0)\n'
        f'      (effects (font (size 1.27 1.27))))\n'
        f'    (property "Value" "ASP-134604-01"\n'
        f'      (at {_f(sx + 8)} {_f(sy - 7)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        f'    (property "Footprint" "Connector_Samtec:Samtec_FMC_ASP-134604-01_4x40_Vertical"\n'
        f'      (at {_f(sx)} {_f(sy)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        f'    (property "Datasheet" "https://suddendocs.samtec.com/prints/asp-134604-01-mkt.pdf"\n'
        f'      (at {_f(sx)} {_f(sy)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        f'    (property "Description" "FMC LPC plug, mezzanine card side"\n'
        f'      (at {_f(sx)} {_f(sy)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        + pin_lines + '\n'
        + f'    (instances (project "{PROJECT}"\n'
        f'      (path "/{top_uuid}" (reference "J1") (unit {unit_n})))))\n'
    )

def _fmc_connector(top_uuid):
    """Return schematic elements for J1: 4 units + labels + GND on unused pins.

    Layout (A1 sheet, connector to the right of the 12 arm sub-sheets):
      Row G (unit 3): Sx=450,  pins carry LA00,LA03,LA08,LA12,LA16,LA20,LA22,LA25,LA29,LA31,LA33
      Row H (unit 4): Sx=530,  pins carry CLK0,LA02,LA04,LA07,LA11,LA15,LA19,LA21,LA24,LA28,LA30,LA32
      Row D (unit 2): Sx=610,  pins carry LA01,LA05,LA09,LA13,LA17,LA23,LA26
      Row C (unit 1): Sx=690,  pins carry LA06,LA10,LA14,LA18,LA27
    """
    UNIT_SX = {'G': 354 * GRID, 'H': 417 * GRID, 'D': 480 * GRID, 'C': 543 * GRID}
    UNIT_N  = {'C': 1,     'D': 2,     'G': 3,     'H': 4}
    SY = 20 * GRID   # top of all connector units (≈25)
    buf = []

    for row in ('G', 'H', 'D', 'C'):
        sx = UNIT_SX[row]
        buf.append(_fmc_unit(UNIT_N[row], row, sx, SY, top_uuid))

        for n in range(1, 41):
            pin_id = f'{row}{n}'
            sy_pin = SY + (n - 1) * 2.54   # screen y of this pin's connection point

            if pin_id in _PIN_NET:
                net = _PIN_NET[pin_id]
                shape = 'input' if net == 'PDM_CLK' else 'output'
                buf.append(_glabel(net, sx, sy_pin, angle=180, shape=shape))
            else:
                # Non-signal pin: tie to GND (covers power GND pins and unused N-side pins)
                # Power supply pins (VADJ, VCC, VREF) are left unconnected; ERC will flag them
                buf.append(_pwr("power:GND", "GND", sx, sy_pin, top_uuid))

    return ''.join(buf)

# ── top sheet generator ───────────────────────────────────────────────────────

def make_top(arm_uuids):
    """Return top.kicad_sch content: 12 arm sub-sheets + J1 FMC LPC connector."""
    top_uuid = _uid()
    buf = []

    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "10.0")\n'
        f'  (uuid "{top_uuid}")\n'
        f'  (paper "A1")\n'
        f'  (title_block\n'
        f'    (title "96-mic IM72D128 PDM Array — Top Level")\n'
        f'    (comment 1 "12 arms × 8 mics  |  48 DATA + 1 PDM_CLK → J1 FMC LPC")\n'
        f'    (comment 2 "FPGA hub: Nexys Video (XC7A200T), FMC LPC port")\n'
        f'    (comment 3 "LA pin mapping: VITA 57.1; verify LA16-LA33 against standard")\n'
        f'  )\n'
        f'  (lib_symbols\n'
        + _lib_gnd()
        + _lib_vdd()
        + _read_fmc_sym()
        + '  )\n'
    )

    # ── 12 arm sub-sheet symbols (4 cols × 3 rows) ───────────────────────────
    SW, SH = 67 * GRID, 35 * GRID
    MARGIN_X, MARGIN_Y = 16 * GRID, 20 * GRID
    SPACING_X, SPACING_Y = SW + 12 * GRID, SH + 16 * GRID

    for arm_idx in range(N_ARMS):
        col = arm_idx % 4
        row = arm_idx // 4
        sx  = MARGIN_X + col * SPACING_X
        sy  = MARGIN_Y + row * SPACING_Y
        sym_uuid = _uid()

        buf.append(
            f'  (sheet\n'
            f'    (at {_f(sx)} {_f(sy)})\n'
            f'    (size {_f(SW)} {_f(SH)})\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (stroke (width 0.1524) (type solid))\n'
            f'    (fill (color 0 0 0 0))\n'
            f'    (uuid "{sym_uuid}")\n'
            f'    (property "Sheetname" "arm_{arm_idx:02d}"\n'
            f'      (at {_f(sx)} {_f(sy - 1.5)} 0)\n'
            f'      (effects (font (size 1.524 1.524)) (justify left bottom)))\n'
            f'    (property "Sheetfile" "arm_{arm_idx:02d}.kicad_sch"\n'
            f'      (at {_f(sx)} {_f(sy + SH + 1.5)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left top)))\n'
            f'    (instances\n'
            f'      (project "{PROJECT}"\n'
            f'        (path "/{top_uuid}"\n'
            f'          (page "{arm_idx + 2}")))))\n'
        )

    # ── J1: FMC LPC connector ─────────────────────────────────────────────────
    buf.append(_fmc_connector(top_uuid))

    buf.append(
        f'  (sheet_instances\n'
        f'    (path "/"\n'
        f'      (page "1")))\n'
        f')\n'
    )

    return ''.join(buf)

# ── KiCad project file ────────────────────────────────────────────────────────

def make_project():
    """Return minimal mic_array.kicad_pro content."""
    return (
        '{\n'
        '  "meta": {\n'
        '    "filename": "mic_array.kicad_pro",\n'
        '    "version": 1\n'
        '  },\n'
        '  "schematic": {\n'
        '    "legacy_lib_dir": "",\n'
        '    "legacy_lib_list": []\n'
        '  },\n'
        '  "boards": [],\n'
        '  "libraries": {\n'
        '    "pinned_footprint_libs": [],\n'
        '    "pinned_symbol_libs": []\n'
        '  }\n'
        '}\n'
    )

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(OUTDIR, exist_ok=True)

    arm_uuids = []
    for i in range(N_ARMS):
        content, sch_uuid = make_arm(i)
        arm_uuids.append(sch_uuid)
        path = os.path.join(OUTDIR, f"arm_{i:02d}.kicad_sch")
        with open(path, 'w') as fh:
            fh.write(content)
        print(f"  {path}")

    top_path = os.path.join(OUTDIR, "top.kicad_sch")
    with open(top_path, 'w') as fh:
        fh.write(make_top(arm_uuids))
    print(f"  {top_path}")

    pro_path = os.path.join(OUTDIR, f"{PROJECT}.kicad_pro")
    with open(pro_path, 'w') as fh:
        fh.write(make_project())
    print(f"  {pro_path}")

    print(f"\nDone — {N_ARMS * N_PER_ARM} mics, {N_ARMS * N_PAIRS} DATA lines.")
    print(f"Open {pro_path} in KiCad 10.")
    print(f"Next steps:")
    print(f"  1. Assign IM72D128 footprint (from Infineon / Ultra Librarian / SnapEDA)")
    print(f"  2. Assign C footprint (e.g., Capacitor_SMD:C_0402_1005Metric)")
    print(f"  3. Run ERC; expected: unconnected VADJ/VCC/VREF pins on J1, missing footprints")
    print(f"  4. Verify LA16-LA33 pin assignments against VITA 57.1 standard before PCB order")

if __name__ == "__main__":
    main()
