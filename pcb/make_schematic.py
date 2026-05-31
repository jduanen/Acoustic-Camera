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
KI_VER  = "20250114"   # KiCad 9/10 schematic format version

# ── arm sheet layout (mm, A3 landscape 420 × 297 mm) ─────────────────────────
#
#  2-column, 2-row arrangement:
#    pairs 0,1 → column 0 (x=80)
#    pairs 2,3 → column 1 (x=230)
#  Within each column: pair in row 0 (y_base=20) or row 1 (y_base=135).
#
COL_X    = [80.0, 230.0]   # mic centre x per column
ROW_BASE = [20.0, 135.0]   # y_base per row
L_DY     = 20.0            # L-mic y offset from y_base
R_DY     = 60.0            # R-mic y offset from y_base
CAP_DX   = 22.0            # cap centre x-offset from mic centre

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

def _lib_mic():
    return (
        '  (symbol "IM72D128"\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0.508))\n'
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
          # VDD: top pin in y-up coords → connection at (0,+6.35), angle 270 = pin points down into body
          '      (pin power_in line (at 0 6.35 270) (length 2.54)\n'
          '        (name "VDD"  (effects (font (size 1.27 1.27))))\n'
          '        (number "1"  (effects (font (size 1.27 1.27)))))\n'
          # GND: bottom pin
          '      (pin power_in line (at 0 -6.35 90) (length 2.54)\n'
          '        (name "GND"  (effects (font (size 1.27 1.27))))\n'
          '        (number "2"  (effects (font (size 1.27 1.27)))))\n'
          # DATA: right-side output
          '      (pin output line (at 6.35 1.27 180) (length 2.54)\n'
          '        (name "DATA" (effects (font (size 1.27 1.27))))\n'
          '        (number "3"  (effects (font (size 1.27 1.27)))))\n'
          # CLK: left-side input, lower
          '      (pin input line (at -6.35 -1.27 0) (length 2.54)\n'
          '        (name "CLK"  (effects (font (size 1.27 1.27))))\n'
          '        (number "4"  (effects (font (size 1.27 1.27)))))\n'
          # SEL: left-side input, upper
          '      (pin input line (at -6.35 1.27 0) (length 2.54)\n'
          '        (name "SEL"  (effects (font (size 1.27 1.27))))\n'
          '        (number "5"  (effects (font (size 1.27 1.27)))))))\n'
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

def make_arm(arm_idx):
    """Return (sch_content, sch_uuid) for arm_NN.kicad_sch."""
    sch_uuid = _uid()
    buf = []

    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "1.0")\n'
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
        + _lib_vdd()
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
        buf.append(_pwr("power:+1V8", "+1V8", x,          yL + VDD_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x,          yL + GND_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x + SEL_DX, yL + SEL_DY,     sch_uuid))  # SEL=GND
        # decoupling cap for L mic: placed so pin 1 aligns with VDD pin
        cap_yL = yL - 2.54   # cap centre → pin 1 at cap_yL + CP1_DY = yL - 6.35 = VDD pin y
        buf.append(_cap(f"C{gL+1}", x + CAP_DX, cap_yL, sch_uuid))
        buf.append(_pwr("power:+1V8", "+1V8", x + CAP_DX, cap_yL + CP1_DY, sch_uuid))  # cap pin 1
        buf.append(_pwr("power:GND",  "GND",  x + CAP_DX, cap_yL + CP2_DY, sch_uuid))  # cap pin 2

        # ── R mic (SEL = +1V8) ───────────────────────────────────────────────
        buf.append(_mic(f"U{gR+1}", x, yR, sch_uuid))
        buf.append(_pwr("power:+1V8", "+1V8", x,          yR + VDD_DY,     sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x,          yR + GND_DY,     sch_uuid))
        buf.append(_pwr("power:+1V8", "+1V8", x + SEL_DX, yR + SEL_DY,     sch_uuid))  # SEL=+1V8
        cap_yR = yR - 2.54
        buf.append(_cap(f"C{gR+1}", x + CAP_DX, cap_yR, sch_uuid))
        buf.append(_pwr("power:+1V8", "+1V8", x + CAP_DX, cap_yR + CP1_DY, sch_uuid))
        buf.append(_pwr("power:GND",  "GND",  x + CAP_DX, cap_yR + CP2_DY, sch_uuid))

        # ── CLK: vertical wire + global label at top ────────────────────────
        clk_x = x + CLK_DX
        buf.append(_wire(clk_x, yL + CLK_DY, clk_x, yR + CLK_DY))
        buf.append(_glabel("PDM_CLK", clk_x, yL + CLK_DY, angle=180, shape="input"))

        # ── DATA: vertical wire + global label at bottom ─────────────────────
        dat_x = x + DATA_DX
        buf.append(_wire(dat_x, yL + DATA_DY, dat_x, yR + DATA_DY))
        buf.append(_glabel(dnet, dat_x, yR + DATA_DY, angle=0, shape="output"))

    buf.append(
        f'  (sheet_instances\n'
        f'    (path "/"\n'
        f'      (page "{arm_idx + 2}")))\n'   # page 1 = top; arms start at 2
        f')\n'
    )

    return ''.join(buf), sch_uuid

# ── top sheet generator ───────────────────────────────────────────────────────

def make_top(arm_uuids):
    """Return top.kicad_sch content referencing all 12 arm sub-sheets."""
    top_uuid = _uid()
    buf = []

    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "1.0")\n'
        f'  (uuid "{top_uuid}")\n'
        f'  (paper "A3")\n'
        f'  (title_block\n'
        f'    (title "96-mic IM72D128 PDM Array — Top Level")\n'
        f'    (comment 1 "12 arms × 8 mics  |  48 DATA lines + 1 PDM_CLK → FMC LPC (J1)")\n'
        f'    (comment 2 "FPGA hub: Nexys Video (XC7A200T) via FMC LPC")\n'
        f'    (comment 3 "See DESIGN.md for pinout and signal assignment")\n'
        f'  )\n'
        f'  (lib_symbols)\n'   # top sheet has no directly-placed components
    )

    # 4 columns × 3 rows of sheet symbols (each 85 × 45 mm)
    SW, SH = 85.0, 45.0    # sheet symbol width / height
    MARGIN_X, MARGIN_Y = 20.0, 25.0
    SPACING_X, SPACING_Y = SW + 15.0, SH + 20.0
    COLS_TOP, ROWS_TOP = 4, 3

    for arm_idx in range(N_ARMS):
        col = arm_idx % COLS_TOP
        row = arm_idx // COLS_TOP
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
    print(f"Open {pro_path} in KiCad 9/10.")
    print(f"Next steps:")
    print(f"  1. Assign IM72D128 footprint (from Infineon / Ultra Librarian / SnapEDA)")
    print(f"  2. Assign C footprint (e.g., Capacitor_SMD:C_0402_1005Metric)")
    print(f"  3. Add FMC LPC connector (J1) to top.kicad_sch")
    print(f"  4. Run ERC; expected warnings: unconnected Footprint fields, single global labels")

if __name__ == "__main__":
    main()
