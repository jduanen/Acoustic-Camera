#!/usr/bin/env python3
"""
make_schematic_multi_fpga.py — generate KiCad 10 schematics for the Phase 4
Multi-FPGA (Clustered) Alternative dev-board wiring (see PHASE4.md).

Scope: board-to-board interconnect (which physical connector pins carry which
signal, real FPGA pin names from the Digilent reference manuals), plus the
per-mic wiring for each cluster's 3 arms — reused directly from
pcb/make_schematic.py's make_arm() (same 96-mic IM72D128 layout as
pcb/mic_array/, not redrawn or reimplemented here) via two overridable
parameters: a per-cluster clock label (each cluster receives its own forwarded
PDM clock copy over its own spoke, not one shared net like the primary
design's single-FPGA case) and a page number (this project nests arms one
level deeper: top -> cluster -> arm, vs. mic_array's top -> arm).

Dev boards are represented as simplified connector-block symbols exposing
only the physical header pins actually used, labeled with their real FPGA
pin names (from the Cmod S7 / Cmod A7-35T reference manuals) — not a redraw
of Digilent's own internal board schematic.

Outputs (in pcb/multi_fpga/):
  top.kicad_sch                    — root sheet: 4 cluster sub-sheets + 1 hub sub-sheet
  cluster_00..03.kicad_sch         — Cmod S7 + spoke bus + 3 arm sub-sheets each
  arm_00..11.kicad_sch             — per-mic wiring, reused from make_schematic.make_arm()
  hub.kicad_sch                    — Cmod A7-35T + 4 spoke buses + TCXO + USB bridge
  multi_fpga.kicad_pro             — KiCad project file

multi_fpga and mic_array are deliberately separate KiCad projects (mutually
exclusive alternative front-ends, not two parts of one build) — only the
arm-level .kicad_sch generation logic is shared, via make_arm(), not the
project files themselves.

Usage (from project root):
  python pcb/make_schematic_multi_fpga.py
"""

import os
import sys
import uuid as _uuid_mod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_schematic as ms  # noqa: E402 — shared #PWR counter, see _pref() below
from make_schematic import make_arm, GRID, _cap, _lib_cap  # noqa: E402 — reuse the primary design's per-mic wiring / cap symbol

# ── project constants ─────────────────────────────────────────────────────────

N_CLUSTERS = 4

OUTDIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "multi_fpga")
PROJECT = "multi_fpga"
KI_VER  = "20260306"   # KiCad 10.0 schematic format version

# ── real pinout data, from Digilent reference manuals ─────────────────────────
# Cmod S7: one "standard" Pmod (JA), 200 ohm series resistors on every pin,
# no differential-capable pins anywhere on the module (see PHASE4.md "Spoke
# link: parallel single-ended bus, not LVDS").
CMOD_S7_PMOD_JA = ["J2", "H2", "H4", "F3", "H3", "H1", "G1", "F4"]  # positions 1,2,3,4,7,8,9,10

# Cmod A7-35T (hub): 48-pin DIP form factor, same as Cmod S7 — see PHASE4.md
# "Why Cmod A7-35T for the hub, not Arty A7-35T". Real pin names from
# Digilent's own CmodA7_Master.xdc (Cmod-A7-35T-GPIO reference project),
# cross-checked against the "Sch=" pin-number comments in that file. All 4
# spokes (32 signals) + the FT232H USB bridge (12) + TCXO clock in (1) = 45
# signals, entirely on the DIP header — no Pmod used at all (see PHASE4.md
# "Why all-DIP, no Pmod" for why this is preferred over splitting spoke 0
# onto the module's one real Pmod, JA).
#
# (fpga_pin, dip_pin_number) — pins 1-14, 17-23, 26-48 in physical order (44
# confirmed-digital pins; 24/25 are the VU/GND power pins, not signal-
# capable) plus DIP pin 16, the 45th signal needed. Pin 16 is documented as
# an XADC auxiliary analog input (vaux12) rather than plain GPIO — H2 is its
# single-ended ("P" leg) ball per the reference material; unused-for-analog
# 7-series aux pins are ordinary fabric I/O (same reasoning already applied
# to unused analog-capable FMC pins in pcb/make_schematic.py's mic array
# connector), but this specific pin has NOT been confirmed against a plain-
# GPIO-mode .xdc example — flag for verification before ordering hardware.
CMOD_A7_35T_DIP = list(zip(
    ["M3", "L3", "A16", "K3", "C15", "H1", "A15", "B15", "A14", "J3", "J1", "K2", "L1", "L2",
     "M1", "N3", "P3", "M2", "N1", "N2", "P1",
     "R3", "T3", "R2", "T1", "T2", "U1", "W2", "V2", "W3", "V3", "W5", "V4", "U4", "V5", "W4",
     "U5", "U2", "W6", "U3", "U7", "W7", "U8", "V8", "H2"],
    list(range(1, 15)) + list(range(17, 24)) + list(range(26, 49)) + [16],
))

# Spoke bus signal order matching Pmod position index 0..7 (see PHASE4.md
# "Spoke link"): 6 data bits + 1 strobe + 1 forwarded PDM clock (hub->cluster).
SPOKE_SIGNAL_SUFFIX = ["D0", "D1", "D2", "D3", "D4", "D5", "STROBE", "CLK"]

# FT232H USB bridge (8 data + 4 control) + TCXO clock in: 13 of the 37 DIP
# signals above, taken from CMOD_A7_35T_DIP in order after spokes 1-3.
USB_TCXO_NETS = [
    "USB_D0", "USB_D1", "USB_D2", "USB_D3", "USB_D4", "USB_D5", "USB_D6", "USB_D7",
    "USB_RXF_N", "USB_TXE_N", "USB_RD_N", "USB_WR_N", "TCXO_CLK",
]

# ── helpers (mirrors pcb/make_schematic.py conventions) ───────────────────────

def _uid():
    return str(_uuid_mod.uuid4())

def _pref():
    # Shares make_schematic.py's counter (imported above) rather than keeping
    # a separate one — main() calls make_arm() (12x, via make_schematic.py's
    # own _pwr()/_pref()) before make_cluster()/make_hub() (via this file's).
    # Two independent counters both starting at #PWR0001 produced duplicate
    # references project-wide (arm files' #PWR000N colliding with cluster/hub
    # files' #PWR000N) since KiCad treats the whole project as one flat
    # reference namespace.
    ms._pwr_n[0] += 1
    return f"#PWR{ms._pwr_n[0]:04d}"

def _f(x):
    """Float -> 2-dp string, trailing zeros stripped."""
    s = f"{x:.2f}"
    if '.' in s:
        s = s.rstrip('0').rstrip('.')
    return s

def _pp(name, val, x=0.0, y=0.0, hide=False, justify=""):
    h = " (hide yes)" if hide else ""
    j = f" (justify {justify})" if justify else ""
    return (f'    (property "{name}" "{val}" (at {_f(x)} {_f(y)} 0)\n'
            f'      (effects (font (size 1.27 1.27)){j}{h}))\n')

def _wire(x1, y1, x2, y2):
    return (f'  (wire (pts (xy {_f(x1)} {_f(y1)}) (xy {_f(x2)} {_f(y2)}))\n'
            f'    (stroke (width 0) (type default))\n'
            f'    (uuid "{_uid()}"))\n')

def _glabel(name, x, y, angle=0, shape="bidirectional"):
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

_flg_n = [0]

def _flag_ref():
    _flg_n[0] += 1
    return f"#FLG{_flg_n[0]:02d}"

def _flag(x, y, sch_uuid):
    """PWR_FLAG instance: declares the net it's wired to as externally
    supplied, satisfying ERC's "power input not driven" rule for rails with
    no on-schematic regulator/source symbol (dev-board-level scope — see
    SCHEMATIC_NOTES.md)."""
    ref = _flag_ref()
    return (f'  (symbol (lib_id "power:PWR_FLAG")\n'
            f'    (at {_f(x)} {_f(y)} 0)\n'
            f'    (unit 1)\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (uuid "{_uid()}")\n'
            f'    (property "Reference" "{ref}"\n'
            f'      (at {_f(x)} {_f(y - 5.08)} 0)\n'
            f'      (effects (font (size 1.27 1.27))))\n'
            f'    (property "Value" "PWR_FLAG"\n'
            f'      (at {_f(x)} {_f(y - 7.62)} 0)\n'
            f'      (effects (font (size 1.27 1.27))))\n'
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

def _pwr_flag_pair(lib_id, val, x, y, sch_uuid):
    """Power symbol + PWR_FLAG stacked at the identical point (both have a
    zero-length pin at their own origin, so co-locating them connects the two
    without a separate wire segment — the standard KiCad idiom for silencing
    "not driven" on a rail with no on-schematic regulator/source symbol).
    Power-symbol nets are global by name across the whole hierarchy, so one
    pair per net name anywhere in the project satisfies ERC for every other
    instance of that same rail."""
    return _pwr(lib_id, val, x, y, sch_uuid) + _flag(x, y, sch_uuid)

def _lib_flag():
    return (
        '  (symbol "power:PWR_FLAG" (power)\n'
        '    (pin_numbers (hide yes)) (pin_names (offset 0) (hide yes))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference", "#FLG", 0, 2.54)
        + _pp("Value",     "PWR_FLAG", 0, 4.826)
        + _pp("Footprint", "", 0, 0, hide=True)
        + _pp("Datasheet", "", 0, 0, hide=True)
        + _pp("Description",
              "Declares its net externally supplied, so ERC does not require "
              "an on-schematic driver for it", 0, 0, hide=True)
        + '    (symbol "PWR_FLAG_0_1"\n'
          '      (circle (center 0 1.905) (radius 0.635)\n'
          '        (stroke (width 0) (type default)) (fill (type none)))\n'
          '      (polyline (pts (xy 0 0) (xy 0 1.27))\n'
          '        (stroke (width 0) (type default)) (fill (type none))))\n'
          '    (symbol "PWR_FLAG_1_1"\n'
          '      (pin power_out line (at 0 0 90) (length 0)\n'
          '        (name "pwr" (effects (font (size 1.27 1.27))))\n'
          '        (number "1" (effects (font (size 1.27 1.27)))))))\n'
    )

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

def _lib_pwr_flag(name):
    """Generic power-rail symbol (+5V / +3V3), same shape as +1V8 in the primary design."""
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

# ── generic connector-block symbol builder ────────────────────────────────────
# All pins on the left side (angle 0 = connection point at smaller x, body to
# the right — same convention as the primary design's CLK/SEL pins), spaced
# 2.54 mm apart, symmetric about y=0 in symbol-local (y-up) coordinates.

PIN_LEN   = 5.08
BODY_HW   = 12.7   # body half-width
STUB      = 8 * GRID   # stub-and-label/pwr wire length (≈10)

# Real footprints for the dev-board module symbols, used when this schematic
# is laid out as a carrier PCB (see pcb/layout_multi_fpga.py). Cmod S7 and
# Cmod A7-35T share the same 48-pin, 2x24, 600mil-row-spacing DIP form factor
# (confirmed from Digilent's Cmod S7 reference manual), which matches the
# stock KiCad DIP-48_W15.24mm_Socket footprint exactly -- no custom footprint
# needed for either module. FT232H_BRK/TCXO_OSC use project-local
# approximate footprints (see pcb/multi_fpga/footprints.pretty/*.kicad_mod
# file descriptions for what's approximated and why).
FOOTPRINTS = {
    "CMOD_S7":      "Package_DIP:DIP-48_W15.24mm_Socket",
    "CMOD_A7_35T":  "Package_DIP:DIP-48_W15.24mm_Socket",
    "FT232H_BRK":   "multi_fpga:FT232H_Breakout",
    "TCXO_OSC":     "multi_fpga:TCXO_Can",
    "LDO_1V8":      "Package_TO_SOT_SMD:SOT-23",
    "LDO_3V3":      "Package_TO_SOT_SMD:SOT-23",
}

# LDO pin order/numbering matches the stock SOT-23 footprint's own pad
# layout (pad1/pad2 on one side, pad3 alone on the other) and Microchip's
# MCP1700 pinout (pin1=GND, pin2=VOUT, pin3=VIN) -- confirm against the
# datasheet before finalizing hardware, same confidence flag this project
# already uses for other real-part pinouts (e.g. CMOD_A7_35T_DIP pin 16).
LDO_PINS = [("GND", "1", "power_in"), ("OUT", "2", "power_out"), ("IN", "3", "power_in")]

def _conn_lib(lib_name, ref_prefix, value, pins, desc=""):
    """pins: list of (pin_name, pin_number, elec_type) in top-to-bottom (screen) order."""
    n = len(pins)
    body_hh = (n - 1) / 2 * 2.54 + 2.54
    buf = [
        f'  (symbol "{lib_name}"\n'
        f'    (pin_numbers (hide no)) (pin_names (offset 0.508))\n'
        '    (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
        + _pp("Reference", ref_prefix, 0, body_hh + 2.54)
        + _pp("Value", value, 0, -(body_hh + 2.54))
        + _pp("Footprint", FOOTPRINTS.get(lib_name, ""), 0, 0, hide=True)
        + _pp("Datasheet", "", 0, 0, hide=True)
        + _pp("Description", desc, 0, 0, hide=True)
        + f'    (symbol "{lib_name}_0_1"\n'
        f'      (rectangle (start {_f(-BODY_HW)} {_f(-body_hh)}) (end {_f(BODY_HW)} {_f(body_hh)})\n'
        '        (stroke (width 0) (type default)) (fill (type background))))\n'
        f'    (symbol "{lib_name}_1_1"\n'
    ]
    for i, (pname, pnum, etype) in enumerate(pins):
        # symbol-local y-up: top pin first, decreasing y going down the list
        y_local = body_hh - 2.54 - i * 2.54
        x_conn = -(BODY_HW + PIN_LEN)
        buf.append(
            f'      (pin {etype} line (at {_f(x_conn)} {_f(y_local)} 0) (length {_f(PIN_LEN)})\n'
            f'        (name "{pname}" (effects (font (size 1.27 1.27))))\n'
            f'        (number "{pnum}" (effects (font (size 1.27 1.27)))))\n'
        )
    buf.append('    ))\n')
    return ''.join(buf)

def _conn_instance(lib_name, ref, x, y, sch_uuid, pins, hide_value=False):
    n = len(pins)
    body_hh = (n - 1) / 2 * 2.54 + 2.54
    val = None
    return (
        f'  (symbol (lib_id "{lib_name}")\n'
        f'    (at {_f(x)} {_f(y)} 0)\n'
        f'    (unit 1)\n'
        f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
        f'    (uuid "{_uid()}")\n'
        f'    (property "Reference" "{ref}"\n'
        f'      (at {_f(x)} {_f(y - body_hh - 2.54)} 0)\n'
        f'      (effects (font (size 1.27 1.27))))\n'
        f'    (property "Footprint" "{FOOTPRINTS.get(lib_name, "")}"\n'
        f'      (at {_f(x)} {_f(y)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        f'    (property "Datasheet" ""\n'
        f'      (at {_f(x)} {_f(y)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        f'    (property "Description" ""\n'
        f'      (at {_f(x)} {_f(y)} 0)\n'
        f'      (effects (font (size 1.27 1.27)) (hide yes)))\n'
        + ''.join(f'    (pin "{pnum}" (uuid "{_uid()}"))\n' for _, pnum, _ in pins)
        + f'    (instances (project "{PROJECT}"\n'
        f'      (path "/{sch_uuid}" (reference "{ref}") (unit 1)))))\n'
    )

def _conn_pin_xy(x, y, i, n):
    """Absolute screen (y-down) coords of pin i's connection point for an instance placed at (x,y)."""
    body_hh = (n - 1) / 2 * 2.54 + 2.54
    y_local = body_hh - 2.54 - i * 2.54     # symbol-local y-up
    y_screen = y - y_local                  # negate for y-down schematic space
    x_screen = x - (BODY_HW + PIN_LEN)
    return x_screen, y_screen

def _stub_and_label(x, y, n_pins_len_left, net, shape="passive", label_angle=180):
    """Short wire stub from a pin's connection point, extending further left, plus a global label."""
    x2 = x - n_pins_len_left
    return _wire(x, y, x2, y) + _glabel(net, x2, y, angle=label_angle, shape=shape)

def _stub_and_pwr(lib_id, val, x, y, stub_len, sch_uuid):
    x2 = x - stub_len
    return _wire(x, y, x2, y) + _pwr(lib_id, val, x2, y, sch_uuid)

# ── symbol definitions used across this project ───────────────────────────────

def _all_lib_symbols():
    cmod_pins = (
        [("PDM_CLK", "D1", "passive")]
        + [(f"PDM_D{i:02d}", f"D{i+2}", "passive") for i in range(12)]
        + [("VU", "D24", "power_in"), ("GND", "D25", "power_in")]
        + [(fpga_pin, f"JA{pos}", "passive")
           for fpga_pin, pos in zip(CMOD_S7_PMOD_JA, [1, 2, 3, 4, 7, 8, 9, 10])]
        # Pmod JA positions 5/6/11/12 are hard-wired on the real Cmod S7 to
        # GND/VCC3V3 (that module's own onboard regulator output, generated
        # locally from ITS OWN VU) -- confirmed from Digilent's public Cmod S7
        # schematic (datasheets/Cmod+S7_sch-public.pdf). An earlier revision
        # of this design modeled JA5/JA6 as "SPOKE_GND"/"SPOKE_VU" carrying
        # the hub's +5V out to this cluster board over the same connector --
        # not physically possible (would drive straight into another
        # regulator's output pin), so removed. Power to this cluster board
        # instead arrives over a separate dedicated connector -- see J2/VR1
        # in make_cluster().
    )
    # Hub: same Cmod A7-35T module as used physically — all 45 signals (4
    # spokes + FT232H + TCXO) on its DIP header, no Pmod (see CMOD_A7_35T_DIP
    # and PHASE4.md "Why all-DIP, no Pmod") + VU/GND power (DIP 24/25).
    hub_pins = (
        [(fpga_pin, f"D{num}", "passive") for fpga_pin, num in CMOD_A7_35T_DIP]
        + [("VU", "D24", "power_in"), ("GND", "D25", "power_in")]
    )

    ft232h_pins = (
        [(f"D{i}", str(i + 1), "passive") for i in range(8)]
        + [("RXF#", "9", "passive"), ("TXE#", "10", "passive"),
           ("RD#", "11", "passive"), ("WR#", "12", "passive")]
        + [("VCCIO", "13", "power_in"), ("GND", "14", "power_in")]
    )

    tcxo_pins = [
        ("VDD", "1", "power_in"), ("GND", "2", "power_in"),
        ("OE", "3", "input"), ("OUT", "4", "output"),
    ]

    return (
        _lib_gnd()
        + _lib_pwr_flag("+5V")
        + _lib_pwr_flag("+3V3")
        + _lib_flag()
        + _lib_cap()
        + _conn_lib("CMOD_S7", "U", "Digilent Cmod S7 (XC7S25-1CSGA225C)", cmod_pins,
                    "Cluster FPGA module — Pmod JA (8 sig, 200ohm series) + DIP header "
                    "(13 of 32 GPIO used for local PDM capture); every pin series-resistor "
                    "protected, no differential-capable I/O (see PHASE4.md Spoke link)")
        + _conn_lib("CMOD_A7_35T", "U", "Digilent Cmod A7-35T (XC7A35T-1CPG236C)", hub_pins,
                    "Hub FPGA module — all 4 spokes + FT232H USB bridge + TCXO clock in "
                    "(45 signals) on its 48-pin DIP header, no Pmod used; same DIP module "
                    "form factor as the cluster's Cmod S7, see PHASE4.md, Why all-DIP, "
                    "no Pmod")
        + _conn_lib("FT232H_BRK", "U", "FTDI FT232H USB-FIFO breakout", ft232h_pins,
                    "USB 2.0 Hi-Speed synchronous 245-mode FIFO bridge to Raspberry Pi 5 "
                    "USB 3.0 port (device is USB2-speed, ~320 Mbps, backward compatible)")
        + _conn_lib("TCXO_OSC", "Y", "12.288 MHz TCXO", tcxo_pins,
                    "Master clock; NDK NZ2520SD or TXC 7M series, +-2.5ppm or better")
        + _conn_lib("LDO_1V8", "VR", "MCP1700T-1802E/TT", LDO_PINS,
                    "1.8V linear regulator for this cluster's own 24-mic array "
                    "(~31mA load, SOT-23, 1uF ceramic in/out per datasheet)")
        + _conn_lib("LDO_3V3", "VR", "MCP1700T-3302E/TT", LDO_PINS,
                    "3.3V linear regulator for the hub's FT232H + TCXO "
                    "(~50mA load, SOT-23, 1uF ceramic in/out per datasheet)")
    )

# ── cluster sheet generator ───────────────────────────────────────────────────

def make_cluster(idx):
    """Return (sch_content, sch_uuid) for cluster_NN.kicad_sch."""
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
        f'    (title "Multi-FPGA Alternative — Cluster {idx} (arms {idx*3}-{idx*3+2}, mics {idx*24}-{idx*24+23})")\n'
        f'    (comment 1 "Digilent Cmod S7 (XC7S25) — CIC/FIR done in HDL, not shown here")\n'
        f'    (comment 2 "Spoke bus: 6 data + strobe + fwd clock, parallel single-ended (not LVDS)")\n'
        f'    (comment 3 "PDM: 12 data lines (2 mics/line) + 1 clock, to this cluster\'s 3 arms")\n'
        f'  )\n'
        f'  (lib_symbols\n' + _all_lib_symbols() + _lib_pwr_flag(f"C{idx}_1V8") + '  )\n'
    )

    cmod_pins = (
        [("PDM_CLK", "D1", "passive")]
        + [(f"PDM_D{i:02d}", f"D{i+2}", "passive") for i in range(12)]
        + [("VU", "D24", "power_in"), ("GND", "D25", "power_in")]
        + [(fpga_pin, f"JA{pos}", "passive")
           for fpga_pin, pos in zip(CMOD_S7_PMOD_JA, [1, 2, 3, 4, 7, 8, 9, 10])]
    )
    n = len(cmod_pins)

    # Arm sub-sheet block geometry, needed here to centre CMOD_S7 against it
    # (used again below where the sheets themselves are placed).
    AW, AH = 47 * GRID, 24 * GRID
    AX, AY0 = 94 * GRID, 64 * GRID
    ARM_GAP = 12 * GRID                        # y-spacing between stacked arm sheets
    ARM_BLOCK_H = 2 * (AH + ARM_GAP) + AH      # 3 arms, 2 gaps between them

    CX = AX + AW + 66 * GRID                   # clear of the arm sheets' right edge
    CY = AY0 + ARM_BLOCK_H / 2                 # vertically centred on the 3-arm block
    # "A" (assembly/board module), not "U" — this project's "U" range is the
    # 96 mics (U1-U96, see make_arm()); reusing "U1" here collided with both
    # the first mic in every cluster's own arm_00-equivalent sheet and with
    # every other cluster's own CMOD_S7 (all 4 clusters previously said "U1").
    buf.append(_conn_instance("CMOD_S7", f"A{idx + 1}", CX, CY, sch_uuid, cmod_pins))

    # PDM_CLK: this cluster's own forwarded-clock copy (not shared with other
    # clusters — see PHASE4.md Spoke link). PDM_D00..D11: tie straight to the
    # real per-mic wiring's global DATA_NN labels (arm sub-sheets, below) —
    # DATA indices are globally unique across all 12 arms (12*idx .. 12*idx+11
    # for this cluster's 3 arms), so no cross-cluster collision.
    clk_net = f"C{idx}_PDM_CLK"
    x, y = _conn_pin_xy(CX, CY, 0, n)
    buf.append(_stub_and_label(x, y, STUB, clk_net, shape="passive"))
    for i in range(12):
        net = f"DATA_{12*idx + i:02d}"
        x, y = _conn_pin_xy(CX, CY, i + 1, n)
        buf.append(_stub_and_label(x, y, STUB, net, shape="passive"))

    # VU / GND power
    i_vu, i_gnd = 13, 14
    x, y = _conn_pin_xy(CX, CY, i_vu, n)
    buf.append(_stub_and_pwr("power:+5V", "+5V", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(CX, CY, i_gnd, n)
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))

    # Spoke bus: pins 15..22 = JA positions 1..10 (skipping 5,6) = D0..D5, STROBE, CLK
    for j, suffix in enumerate(SPOKE_SIGNAL_SUFFIX):
        net = f"SPOKE{idx}_{suffix}"
        x, y = _conn_pin_xy(CX, CY, 15 + j, n)
        shape = "input" if suffix == "CLK" else "output"
        buf.append(_stub_and_label(x, y, STUB, net, shape=shape, label_angle=180))

    # VR1: this cluster's own +1.8V LDO for its 24 mics -- fed from this
    # board's own +5V (arriving over the new dedicated power connector, see
    # pcb/layout_multi_fpga.py -- not modelled as a schematic symbol, same as
    # the spoke connector itself: see CMOD_S7_PMOD_JA's comment on why
    # board-to-board connectors are PCB-layout-only in this project),
    # output on a per-cluster-scoped net (C{idx}_1V8, not the single-FPGA
    # design's shared "+1V8") so 4 independent LDOs don't drive one net.
    vdd_net = f"C{idx}_1V8"
    VX, VY = CX + 30 * GRID, CY + 60 * GRID
    vr_n = len(LDO_PINS)
    buf.append(_conn_instance("LDO_1V8", "VR1", VX, VY, sch_uuid, LDO_PINS))
    x, y = _conn_pin_xy(VX, VY, 0, vr_n)   # pin 1 = GND
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(VX, VY, 1, vr_n)   # pin 2 = OUT
    buf.append(_stub_and_pwr(f"power:{vdd_net}", vdd_net, x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(VX, VY, 2, vr_n)   # pin 3 = IN
    buf.append(_stub_and_pwr("power:+5V", "+5V", x, y, STUB, sch_uuid))

    # Bypass caps, 1uF ceramic in/out per MCP1700's typical application
    # circuit -- same _cap()/_lib_cap() 2-pin symbol as the mic decoupling
    # caps, offsets (-3.81/+3.81) matching that symbol's own pin geometry.
    buf.append(_cap("C25", VX - 15 * GRID, VY, sch_uuid))
    buf.append(_pwr("power:+5V", "+5V", VX - 15 * GRID, VY - 3.81, sch_uuid))
    buf.append(_pwr("power:GND", "GND", VX - 15 * GRID, VY + 3.81, sch_uuid))
    buf.append(_cap("C26", VX + 15 * GRID, VY, sch_uuid))
    buf.append(_pwr(f"power:{vdd_net}", vdd_net, VX + 15 * GRID, VY - 3.81, sch_uuid))
    buf.append(_pwr("power:GND", "GND", VX + 15 * GRID, VY + 3.81, sch_uuid))

    # 3 arm sub-sheets (this cluster's share of the 96-mic array, reused from
    # pcb/mic_array/'s per-mic wiring via make_arm() — see module docstring).
    for k in range(3):
        arm_idx = idx * 3 + k
        ay = AY0 + k * (AH + ARM_GAP)
        arm_sheet_uuid = _uid()
        page_num = arm_idx + 7   # pages: 1=top, 2-5=clusters, 6=hub, 7-18=arms
        buf.append(
            f'  (sheet\n'
            f'    (at {_f(AX)} {_f(ay)})\n'
            f'    (size {_f(AW)} {_f(AH)})\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (stroke (width 0.1524) (type solid))\n'
            f'    (fill (color 0 0 0 0))\n'
            f'    (uuid "{arm_sheet_uuid}")\n'
            f'    (property "Sheetname" "arm_{arm_idx:02d}"\n'
            f'      (at {_f(AX)} {_f(ay - 1.5)} 0)\n'
            f'      (effects (font (size 1.524 1.524)) (justify left bottom)))\n'
            f'    (property "Sheetfile" "arm_{arm_idx:02d}.kicad_sch"\n'
            f'      (at {_f(AX)} {_f(ay + AH + 1.5)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left top)))\n'
            f'    (instances\n'
            f'      (project "{PROJECT}"\n'
            f'        (path "/{sch_uuid}"\n'
            f'          (page "{page_num}")))))\n'
        )

    buf.append(
        f'  (sheet_instances\n'
        f'    (path "/"\n'
        f'      (page "{idx + 2}")))\n'
        f')\n'
    )
    return ''.join(buf), sch_uuid

# ── hub sheet generator ────────────────────────────────────────────────────────

def make_hub():
    sch_uuid = _uid()
    buf = []
    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "10.0")\n'
        f'  (uuid "{sch_uuid}")\n'
        f'  (paper "A2")\n'
        f'  (title_block\n'
        f'    (title "Multi-FPGA Alternative — Hub (Cmod A7-35T)")\n'
        f'    (comment 1 "4x spoke bus (all on DIP header, no Pmod) -- reassembles 96ch")\n'
        f'    (comment 2 "TCXO forwards 3.072 MHz PDM clock to all 4 clusters over spoke CLK")\n'
        f'    (comment 3 "USB FIFO bridge (FT232H) to Raspberry Pi 5, also on DIP -- no GbE on hub")\n'
        f'  )\n'
        f'  (lib_symbols\n' + _all_lib_symbols() + '  )\n'
    )

    # Same Cmod A7-35T module used in _all_lib_symbols() — all 45 signals (4
    # spokes + FT232H + TCXO) on its DIP header, no Pmod, + VU/GND power.
    hub_pins = (
        [(fpga_pin, f"D{num}", "passive") for fpga_pin, num in CMOD_A7_35T_DIP]
        + [("VU", "D24", "power_in"), ("GND", "D25", "power_in")]
    )
    n = len(hub_pins)

    AX, AY = 118 * GRID, 197 * GRID
    # "A" (assembly/board module) continuing the clusters' A1-A4 — see
    # make_cluster()'s comment on why this isn't "U1" (mic-range collision).
    buf.append(_conn_instance("CMOD_A7_35T", f"A{N_CLUSTERS + 1}", AX, AY, sch_uuid, hub_pins))

    # 4 spokes (32 signals): DIP pins, index 0-31
    for cidx in range(N_CLUSTERS):
        for j, suffix in enumerate(SPOKE_SIGNAL_SUFFIX):
            pin_i = cidx * 8 + j
            net = f"SPOKE{cidx}_{suffix}"
            x, y = _conn_pin_xy(AX, AY, pin_i, n)
            shape = "output" if suffix == "CLK" else "input"
            buf.append(_stub_and_label(x, y, STUB, net, shape=shape))

    # FT232H/TCXO (13 signals): DIP pins, index 32-44
    for k, net in enumerate(USB_TCXO_NETS):
        x, y = _conn_pin_xy(AX, AY, 32 + k, n)
        shape = "input" if net == "TCXO_CLK" else "passive"
        buf.append(_stub_and_label(x, y, STUB, net, shape=shape))

    # Power: VU/GND, DIP pins 24/25 (pin index 45/46)
    x, y = _conn_pin_xy(AX, AY, 45, n)
    buf.append(_stub_and_pwr("power:+5V", "+5V", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(AX, AY, 46, n)
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))

    # TCXO
    tcxo_pins = [
        ("VDD", "1", "power_in"), ("GND", "2", "power_in"),
        ("OE", "3", "input"), ("OUT", "4", "output"),
    ]
    TX, TY = 276 * GRID, 145 * GRID
    buf.append(_conn_instance("TCXO_OSC", "Y1", TX, TY, sch_uuid, tcxo_pins))
    x, y = _conn_pin_xy(TX, TY, 0, 4)
    buf.append(_stub_and_pwr("power:+3V3", "+3V3", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(TX, TY, 1, 4)
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(TX, TY, 2, 4)
    buf.append(_stub_and_pwr("power:+3V3", "+3V3", x, y, STUB, sch_uuid))   # OE tied high (always enabled)
    x, y = _conn_pin_xy(TX, TY, 3, 4)
    buf.append(_stub_and_label(x, y, STUB, "TCXO_CLK", shape="output", label_angle=180))

    # FT232H USB bridge
    ft232h_pins = (
        [(f"D{i}", str(i + 1), "passive") for i in range(8)]
        + [("RXF#", "9", "passive"), ("TXE#", "10", "passive"),
           ("RD#", "11", "passive"), ("WR#", "12", "passive")]
        + [("VCCIO", "13", "power_in"), ("GND", "14", "power_in")]
    )
    FX, FY = 276 * GRID, 232 * GRID
    # "A" (assembly/board module) — was "U2", colliding with the 2nd mic; see
    # make_cluster()'s comment.
    buf.append(_conn_instance("FT232H_BRK", f"A{N_CLUSTERS + 2}", FX, FY, sch_uuid, ft232h_pins))
    ft232h_nets = [f"USB_D{i}" for i in range(8)] + ["USB_RXF_N", "USB_TXE_N", "USB_RD_N", "USB_WR_N"]
    for i, net in enumerate(ft232h_nets):
        x, y = _conn_pin_xy(FX, FY, i, len(ft232h_pins))
        buf.append(_stub_and_label(x, y, STUB, net, shape="passive", label_angle=180))
    x, y = _conn_pin_xy(FX, FY, 12, len(ft232h_pins))
    buf.append(_stub_and_pwr("power:+3V3", "+3V3", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(FX, FY, 13, len(ft232h_pins))
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))

    # VR2: the hub's own +3.3V LDO for FT232H + TCXO, fed from the hub's own
    # +5V (Cmod A7-35T's own VU, above). Unlike +1V8, +3V3 only ever exists
    # on this one board, so no per-cluster-scoped net naming is needed here.
    VX2, VY2 = 108 * GRID, 180 * GRID
    vr_n = len(LDO_PINS)
    buf.append(_conn_instance("LDO_3V3", f"VR{N_CLUSTERS + 1}", VX2, VY2, sch_uuid, LDO_PINS))
    x, y = _conn_pin_xy(VX2, VY2, 0, vr_n)   # pin 1 = GND
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(VX2, VY2, 1, vr_n)   # pin 2 = OUT
    buf.append(_stub_and_pwr("power:+3V3", "+3V3", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(VX2, VY2, 2, vr_n)   # pin 3 = IN
    buf.append(_stub_and_pwr("power:+5V", "+5V", x, y, STUB, sch_uuid))

    buf.append(_cap("C1", VX2 - 15 * GRID, VY2, sch_uuid))
    buf.append(_pwr("power:+5V", "+5V", VX2 - 15 * GRID, VY2 - 3.81, sch_uuid))
    buf.append(_pwr("power:GND", "GND", VX2 - 15 * GRID, VY2 + 3.81, sch_uuid))
    buf.append(_cap("C2", VX2 + 15 * GRID, VY2, sch_uuid))
    buf.append(_pwr("power:+3V3", "+3V3", VX2 + 15 * GRID, VY2 - 3.81, sch_uuid))
    buf.append(_pwr("power:GND", "GND", VX2 + 15 * GRID, VY2 + 3.81, sch_uuid))

    # Power flags: GND/+5V only -- these remain genuinely externally supplied
    # (from the Pi 5, no on-schematic source, see SCHEMATIC_NOTES.md). +3V3
    # and +1V8 used to be flagged here/on arm_00.kicad_sch too, but both now
    # have a real driver (VR2's/VR1's OUT pin), so ERC's "power input not
    # driven" rule is satisfied naturally -- flagging them as well would be
    # redundant/confusing.
    for i, (lib_id, val) in enumerate(
            [("power:GND", "GND"), ("power:+5V", "+5V")]):
        buf.append(_pwr_flag_pair(lib_id, val, 108 * GRID, 84 * GRID + i * 12 * GRID, sch_uuid))

    buf.append(
        '  (text "USB-C/Micro-B cable to Raspberry Pi 5 USB 3.0 port\\n'
        'Standalone: Pi 5 runs beamforming locally\\n'
        'Tethered: Pi 5 relays stream out its own on-board GbE to external host"\n'
        f'    (at {_f(FX + 30)} {_f(FY)} 0)\n'
        '    (effects (font (size 1.27 1.27)) (justify left))\n'
        f'    (uuid "{_uid()}"))\n'
    )

    buf.append(
        f'  (sheet_instances\n'
        f'    (path "/"\n'
        f'      (page "{N_CLUSTERS + 2}")))\n'
        f')\n'
    )
    return ''.join(buf), sch_uuid

# ── top sheet generator ───────────────────────────────────────────────────────

def make_top():
    top_uuid = _uid()
    buf = []
    buf.append(
        f'(kicad_sch\n'
        f'  (version {KI_VER})\n'
        f'  (generator "{PROJECT}_make_schematic")\n'
        f'  (generator_version "10.0")\n'
        f'  (uuid "{top_uuid}")\n'
        f'  (paper "A3")\n'
        f'  (title_block\n'
        f'    (title "Phase 4 — Multi-FPGA (Clustered) Alternative — Top Level")\n'
        f'    (comment 1 "4x cluster (Cmod S7, 24 mics each) + 1x hub (Cmod A7-35T) + Pi 5 via USB")\n'
        f'    (comment 2 "See PHASE4.md #fpga--multi-fpga-clustered-alternative for full reasoning")\n'
        f'    (comment 3 "Dev-board interconnect only -- not the 96-mic array itself (pcb/mic_array/)")\n'
        f'  )\n'
        f'  (lib_symbols)\n'
    )

    SW, SH = 55 * GRID, 32 * GRID
    MARGIN_X, MARGIN_Y = 16 * GRID, 20 * GRID
    SPACING_X = SW + 12 * GRID
    HUB_ROW_GAP = 24 * GRID   # clear space below the cluster row before the hub row

    names = [f"cluster_{i:02d}" for i in range(N_CLUSTERS)] + ["hub"]
    for i, name in enumerate(names):
        if name == "hub":
            sx, sy = MARGIN_X, MARGIN_Y + SH + HUB_ROW_GAP   # own row, not inline with clusters
        else:
            sx, sy = MARGIN_X + i * SPACING_X, MARGIN_Y
        sym_uuid = _uid()
        buf.append(
            f'  (sheet\n'
            f'    (at {_f(sx)} {_f(sy)})\n'
            f'    (size {_f(SW)} {_f(SH)})\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (stroke (width 0.1524) (type solid))\n'
            f'    (fill (color 0 0 0 0))\n'
            f'    (uuid "{sym_uuid}")\n'
            f'    (property "Sheetname" "{name}"\n'
            f'      (at {_f(sx)} {_f(sy - 1.5)} 0)\n'
            f'      (effects (font (size 1.524 1.524)) (justify left bottom)))\n'
            f'    (property "Sheetfile" "{name}.kicad_sch"\n'
            f'      (at {_f(sx)} {_f(sy + SH + 1.5)} 0)\n'
            f'      (effects (font (size 1.27 1.27)) (justify left top)))\n'
            f'    (instances\n'
            f'      (project "{PROJECT}"\n'
            f'        (path "/{top_uuid}"\n'
            f'          (page "{i + 2}")))))\n'
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
    return (
        '{\n'
        '  "meta": {\n'
        '    "filename": "multi_fpga.kicad_pro",\n'
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

    # 12 arm sheets: real per-mic wiring, reused from make_schematic.make_arm().
    # clk_label is cluster-specific (each cluster gets its own forwarded-clock
    # copy), vdd_label likewise gives each cluster its own independent +1.8V
    # rail (fed by that cluster's own local LDO, VR1 -- see make_cluster()),
    # and page_num matches this project's deeper nesting (top -> cluster ->
    # arm). make_arm() bakes in make_schematic.py's own PROJECT ("mic_array")
    # in its symbol-instance paths, so patch that to this project's name.
    for arm_idx in range(N_CLUSTERS * 3):
        cluster_idx = arm_idx // 3
        content, _arm_uuid = make_arm(arm_idx, clk_label=f"C{cluster_idx}_PDM_CLK",
                                       page_num=arm_idx + 7, vdd_label=f"C{cluster_idx}_1V8")
        content = content.replace('(project "mic_array"', f'(project "{PROJECT}"')
        # No PWR_FLAG needed for C{cluster_idx}_1V8: that cluster's own VR1
        # (make_cluster()) drives it with a real output pin.
        path = os.path.join(OUTDIR, f"arm_{arm_idx:02d}.kicad_sch")
        with open(path, 'w') as fh:
            fh.write(content)
        print(f"  {path}")

    for i in range(N_CLUSTERS):
        content, _ = make_cluster(i)
        path = os.path.join(OUTDIR, f"cluster_{i:02d}.kicad_sch")
        with open(path, 'w') as fh:
            fh.write(content)
        print(f"  {path}")

    content, _ = make_hub()
    path = os.path.join(OUTDIR, "hub.kicad_sch")
    with open(path, 'w') as fh:
        fh.write(content)
    print(f"  {path}")

    top_path = os.path.join(OUTDIR, "top.kicad_sch")
    with open(top_path, 'w') as fh:
        fh.write(make_top())
    print(f"  {top_path}")

    pro_path = os.path.join(OUTDIR, f"{PROJECT}.kicad_pro")
    with open(pro_path, 'w') as fh:
        fh.write(make_project())
    print(f"  {pro_path}")

    print(f"\nDone — {N_CLUSTERS} clusters (12 arms, 96 mics total) + 1 hub.")
    print(f"Open {pro_path} in KiCad 10.")
    print("Next steps:")
    print("  1. Run ERC — expected clean; see SCHEMATIC_NOTES.md for accepted warning classes")
    print("  2. Module footprints assigned (DIP-48 socket / FT232H_Breakout / TCXO_Can) -- see pcb/layout_multi_fpga.py")
    print("  3. Cross-check FPGA pin names here against a synthesized .xdc before wiring hardware")

if __name__ == "__main__":
    main()
