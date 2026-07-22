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
pin names (from the Cmod S7 / Arty A7 reference manuals) — not a redraw of
Digilent's own internal board schematic.

Outputs (in pcb/multi_fpga/):
  top.kicad_sch                    — root sheet: 4 cluster sub-sheets + 1 hub sub-sheet
  cluster_00..03.kicad_sch         — Cmod S7 + spoke bus + 3 arm sub-sheets each
  arm_00..11.kicad_sch             — per-mic wiring, reused from make_schematic.make_arm()
  hub.kicad_sch                    — Arty A7-35T + 4 spoke buses + TCXO + USB bridge
  multi_fpga.kicad_pro             — KiCad project file

multi_fpga and mic_array are deliberately separate KiCad projects (mutually
exclusive alternative front-ends, not two parts of one build) — only the
arm-level .kicad_sch generation logic is shared, via make_arm(), not the
project files themselves.

Usage (from project root):
  python pcb/make_schematic_multi_fpga.py
"""

import os
import re
import sys
import uuid as _uuid_mod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_schematic as ms  # noqa: E402 — shared #PWR counter, see _pref() below
from make_schematic import make_arm, GRID  # noqa: E402 — reuse the primary design's per-mic wiring

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

# Arty A7: 4 Pmod connectors, 200 ohm series resistors on every pin (JA/JD
# "standard"; JB/JC "high-speed" with 0-ohm shunts instead — irrelevant here
# since we no longer need differential signaling on the hub either).
ARTY_PMOD = {
    "JA": ["G13", "B11", "A11", "D12", "D13", "B18", "A18", "K16"],
    "JB": ["E15", "E16", "D15", "C15", "J17", "J18", "K15", "J15"],
    "JC": ["U12", "V12", "V10", "V11", "U14", "V14", "T13", "U13"],
    "JD": ["D4",  "D3",  "F4",  "F3",  "E2",  "D2",  "H2",  "G2"],
}
ARTY_PMOD_ORDER = ["JA", "JB", "JC", "JD"]  # spoke 0,1,2,3

# Spoke bus signal order matching Pmod position index 0..7 (see PHASE4.md
# "Spoke link"): 6 data bits + 1 strobe + 1 forwarded PDM clock (hub->cluster).
SPOKE_SIGNAL_SUFFIX = ["D0", "D1", "D2", "D3", "D4", "D5", "STROBE", "CLK"]

# Arty shield connector (49 GPIO, "IOx" names per RM section 11) — subset used
# for the FT232H USB-FIFO bridge (8 data + 4 control) and the TCXO clock input.
SHIELD_PINS_USED = [
    ("IO0", "USB_D0"), ("IO1", "USB_D1"), ("IO2", "USB_D2"), ("IO3", "USB_D3"),
    ("IO4", "USB_D4"), ("IO5", "USB_D5"), ("IO6", "USB_D6"), ("IO7", "USB_D7"),
    ("IO8", "USB_RXF_N"), ("IO9", "USB_TXE_N"), ("IO10", "USB_RD_N"), ("IO11", "USB_WR_N"),
    ("IO12", "TCXO_CLK"),
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

def _inject_pwr_flag_onto_net(content, sch_uuid, lib_id):
    """Stack a PWR_FLAG onto an existing power-symbol instance already present
    in a generated arm sheet's content (used for +1V8, the mic-local supply
    rail — see main()). +1V8 has no natural home at the cluster/hub level, and
    a fresh isolated flag pair for a net with no other same-named usage in
    that file isn't reliably recognized as connected by ERC; stacking onto an
    already-used instance avoids that."""
    m = re.search(
        r'\(symbol \(lib_id "' + re.escape(lib_id) + r'"\)\n'
        r'    \(at ([\d.\-]+) ([\d.\-]+) 0\)',
        content)
    if not m:
        raise ValueError(f"no existing {lib_id} instance found to attach a PWR_FLAG to")
    x, y = float(m.group(1)), float(m.group(2))
    content = content.replace('  (lib_symbols\n', '  (lib_symbols\n' + _lib_flag(), 1)
    content = content.replace('  (sheet_instances\n', _flag(x, y, sch_uuid) + '  (sheet_instances\n', 1)
    return content

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
        + _pp("Footprint", "", 0, 0, hide=True)
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
        f'    (property "Footprint" ""\n'
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
    )
    arty_pins = []
    for pmod in ARTY_PMOD_ORDER:
        for fpga_pin, pos in zip(ARTY_PMOD[pmod], [1, 2, 3, 4, 7, 8, 9, 10]):
            arty_pins.append((f"{pmod}_{fpga_pin}", f"{pmod}{pos}", "passive"))
    for shield_name, _ in SHIELD_PINS_USED:
        arty_pins.append((shield_name, f"SH_{shield_name}", "passive"))
    arty_pins += [("5V0", "PWR1", "power_in"), ("GND", "PWR2", "power_in")]

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
        + _conn_lib("CMOD_S7", "U", "Digilent Cmod S7 (XC7S25-1CSGA225C)", cmod_pins,
                    "Cluster FPGA module — Pmod JA (8 sig, 200ohm series) + DIP header "
                    "(13 of 32 GPIO used for local PDM capture); every pin series-resistor "
                    "protected, no differential-capable I/O (see PHASE4.md Spoke link)")
        + _conn_lib("ARTY_A7_35T", "U", "Digilent Arty A7-35T (XC7A35TICSG324-1L)", arty_pins,
                    "Hub FPGA module — 4x Pmod (32 pins, one per cluster spoke) + 13 shield "
                    "pins (USB FIFO bridge + TCXO clock in); on-board 10/100 Ethernet PHY "
                    "(TI DP83848J) left unused, see PHASE4.md Host interface")
        + _conn_lib("FT232H_BRK", "U", "FTDI FT232H USB-FIFO breakout", ft232h_pins,
                    "USB 2.0 Hi-Speed synchronous 245-mode FIFO bridge to Raspberry Pi 5 "
                    "USB 3.0 port (device is USB2-speed, ~320 Mbps, backward compatible)")
        + _conn_lib("TCXO_OSC", "Y", "12.288 MHz TCXO", tcxo_pins,
                    "Master clock; NDK NZ2520SD or TXC 7M series, +-2.5ppm or better")
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
        f'  (lib_symbols\n' + _all_lib_symbols() + '  )\n'
    )

    cmod_pins = (
        [("PDM_CLK", "D1", "passive")]
        + [(f"PDM_D{i:02d}", f"D{i+2}", "passive") for i in range(12)]
        + [("VU", "D24", "power_in"), ("GND", "D25", "power_in")]
        + [(fpga_pin, f"JA{pos}", "passive")
           for fpga_pin, pos in zip(CMOD_S7_PMOD_JA, [1, 2, 3, 4, 7, 8, 9, 10])]
    )
    n = len(cmod_pins)
    CX, CY = 94 * GRID, 118 * GRID
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

    # 3 arm sub-sheets (this cluster's share of the 96-mic array, reused from
    # pcb/mic_array/'s per-mic wiring via make_arm() — see module docstring).
    AW, AH = 47 * GRID, 24 * GRID
    AX, AY0 = 16 * GRID, 32 * GRID
    for k in range(3):
        arm_idx = idx * 3 + k
        ay = AY0 + k * (AH + 12 * GRID)
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
        f'    (title "Multi-FPGA Alternative — Hub (Arty A7-35T)")\n'
        f'    (comment 1 "4x spoke bus (one per cluster, via Pmod JA-JD) -- reassembles 96ch")\n'
        f'    (comment 2 "TCXO forwards 3.072 MHz PDM clock to all 4 clusters over spoke CLK")\n'
        f'    (comment 3 "USB FIFO bridge (FT232H) to Raspberry Pi 5 -- no GbE MAC/PHY on hub")\n'
        f'  )\n'
        f'  (lib_symbols\n' + _all_lib_symbols() + '  )\n'
    )

    arty_pins = []
    for pmod in ARTY_PMOD_ORDER:
        for fpga_pin, pos in zip(ARTY_PMOD[pmod], [1, 2, 3, 4, 7, 8, 9, 10]):
            arty_pins.append((f"{pmod}_{fpga_pin}", f"{pmod}{pos}", "passive"))
    for shield_name, _ in SHIELD_PINS_USED:
        arty_pins.append((shield_name, f"SH_{shield_name}", "passive"))
    arty_pins += [("5V0", "PWR1", "power_in"), ("GND", "PWR2", "power_in")]
    n = len(arty_pins)

    AX, AY = 118 * GRID, 197 * GRID
    # "A" (assembly/board module) continuing the clusters' A1-A4 — see
    # make_cluster()'s comment on why this isn't "U1" (mic-range collision).
    buf.append(_conn_instance("ARTY_A7_35T", f"A{N_CLUSTERS + 1}", AX, AY, sch_uuid, arty_pins))

    # 4x Pmod spoke buses: pins 0..31, 8 per cluster in ARTY_PMOD_ORDER order
    for cidx, pmod in enumerate(ARTY_PMOD_ORDER):
        for j, suffix in enumerate(SPOKE_SIGNAL_SUFFIX):
            pin_i = cidx * 8 + j
            net = f"SPOKE{cidx}_{suffix}"
            x, y = _conn_pin_xy(AX, AY, pin_i, n)
            shape = "output" if suffix == "CLK" else "input"
            buf.append(_stub_and_label(x, y, STUB, net, shape=shape))

    # Shield pins 32..44: USB FIFO bus + TCXO clock in
    base = 32
    for k, (shield_name, net) in enumerate(SHIELD_PINS_USED):
        x, y = _conn_pin_xy(AX, AY, base + k, n)
        shape = "input" if net == "TCXO_CLK" else "passive"
        buf.append(_stub_and_label(x, y, STUB, net, shape=shape))

    # Power
    x, y = _conn_pin_xy(AX, AY, base + len(SHIELD_PINS_USED), n)
    buf.append(_stub_and_pwr("power:+5V", "+5V", x, y, STUB, sch_uuid))
    x, y = _conn_pin_xy(AX, AY, base + len(SHIELD_PINS_USED) + 1, n)
    buf.append(_stub_and_pwr("power:GND", "GND", x, y, STUB, sch_uuid))

    # TCXO
    tcxo_pins = [
        ("VDD", "1", "power_in"), ("GND", "2", "power_in"),
        ("OE", "3", "input"), ("OUT", "4", "output"),
    ]
    TX, TY = 252 * GRID, 47 * GRID
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
    FX, FY = 252 * GRID, 134 * GRID
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

    # Power flags: one pair per rail used at this level (GND, +5V, +3V3).
    # Power-symbol nets are global by name, so placing these once here
    # satisfies ERC's "power input not driven" rule hierarchy-wide — no
    # regulator/source symbol is modeled at this dev-board-interconnect scope
    # (see SCHEMATIC_NOTES.md). +1V8 (mic-local supply) is flagged separately
    # in arm_00.kicad_sch, where it's already used, rather than as a fresh
    # isolated island here — see main()/_inject_pwr_flag_onto_net().
    for i, (lib_id, val) in enumerate(
            [("power:GND", "GND"), ("power:+5V", "+5V"), ("power:+3V3", "+3V3")]):
        buf.append(_pwr_flag_pair(lib_id, val, 24 * GRID, 16 * GRID + i * 12 * GRID, sch_uuid))

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
        f'    (comment 1 "4x cluster (Cmod S7, 24 mics each) + 1x hub (Arty A7-35T) + Pi 5 via USB")\n'
        f'    (comment 2 "See PHASE4.md #fpga--multi-fpga-clustered-alternative for full reasoning")\n'
        f'    (comment 3 "Dev-board interconnect only -- not the 96-mic array itself (pcb/mic_array/)")\n'
        f'  )\n'
        f'  (lib_symbols)\n'
    )

    SW, SH = 55 * GRID, 32 * GRID
    MARGIN_X, MARGIN_Y = 16 * GRID, 20 * GRID
    SPACING_X = SW + 12 * GRID

    names = [f"cluster_{i:02d}" for i in range(N_CLUSTERS)] + ["hub"]
    for i, name in enumerate(names):
        sx = MARGIN_X + i * SPACING_X
        sy = MARGIN_Y
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
    # copy) and page_num matches this project's deeper nesting (top -> cluster
    # -> arm). make_arm() bakes in make_schematic.py's own PROJECT ("mic_array")
    # in its symbol-instance paths, so patch that to this project's name.
    for arm_idx in range(N_CLUSTERS * 3):
        cluster_idx = arm_idx // 3
        content, arm_uuid = make_arm(arm_idx, clk_label=f"C{cluster_idx}_PDM_CLK", page_num=arm_idx + 7)
        content = content.replace('(project "mic_array"', f'(project "{PROJECT}"')
        if arm_idx == 0:
            # +1V8 (mic-local supply) has no natural home at the cluster/hub
            # level, unlike GND/+5V/+3V3 — flag it here, where it's already
            # used, instead of as a fresh isolated island (see
            # _inject_pwr_flag_onto_net()).
            content = _inject_pwr_flag_onto_net(content, arm_uuid, "power:+1V8")
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
    print("  1. Run ERC; expected: unconnected DP83848J PHY pins (unused, see PHASE4.md)")
    print("  2. Assign real footprints if this is laid out as a carrier PCB, not point-to-point wiring")
    print("  3. Cross-check FPGA pin names here against a synthesized .xdc before wiring hardware")

if __name__ == "__main__":
    main()
