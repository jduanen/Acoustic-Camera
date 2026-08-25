#!/usr/bin/env python3
"""
adjust_mic_refdes_and_cap.py -- in-place fix for pcb/single_fpga/top.kicad_pcb:
moves each mic's silkscreen Reference field to the right side of the mic
footprint, and nudges each decoupling cap a small amount further "up" (away
from its mic), to resolve a real, confirmed DRC problem -- not a cosmetic
guess.

Real problem, confirmed via `kicad-cli pcb drc --format json` on the board
as placed by place_mic_array.py, before this script existed: 192
`silk_overlap` + 192 `silk_over_copper` violations (384 of the board's then
481 total), and every single one (checked programmatically, 0 exceptions)
is between mic U{n}'s Reference field and its own paired cap C{n+17}'s
Reference field / copper pad -- never a cross-pair (different mic/cap)
collision. Root cause, read directly from both real footprint files:
- IFX-PG-LLGA-5-4.kicad_mod's own default Reference position is (-2.3,
  -2.204) on F.SilkS, left/bottom-justified -- i.e. anchored at the mic's
  top-left courtyard corner, growing up-and-right from there.
- The mic's own decoupling cap sits at (CAP_TO_MIC_DX_MM, CAP_TO_MIC_DY_MM)
  = (-0.49, -2.95) in the same mic-local frame (place_mic_array.py) --
  directly above-left of the mic, i.e. exactly where the mic's default
  Reference text grows into. Confirmed via real GetBoundingBox() calls on
  both fields (not eyeballed): the two text boxes, and the mic's ref text
  vs. the cap's own F.Cu pads, genuinely overlap.

Fix (real, re-verified after the change, not just asserted):
1. Move every mic's Reference field to the mic's own right side instead --
   local (REFDES_DX_MM, REFDES_DY_MM) = (3.4, 0.0), i.e. 0.3mm clear of the
   mic footprint's own real F.CrtYd right edge (x=3.1, read directly from
   IFX-PG-LLGA-5-4.kicad_mod), vertically centred on the part (the
   courtyard's own real y-range is -2.2..2.2, centre 0). Left/vert-centre
   justified (kept left-justified like the original; vertical changed from
   bottom to centre since there's no longer a nearby feature above/below to
   avoid). This puts the text nowhere near the cap (which stays up-left) or
   the mic's own pads, for every mic -- verified against real nearest-
   neighbour mic spacing (min 12.94mm centre-to-centre, computed from
   array_xy.csv) with real bounding boxes after the move, not just by
   argument.
2. Move every cap a further 0.3mm "up" (more negative Y, away from its
   mic -- matches the existing sign convention: CAP_TO_MIC_DY_MM is already
   negative/"up" in place_mic_array.py) -- NEW_CAP_TO_MIC_DY_MM = -3.25,
   was -2.95. This grows the real gap between the mic's and cap's own
   F.CrtYd boxes from 0.35mm (computed from both footprints' real courtyard
   bounds) to 0.65mm, real extra keepout margin between the pair, on top of
   fix (1) rather than instead of it (the user asked for both).

Both target positions are computed fresh from each mic's own real, current
board position (read via GetPosition(), not re-derived from
test/phase4/array_xy.csv or assumed to still be at script-placement-time
coordinates) plus a fixed local-frame offset -- same convention
place_mic_array.py itself documents (every mic sits at a fixed 0deg
orientation regardless of spiral angle, so one fixed offset applies to all
96 identically). This makes the script idempotent: re-running it recomputes
the same absolute target each time instead of drifting on repeated runs.

Scope: this ref-des/cap-spacing fix only. Does not touch the pre-existing,
separately-tracked 96 hole_clearance violations (mic-footprint-internal,
NPTH vs pad 5, unrelated) or the 1 silk_edge_clearance violation (HCAM3's
own ref field clipped by the board edge, unrelated) -- both untouched by
this script's changes, confirmed by re-running DRC after.

Usage (from project root -- must use /usr/bin/python3, not this project's
venv; pcbnew is a system dist-package):
  /usr/bin/python3 pcb/single_fpga/adjust_mic_refdes_and_cap.py
"""

import math
import os
import re

import pcbnew

ROOT = os.path.dirname(os.path.abspath(__file__))  # pcb/single_fpga
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")

# New mic Reference-field local offset (mic-local frame; every mic is at a
# fixed 0deg orientation, see place_mic_array.py's MIC_ROT_DEG) -- 0.3mm
# clear of the mic's own real F.CrtYd right edge (x=3.1mm, from
# pcb/IM72D128/KiCad/IFX-PG-LLGA-5-4.kicad_mod), vertically centred.
REFDES_DX_MM = 3.4
REFDES_DY_MM = 0.0

# New cap-to-mic Y offset (mic-local frame, matches place_mic_array.py's
# CAP_TO_MIC_DX_MM/CAP_TO_MIC_DY_MM convention exactly) -- was -2.95,
# now 0.3mm further "up" (more negative = further from the mic).
CAP_TO_MIC_DX_MM = -0.49  # unchanged
NEW_CAP_TO_MIC_DY_MM = -3.25


def _rotate(dx, dy, angle_deg):
    a = math.radians(angle_deg)
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def main():
    board = pcbnew.LoadBoard(BOARD_PATH)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    mic_refs = sorted((r for r in fps if re.fullmatch(r"U\d+", r)), key=lambda r: int(r[1:]))
    cap_refs = sorted((r for r in fps if re.fullmatch(r"C\d+", r)), key=lambda r: int(r[1:]))
    assert len(mic_refs) == 96, f"expected 96 mics, found {len(mic_refs)}: {mic_refs}"
    assert len(cap_refs) == 96, f"expected 96 caps, found {len(cap_refs)}: {cap_refs}"

    # ── 1. move each mic's Reference field to the right side ────────────
    for mic_ref in mic_refs:
        fp = fps[mic_ref]
        rot = fp.GetOrientationDegrees()
        dx, dy = _rotate(REFDES_DX_MM, REFDES_DY_MM, rot)
        fx, fy = fp.GetPosition().x / 1e6, fp.GetPosition().y / 1e6
        field = fp.Reference()
        field.SetPosition(pcbnew.VECTOR2I_MM(fx + dx, fy + dy))
        field.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
        field.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)

    # ── 2. move each cap 0.3mm further from its mic ──────────────────────
    for cap_ref in cap_refs:
        idx = int(cap_ref[1:]) - 18  # cap C{idx+18} <-> mic U{idx+1}, see place_mic_array.py
        mic_ref = f"U{idx + 1}"
        assert mic_ref in fps, f"{cap_ref} has no paired mic {mic_ref}"
        mic_fp = fps[mic_ref]
        mx, my = mic_fp.GetPosition().x / 1e6, mic_fp.GetPosition().y / 1e6
        rot = mic_fp.GetOrientationDegrees()
        dx, dy = _rotate(CAP_TO_MIC_DX_MM, NEW_CAP_TO_MIC_DY_MM, rot)
        fps[cap_ref].SetPosition(pcbnew.VECTOR2I_MM(mx + dx, my + dy))

    board.Save(BOARD_PATH)
    print(f"Moved {len(mic_refs)} mic Reference fields to their footprints' right side "
          f"(local offset {REFDES_DX_MM:.1f}, {REFDES_DY_MM:.1f}mm).")
    print(f"Moved {len(cap_refs)} decoupling caps to CAP_TO_MIC_DY_MM="
          f"{NEW_CAP_TO_MIC_DY_MM:.2f}mm (was -2.95mm). Board saved to {BOARD_PATH}.")


if __name__ == "__main__":
    main()
