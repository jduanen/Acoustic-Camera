#!/usr/bin/env python3
"""
place_cdclvc1108_drivers.py -- places the 12 arm-local CDCLVC1108 PDM clock
buffer footprints (U99-U110, one per arm) onto pcb/single_fpga/top.kicad_pcb,
each centred in the real physical gap between that arm's 4th and 5th mic.

Scope: footprint placement only, no routing -- the user routes from here to
the mics by hand (same placement-only precedent as place_mic_array.py and
place_ac7200_connectors.py in this directory).

Real data sources (no invented numbers):
- Mic positions: test/phase4/array_xy.csv (same 96-row real acoustic-array
  geometry every other placement script in this project uses -- see
  place_mic_array.py's own header for full provenance). Grouped by
  arm_idx (0-11); within each arm the 8 rows are already mic_idx-ordered,
  which is also radius-ordered (mic_idx 0 = innermost r=25mm ... mic_idx 7 =
  outermost r=150mm, confirmed directly from the CSV) -- so "4th mic" /
  "5th mic" (1-indexed, per the user's own request wording) are simply
  arm_rows[3] / arm_rows[4].
- Driver position: the real midpoint of the mic4-mic5 line segment (not a
  fixed radius/angle formula -- the array is an Underbrink spiral, not a
  straight radial line: mic_idx 0's own polar angle is 0deg but mic_idx 7's
  is ~41.5deg on the same arm, confirmed directly from the CSV, so "radial
  line" isn't a real description of an arm's shape), PLUS the board's real
  origin offset (see below) -- NOT the raw CSV midpoint on its own. Real
  mic4-mic5 gap: 19.254mm on every arm (confirmed identical across all 12
  by the 12-fold rotational symmetry the whole array is built from),
  comfortably clear of the CDCLVC1108 footprint's own real F.CrtYd extent
  (7.8 x 5.6mm, read directly from
  pcb/libraries/CDCLVC1108/footprints.pretty/PW0016A_N.kicad_mod).
- Board origin offset: array_xy.csv's coordinates are centred on the
  array's own local origin (0,0), but this board's real disc centre on the
  page is (268.863, 205.363)mm -- confirmed directly from the real
  Edge.Cuts bounding box, and independently cross-checked against U1's own
  real placed position (293.863, 205.363 = disc centre + CSV-local
  (25.0, 0.0)). Derived at runtime from U1's real position rather than
  hardcoded (see main() below), so this stays correct even if the board is
  recentred again later -- a hardcoded (0,0) assumption is exactly what
  put every driver in the wrong place (clustered near the page origin
  instead of on the disc) the first time this script ran for real.
- Driver rotation: fixed 0deg (DRIVER_ROT_DEG below) for every driver,
  matching place_mic_array.py's own MIC_ROT_DEG convention -- same "facing
  up" orientation on all 12, confirmed with the user. An earlier version of
  this script instead aligned each driver to its own local mic4->mic5
  segment direction (a per-arm angle, for consistent routing geometry
  arm-to-arm); the user asked for a fixed orientation instead, so that
  choice was dropped. Re-verified via kicad-cli DRC after the change (no
  new courtyard/silkscreen violations from the rotation itself -- see the
  ref-des repositioning below, needed because the previous per-arm-rotation
  fixes for a few instances' Reference text no longer applied once every
  driver rotated to the same fixed angle).
- Driver reference designators: U(99+arm_idx), confirmed against each
  mic_arm_NN.kicad_sch's own already-placed CDCLVC1108 symbol (arm 0 ->
  U99 ... arm 11 -> U110) -- see renumber_mic_arm.py's docstring for how
  this numbering was established project-wide.
- Footprint: CDCLVC1108:PW0016A_N (pcb/libraries/CDCLVC1108/footprints.pretty/
  PW0016A_N.kicad_mod), the exact footprint every mic_arm_NN.kicad_sch's
  CDCLVC1108PW symbol now points at (see the CDCLVC1108: library-prefix fix
  made earlier this session).

Usage (from project root, plain python3 -- must use /usr/bin/python3, not a
venv's python3; pcbnew is a system dist-package, confirmed not importable
from this project's venv):
  /usr/bin/python3 pcb/single_fpga/tools/place_cdclvc1108_drivers.py
"""

import csv
import os

import pcbnew

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))       # pcb/single_fpga/tools
ROOT = os.path.dirname(TOOLS_DIR)                             # pcb/single_fpga
REPO_ROOT = os.path.dirname(os.path.dirname(ROOT))             # repo root
CSV_PATH = os.path.join(REPO_ROOT, "test", "phase4", "array_xy.csv")
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")

LIB_ROOT = os.path.join(ROOT, "..", "libraries")
DRIVER_FP_LIB = os.path.join(LIB_ROOT, "CDCLVC1108", "footprints.pretty")
DRIVER_FP_NAME = "PW0016A_N"
DRIVER_ROT_DEG = 0.0  # every driver "facing up", same orientation on all 12 -- confirmed with the user


def load_fp(lib_path, name, ref, x_mm, y_mm, rot_deg=0.0, lib_nickname=None):
    fp = pcbnew.FootprintLoad(lib_path, name)
    if fp is None:
        raise RuntimeError(f"footprint not found: {lib_path} / {name}")
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
    fp.SetOrientationDegrees(rot_deg)
    if lib_nickname:
        # FootprintLoad() by raw path (not by project fp-lib-table nickname)
        # leaves FPID's library nickname empty -- same "footprint_link_issues:
        # library ''" gap already found and fixed on the schematic side
        # earlier this session. Set it explicitly so the placed footprint
        # stays linked to its real fp-lib-table entry.
        fp.SetFPID(pcbnew.LIB_ID(lib_nickname, name))
    return fp


def footprint_bbox_mm(fp):
    bb = fp.GetBoundingBox(False)
    return (bb.GetLeft() / 1e6, bb.GetTop() / 1e6, bb.GetRight() / 1e6, bb.GetBottom() / 1e6)


def rects_overlap(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return ax0 < bx1 and ax1 > bx0 and ay0 < by1 and ay1 > by0


def main():
    rows = list(csv.DictReader(open(CSV_PATH, newline="")))
    assert len(rows) == 96, f"expected 96 mics in {CSV_PATH}, found {len(rows)}"
    by_arm = {}
    for r in rows:
        by_arm.setdefault(int(r["arm_idx"]), []).append(r)
    assert set(by_arm) == set(range(12)), sorted(by_arm)

    board = pcbnew.LoadBoard(BOARD_PATH)

    # Idempotency guard -- same pattern/reasoning as place_ac7200_connectors.py's
    # own guard (this pcbnew binding's board.Remove() poisons later
    # FootprintLoad()/board.Add() calls, confirmed there directly).
    existing_refs = {fp.GetReference() for fp in list(board.GetFootprints())}
    if "U99" in existing_refs:
        raise RuntimeError(
            f"{BOARD_PATH} already has U99 -- refusing to run again "
            "(this script's own idempotency guard). Remove the existing "
            "U99-U110 CDCLVC1108 footprints by hand in KiCad first if you "
            "want to regenerate."
        )

    # Real courtyard boxes of everything already on the board, for the real
    # post-placement overlap check below (same technique place_mic_array.py
    # uses for its own 192-footprint placement pass).
    existing_boxes = [(fp.GetReference(), footprint_bbox_mm(fp)) for fp in board.GetFootprints()]

    # Real board-origin offset, derived from U1's own real placed position on
    # THIS board vs its known CSV-local position (mic_idx=0, arm_idx=0 ->
    # (25.0, 0.0)mm) -- NOT assumed to be (0,0). array_xy.csv's own
    # coordinates are centred on the array's own local origin, but this
    # board's actual disc was recentred on the page after place_mic_array.py
    # first generated it (confirmed: real Edge.Cuts bounding-box centre is
    # (268.863, 205.363)mm, not (0,0) -- the same real centre already
    # established independently via HCAM1-4's own recovered position, see
    # fix_camera_mount_holes.py). A prior version of this script placed
    # drivers at raw CSV-local coordinates directly and put all 12 in the
    # wrong place (clustered near the page origin instead of on the disc) --
    # deriving the offset from the board's own live state instead of a
    # hardcoded constant means this stays correct even if the board is
    # recentred again later.
    existing_fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    if "U1" not in existing_fps:
        raise RuntimeError("U1 not found on board -- can't derive the real board-origin offset")
    u1_pos = existing_fps["U1"].GetPosition()
    offset_x = u1_pos.x / 1e6 - 25.0
    offset_y = u1_pos.y / 1e6 - 0.0
    print(f"real board-origin offset (from U1): ({offset_x:.6f}, {offset_y:.6f})mm")

    placed = []
    for arm in range(12):
        arm_rows = sorted(by_arm[arm], key=lambda r: int(r["mic_idx"]))
        assert len(arm_rows) == 8
        x4, y4 = float(arm_rows[3]["x_mm"]), float(arm_rows[3]["y_mm"])  # 4th mic (1-indexed)
        x5, y5 = float(arm_rows[4]["x_mm"]), float(arm_rows[4]["y_mm"])  # 5th mic (1-indexed)
        mx, my = (x4 + x5) / 2.0 + offset_x, (y4 + y5) / 2.0 + offset_y
        rot_deg = DRIVER_ROT_DEG

        ref = f"U{99 + arm}"
        fp = load_fp(DRIVER_FP_LIB, DRIVER_FP_NAME, ref, mx, my, rot_deg=rot_deg,
                     lib_nickname="CDCLVC1108")
        board.Add(fp)
        placed.append((ref, footprint_bbox_mm(fp)))

    # Real overlap check: every newly-placed driver's courtyard against every
    # other footprint already on the board (mics, caps, camera holes, ...)
    # and against each other.
    all_boxes = existing_boxes + placed
    n_checks = 0
    for i, (ref_a, box_a) in enumerate(placed):
        for ref_b, box_b in all_boxes:
            if ref_b == ref_a:
                continue
            n_checks += 1
            if rects_overlap(box_a, box_b):
                raise RuntimeError(f"courtyard overlap: {ref_a} collides with {ref_b}")

    board.Save(BOARD_PATH)

    print(f"Placed {len(placed)} CDCLVC1108 drivers (U99-U110), one per arm, "
          f"centred between each arm's 4th and 5th mic, all facing up "
          f"(rot={DRIVER_ROT_DEG}deg). {n_checks} courtyard-overlap checks, "
          f"all clear. Board saved to {BOARD_PATH}.")
    for ref, (x0, y0, x1, y1) in placed:
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        print(f"  {ref}: centre ({cx:.3f}, {cy:.3f})mm")


if __name__ == "__main__":
    main()
