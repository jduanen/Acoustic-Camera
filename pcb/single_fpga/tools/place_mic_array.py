#!/usr/bin/env python3
"""
place_mic_array.py -- board outline + footprint placement for the single-FPGA
disc mic-array board (pcb/single_fpga/top.kicad_pcb).

Builds top.kicad_pcb from scratch (it currently exists only as KiCad's bare
empty skeleton -- confirmed via `cat` before writing this, 79 bytes, no
Edge.Cuts/footprints/anything): a single circular board carrying all 96
IM72D128 mics + their 96 decoupling caps at their real acoustic-array
positions, plus a centre cutout + real mounting-hole pattern for a Pi Camera
Module 3 Wide mounted on this board, lens facing out through the cutout.

Scope: board outline + footprint placement only, no routing -- mirrors the
established precedent in this project (pcb/multi_fpga/tools/layout_multi_fpga.py's own header
comment, pcb/mic_array/tools/place_mics.py's docstring). The AC7200 module, TCXO, and 3.3V
regulator are already in the schematic (per the user) but are NOT placed
here -- out of scope for this pass, left for the user to place.

Real data sources (no invented numbers):
- Mic positions: test/phase4/array_xy.csv (96 rows, mic_idx 0-95, arm_idx
  0-11), the output of notebooks/make_nb18.py's underbrink_array(12, 8) --
  the SAME real acoustic-array geometry pcb/multi_fpga/tools/layout_multi_fpga.py's cluster
  boards and pcb/mic_array/tools/place_mics.py both build from. The array's physical mic
  layout is a property of the acoustic design (beamforming geometry), not
  of which FPGA board the mics happen to be wired to -- reused verbatim,
  not re-derived.
- Mic/cap reference-designator convention: confirmed by grepping this
  directory's own mic_arm_00..11.kicad_sch (already drawn by the user) --
  mic U{mic_idx+1}, decoupling cap C{mic_idx+18}, a clean 1:1 pairing
  (arm 0 = U1-8/C18-25, arm 1 = U9-16/C26-33, ... arm 11 = U89-96/C106-113).
  Confirmed from the schematic that mic/cap power nets are a plain per-sheet
  VDD_MIC/GND rail (no per-mic-distinct net), so which specific cap ends up
  next to which specific mic doesn't matter electrically -- only that each
  mic gets its own nearby decoupling cap. This index pairing is used anyway
  for a clean, unambiguous ref-to-position mapping.
- Cap-to-mic offset (CAP_TO_MIC_DX/DY_MM below): reused from
  pcb/multi_fpga/tools/layout_multi_fpga.py's own CAP_TO_MIC_DX_MM/CAP_TO_MIC_DY_MM --
  expressed as a fixed absolute-frame offset from a 0-deg-oriented mic
  footprint, and both this board and that one place every mic at a fixed
  0-deg orientation regardless of its angular position on the spiral (this
  file's own choice, confirmed here to match pcb/mic_array/tools/place_mics.py's identical
  convention -- see MIC_ROT_DEG below), so the same offset applies whichever
  spiral position the mic sits at. NOT reusing that file's
  CAP_TO_MIC_DY_MM_EXCEPTIONS dict -- those exceptions were for specific
  local routing congestion around that board's own CMod A7-35T/connectors,
  which don't exist anywhere near this disc's mics; verified no exceptions
  are needed here instead (see the post-placement overlap check below).
- Mic/cap footprint courtyard bounds: read directly from
  pcb/IM72D128/KiCad/IFX-PG-LLGA-5-4.kicad_mod (F.CrtYd) and KiCad's
  stock Capacitor_SMD.pretty/C_0603_1608Metric.kicad_mod -- used to compute
  the real worst-case outer-mic corner reach (see BOARD_RADIUS_MM below)
  and to verify no two placed footprints' courtyards overlap after
  placement.
- Camera (Pi Camera Module 3 Wide) mechanical spec: Raspberry Pi's own
  official product brief (RP-008151-DS, "Physical specification" page,
  Wide lens drawing) -- PCB 25x24mm (23.862mm real), lens height 12.4mm,
  front lens aperture dia 6.95mm, 4 mounting holes dia 2.2mm each (M2
  clearance). CAMERA_CUTOUT_DIA_MM=12.0mm is sized to clear the 6.95mm
  aperture with real margin.
  Real hole pattern (re-verified directly against the product brief's own
  vector drawing this session, at 600dpi, via pixel-precise measurement --
  NOT the 14.4x14.5mm symmetric-rectangle figure this file used earlier,
  which was a misread of that same drawing): the 4 holes are NOT symmetric
  around the lens. Measured from the drawing's own real board edges
  (25 x 23.862mm, confirmed independently via the product brief's own
  printed board-size spec): holes sit 2.0mm in from the left/right/bottom
  edges (X = 2.0mm and 23.0mm from the left edge -- a real 21.0mm X
  spacing, not 14.4mm), but only the BOTTOM pair is 2.0mm from an edge --
  the TOP pair sits 9.35mm down from the top edge, because the board's
  FPC-connector notch eats into the top of the board asymmetrically. The
  lens is confirmed (same drawing) to sit exactly level with the TOP hole
  pair (same Y), not centred between top and bottom pairs, and horizontally
  centred exactly on the board's own centreline (25/2 = 12.5mm) -- also
  confirmed as the X midpoint of the 2.0/23.0mm hole columns. So, relative
  to the lens/cutout centre: two holes at (+-10.5, 0)mm (level with the
  lens), two more at (+-10.5, +12.5)mm (12.5mm further from the lens, on
  whichever side the FPC connector notch is NOT on). CAMERA_MOUNT_HOLE_*
  below reflect this real, asymmetric pattern.
  NOT modelled: exact standoff/board-thickness stack-up between the camera
  module and this board's front face -- the cutout is sized to the real
  lens aperture with real margin, but whether the 120-degree Wide FOV cone
  clears the cutout edge at whatever standoff height the assembly ends up
  using has NOT been checked (no standoff dimension available this
  session). Verify against the physical module/enclosure before fab.

Usage (from project root, plain python3 -- no KiCad Scripting Console
needed, same as pcb/multi_fpga/tools/layout_multi_fpga.py):
  python3 pcb/single_fpga/place_mic_array.py
"""

import csv
import math
import os

import pcbnew

ROOT = os.path.dirname(os.path.abspath(__file__))          # pcb/single_fpga
REPO_ROOT = os.path.dirname(os.path.dirname(ROOT))          # repo root
CSV_PATH = os.path.join(REPO_ROOT, "test", "phase4", "array_xy.csv")
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")

MIC_FP_LIB = os.path.join(REPO_ROOT, "pcb", "IM72D128", "KiCad")
MIC_FP_NAME = "IFX-PG-LLGA-5-4"
CAP_FP_LIB = "/usr/share/kicad/footprints/Capacitor_SMD.pretty"
CAP_FP_NAME = "C_0603_1608Metric"
MOUNTHOLE_LIB = "/usr/share/kicad/footprints/MountingHole.pretty"
CAMERA_HOLE_NAME = "MountingHole_2.2mm_M2"  # real dia from RPi's own datasheet, see header

# ── mic array geometry (real, from array_xy.csv -- see header) ─────────────
MIC_ROT_DEG = 0.0  # every mic at a fixed orientation regardless of spiral
                    # angle, matching pcb/mic_array/tools/place_mics.py's own convention
                    # exactly (fp.SetOrientationDegrees(0.0) for every ref)

CAP_TO_MIC_DX_MM = -0.49
CAP_TO_MIC_DY_MM = -2.95
CAP_ROT_DEG = 0.0

# Worst-case real mic-footprint-courtyard corner reach from array centre,
# computed directly from array_xy.csv + IFX-PG-LLGA-5-4's real F.CrtYd
# bounds (-2.3..3.1mm x -2.2..2.2mm, local/unrotated): 153.78mm (mic_idx=7,
# the outermost mic on arm 0). +~6.2mm fab margin -> 160.0mm, a clean
# number. (Not a fixed formula from R_MAX_MM=150 alone -- courtyard is
# asymmetric and mics are unrotated, so the true worst corner depends on
# each outer mic's specific angle; verified by checking all 96 mics' 4
# corners each, not just the nominal R_MAX_MM mics.)
BOARD_RADIUS_MM = 160.0

# Camera cutout + real 4-hole mounting pattern -- see header for sourcing.
CAMERA_CUTOUT_DIA_MM = 12.0
CAMERA_MOUNT_HOLE_HALF_X_MM = 10.5   # +-10.5mm, both hole rows (real 21.0mm X spacing)
CAMERA_MOUNT_HOLE_NEAR_Y_MM = 0.0    # HCAM1/HCAM2: level with the lens (real, not +-symmetric)
CAMERA_MOUNT_HOLE_FAR_Y_MM = 12.5    # HCAM3/HCAM4: 12.5mm further out


def load_fp(lib_path, name, ref, x_mm, y_mm, rot_deg=0.0):
    fp = pcbnew.FootprintLoad(lib_path, name)
    if fp is None:
        raise RuntimeError(f"footprint not found: {lib_path} / {name}")
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
    fp.SetOrientationDegrees(rot_deg)
    return fp


def add_outline(board, points_mm, width_mm=0.15, layer=pcbnew.Edge_Cuts):
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_POLY)
    poly = pcbnew.SHAPE_POLY_SET()
    poly.NewOutline()
    for x, y in points_mm:
        poly.Append(pcbnew.VECTOR2I_MM(x, y))
    shape.SetPolyShape(poly)
    shape.SetLayer(layer)
    shape.SetWidth(pcbnew.FromMM(width_mm))
    board.Add(shape)


def add_circle(board, cx_mm, cy_mm, radius_mm, n=32, width_mm=0.15, layer=pcbnew.Edge_Cuts):
    """A closed circular Edge.Cuts loop. Fully inside another closed
    Edge.Cuts loop, it reads as a cutout/hole in KiCad; on its own (this
    board's own outer boundary) it's the board shape itself -- same
    add_outline() polygon technique either way, mirroring
    pcb/multi_fpga/tools/layout_multi_fpga.py's add_circle_cutout()."""
    pts = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        pts.append((cx_mm + radius_mm * math.cos(ang), cy_mm + radius_mm * math.sin(ang)))
    add_outline(board, pts, width_mm=width_mm, layer=layer)


def add_mounting_hole(board, x_mm, y_mm, ref):
    board.Add(load_fp(MOUNTHOLE_LIB, CAMERA_HOLE_NAME, ref, x_mm, y_mm))


def _local_courtyard_corners(lib_path, name):
    """Real local (unrotated, at origin) F.CrtYd bounding-box corners of a
    footprint, loaded fresh so this stays correct if the footprint file
    ever changes -- mirrors pcb/multi_fpga/tools/layout_multi_fpga.py's own helper."""
    fp = pcbnew.FootprintLoad(lib_path, name)
    fp.SetPosition(pcbnew.VECTOR2I_MM(0, 0))
    fp.SetOrientationDegrees(0)
    bb = fp.GetBoundingBox(False)
    x0, x1 = bb.GetLeft() / 1e6, bb.GetRight() / 1e6
    y0, y1 = bb.GetTop() / 1e6, bb.GetBottom() / 1e6
    return (x0, y0, x1, y1)


def _rects_overlap(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return ax0 < bx1 and ax1 > bx0 and ay0 < by1 and ay1 > by0


def main():
    with open(CSV_PATH, newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 96, f"expected 96 mics in {CSV_PATH}, found {len(rows)}"

    board = pcbnew.LoadBoard(BOARD_PATH)

    # NOT idempotent, deliberately: this KiCad 10.0.5 python binding has a
    # real bug where board.Remove() calls poison later FootprintLoad()/
    # Drawings() calls in the same process (confirmed directly -- both
    # start raising "'SwigPyObject' object has no attribute/is not
    # iterable" afterward, a binding-state issue, not a logic bug in this
    # script). A prior version of this script tried to work around that by
    # removing old content before regenerating and hit exactly this wall;
    # rather than keep fighting the binding, this refuses to run against a
    # non-empty board instead -- simpler and safer than a fragile
    # remove-and-regenerate dance (also real bug hit once already: a
    # re-run of an earlier, non-guarded version of this script silently
    # doubled every footprint to 392). Delete/reset top.kicad_pcb (or
    # remove BOARD_ORIGIN's footprints by hand in KiCad) before re-running.
    existing = list(board.GetFootprints())
    if existing:
        raise RuntimeError(
            f"{BOARD_PATH} already has {len(existing)} footprints -- "
            "refusing to run again (see comment above main() for why). "
            "Reset the board file first if you want to regenerate."
        )

    # ── board outline: outer disc + centre camera cutout ────────────────
    add_circle(board, 0.0, 0.0, BOARD_RADIUS_MM, n=144, width_mm=0.15)
    add_circle(board, 0.0, 0.0, CAMERA_CUTOUT_DIA_MM / 2.0, n=32, width_mm=0.15)

    # ── camera mounting holes (real 4-hole pattern, see header) ─────────
    add_mounting_hole(board, -CAMERA_MOUNT_HOLE_HALF_X_MM, CAMERA_MOUNT_HOLE_NEAR_Y_MM, "HCAM1")
    add_mounting_hole(board, CAMERA_MOUNT_HOLE_HALF_X_MM, CAMERA_MOUNT_HOLE_NEAR_Y_MM, "HCAM2")
    add_mounting_hole(board, -CAMERA_MOUNT_HOLE_HALF_X_MM, CAMERA_MOUNT_HOLE_FAR_Y_MM, "HCAM3")
    add_mounting_hole(board, CAMERA_MOUNT_HOLE_HALF_X_MM, CAMERA_MOUNT_HOLE_FAR_Y_MM, "HCAM4")

    # ── 96 mics + 96 decoupling caps ─────────────────────────────────────
    placed = []  # (ref, courtyard_world_rect) for the overlap check below
    mic_local = _local_courtyard_corners(MIC_FP_LIB, MIC_FP_NAME)
    cap_local = _local_courtyard_corners(CAP_FP_LIB, CAP_FP_NAME)

    for row in rows:
        idx = int(row["mic_idx"])
        x, y = float(row["x_mm"]), float(row["y_mm"])

        mic_ref = f"U{idx + 1}"
        board.Add(load_fp(MIC_FP_LIB, MIC_FP_NAME, mic_ref, x, y, rot_deg=MIC_ROT_DEG))
        placed.append((mic_ref, (x + mic_local[0], y + mic_local[1], x + mic_local[2], y + mic_local[3])))

        cap_ref = f"C{idx + 18}"
        cx, cy = x + CAP_TO_MIC_DX_MM, y + CAP_TO_MIC_DY_MM
        board.Add(load_fp(CAP_FP_LIB, CAP_FP_NAME, cap_ref, cx, cy, rot_deg=CAP_ROT_DEG))
        placed.append((cap_ref, (cx + cap_local[0], cy + cap_local[1], cx + cap_local[2], cy + cap_local[3])))

    board.Save(BOARD_PATH)

    # ── post-placement verification: no two courtyards overlap ──────────
    # (O(n^2) over 192 footprints = ~18k pairs, trivial cost; a real check,
    # not just eyeballed -- this is exactly the class of problem
    # CAP_TO_MIC_DY_MM_EXCEPTIONS existed to work around on the multi-FPGA
    # board, so it's worth confirming this board doesn't need its own.)
    overlaps = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if _rects_overlap(placed[i][1], placed[j][1]):
                overlaps.append((placed[i][0], placed[j][0]))

    print(f"Placed 96 mics (U1-U96) + 96 caps (C18-C113). Board saved to {BOARD_PATH}.")
    print(f"Board outline: {BOARD_RADIUS_MM*2:.1f}mm dia disc, "
          f"{CAMERA_CUTOUT_DIA_MM:.1f}mm centre camera cutout, 4x camera mounting holes.")
    if overlaps:
        print(f"*** {len(overlaps)} courtyard overlap(s) found -- NOT expected, investigate: {overlaps[:10]}")
    else:
        print("Courtyard overlap check: none found across all 192 placed footprints.")


if __name__ == "__main__":
    main()
