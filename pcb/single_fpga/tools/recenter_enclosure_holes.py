#!/usr/bin/env python3
"""
recenter_enclosure_holes.py -- one-time fix: place_enclosure_holes.py
placed HENC1-4 at literal (+-50, +-50)mm, assuming world (0,0) was the
disc's own centre. The user has since manually moved the whole board's
content in KiCad to actually centre it on the page -- so the disc's real
centre is no longer world (0,0), and HENC1-4 are now offset from the
disc by whatever translation was applied to everything else.

Real disc centre recovered from HCAM1-4 (the 4 camera mounting holes),
NOT hardcoded: those were originally placed at exactly (+-7.2, +-7.25)mm
relative to world (0,0) (see place_mic_array.py's
CAMERA_MOUNT_HOLE_HALF_X/Y_MM) and moved along with everything else, so
back-solving disc_centre = HCAMn_current_pos - HCAMn_original_offset from
all 4 gives the real current centre robustly (confirmed all 4 agree
exactly before trusting it, not just using one).

Usage (from project root -- must use /usr/bin/python3, not this
project's venv):
  /usr/bin/python3 pcb/single_fpga/recenter_enclosure_holes.py
"""

import os

import pcbnew

ROOT = os.path.dirname(os.path.abspath(__file__))
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")

# Original (world-origin-relative) offsets used when HCAM1-4 were placed --
# see place_mic_array.py's CAMERA_MOUNT_HOLE_HALF_X_MM/_Y_MM (7.2, 7.25).
HCAM_ORIGINAL_OFFSETS = {
    "HCAM1": (-7.2, -7.25),
    "HCAM2": (7.2, -7.25),
    "HCAM3": (-7.2, 7.25),
    "HCAM4": (7.2, 7.25),
}

HALF_SIDE_MM = 50.0


def main():
    board = pcbnew.LoadBoard(BOARD_PATH)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    centres = []
    for ref, (ox, oy) in HCAM_ORIGINAL_OFFSETS.items():
        pos = fps[ref].GetPosition()
        cx = pos.x / 1e6 - ox
        cy = pos.y / 1e6 - oy
        centres.append((cx, cy))

    cx0, cy0 = centres[0]
    for cx, cy in centres[1:]:
        assert abs(cx - cx0) < 0.001 and abs(cy - cy0) < 0.001, (
            f"HCAM1-4 don't agree on a single disc centre -- {centres} "
            "(expected all 4 to back-solve to the same point if they were "
            "moved together as one rigid translation)"
        )
    print(f"Real disc centre recovered from HCAM1-4: ({cx0:.4f}, {cy0:.4f})mm")

    corners = {
        "HENC1": (HALF_SIDE_MM, HALF_SIDE_MM),
        "HENC2": (HALF_SIDE_MM, -HALF_SIDE_MM),
        "HENC3": (-HALF_SIDE_MM, HALF_SIDE_MM),
        "HENC4": (-HALF_SIDE_MM, -HALF_SIDE_MM),
    }
    for ref, (dx, dy) in corners.items():
        fps[ref].SetPosition(pcbnew.VECTOR2I_MM(cx0 + dx, cy0 + dy))
        print(f"  {ref} -> ({cx0 + dx:.4f}, {cy0 + dy:.4f})mm")

    board.Save(BOARD_PATH)
    print(f"Board saved to {BOARD_PATH}.")


if __name__ == "__main__":
    main()
