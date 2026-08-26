#!/usr/bin/env python3
"""
fix_camera_mount_holes.py -- one-time fix: HCAM1-4 on
pcb/single_fpga/top.kicad_pcb were placed at a symmetric +-7.2/+-7.25mm
rectangle (place_mic_array.py's original CAMERA_MOUNT_HOLE_HALF_X/Y_MM),
which was a MISREAD of the Pi Camera Module 3 Wide's real mounting-hole
pattern. The real pattern (re-measured this session directly from Raspberry
Pi's own product-brief vector drawing, RP-008151-DS, at 600dpi, pixel-
precise, cross-validated two independent ways -- direct hole-circle
centroid measurement, and edge-inset arithmetic against the drawing's own
printed 25 x 23.862mm board size) is NOT symmetric around the lens:

  - Real X spacing: 21.0mm (holes at lens-centre +-10.5mm), not 14.4mm.
  - Real Y: the lens sits level with the TOP hole pair (0mm offset), and
    the BOTTOM hole pair is a further 12.5mm out -- not a symmetric
    +-7.25mm rectangle. (The board's FPC-connector notch eats into one
    edge asymmetrically, which is what pulls the near hole pair in level
    with the lens instead of centred between both pairs.)

See place_mic_array.py's own header docstring for the full derivation.
This script re-derives the disc's real current centre from HCAM1-4's own
ORIGINAL (wrong) offsets -- same back-solve technique
recenter_enclosure_holes.py already used for HENC1-4 -- so it's independent
of any manual re-centring the user has done in KiCad since these holes
were first placed, then repositions (via SetPosition, not recreate) each
hole to its real, corrected location relative to that same real centre.

Provisional orientation choice (not re-derivable from the datasheet, see
place_mic_array.py's docstring): HCAM1/HCAM2 keep their original "near"
role (now Y=0, level with the lens) and HCAM3/HCAM4 keep their original
"far" role (now Y=+12.5mm) -- i.e. this only fixes the real distances, not
which side of the disc the camera module's FPC-connector notch ends up
facing. Flip the whole group in KiCad afterward if the notch needs to face
the other way for your ribbon-cable routing.

Usage (from project root -- must use /usr/bin/python3, not this project's
venv):
  /usr/bin/python3 pcb/single_fpga/fix_camera_mount_holes.py
"""

import os

import pcbnew

ROOT = os.path.dirname(os.path.abspath(__file__))
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")

# Original (wrong) offsets HCAM1-4 were placed at -- used only to back-solve
# the disc's real current centre (see recenter_enclosure_holes.py for the
# identical technique/reasoning). NOT the fix itself.
HCAM_ORIGINAL_OFFSETS = {
    "HCAM1": (-7.2, -7.25),
    "HCAM2": (7.2, -7.25),
    "HCAM3": (-7.2, 7.25),
    "HCAM4": (7.2, 7.25),
}

# Real corrected offsets -- see header docstring for the real-drawing
# derivation.
HALF_X_MM = 10.5
NEAR_Y_MM = 0.0
FAR_Y_MM = 12.5
NEW_OFFSETS = {
    "HCAM1": (-HALF_X_MM, NEAR_Y_MM),
    "HCAM2": (HALF_X_MM, NEAR_Y_MM),
    "HCAM3": (-HALF_X_MM, FAR_Y_MM),
    "HCAM4": (HALF_X_MM, FAR_Y_MM),
}


def main():
    board = pcbnew.LoadBoard(BOARD_PATH)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    for ref in HCAM_ORIGINAL_OFFSETS:
        assert ref in fps, f"{BOARD_PATH} has no {ref} footprint -- nothing to fix"

    centres = []
    for ref, (ox, oy) in HCAM_ORIGINAL_OFFSETS.items():
        pos = fps[ref].GetPosition()
        cx = pos.x / 1e6 - ox
        cy = pos.y / 1e6 - oy
        centres.append((cx, cy))

    cx0, cy0 = centres[0]
    for cx, cy in centres[1:]:
        assert abs(cx - cx0) < 0.001 and abs(cy - cy0) < 0.001, (
            f"HCAM1-4 don't agree on a single real centre -- {centres} "
            "(expected all 4 to back-solve to the same point)"
        )
    print(f"Real camera/lens centre recovered from HCAM1-4: ({cx0:.4f}, {cy0:.4f})mm")

    for ref, (dx, dy) in NEW_OFFSETS.items():
        fps[ref].SetPosition(pcbnew.VECTOR2I_MM(cx0 + dx, cy0 + dy))
        print(f"  {ref} -> ({cx0 + dx:.4f}, {cy0 + dy:.4f})mm")

    board.Save(BOARD_PATH)
    print(f"Board saved to {BOARD_PATH}.")


if __name__ == "__main__":
    main()
