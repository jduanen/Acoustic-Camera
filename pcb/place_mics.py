#!/usr/bin/env python3
"""KiCad scripting console script — place 96 IM72D128 mic footprints.

Workflow:
  1. Draw schematic (see SCHEMATIC_NOTES.md in this directory).
  2. Tools → Update PCB from Schematic → all 96 U* footprints land at (0, 0).
  3. Open Tools → Scripting Console in pcbnew.
  4. Run:
       exec(open('/path/to/place_mics.py').read())

The script reads test/phase4/array_xy.csv and moves each U{n} footprint
to the correct spiral XY position.  Adjust BOARD_ORIGIN_MM if your board
origin differs from the default (160, 120).

Footprint references must be U1–U96 (mic_idx 0 → U1, mic_idx 95 → U96).
"""
import csv
import os
import pcbnew  # only available inside KiCad scripting console

# ── configuration ─────────────────────────────────────────────────────────────

# Centre of the mic array on the PCB (mm).  Adjust to match your board outline.
# Default puts the array centre at (160, 120) on a 320×240 mm board.
BOARD_ORIGIN_MM = (160.0, 120.0)

# Path to the coordinate CSV, relative to the .kicad_pcb file location.
# Change to an absolute path if needed.
_board_dir = os.path.dirname(pcbnew.GetBoard().GetFileName())
CSV_PATH = os.path.join(_board_dir, '..', 'test', 'phase4', 'array_xy.csv')

# ── load coordinates ──────────────────────────────────────────────────────────

with open(CSV_PATH, newline='') as f:
    rows = list(csv.DictReader(f))

# Build reference → (x_mm, y_mm) map.  mic_idx 0 → U1, etc.
coord_map = {}
for row in rows:
    ref = f"U{int(row['mic_idx']) + 1}"
    coord_map[ref] = (float(row['x_mm']), float(row['y_mm']))

# ── move footprints ────────────────────────────────────────────────────────────

board = pcbnew.GetBoard()
placed = 0
skipped = []

for fp in board.GetFootprints():
    ref = fp.GetReference()
    if ref not in coord_map:
        continue
    dx, dy = coord_map[ref]
    x = BOARD_ORIGIN_MM[0] + dx
    y = BOARD_ORIGIN_MM[1] + dy
    fp.SetPosition(pcbnew.VECTOR2I_MM(x, y))
    fp.SetOrientationDegrees(0.0)
    placed += 1

if placed < len(coord_map):
    missing = sorted(set(coord_map) - {fp.GetReference() for fp in board.GetFootprints()})
    print(f"[warn] {len(missing)} refs not found on board: {missing[:5]}{'...' if len(missing)>5 else ''}")

board.Save(board.GetFileName())
pcbnew.Refresh()
print(f"Placed {placed} / {len(coord_map)} mic footprints.  Board saved.")
