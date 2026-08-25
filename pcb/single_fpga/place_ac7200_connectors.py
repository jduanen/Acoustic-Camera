#!/usr/bin/env python3
"""
place_ac7200_connectors.py -- adds a real, self-contained "footprint group"
to pcb/single_fpga/top.kicad_pcb: the 4 carrier-side AXK680337YG header
connectors + 4 real mounting holes that mate with the ALINX AC7200 module,
grouped together as one movable KiCad PCB_GROUP ("AC7200_MOUNT").

Real data sources (no invented numbers):
- Connector centers + rotations, and mounting-hole positions: computed
  directly from pcb/single_fpga/footprints.pretty/AC7200.kicad_mod's own
  real pad data (bounding-box centre of each connector's 80 real SMD pads;
  the 4 real NPTH mounting-hole pads) -- that footprint's own positions
  are themselves real, STEP-model-derived data (see its own `descr` field
  and pcb/single_fpga/AC7200/AC7200_README.md for full provenance), not
  re-derived or guessed here.
- Header connector footprint: pcb/single_fpga/footprints.pretty/
  AXK680337YG.kicad_mod (real Panasonic P5K catalog geometry, built and
  kicad-cli-validated earlier this session -- see that file's own descr).
- Mounting hole size: 2.7mm, matching AC7200.kicad_mod's own 4 real NPTH
  pads exactly (same STANDOFF_HOLE_NAME convention pcb/layout_multi_fpga.py
  already uses elsewhere in this project for M2.5 clearance holes).

Placement choice (NOT derived from any real spec -- a real, but provisional,
design decision, flagged clearly): the group is placed on B.Cu (the BACK
of the disc), not the front. Reasoning: the front (F.Cu) is the mic-facing
side, and the 96-mic Underbrink spiral genuinely covers the full 360
degrees of the disc from r=25mm to r=150mm (confirmed by inspecting the
placed board, not assumed) -- there is no open front-side pocket a 55x45mm
module could sit in without colliding with a mic's courtyard somewhere
around the full circle. Mounting electronics on the back (mics facing the
scene, electronics tucked behind) is also the standard pattern this
project already uses for the multi-FPGA hub (Pi 5 stacks behind the hub
board on its own standoffs). The group's own centre position on the back
(AC7200_MOUNT_XY_MM below) is a simple placeholder chosen only to sit
clear of the centre camera-mount keepout (see place_mic_array.py) --
NOT tied to any real routing/connector-reach analysis to the AC7200's own
GPIO banks. Being a real KiCad group, it can be selected and dragged
anywhere as one unit afterward without breaking the internal real
connector-to-connector/hole-to-hole alignment.

NOT verified this session (flag before fab): the exact mating handedness
(pin-1-to-pin-1 alignment) across the front-to-back Flip() applied to each
header -- this assumes KiCad's standard flip convention produces a
correctly-mating pair with the module's own real socket footprint, but no
physical/3D check was done to confirm it. Also not addressed: AC7200's own
required rotation (if any) once actually placed on the board -- this
script places the connector GROUP using the module's own unrotated (as
extracted) relative geometry; if the final board-level orientation needs
a different rotation, rotate the whole group as one unit (real KiCad group
rotate), not the pieces individually.

Scope: this connector/hole group only -- no routing, no AC7200 footprint
itself (still up to the user to place, aligned to this group, mirroring
the placement-only precedent established elsewhere in this project).

Usage (from project root, plain python3 -- must use /usr/bin/python3, not
a venv's python3; pcbnew is a system dist-package, confirmed not importable
from this project's venv):
  /usr/bin/python3 pcb/single_fpga/place_ac7200_connectors.py
"""

import os
import re

import pcbnew

ROOT = os.path.dirname(os.path.abspath(__file__))  # pcb/single_fpga
BOARD_PATH = os.path.join(ROOT, "top.kicad_pcb")
AC7200_FP_PATH = os.path.join(ROOT, "footprints.pretty", "AC7200.kicad_mod")

HEADER_FP_LIB = os.path.join(ROOT, "footprints.pretty")
HEADER_FP_NAME = "AXK680337YG"
MOUNTHOLE_LIB = "/usr/share/kicad/footprints/MountingHole.pretty"
MOUNTHOLE_NAME = "MountingHole_2.7mm_M2.5"  # matches AC7200's own real 2.7mm NPTH holes

# Provisional placement of the whole group's local origin on the carrier
# board -- see header docstring. Clear of the centre camera-mount keepout
# (camera holes at r=~10.2mm); AC7200's own connector/hole extent reaches
# ~28mm from its own origin, so this leaves real margin either way.
AC7200_MOUNT_XY_MM = (0.0, -75.0)
AC7200_MOUNT_ROT_DEG = 0.0  # whole-group rotation; see header docstring


def _real_ac7200_geometry():
    """Extract real connector centres/rotations + mounting-hole positions
    directly from AC7200.kicad_mod's own pad data (bounding-box centre of
    each connector's 80 pads; the 4 real NPTH pads) -- see header
    docstring for why this is more robust than hardcoding numbers by hand."""
    text = open(AC7200_FP_PATH).read()

    pad_re = re.compile(r'\(pad "(CON\d)_(\d+)" smd rect \(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)')
    from collections import defaultdict
    conns = defaultdict(list)
    for con, pin, x, y, rot in pad_re.findall(text):
        conns[con].append((float(x), float(y), float(rot) if rot else 0.0))

    connectors = {}
    for con, pads in conns.items():
        xs = [p[0] for p in pads]
        ys = [p[1] for p in pads]
        rots = {p[2] for p in pads}
        assert len(rots) == 1, f"{con} pads have mixed rotations: {rots}"
        connectors[con] = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0, rots.pop())

    hole_re = re.compile(r'\(pad "" np_thru_hole circle \(at ([\-\d.]+) ([\-\d.]+)\) \(size ([\d.]+)')
    holes = [(float(x), float(y), float(size)) for x, y, size in hole_re.findall(text)]
    assert len(holes) == 4, f"expected 4 real mounting holes in {AC7200_FP_PATH}, found {len(holes)}"

    return connectors, holes


def load_fp(lib_path, name, ref, x_mm, y_mm, rot_deg=0.0):
    fp = pcbnew.FootprintLoad(lib_path, name)
    if fp is None:
        raise RuntimeError(f"footprint not found: {lib_path} / {name}")
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
    fp.SetOrientationDegrees(rot_deg)
    return fp


def main():
    connectors, holes = _real_ac7200_geometry()
    assert set(connectors) == {"CON1", "CON2", "CON3", "CON4"}, connectors

    board = pcbnew.LoadBoard(BOARD_PATH)

    # Idempotency guard (same pattern/reasoning as place_mic_array.py's own
    # guard -- this pcbnew binding's board.Remove() poisons later
    # FootprintLoad()/Drawings() calls, confirmed directly there, so this
    # refuses to run again rather than trying to auto-clear prior content).
    existing_refs = {fp.GetReference() for fp in list(board.GetFootprints())}
    if "J1" in existing_refs or "HAC1" in existing_refs:
        raise RuntimeError(
            f"{BOARD_PATH} already has J1/HAC1 -- refusing to run again "
            "(this script's own idempotency guard, see place_mic_array.py's "
            "identical pattern for why). Remove the existing AC7200_MOUNT "
            "group by hand in KiCad first if you want to regenerate."
        )

    ox, oy = AC7200_MOUNT_XY_MM
    group = pcbnew.PCB_GROUP(board)
    group.SetName("AC7200_MOUNT")

    con_order = ["CON1", "CON2", "CON3", "CON4"]
    for i, con in enumerate(con_order):
        cx, cy, crot = connectors[con]
        ref = f"J{i + 1}"
        fp = load_fp(HEADER_FP_LIB, HEADER_FP_NAME, ref, ox + cx, oy + cy, rot_deg=crot)
        # board.Add() MUST happen before Flip() -- Flip() on a footprint
        # not yet attached to a board segfaults in this KiCad 10.0.5
        # python binding (confirmed directly, real crash, not guesswork).
        board.Add(fp)
        fp.Flip(pcbnew.VECTOR2I_MM(ox + cx, oy + cy), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        group.AddItem(fp)

    for i, (hx, hy, hsize) in enumerate(holes):
        ref = f"HAC{i + 1}"
        fp = load_fp(MOUNTHOLE_LIB, MOUNTHOLE_NAME, ref, ox + hx, oy + hy)
        board.Add(fp)
        group.AddItem(fp)

    board.Add(group)
    board.Save(BOARD_PATH)

    print(f"Added AC7200_MOUNT group: 4x {HEADER_FP_NAME} headers (J1-J4, on B.Cu) "
          f"+ 4x real 2.7mm mounting holes (HAC1-HAC4), centred at "
          f"({ox:.1f}, {oy:.1f})mm on the carrier board. Board saved to {BOARD_PATH}.")
    print("Provisional placement -- see this script's header docstring. "
          "The whole assembly is a real KiCad group (AC7200_MOUNT), so it "
          "can be selected and moved as one unit without breaking the "
          "real connector/hole alignment extracted from AC7200.kicad_mod.")


if __name__ == "__main__":
    main()
