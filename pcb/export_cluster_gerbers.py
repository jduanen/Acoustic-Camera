#!/usr/bin/env python3
"""export_cluster_gerbers.py -- exports 4 Gerber sets from the single
pcb/multi_fpga/cluster_00.kicad_pcb layout, one per physical unit, differing
only in the UNIT_ID silkscreen text (a KiCad project text variable,
${UNIT_ID}, resolved per-run via kicad-cli's --define-var). See
pcb/multi_fpga/Readme.txt and SCHEMATIC_NOTES.md: this is the one physical
board design, fabricated 4x -- there is intentionally no cluster_01/02/03
.kicad_pcb any more (they were stale hand-copied duplicates; net names were
already generic/unit-agnostic in all 4, and fpga/cluster/rtl/cluster_top.v's
port list confirms the bitstream is identical regardless of which spoke slot
a given physical unit ends up in).

Drill files don't depend on UNIT_ID (no silkscreen content), so they're
generated once and shared across all 4 units.
"""
import subprocess
import sys
from pathlib import Path

PCB_DIR = Path(__file__).parent / "multi_fpga"
PCB_FILE = PCB_DIR / "cluster_00.kicad_pcb"
OUT_DIR = PCB_DIR / "gerbers"
UNITS = [0, 1, 2, 3]


def run(cmd):
    print(f"$ {' '.join(str(c) for c in cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"FAIL: command exited {result.returncode}")
        sys.exit(1)


def main():
    for unit in UNITS:
        out = OUT_DIR / f"cluster_unit{unit}"
        run([
            "kicad-cli", "pcb", "export", "gerbers",
            "--define-var", f"UNIT_ID={unit}",
            "-o", str(out),
            str(PCB_FILE),
        ])
        print(f"unit {unit}: -> {out}")

    drill_out = OUT_DIR / "drill"
    run([
        "kicad-cli", "pcb", "export", "drill",
        "-o", str(drill_out),
        str(PCB_FILE),
    ])
    print(f"drill (shared across all units): -> {drill_out}")

    print(f"\n{len(UNITS)} Gerber sets + 1 shared drill set exported to {OUT_DIR}")


if __name__ == "__main__":
    main()
