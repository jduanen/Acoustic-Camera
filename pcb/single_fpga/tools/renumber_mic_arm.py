#!/usr/bin/env python3
"""
renumber_mic_arm.py -- text-only renumbering for a mic_arm_NN.kicad_sch file
that started life as a copy of the correctly-wired mic_arm_00.kicad_sch
template (the only arm sheet whose net names/refs are already right --
every other arm sheet was found this session to still carry mic_arm_00's
own literal labels/references, unrenamed).

N is read from the numeric part of the file's own name (e.g. "03" from
"mic_arm_03.kicad_sch"). Seven plain regex substitutions, run against the
file's raw text -- no pcbnew/eeschema API involved, this is intentionally
just a text transform:

  ARM_00_CLK              -> ARM_{N:02d}_CLK          (single literal token)
  U1 .. U8                -> U(n + 8*N)                (mic refs, n=1..8)
  U99                     -> U(99 + N)                 (this arm's own
                                                          CDCLVC1108 buffer)
  C18 .. C25               -> C(n + 8*N)                (decoupling caps,
                                                          n=18..25)
  R1                      -> R(5 + N)                  (this arm's own
                                                          CDCLVC1108 1G
                                                          pull-up)
  DATA_00 .. DATA_03      -> DATA_{n + 4*N:02d}         (n=0..3)
  MIC_CLK_00 .. MIC_CLK_07 -> MIC_CLK_{n + 8*N:02d}      (n=0..7)

R1 -> R(5+N), not R(1+N): R1..R4 are already used project-wide by
power_clock.kicad_sch's own buck-regulator/enable resistors (confirmed
by grepping every R-reference in the project before picking this base --
R5..R16 were free), so this base is chosen to avoid colliding with those,
not derived from any per-mic/per-arm index formula the other rules use.

The U1-U8/C18-25/DATA_00-03/MIC_00-07_CLK formulas exactly reproduce this
project's own established numbering (mic U{mic_idx+1}, decoupling cap
C{mic_idx+18}, DATA index = arm*4 + pair, mic_idx = arm*8 + position --
see place_mic_array.py's own docstring and pcb/SCHEMATIC_NOTES.md's Data
Line Assignment table), so running this against a fresh mic_arm_00.kicad_sch
copy for a given arm N reproduces exactly the reference/net numbering that
arm should have.

All 7 patterns use \\b-anchored regexes (not plain substring replace), so
e.g. U1 cannot match inside U10/U18, C18 cannot match inside C180, and
DATA_00 cannot match inside a longer token -- safe to run against the
literal, unrenamed template content this script expects, not safe to
assume idempotent against arbitrary other content.

Usage (plain python3, no pcbnew needed -- this never touches KiCad's
Python API, just file text):
  python3 pcb/single_fpga/renumber_mic_arm.py pcb/single_fpga/mic_arm_03.kicad_sch [more files...]

Each file is modified in place. Intended precondition (not checked by this
script): the target file is already a copy of mic_arm_00.kicad_sch's real
content, saved under its own mic_arm_NN.kicad_sch name -- this script only
fixes the numbering inside it, it does not create or copy files itself.
"""

import re
import sys
from pathlib import Path

RULES = [
    ("ARM_00_CLK",
     re.compile(r"\bARM_00_CLK\b"),
     lambda m, n: f"ARM_{n:02d}_CLK"),
    ("U1..U8",
     re.compile(r"\bU([1-8])\b"),
     lambda m, n: f"U{int(m.group(1)) + 8 * n}"),
    ("U99",
     re.compile(r"\bU99\b"),
     lambda m, n: f"U{99 + n}"),
    ("C18..C25",
     re.compile(r"\bC(18|19|20|21|22|23|24|25)\b"),
     lambda m, n: f"C{int(m.group(1)) + 8 * n}"),
    ("R1",
     re.compile(r"\bR1\b"),
     lambda m, n: f"R{5 + n}"),
    ("DATA_00..DATA_03",
     re.compile(r"\bDATA_(0[0-3])\b"),
     lambda m, n: f"DATA_{int(m.group(1)) + 4 * n:02d}"),
    ("MIC_CLK_00..MIC_CLK_07",
     re.compile(r"\bMIC_CLK_(0[0-7])\b"),
     lambda m, n: f"MIC_CLK_{int(m.group(1)) + 8 * n:02d}"),
]


def arm_number(path: Path) -> int:
    m = re.search(r"(\d+)", path.stem)
    if not m:
        raise ValueError(f"no numeric part found in filename: {path.name}")
    return int(m.group(1))


def renumber(path: Path) -> None:
    n = arm_number(path)
    text = path.read_text()
    print(f"{path.name}: N={n:02d}")
    for label, pattern, repl in RULES:
        text, count = pattern.subn(lambda m: repl(m, n), text)
        print(f"  {label}: {count} replaced")
    path.write_text(text)


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <mic_arm_NN.kicad_sch> [more files...]", file=sys.stderr)
        sys.exit(1)
    for arg in sys.argv[1:]:
        renumber(Path(arg))


if __name__ == "__main__":
    main()
