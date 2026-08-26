#!/usr/bin/env python3
"""
renumber_power_refs.py -- reassigns fresh, globally-unique reference
designators to every power/ground symbol instance (Reference starting
"#PWR" -- GND, +3V3, +5V, +3V6/+3V8, MIC_3V3, CLK_3V3, PWR_FLAG, etc.)
across a set of .kicad_sch files, processed together with one shared
counter so uniqueness holds project-wide, not just per file.

Why this is needed: several of the mic_arm_NN.kicad_sch sheets were
created as literal file copies of mic_arm_00.kicad_sch (see
renumber_mic_arm.py's own docstring for the matching net/ref renumbering
that problem also needed) -- their power/ground symbols still carry
mic_arm_00's own literal "#PWRxxx" values, which collide project-wide once
KiCad flattens the hierarchy (multiple different physical symbols in
different sheets claiming the same reference is a real ERC "duplicate
reference" condition, not just a display quirk).

This is a plain text transform, not a KiCad API call: each candidate is a
top-level `(symbol\\n ...)` instance block (identified by having NO quoted
name directly after the `symbol` keyword -- library-cached template
definitions inside `(lib_symbols ...)` always do, e.g.
`(symbol "power:GND" ...)`, so they're never touched). A block is renamed
only if it contains `(property "Reference" "#PWR...")`. The block's own
matching `(instances (project ... (path ... (reference "#PWRxxx")))))`
sub-block is updated to the same new value in the same pass, since both
must always agree.

Block boundaries are found with a real depth-counting scan (respecting
quoted strings, so a `)` or `(` inside a property string like a
Description can't be mistaken for structure) -- not a naive regex over the
whole file, which would risk matching across block boundaries.

Usage (plain python3, no pcbnew/eeschema API involved):
  python3 pcb/single_fpga/renumber_power_refs.py pcb/single_fpga/*.kicad_sch

Every given file is modified in place. The new numbering starts at
#PWR001 and increments across all files in the order given on the command
line -- run it with every sheet that shares the project in one invocation,
or uniqueness across the set isn't guaranteed.
"""

import re
import sys
from pathlib import Path

REF_RE = re.compile(r'\(property "Reference" "(#PWR\d*)"')


def find_blocks(text: str):
    """Yield (start, end) spans of each top-level `(symbol\\n ...)`
    instance block -- i.e. `(symbol` immediately followed by a newline,
    not `(symbol "some:name"` (which marks a lib_symbols template,
    always skipped). end is exclusive, one past the matching `)`."""
    for m in re.finditer(r"\(symbol\n", text):
        start = m.start()
        depth = 0
        in_str = False
        i = start
        n = len(text)
        while i < n:
            c = text[i]
            if in_str:
                if c == "\\":
                    i += 2
                    continue
                if c == '"':
                    in_str = False
            else:
                if c == '"':
                    in_str = True
                elif c == "(":
                    depth += 1
                elif c == ")":
                    depth -= 1
                    if depth == 0:
                        yield start, i + 1
                        break
            i += 1


def renumber_file(path: Path, counter: int) -> int:
    text = path.read_text()
    out = []
    last_end = 0
    n_renamed = 0
    for start, end in find_blocks(text):
        block = text[start:end]
        m = REF_RE.search(block)
        if not m:
            continue
        old_ref = m.group(1)
        new_ref = f"#PWR{counter:03d}"
        counter += 1
        n_renamed += 1
        new_block = block.replace(f'"{old_ref}"', f'"{new_ref}"')
        out.append(text[last_end:start])
        out.append(new_block)
        last_end = end
    out.append(text[last_end:])
    path.write_text("".join(out))
    print(f"{path.name}: {n_renamed} power/ground refs renumbered")
    return counter


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <file.kicad_sch> [more files...]", file=sys.stderr)
        sys.exit(1)
    counter = 1
    for arg in sys.argv[1:]:
        counter = renumber_file(Path(arg), counter)
    print(f"Total: {counter - 1} power/ground refs renumbered (#PWR001..#PWR{counter - 1:03d}).")


if __name__ == "__main__":
    main()
