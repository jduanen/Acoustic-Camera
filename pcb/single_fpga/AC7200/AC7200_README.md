# AC7200 KiCad symbol + footprint -- provenance and caveats

`AC7200.kicad_sym` (this directory) + `footprints.pretty/AC7200.kicad_mod` for the ALINX
AC7200 XC7A200T-2FBG484I FPGA SOM. Both parse and export cleanly via `kicad-cli sym export
svg` / `kicad-cli fp export svg` (real tool check, not just eyeballed).

## Pin data: cross-checked against ALINX's real manual

Every one of the 320 connector pins (CON1-4, 80 each) was cross-checked against ALINX's own
real user manual (`docs/AC7200_User_Manual.pdf`, Part 10). Result: an earlier pass built from
a third-party web transcription (manuals.plus) turned out to be accurate for CON2, CON3, CON4
(all 240 pins) and CON1 pins 1-72 -- every one matched the real manual exactly. Two real
corrections came out of the check:

- **CON1 pins 73-80** were previously unverified placeholders (three separate web re-fetches
  had returned three different, contradictory answers) -- now filled in with the real values
  from ALINX's manual: `B16_L6_P, NC, B16_L8_P, NC, B16_L8_N, NC, NC, NC`.
- **CON1 pin 27**: ALINX's own manual prints `B13_L7_P` at *both* pin 25 (FPGA ball AB11) and
  pin 27 (FPGA ball AB12) -- a real duplicate/typo in their own document, not a transcription
  artifact (confirmed present in the actual PDF table, not just the web mirror). Every other
  one of the 320 pins alternates differential P/N without exception, and AB11/AB12 is a
  typical adjacent-ball pair, so pin 27 is corrected here to `B13_L7_N`. Flagged in both
  parts' `Description` fields as a corrected manual typo, not silently changed.

Also resolved by the real manual: the chip is confirmed **XC7A200T-2FBG484I** (FBG package --
matches Vivado's own real part table; an earlier secondary source had this as "FGG484", which
doesn't exist in Vivado's part list and was flagged as a likely transcription artifact at the
time -- now confirmed wrong, FBG is correct).

**Not modeled here** (real, and worth knowing about, but out of scope for a carrier-facing
symbol/footprint): the module also has its own on-board 6-pin 2.54mm JTAG header (J1: TMS,
TDI, TDO, TCK, GND, +3.3V -- a second, on-module-only path to the same JTAG signals CON3
pins 77-80 already break out to the carrier) and a 2-pin standalone-power header (J3,
+5V/GND, mutually exclusive with powering the module through CON1 -- per the manual, "do not
supply power via J3 and the carrier board at the same time").

## Mechanical placement: real, cross-validated two independent ways

The board outline, 4x mounting holes, and all 4 connectors' real center position + rotation
were extracted directly from `cad/AC7200.3.0.stp` -- the user's own reference 3D model
already in this directory -- by hand-tracing its real STEP AP214 assembly structure
(`PRODUCT` -> `NEXT_ASSEMBLY_USAGE_OCCURRENCE` -> `PRODUCT_DEFINITION_SHAPE` ->
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` -> `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`
-> `ITEM_DEFINED_TRANSFORMATION` -> `AXIS2_PLACEMENT_3D` -> `CARTESIAN_POINT`/`DIRECTION`,
for each of CON1-4 and its 4 real, individually-named mounting holes).

The board size this produced (55.00 x 45.00mm) is now confirmed **twice, independently**:
matches ALINX's manual's own dimensioned Part 12 "Structure Diagram" drawing exactly, in
addition to matching the symmetry of the STEP-derived connector/hole positions. High
confidence.

(`AC7200_PCB_Reference/AC7200.brd` in this directory is ALINX's own reference carrier board,
but it's a Cadence Allegro binary file -- not parseable without Allegro 16.6+, which isn't
available here. If you have access to Allegro, it's a strictly better source than anything
above for double-checking exact connector pad geometry specifically.)

The connector part number is real: the STEP model's own component names are `AXK680137YG`
(Panasonic AXK6/P5K series, 0.5mm pitch, 80-pin -- the part *on the module*). ALINX's manual
states the module's B2B connectors have 0.5mm pin spacing and mate with the carrier board;
the carrier-side part is the complementary `AXK680337YG` (same family), matching what an
earlier web search independently found as "the recommended connector for designing a base
board." This is what the footprint's pads are built around.

## What's still estimated, not verified

- **Per-pad geometry** (pad size 0.3 x 1.0mm, 40 pins/row at 0.5mm pitch): built from real
  partial specs (23.20 x 4.60mm body, 0.5mm pitch, 5.60mm "recommended PC board pattern
  width" -- all from Panasonic's AXK680337YG product page) but the *exact* pad size and
  row-to-row pitch were not independently obtained (not in ALINX's manual, which documents
  signal assignment, not carrier-side land-pattern dimensions). **Verify against Panasonic's
  real land pattern (or the Allegro reference board, if you can open it) before
  fabrication.**
- **Mounting hole diameter** (2.7mm): assumed M2.5 clearance, matching this project's own
  standoff convention elsewhere (`pcb/layout_multi_fpga.py`'s `STANDOFF_HOLE_NAME`) -- ALINX's
  manual doesn't state a hole diameter.

## Bottom line

The pinout is now fully real, cross-checked against ALINX's own manual, pin-by-pin. The
mechanical placement is real, cross-validated two independent ways. The only remaining gaps
are connector *pad-level* geometry (size/pitch estimated from partial specs) and mounting
hole diameter (assumed) -- both flagged inline above and in both parts' own `Description`
fields. Still worth a final pad-geometry check against Panasonic's real datasheet before
sending this to fabrication, but the signal-level correctness that would matter for a
schematic (which pin is which net) is now solid.
