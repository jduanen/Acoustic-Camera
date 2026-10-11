# Board-stack assembly animation

`stack_assembly.mp4` (22 s, 1280×720, 24 fps) and `stack_assembly.blend` show the
single-FPGA hardware stack being assembled: the 320 mm `mic_array` disc, then the
`front_end` carrier, then the ALINX AC7200 module. Each board drops onto guide rods
running through its mounting holes, landing at its real connector mated height.

| File | Purpose |
|---|---|
| `make_assembly.sh` | Regenerates everything from the current PCBs/STEP files |
| `build_assembly.py` | Blender script: imports the models, places them, animates, sets up the camera, lights and captions |
| `stack_assembly.blend` | Generated scene (Blender 5.x); open it to scrub the timeline or re-render |
| `stack_assembly.mp4` | Rendered animation |
| `build/` | Intermediates (GLBs, venv, PNG frames): disposable, don't commit |

## Regenerating

```bash
cd cad/assembly
./make_assembly.sh              # export + build .blend + render (~45 min)
./make_assembly.sh --no-render  # export + build .blend only (~1 min)
```

Requires `kicad-cli`, `blender` (5.x), `ffmpeg` and `python3`. The script creates
`build/venv` with [`cascadio`](https://pypi.org/project/cascadio/) (OpenCASCADE
STEP→GLB) because Blender can't import STEP. Rendering uses Cycles on the CPU: this
machine has only software OpenGL (llvmpipe), where EEVEE takes minutes per frame.

## Inputs

| Model | Source | Conversion |
|---|---|---|
| mic_array | `pcb/single_fpga/mic_array.kicad_pcb` | `kicad-cli pcb export glb`, origin at (268.863, 205.363) mm |
| front_end | `pcb/single_fpga/front_end.kicad_pcb` | `kicad-cli pcb export glb`, origin at (100, 100) mm |
| AC7200 module | `cad/AC7200-step.stp` (ALINX) | cascadio |
| J2 socket (front_end) | `pcb/libraries/AXK5S80347YG/axk5s80347y.stp` | cascadio |
| J1 header (mic_array) | no vendor model available | black proxy block |

## Placement

World origin is the centre of the 100 mm square formed by the four M4 enclosure holes
HENC1–4, which exist on both boards. Each board is exported with that point as its
origin, so the holes coincide by construction. World Y = −KiCad Y; units are mm.

| Part | Z (mm) | Basis |
|---|---|---|
| mic_array | 0 – 1.6 | — |
| front_end | 8.6 – 10.2 | AXK6S80347YG / AXK5S80347YG pair, 7.0 mm mated height |
| AC7200 PCB | 13.2 – 14.8 | AXK680337YG / AXK580137YG pair, 3.0 mm mated height |

The AC7200 is placed connector-side down, with its hole-pattern centre on the U2
footprint origin (0.092, 9.905) and its vendor top view (JTAG header at top-left)
facing up. Measured in the final frame of the built scene: every height above matches,
the AC7200 centre lands on U2, and J2 sits within 8 µm of J1.

Cosmetic overrides (render only, the PCBs are untouched):
- KiCad exports the stackup colours: dielectric "PTFE natural" (near-white) and a mask at
  83% alpha, which renders washed out. The script makes the mask opaque green and the
  board body FR4-coloured.
- The AC7200 STEP's board body is light grey and has no material; it is shown black.

These front_end parts have no 3D model on disk, so they don't appear: 2× Taiyo-Yuden
NR-20xx inductors, the VQFN-48, a 1206 cap and 3× 0402 resistors (the last two from
`${SPARKFUN_KICAD_LIBRARY}`).

## Findings from the 3D check (2026-10-11): open, fix before fab

### 1. U2 (AC7200) footprint is mirrored: CON1–4 pads are Y-flipped

The footprint `pcb/libraries/single_fpga/footprints.pretty/AC7200.kicad_mod` puts
every pad at the vendor position with its Y negated. Three independent sources
agree on the vendor layout. In top view (FPGA side up), CON2 is on the JTAG-header edge:

- ALINX's dimensioned top view, `docs/AC7200核心板结构图.pdf`: JTAG header top-left,
  FPGA left, DDR right.
- ALINX's bottom-component drawing, `docs/AC7200核心板PCB图.pdf` page 2. It is drawn as
  seen through the board from the top (silkscreen text reads mirrored; JTAG holes
  top-left). It shows CON2 top, CON1 right, CON3 left, CON4 bottom, plus a pin-1 dot on
  each connector.
- `cad/AC7200.3.0.stp` (CON-labelled) and `cad/AC7200-step.stp`: same layout.

In the same top-view frame (x right, y up, relative to U2), the footprint has:

| Connector | Footprint | Vendor |
|---|---|---|
| CON2 | bottom edge, pin 1 at (−10.68, −16.30) | top edge, pin 1 ≈ (−10.7, +15.0) |
| CON4 | top edge, pin 1 at (8.82, +16.44) | bottom edge, pin 1 ≈ (8.8, −15.1) |
| CON1 | pin 1 at (21.41, −9.81), bottom end | pin 1 ≈ (20.0, +9.8), top end |
| CON3 | pin 1 at (−21.37, +9.69), top end | pin 1 ≈ (−20.0, −9.7), bottom end |

That is (x, y) → (x, −y) for all four connectors. No rotation of the module can
compensate for a mirror. The cause looks like the STEP's Y-up values being copied into
KiCad's Y-down frame without negation. The `descr` says the positions came from
`AC7200.3.0.stp`, and the footprint's Y values equal the STEP's Y values. The 3D-model
offsets on the footprint are correct (KiCad model offsets are Y-up), which is why the
3D viewer doesn't show the problem.

Consequence: the module would physically fit (the four holes are symmetric), but each
carrier pad would touch roughly the mirrored module pin. For example:
- Carrier +5V on CON1_3…8 would land on module bank-34/bank-13 I/O pins (≈CON1_33…38),
  and the module's own VCCIN pins would land on unconnected pads.
- Module GND pins would land on carrier signals such as RGMII_TX_CTL, PHY_INT_N, and
  DATA_02/04/06/12/13/28/36/44.

This would very likely damage the module at power-up.

Fix (drafted, not yet applied to the board): `pcb/libraries/single_fpga/footprints.pretty/AC7200_corrected.kicad_mod`
is `AC7200.kicad_mod` with every Y coordinate and pad/text angle negated. That covers
the 320 pads, the 4 holes, the silkscreen outlines and the labels; the 3D-model offsets
are unchanged. It parses in `kicad-cli`, and it is an exact mirror of the original (0 nm
error, with pad sizes, layers and pad numbers unchanged). It also matches the
references: CON2 is on the JTAG edge, and each connector's pin 1 is at the end ALINX
marks.

To apply it: in the front_end, change U2's footprint to
`single_fpga:AC7200_corrected` (Edit → Change Footprints), then re-route the
connector fan-out. Every pad moves, so the existing U2 traces will need redoing. Once
it's adopted, retire `AC7200.kicad_mod`.

### 2. J1/J2 (mic_array ↔ front_end): DATA lines cross in even/odd pairs

The schematic connects J1 pin *n* to J2 pin *n*, and the nets match by pin number. But
when the boards are stacked, J1 pin *n* sits over J2's pad on the *opposite* row (e.g.
J1 pin 1 at y = −15.86 mm, J2 pin 1 at y = −11.68 mm). Mating contacts face each
other directly, so physically:
- All 48 DATA nets swap pairwise: DATA_00↔DATA_01, …, DATA_46↔DATA_47.
- Power and GND are unaffected because they occupy adjacent odd/even pin pairs.

The header footprint `CONN80_AXK6S80347YG_PAN` was derived from the vendor socket
footprint (see its description), not taken from a vendor header pattern, so its
odd/even row assignment is the likely culprit. Check it against Panasonic's header
land pattern.

Severity is lower than finding 1: it doesn't damage anything and could even be absorbed
in the XDC pin map. Fixing the footprint is cleaner.
