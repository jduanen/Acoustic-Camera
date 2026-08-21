"""
Generates lib/UM232H/UM232H.step -- a representative 3-D model of the FTDI
UM232H module, built from the dimensions in DS_UM232H.pdf Section 5 /
Figure 3 (Module Dimensions). Run with the repo's venv (has cadquery):
    ./venv/bin/python3 lib/UM232H/generate_step.py

Coordinate system ("natural" landscape orientation, matching how Figure 3
itself is drawn -- USB connector at the -X end, pin field centered on the
origin so this model's origin lines up with the UM232H footprint's own
origin (pin-field center) after a 90deg Z rotation -- see the "(model ...)"
block in footprints.pretty/UM232H.kicad_mod):
    X : along the board's length (44.5mm). Pin 1 is nearest the connector
        (-X), pin 14 is farthest (+X).
    Y : along the board's width (18.25mm). The two pin rows sit at
        Y=+/-7.62 (15.24mm row spacing), centered on the origin.
    Z : up. Z=0 is the module's OWN pcb top surface (where its pins are
        soldered into its own board) -- not the host board's surface. This
        module stands on its own long pins, proud of whatever socket it
        plugs into; Z=0 is just this part's local reference plane, same as
        this repo's other 3-D models (ECS-TXO, Adafruit_FT232H) are built
        around their own footprint origin rather than the host board's Z.

Dimensions used, all taken directly from Figure 3 (mm):
    44.5 x 18.25 x 1.6    overall board L x W x thickness
    15.24 / 2.54          pin row spacing / pin pitch
    10.0 / 43.0           first/last pin X, measured from the board edge
    8.0                   USB connector depth (mini-B shell, along X)
    13.0 / 5.3            connector Y-span / bare-board gap on the far side
    4.1                   connector height above the pcb top surface
    0.50                  pin diameter
    14.9                  board-top-surface to pin-tip -- interpreted as the
                           single unambiguous total drop given in the side
                           view (the 4.1/5.8/5.0 front-view sub-breakdown is
                           genuinely ambiguous from the figure's dashed
                           reference lines and is NOT used for the total --
                           see the "Description" property in
                           footprints.pretty/UM232H.kicad_mod for the full
                           list of interpretive choices made here).

Everything not explicitly dimensioned (connector overhang past the board
edge, the pin-housing collar thickness, the extra unlabeled GND through-hole
and resistor-shaped silkscreen icon near the bottom of the real board) is a
documented simplification -- this is a representative mechanical-fit model,
not a vendor-exact reproduction.
"""
import cadquery as cq

# ---- board ------------------------------------------------------------
BOARD_L = 44.5
BOARD_W = 18.25
BOARD_T = 1.6

PIN_PITCH = 2.54
ROW_SPACING = 15.24
N_PINS = 14
PIN_DIA = 0.50

FIRST_PIN_X = 10.0   # from the board edge nearest the connector
LAST_PIN_X = 43.0

# pin field centered on the origin
pin_field_len = LAST_PIN_X - FIRST_PIN_X            # 33.0 (Fig 3's own
                                                     # rounding; 13*2.54=33.02)
x0 = -pin_field_len / 2                             # first pin X
row_y = ROW_SPACING / 2                             # +/-7.62

# board edges, in the same origin-centered frame: pin1 sits FIRST_PIN_X in
# from the connector-side edge, pin14 sits (BOARD_L-LAST_PIN_X) in from the
# far edge.
board_x0 = x0 - FIRST_PIN_X                         # connector-side edge
board_x1 = board_x0 + BOARD_L                       # far edge
board_y0 = -BOARD_W / 2
board_y1 = BOARD_W / 2

# ---- connector ----------------------------------------------------------
CONN_DEPTH = 8.0       # X extent of the connector shell
CONN_OVERHANG = 1.5    # documented assumption: shell protrudes past the
                       # board edge for the cable-insertion face (matches
                       # Figure 1's photo; not itself a dimensioned value)
CONN_HEIGHT = 4.1      # above pcb top surface
CONN_Y_SPAN = 13.0     # from Fig 3's top-view vertical bracket
# connector's Y-span starts flush with one long edge of the board (per
# Figure 3) and leaves the 5.3mm gap on the other side
conn_y0 = board_y0
conn_x0 = board_x0 - CONN_OVERHANG

# ---- pins below the board -----------------------------------------------
TOTAL_DROP = 14.9      # board top surface -> pin tip (see docstring)
HOUSING_T = 1.27       # assumed pin-collar thickness (not separately
                       # dimensioned in the datasheet)
pin_bottom_z = -TOTAL_DROP

# ---- build ----------------------------------------------------------------
board = (
    cq.Workplane("XY")
    .box(BOARD_L, BOARD_W, BOARD_T, centered=(False, False, False))
    .translate((board_x0, board_y0, -BOARD_T))
)

connector = (
    cq.Workplane("XY")
    .box(CONN_DEPTH + CONN_OVERHANG, CONN_Y_SPAN, CONN_HEIGHT, centered=(False, False, False))
    .translate((conn_x0, conn_y0, 0))
)

# housing sized to the pin field itself (with a small margin) -- must stay
# within the board's own footprint, not an arbitrary fraction of board length
housing_margin = 1.5
housing_x0 = x0 - housing_margin
housing_x1 = (x0 + (N_PINS - 1) * PIN_PITCH) + housing_margin
housing = (
    cq.Workplane("XY")
    .box(housing_x1 - housing_x0, ROW_SPACING + 3.0, HOUSING_T, centered=(False, False, False))
    .translate((housing_x0, -(ROW_SPACING + 3.0) / 2, -BOARD_T - HOUSING_T))
)

pins = cq.Workplane("XY")
for row in (-row_y, row_y):
    for i in range(N_PINS):
        px = x0 + i * PIN_PITCH
        pins = pins.union(
            cq.Workplane("XY")
            .moveTo(px, row)
            .circle(PIN_DIA / 2)
            .extrude(-TOTAL_DROP)
        )

assy = cq.Assembly()
assy.add(board, name="pcb", color=cq.Color(0.09, 0.35, 0.18, 1.0))
assy.add(connector, name="usb_connector", color=cq.Color(0.75, 0.75, 0.78, 1.0))
assy.add(housing, name="pin_housing", color=cq.Color(0.05, 0.05, 0.05, 1.0))
assy.add(pins, name="pins", color=cq.Color(0.85, 0.68, 0.25, 1.0))

out_dir = "/home/jdn/Code/Acoustic-Camera/lib/UM232H"
cq.exporters.assembly.exportAssembly(assy, f"{out_dir}/UM232H.step")
print(f"wrote {out_dir}/UM232H.step")
print("board X", board_x0, board_x1, " Y", board_y0, board_y1)
print("pin field X", x0, x0 + (N_PINS - 1) * PIN_PITCH, " rows Y", -row_y, row_y)
print("pin tip Z", pin_bottom_z)
