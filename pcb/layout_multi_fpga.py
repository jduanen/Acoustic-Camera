#!/usr/bin/env python3
"""
layout_multi_fpga.py -- board outline + footprint placement for the Phase 4
Multi-FPGA (Clustered) alternative: 4 quadrant cluster boards (Cmod S7 + 24
mics + their decoupling caps each) and 1 hub board (Cmod A7-35T + FT232H
breakout + TCXO), written as 5 SEPARATE, independently-fabricable files --
cluster_00..03.kicad_pcb + hub.kicad_pcb, matching the schematic's own
per-sheet file naming (pcb/multi_fpga/cluster_NN.kicad_sch, hub.kicad_sch).
These are 5 real physical boards (hub mounted behind the 4 clusters on its
own standoffs -- see place_hub()), not one panel, so they get 5 files, not
1 file with 5 outlines in it.

All 5 files share the same (array-centre-origin) coordinate system, even
though each is its own board/project -- this is deliberate, not an oversight:
it keeps a cluster's spoke-connector position numerically identical to the
matching position on the hub file (see cluster_standoff_xy()), which is
useful for sanity-checking the two boards align, and costs nothing since a
fab house doesn't care where a board's content sits within its own file.

Scope: board outlines + footprint placement only, no routing (mirrors
pcb/place_mics.py's placement-only precedent for the single-FPGA board).
Unlike place_mics.py, this does not require "Update PCB from Schematic" or
KiCad's in-app Scripting Console first -- it builds each board from scratch,
loading footprints directly by library path via pcbnew (confirmed importable
from plain python3 in this environment). Pads are left netless (no netlist
import); wiring/routing is a separate follow-up pass.

Each cluster's board outline is bounded by two Underbrink log-spiral curves
(the same shape used to generate the mic positions themselves, see
notebooks/make_nb18.py's underbrink_array()), offset by a constant 90°
rotation from each other and by 15° from that cluster's own 3 arms. Because
all 12 arms are the same spiral curve rotated by multiples of 30°, a cut
curve offset by +-15° from the outermost/innermost arm of a cluster stays
exactly 90° of arc from its mirror image at every radius -- so the 4
cluster outlines tile the full circle with no gaps or overlaps, even though
each individual arm curls well past its own 30° nominal spacing by the time
it reaches its outermost mic (up to ~41° of spiral sweep at these
R_MIN/R_MAX/SPIRAL_DEG values) -- a straight radial cut would clip mics.

Usage (from project root):
  python3 pcb/layout_multi_fpga.py
"""

import csv
import math
import os

import pcbnew

# ── array geometry constants -- MUST match notebooks/make_nb18.py's
# underbrink_array(12, 8) call exactly, since these boards are built around
# the same mic positions that notebook derives (test/phase4/array_xy.csv is
# that notebook's own output). ─────────────────────────────────────────────
R_MIN_MM     = 25.0
R_MAX_MM     = 150.0
SPIRAL_DEG   = 22.0
N_ARMS_TOTAL = 12
ARMS_PER_CLUSTER = 3
N_CLUSTERS   = 4

# The board's own inner edge has to sit CLOSER to centre than R_MIN_MM, not
# AT it: mics are placed unrotated (fixed orientation regardless of their
# angular position, matching pcb/place_mics.py's precedent for the
# single-FPGA board), so an innermost mic's asymmetric courtyard
# (IM72D128, -2.7..3.15mm x -2.25..2.25mm around its centre) doesn't sit
# symmetrically about its own radial direction -- at some of the 12 arms'
# angles, a footprint corner reaches as close as r=21.15mm to the array
# centre even though the mic's own centre is at r=R_MIN_MM=25mm. Checked
# across all 12 arms' innermost mics, not just arm 0. 20.0mm leaves ~1.1mm
# margin past the worst case.
BOARD_INNER_RADIUS_MM = 20.0

ROOT       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(ROOT, "..", "test", "phase4", "array_xy.csv")
OUTDIR     = os.path.join(ROOT, "multi_fpga")


def board_pcb_path(name):
    return os.path.join(OUTDIR, f"{name}.kicad_pcb")

MIC_FP_LIB  = os.path.join(ROOT, "IM72D128", "KiCad")
MIC_FP_NAME = "IFX-PG-LLGA-5-4"
DIP48_LIB   = "/usr/share/kicad/footprints/Package_DIP.pretty"
DIP48_NAME  = "DIP-48_W15.24mm_Socket"
LOCAL_LIB   = os.path.join(OUTDIR, "footprints.pretty")
CAP_FP_LIB  = "/usr/share/kicad/footprints/Capacitor_SMD.pretty"
CAP_FP_NAME = "C_0603_1608Metric"
MOUNTHOLE_LIB       = "/usr/share/kicad/footprints/MountingHole.pretty"
STANDOFF_HOLE_NAME  = "MountingHole_2.7mm_M2.5"  # cluster<->hub standoffs + Pi 5's own holes (both M2.5)
ENCLOSURE_HOLE_NAME = "MountingHole_3.2mm_M3"     # cluster board -> outer enclosure
SPOKE_SOCKET_LIB    = "/usr/share/kicad/footprints/Connector_PinSocket_2.54mm.pretty"
SPOKE_SOCKET_NAME   = "PinSocket_2x06_P2.54mm_Vertical"

# Cmod S7 module: mounted RADIALLY (long axis running outward along the
# gap between two arms, not across it -- see find_module_placement() for
# why), tucked into the natural gap between this cluster's 2nd and 3rd
# arms, as far in towards the array centre as it can go without colliding
# with a mic or its decoupling cap (found by search in
# find_module_placement(), not a fixed margin). R_BOARD_MAX_MM is
# therefore no longer a fixed constant -- it's derived in main() from
# where that search actually lands the 4 (identical, just rotated)
# cluster boards, then applied uniformly to all 4 so they stay identical
# boards. This module-level value is a placeholder overwritten by main()
# before anything reads it; cluster_outline_points()/_cut_curve_points()
# depend on it via the globals below.
R_BOARD_MAX_MM = R_MAX_MM + 40.0

# ── Underbrink log-spiral helpers (mirrors notebooks/make_nb18.py exactly) ──

_B = 1.0 / math.tan(math.radians(SPIRAL_DEG))


def _t_at_r(r_mm):
    """Spiral parameter t (radians of sweep from r_min) at radius r_mm."""
    return math.log(r_mm / R_MIN_MM) / _B


def _r_at_t(t):
    return R_MIN_MM * math.exp(_B * t)


def _cut_curve_points(offset_deg, n=24, t_min=None, t_max=None):
    """Sample points (mm) along the boundary spiral rotated by offset_deg,
    from r=BOARD_INNER_RADIUS_MM (t=t_min) out to r=R_BOARD_MAX_MM
    (t=t_max). t_min is negative (BOARD_INNER_RADIUS_MM < R_MIN_MM, i.e.
    before the spiral's own t=0 point) -- the log-spiral formula is smooth
    and well-defined there too, so this just continues the same curve
    shape a little further inward; the "same shape rotated by a constant
    offset" tiling argument doesn't care what t range is used, only that
    both boundary curves use the same one. Looks up the current
    R_BOARD_MAX_MM fresh each call (rather than a bound default argument)
    since main() finalises that value only after searching for where the
    Cmod S7 modules actually land -- see R_BOARD_MAX_MM comment."""
    if t_min is None:
        t_min = _t_at_r(BOARD_INNER_RADIUS_MM)
    if t_max is None:
        t_max = _t_at_r(R_BOARD_MAX_MM)
    pts = []
    for i in range(n):
        t = t_min + (t_max - t_min) * i / (n - 1)
        r = _r_at_t(t)
        theta = t + math.radians(offset_deg)
        pts.append((r * math.cos(theta), r * math.sin(theta)))
    return pts


def _arc_points(r_mm, deg_from, deg_to, n=16):
    pts = []
    for i in range(n):
        deg = deg_from + (deg_to - deg_from) * i / (n - 1)
        theta = math.radians(deg)
        pts.append((r_mm * math.cos(theta), r_mm * math.sin(theta)))
    return pts


def cluster_bounds_deg(c):
    """(before, after) rotation offsets in degrees for cluster c's two
    boundary cut curves -- see module docstring for why +-15deg."""
    before = 90.0 * c - 15.0
    after = 90.0 * c + 75.0
    return before, after


def cluster_outline_points(c):
    before, after = cluster_bounds_deg(c)
    cut_before = _cut_curve_points(before)
    cut_after = _cut_curve_points(after)
    # Inner arc/cut curves now start at BOARD_INNER_RADIUS_MM (< R_MIN_MM),
    # not R_MIN_MM itself -- see that constant's comment for why (innermost
    # mics' unrotated, asymmetric courtyards otherwise poke past the inner
    # edge). Same "actual endpoint, not the un-swept angle" fix as the outer
    # arc below: at t=t_min (now negative), each cut curve's angle is
    # offset_deg + degrees(t_min), not just offset_deg.
    t_min_deg = math.degrees(_t_at_r(BOARD_INNER_RADIUS_MM))
    inner_arc = _arc_points(BOARD_INNER_RADIUS_MM, before + t_min_deg, after + t_min_deg)
    # The outer arc must connect the two cut curves' actual endpoints at
    # r=R_BOARD_MAX_MM, not the "before"/"after" angles -- those only apply
    # at r=R_MIN_MM (t=0). Both curves sweep an extra degrees(t_max) of
    # angle by the outer radius (the same spiral-drift mechanism this file
    # relies on elsewhere), so using the un-swept angles here left a
    # degrees(t_max)-wide gap at each shoulder that add_outline() drew as a
    # straight chord cutting across the board near its outer edge.
    t_max_deg = math.degrees(_t_at_r(R_BOARD_MAX_MM))
    outer_arc = _arc_points(R_BOARD_MAX_MM, after + t_max_deg, before + t_max_deg)
    return inner_arc + cut_after + outer_arc + list(reversed(cut_before))


# ── footprint loading ────────────────────────────────────────────────────

def load_fp(lib_path, name, ref, x_mm, y_mm, rot_deg=0.0):
    fp = pcbnew.FootprintLoad(lib_path, name)
    if fp is None:
        raise RuntimeError(f"footprint not found: {lib_path} / {name}")
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
    fp.SetOrientationDegrees(rot_deg)
    return fp


def _local_courtyard_corners(lib_path, name):
    """The 4 corners (mm, footprint-local coords) of a freshly-loaded
    footprint's bounding box at 0 rotation/position -- used to test where
    its TRUE corners land once placed+rotated (see _true_corners() below).
    Loaded fresh rather than hardcoded so this stays correct if the
    footprint file changes."""
    fp = pcbnew.FootprintLoad(lib_path, name)
    fp.SetPosition(pcbnew.VECTOR2I_MM(0, 0))
    fp.SetOrientationDegrees(0)
    bb = fp.GetBoundingBox(False)
    x0, x1 = bb.GetLeft() / 1e6, bb.GetRight() / 1e6
    y0, y1 = bb.GetTop() / 1e6, bb.GetBottom() / 1e6
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _true_corners(px_mm, py_mm, rot_deg, local_corners):
    """World-space (mm) positions of a footprint's true corners, given its
    placement and rotation. NB: FOOTPRINT.GetBoundingBox() on an already-
    rotated instance returns an axis-aligned box that circumscribes the
    rotated shape -- combining its Left/Right/Top/Bottom independently
    produces phantom points further out than any real corner. This instead
    transforms the known local corners directly, using the empirically
    confirmed mapping local+X -> world (cos t, -sin t), local+Y -> world
    (sin t, cos t) (see find_module_placement()'s Cmod S7 comment)."""
    t = math.radians(rot_deg)
    xdir = (math.cos(t), -math.sin(t))
    ydir = (math.sin(t), math.cos(t))
    return [(px_mm + lx * xdir[0] + ly * ydir[0],
             py_mm + lx * xdir[1] + ly * ydir[1])
            for lx, ly in local_corners]


def add_outline(board, points_mm, width_mm=0.15):
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_POLY)
    poly = pcbnew.SHAPE_POLY_SET()
    poly.NewOutline()
    for x, y in points_mm:
        poly.Append(pcbnew.VECTOR2I_MM(x, y))
    shape.SetPolyShape(poly)
    shape.SetLayer(pcbnew.Edge_Cuts)
    shape.SetWidth(pcbnew.FromMM(width_mm))
    board.Add(shape)


def add_circle_cutout(board, cx_mm, cy_mm, radius_mm, n=32, width_mm=0.15):
    """A closed circular Edge.Cuts loop fully inside the board's own outline
    reads as a cutout/hole in KiCad, not a second board -- used for the
    hub's camera lens opening."""
    pts = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        pts.append((cx_mm + radius_mm * math.cos(ang), cy_mm + radius_mm * math.sin(ang)))
    add_outline(board, pts, width_mm=width_mm)


def add_mounting_hole(board, lib_name, x_mm, y_mm, ref):
    board.Add(load_fp(MOUNTHOLE_LIB, lib_name, ref, x_mm, y_mm))


# ── cluster board ────────────────────────────────────────────────────────

def _mic_and_cap_xy(x, y):
    """Cap position for a mic at (x,y): offset 3mm further out along the
    mic's own radial direction from the array centre. Shared by the
    obstacle check (find_module_placement()) and the actual placement
    (build_cluster()) so the check sees exactly what gets placed."""
    r = math.hypot(x, y)
    if r > 0:
        return x + 3.0 * x / r, y + 3.0 * y / r
    return x + 3.0, y


# Cmod S7 placement: fixed constants approximating a manual layout the
# user dragged/rotated in KiCad (recovered from
# pcb/multi_fpga/.history's local-history git repo, commit 9d35787 "PCB
# Save", after it had been overwritten in the working tree by rerunning
# this script's previous (collision-search-based) version -- the .history
# snapshot was the only surviving copy of the manual edit). Averaged
# across the 4 (independently dragged, so not perfectly identical)
# instances found there:
#   radius: 87.41, 88.33, 86.56, 88.88 -> avg 87.8mm
#   position angle, relative to each cluster's own 90c base: 49.53,
#     51.18, 50.53, 49.44 -> avg 50.2deg (jitter of ~1deg either way,
#     consistent with a hand-dragged position, same pattern seen earlier
#     in this project's schematic repositioning)
#   rotation: exactly 30, -60, -150, 120 deg for c=0..3 -- i.e. exactly
#     30-90c with NO jitter at all, unlike position (suggests the
#     rotation was set precisely, e.g. via the footprint properties
#     dialog, rather than free-dragged)
# Confirmed collision-free against this project's actual mic/cap
# footprints and fully contained in each module's own cluster wedge (see
# chat verification) before adopting this as the fixed layout.
CMOD_S7_PLACEMENT_R_MM     = 87.8
CMOD_S7_PLACEMENT_ANGLE_DEG = 50.2  # offset from each cluster's own 90*c base
CMOD_S7_PLACEMENT_ROT_DEG   = 30.0  # rot_deg = CMOD_S7_PLACEMENT_ROT_DEG - 90*c


def find_module_placement(c, mic_rows):
    """Cmod S7 placement for cluster c -- see CMOD_S7_PLACEMENT_* comment
    above. mic_rows is unused (kept in the signature since build_cluster()
    and main() already pass it, and a future revision may want it again
    for a collision check) but the placement itself is now a fixed formula,
    not searched. Returns (mx, my, rot_deg, max_corner_radius_mm)."""
    angle_deg = 90.0 * c + CMOD_S7_PLACEMENT_ANGLE_DEG
    rot_deg = CMOD_S7_PLACEMENT_ROT_DEG - 90.0 * c
    theta = math.radians(angle_deg)
    mx, my = CMOD_S7_PLACEMENT_R_MM * math.cos(theta), CMOD_S7_PLACEMENT_R_MM * math.sin(theta)

    local_corners = _local_courtyard_corners(DIP48_LIB, DIP48_NAME)
    corners = _true_corners(mx, my, rot_deg, local_corners)
    max_r = max(math.hypot(x, y) for x, y in corners)
    return mx, my, rot_deg, max_r


def cluster_standoff_xy(c):
    """Mechanical standoff position for cluster c's hub connection: offset
    to one side of that cluster's Cmod S7 module, along its short (local
    +X) axis, clear of the module's own footprint -- so the board-to-board
    header/socket at the Cmod's own position isn't left bearing the
    mechanical load of the stack by itself. Same (x,y) is used on both the
    cluster board (build_cluster()) and the hub board (place_hub()) so the
    standoff bridges the two."""
    angle_deg = 90.0 * c + CMOD_S7_PLACEMENT_ANGLE_DEG
    rot_deg = CMOD_S7_PLACEMENT_ROT_DEG - 90.0 * c
    theta = math.radians(angle_deg)
    mx = CMOD_S7_PLACEMENT_R_MM * math.cos(theta)
    my = CMOD_S7_PLACEMENT_R_MM * math.sin(theta)
    rot_rad = math.radians(rot_deg)
    xdir = (math.cos(rot_rad), -math.sin(rot_rad))  # local +X world direction (short axis)
    # Local-X extent of the DIP-48 footprint runs from its origin out to
    # ~16.8mm (see _local_courtyard_corners) -- offset from THAT edge, not
    # from the origin, plus a clearance margin, so the hole doesn't land
    # inside the module's own courtyard.
    local_corners = _local_courtyard_corners(DIP48_LIB, DIP48_NAME)
    x_extent = max(lx for lx, _ in local_corners)
    # Negative direction checked clearer of nearby mics than positive (the
    # module isn't centred on local x=0, so the two sides aren't
    # symmetric) -- verified computationally, not just assumed.
    offset = -(x_extent + 8.0)
    return mx + offset * xdir[0], my + offset * xdir[1]


# Enclosure-mounting holes: 2 per cluster board, near the outer rim, at
# +-40deg from that cluster's wedge bisector (90c+30) -- clear of both the
# Cmod S7 (which sits at +20.2deg from the bisector, see
# CMOD_S7_PLACEMENT_ANGLE_DEG=50.2 vs the 30deg bisector) and the wedge's
# own +-45deg boundary (5deg margin so a hole doesn't land right at the
# board edge).
ENCLOSURE_HOLE_OFFSET_DEG = 40.0


def build_cluster(board, c, mic_rows, module_placement):
    add_outline(board, cluster_outline_points(c))

    for row in mic_rows:
        mic_idx = int(row["mic_idx"])
        x, y = float(row["x_mm"]), float(row["y_mm"])
        board.Add(load_fp(MIC_FP_LIB, MIC_FP_NAME, f"U{mic_idx + 1}", x, y))
        cx, cy = _mic_and_cap_xy(x, y)
        board.Add(load_fp(CAP_FP_LIB, CAP_FP_NAME, f"C{mic_idx + 1}", cx, cy))

    mx, my, rot_deg = module_placement
    board.Add(load_fp(DIP48_LIB, DIP48_NAME, f"A{c + 1}", mx, my, rot_deg=rot_deg))

    # Cluster<->hub mechanical standoff.
    sx, sy = cluster_standoff_xy(c)
    add_mounting_hole(board, STANDOFF_HOLE_NAME, sx, sy, f"H{c + 1}A")

    # Enclosure mounting, 2 per board, near the outer rim. The wedge centre
    # drifts with radius (same spiral-drift reasoning as cluster_outline_
    # points()/find_module_placement()), so it's evaluated at r_enc, not
    # taken as the fixed 90c+30 that only holds at r=R_MIN.
    r_enc = R_BOARD_MAX_MM - 15.0
    bisector_deg_at_r = 90.0 * c + 30.0 + math.degrees(_t_at_r(r_enc))
    for i, sign in enumerate((-1.0, 1.0)):
        deg = bisector_deg_at_r + sign * ENCLOSURE_HOLE_OFFSET_DEG
        ex = r_enc * math.cos(math.radians(deg))
        ey = r_enc * math.sin(math.radians(deg))
        add_mounting_hole(board, ENCLOSURE_HOLE_NAME, ex, ey, f"H{c + 1}E{i + 1}")


# ── hub board ────────────────────────────────────────────────────────────
# The hub now mounts at the physical centre of the array (not a
# disconnected board off to one side): a circular board, standoff- and
# connector-mated to all 4 cluster boards, with the camera looking forward
# through a centre cutout and the Pi 5 stacked on the same standoffs
# behind it. See PHASE4.md / SCHEMATIC_NOTES.md "Spoke connector" for the
# board-to-board connector choice (plain 2x6 2.54mm header/socket, plugged
# directly into each Cmod S7's Pmod JA -- no cable) and its flagged
# signal-integrity caveat (shared Pmod ground / spoke CLK jitter risk, not
# solved in this pass).

HUB_RADIUS_MM = 105.0  # covers the 4 spoke-connector positions (r=87.8, see
                       # CMOD_S7_PLACEMENT_R_MM) plus room for the hub's own
                       # components and the enclosure standoffs

# Camera (Pi Camera Module 3 Wide): PCB 25x24mm, lens module ~12.4mm tall
# (Raspberry Pi's spec sheet). Lens-barrel diameter wasn't found this
# session -- cutout sized generously and flagged to confirm against
# Raspberry Pi's mechanical drawing before fab, same for the mounting-hole
# spacing (placeholder, camera modules typically use 2 small holes).
CAMERA_CUTOUT_DIA_MM = 20.0
CAMERA_MOUNT_HOLE_SPACING_MM = 21.0

# Raspberry Pi 5: 85x56mm board, M2.5 mounting holes on the same pattern as
# Pi 4 (official mechanical drawing referenced but exact offsets not
# pulled this session -- placeholder centred rectangle, confirm before
# fab). Stacks behind the hub on its own standoffs (own hole pattern, not
# the 4 cluster-facing ones), connected by a short USB cable (Pi 5's USB
# port is fixed hardware, not a rigid board-to-board link) -- so only its
# mounting holes are modelled here, not a fabricated outline (it's a
# purchased board, not one this project cuts).
RPI5_HOLE_HALF_SPACING_MM = (29.0, 24.5)  # placeholder half-spacing (58x49mm pattern)
RPI5_CENTRE_XY_MM = (0.0, -160.0)  # clear of HUB_RADIUS_MM entirely (105mm) -- Pi 5 is a
                                    # purchased board on its own standoffs, doesn't need to
                                    # share the hub board's own footprint at all


def _radial_dip48_pose(r_mm, angle_deg):
    """Placement math for a DIP-48 module mounted radially (pin 1 at r_mm
    along angle_deg, body extending further out) -- same convention as
    find_module_placement(), reused here for the hub's own CMOD_A7_35T.
    No neighbouring-arm collision concern on the hub, so a direct formula
    rather than a search."""
    rot_deg = 90.0 - angle_deg
    theta = math.radians(angle_deg)
    return r_mm * math.cos(theta), r_mm * math.sin(theta), rot_deg


def place_hub(board):
    add_outline(board, [
        (HUB_RADIUS_MM * math.cos(2 * math.pi * i / 64),
         HUB_RADIUS_MM * math.sin(2 * math.pi * i / 64))
        for i in range(64)
    ])
    add_circle_cutout(board, 0.0, 0.0, CAMERA_CUTOUT_DIA_MM / 2.0)

    # 4x spoke connector sockets -- exact same (x,y) as each cluster's own
    # Cmod S7 (find_module_placement() reused directly, mic_rows argument
    # unused by that function), so the socket lines up under the header
    # plugged into that cluster's Pmod. Rotation matched too, though pin-1
    # alignment between the two facing connectors still needs a physical
    # check once parts are in hand (flagged, not verified here).
    for c in range(N_CLUSTERS):
        sx, sy, srot, _ = find_module_placement(c, [])
        board.Add(load_fp(SPOKE_SOCKET_LIB, SPOKE_SOCKET_NAME, f"J{c + 1}", sx, sy, rot_deg=srot))
        hx, hy = cluster_standoff_xy(c)
        add_mounting_hole(board, STANDOFF_HOLE_NAME, hx, hy, f"H{c + 1}B")

    # Hub's own CMOD_A7_35T + FT232H + TCXO, tucked into the gaps between
    # the 4 spoke sockets (each socket sits at 90c+50.2deg -- see
    # CMOD_S7_PLACEMENT_ANGLE_DEG -- so the gap midpoints are at +45deg
    # from that, i.e. 90c+95.2deg).
    ax, ay, arot = _radial_dip48_pose(CMOD_S7_PLACEMENT_R_MM, 95.2)
    board.Add(load_fp(DIP48_LIB, DIP48_NAME, f"A{N_CLUSTERS + 1}", ax, ay, rot_deg=arot))

    ft_r, ft_deg = 65.0, 185.2
    board.Add(load_fp(LOCAL_LIB, "FT232H_Breakout", f"A{N_CLUSTERS + 2}",
                       ft_r * math.cos(math.radians(ft_deg)), ft_r * math.sin(math.radians(ft_deg)),
                       rot_deg=90.0 - ft_deg))

    tcxo_r, tcxo_deg = 65.0, 275.2
    board.Add(load_fp(LOCAL_LIB, "TCXO_Can", "Y1",
                       tcxo_r * math.cos(math.radians(tcxo_deg)), tcxo_r * math.sin(math.radians(tcxo_deg))))

    # Camera mount (placeholder mounting holes either side of the cutout).
    cx0, cy0 = CAMERA_MOUNT_HOLE_SPACING_MM / 2.0, 0.0
    add_mounting_hole(board, STANDOFF_HOLE_NAME, -cx0, cy0, "HCAM1")
    add_mounting_hole(board, STANDOFF_HOLE_NAME, cx0, cy0, "HCAM2")

    # Raspberry Pi 5 mounting holes (own standoffs, stacked behind the hub).
    prx, pry = RPI5_CENTRE_XY_MM
    hx, hy = RPI5_HOLE_HALF_SPACING_MM
    for i, (sx, sy) in enumerate([(-hx, -hy), (hx, -hy), (-hx, hy), (hx, hy)]):
        add_mounting_hole(board, STANDOFF_HOLE_NAME, prx + sx, pry + sy, f"HPI{i + 1}")


# ── main ─────────────────────────────────────────────────────────────────

def main():
    with open(CSV_PATH, newline="") as f:
        rows = list(csv.DictReader(f))

    # Plan pass: find each cluster's mic rows + its Cmod S7 placement
    # before drawing anything. R_BOARD_MAX_MM depends on how far out the
    # search actually needs to go, and the 4 boards must stay identical
    # (just rotated 90deg each) -- so every cluster_outline_points() call
    # has to use the SAME final R_BOARD_MAX_MM, decided only after all 4
    # searches are in.
    cluster_mic_rows = []
    placements = []
    max_reach = 0.0
    for c in range(N_CLUSTERS):
        arms = range(c * ARMS_PER_CLUSTER, (c + 1) * ARMS_PER_CLUSTER)
        mic_rows = [r for r in rows if int(r["arm_idx"]) in arms]
        cluster_mic_rows.append(mic_rows)

        mx, my, rot_deg, max_r = find_module_placement(c, mic_rows)
        placements.append((mx, my, rot_deg))
        max_reach = max(max_reach, max_r)

    global R_BOARD_MAX_MM
    R_BOARD_MAX_MM = max(R_MAX_MM + 5.0, max_reach + 3.0)  # 3mm clearance past the furthest module corner found

    # Build pass: now that R_BOARD_MAX_MM is final, draw outlines + place
    # everything -- 5 separate boards (5 separate pcbnew.BOARD() objects,
    # each saved to its own file), not one file with 5 outlines in it. See
    # module docstring for why.
    os.makedirs(OUTDIR, exist_ok=True)
    total_fp = 0
    # 24 mics + 24 caps + 1 Cmod S7 + 1 cluster<->hub standoff + 2 enclosure holes
    expected_per_cluster = 24 + 24 + 1 + 1 + 2
    for c in range(N_CLUSTERS):
        board = pcbnew.CreateEmptyBoard()
        build_cluster(board, c, cluster_mic_rows[c], placements[c])
        path = board_pcb_path(f"cluster_{c:02d}")
        board.Save(path)
        n_fp = len(board.GetFootprints())
        total_fp += n_fp
        print(f"Saved {path} -- {n_fp} footprints (expected {expected_per_cluster})")

    hub_board = pcbnew.CreateEmptyBoard()
    place_hub(hub_board)
    hub_path = board_pcb_path("hub")
    hub_board.Save(hub_path)
    n_fp_hub = len(hub_board.GetFootprints())
    total_fp += n_fp_hub
    # CMOD_A7_35T + FT232H + TCXO + 4 spoke sockets + 4 hub-side standoffs
    # + 2 camera holes + 4 Pi 5 holes
    expected_hub = 3 + N_CLUSTERS + N_CLUSTERS + 2 + 4
    print(f"Saved {hub_path} -- {n_fp_hub} footprints (expected {expected_hub})")

    print(f"Total footprints across all 5 boards: {total_fp}")
    print(f"R_BOARD_MAX_MM = {R_BOARD_MAX_MM:.2f}")


if __name__ == "__main__":
    main()
