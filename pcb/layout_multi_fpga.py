#!/usr/bin/env python3
"""
layout_multi_fpga.py -- board outline + footprint placement for the Phase 4
Multi-FPGA (Clustered) alternative: 4 quadrant cluster boards (Cmod S7 + 24
mics + their decoupling caps each) and 1 hub board (Cmod A7-35T + FT232H
breakout + TCXO), all in one pcb/multi_fpga/layout.kicad_pcb file.

Scope: board outlines + footprint placement only, no routing (mirrors
pcb/place_mics.py's placement-only precedent for the single-FPGA board).
Unlike place_mics.py, this does not require "Update PCB from Schematic" or
KiCad's in-app Scripting Console first -- it builds the board from scratch,
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

ROOT       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(ROOT, "..", "test", "phase4", "array_xy.csv")
OUTDIR     = os.path.join(ROOT, "multi_fpga")
OUT_PCB    = os.path.join(OUTDIR, "layout.kicad_pcb")

MIC_FP_LIB  = os.path.join(ROOT, "IM72D128", "KiCad")
MIC_FP_NAME = "IFX-PG-LLGA-5-4"
DIP48_LIB   = "/usr/share/kicad/footprints/Package_DIP.pretty"
DIP48_NAME  = "DIP-48_W15.24mm_Socket"
LOCAL_LIB   = os.path.join(OUTDIR, "footprints.pretty")
CAP_FP_LIB  = "/usr/share/kicad/footprints/Capacitor_SMD.pretty"
CAP_FP_NAME = "C_0603_1608Metric"

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


def _cut_curve_points(offset_deg, n=24, t_max=None):
    """Sample points (mm) along the boundary spiral rotated by offset_deg,
    from r=R_MIN_MM (t=0) out to r=R_BOARD_MAX_MM (t=t_max). Looks up the
    current R_BOARD_MAX_MM fresh each call (rather than a bound default
    argument) since main() finalises that value only after searching for
    where the Cmod S7 modules actually land -- see R_BOARD_MAX_MM comment."""
    if t_max is None:
        t_max = _t_at_r(R_BOARD_MAX_MM)
    pts = []
    for i in range(n):
        t = t_max * i / (n - 1)
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
    inner_arc = _arc_points(R_MIN_MM, before, after)
    outer_arc = _arc_points(R_BOARD_MAX_MM, after, before)
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


def _point_in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i, (xi, yi) in enumerate(poly):
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


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


# ── exact (closed-form) wedge membership, for the Cmod S7 placement search ──
# cluster_outline_points()'s polygon is sampled (24 points/curve) and good
# enough for drawing Edge.Cuts and the earlier corner-containment check, but
# the wedge boundary has a closed form: at any radius r, the wedge is
# exactly the +-45deg band around wedge_center_deg(c, r) (both boundary
# curves are the same spiral offset by a constant 90deg, so the band width
# is exactly 90deg at every radius -- see module docstring). Using the
# closed form here avoids sampling-resolution error while searching.

def _wedge_center_deg(c, r_mm):
    return 90.0 * c + 30.0 + math.degrees(_t_at_r(r_mm))


def _angle_diff(a_deg, b_deg):
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def _point_in_own_wedge(c, x, y, half_width_deg=45.0):
    r = math.hypot(x, y)
    if r < R_MIN_MM:
        return False
    ang = math.degrees(math.atan2(y, x))
    return abs(_angle_diff(ang, _wedge_center_deg(c, r))) <= half_width_deg


# ── obstacle collision (mic/cap footprints vs. a candidate module pose) ─────

def _to_local(px_mm, py_mm, rot_deg, ox_mm, oy_mm):
    """World point (ox,oy) expressed in the local frame of a footprint
    placed at (px,py) with the given rotation -- inverse of _true_corners()'s
    transform. xdir/ydir are orthonormal, so the inverse is just a dot
    product against each."""
    t = math.radians(rot_deg)
    xdir = (math.cos(t), -math.sin(t))
    ydir = (math.sin(t), math.cos(t))
    dx, dy = ox_mm - px_mm, oy_mm - py_mm
    return dx * xdir[0] + dy * xdir[1], dx * ydir[0] + dy * ydir[1]


def _collides(px_mm, py_mm, rot_deg, local_bbox, obstacles):
    """obstacles: list of (x_mm, y_mm, radius_mm) circular proxies (mic/cap
    courtyards). local_bbox: (x0,y0,x1,y1) of the module's own footprint in
    its local frame."""
    x0, y0, x1, y1 = local_bbox
    for ox, oy, orad in obstacles:
        lx, ly = _to_local(px_mm, py_mm, rot_deg, ox, oy)
        if (x0 - orad) <= lx <= (x1 + orad) and (y0 - orad) <= ly <= (y1 + orad):
            return True
    return False


# ── cluster board ────────────────────────────────────────────────────────

MIC_COURTYARD_RADIUS_MM = 3.2   # IM72D128 courtyard half-extent, rounded up
CAP_COURTYARD_RADIUS_MM = 1.55  # C_0603 courtyard half-extent, rounded up

# Cmod S7 search range/resolution -- see find_module_placement().
_GAP_R_MIN_MM   = R_MIN_MM + 5.0
_GAP_R_MAX_MM   = R_MAX_MM + 60.0
_GAP_R_STEP_MM  = 0.5
_GAP_DELTA_DEG  = 60.0
_GAP_DELTA_STEP = 0.5


def _mic_and_cap_xy(x, y):
    """Cap position for a mic at (x,y): offset 3mm further out along the
    mic's own radial direction from the array centre. Shared by the
    obstacle list (find_module_placement()) and the actual placement
    (build_cluster()) so the search sees exactly what gets placed."""
    r = math.hypot(x, y)
    if r > 0:
        return x + 3.0 * x / r, y + 3.0 * y / r
    return x + 3.0, y


def find_module_placement(c, mic_rows):
    """Search for where cluster c's Cmod S7 module goes: tucked into the
    natural gap between this cluster's 2nd and 3rd arms (arm indices
    3c+1, 3c+2 -- an arbitrary but fixed choice of which of the cluster's
    2 internal arm-to-arm gaps to use, consistent across all 4 clusters),
    as far in towards the array centre as it can go without colliding
    with a mic or its decoupling cap.

    Orientation is now RADIAL (long axis running along the gap, outward
    from the centre), not tangential (across it) -- that gap is a wedge
    that narrows towards the centre and widens outward (the two arms
    diverge with radius, same as every arm-to-arm gap), so a long thin
    module fits it lengthwise, the way a book fits a shelf, rather than
    lying across it. Short axis (radial version: local +X) sits centred
    on the gap's angular bisector; long axis (local +Y) points straight
    out along it, pin-1 end innermost.

    The gap's angular centre is tracked with radius the same way the
    wedge bisector is (see cluster_outline_points()'s docstring on spiral
    drift): at r=R_MAX_MM this is exactly 15deg off the wedge's own
    bisector (which sits on arm 3c+1's own curve) -- i.e. comfortably
    inside this cluster's own wedge (half-width 45deg), not near either
    boundary, so the binding constraint here is mic/cap collision, not the
    board edge.

    Returns (mx, my, rot_deg, max_corner_radius_mm)."""
    obstacles = []
    for row in mic_rows:
        x, y = float(row["x_mm"]), float(row["y_mm"])
        obstacles.append((x, y, MIC_COURTYARD_RADIUS_MM))
        cx, cy = _mic_and_cap_xy(x, y)
        obstacles.append((cx, cy, CAP_COURTYARD_RADIUS_MM))

    local_corners = _local_courtyard_corners(DIP48_LIB, DIP48_NAME)
    xs = [p[0] for p in local_corners]
    ys = [p[1] for p in local_corners]
    local_bbox = (min(xs), min(ys), max(xs), max(ys))
    # Pin 1 (the footprint's local origin) sits at one END of the SHORT
    # (local X) axis too, not its centre -- shift so the short axis's
    # true midpoint, not the origin, lands on the gap's bisector line.
    short_axis_centre_shift = -(local_bbox[0] + local_bbox[2]) / 2.0

    def place_at(r0, extra_delta_deg):
        """r0: radius of the pin-1 (innermost) end of the module, along
        the gap's own bisector line."""
        angle_deg = 90.0 * c + 45.0 + math.degrees(_t_at_r(r0)) + extra_delta_deg
        rot_deg = 90.0 - angle_deg  # local +Y -> world angle angle_deg (radial); see module docstring's rotation derivation
        theta = math.radians(angle_deg)
        rot_rad = math.radians(rot_deg)
        xdir = (math.cos(rot_rad), -math.sin(rot_rad))  # local +X world direction (tangential)
        mx = r0 * math.cos(theta) + short_axis_centre_shift * xdir[0]
        my = r0 * math.sin(theta) + short_axis_centre_shift * xdir[1]
        return mx, my, rot_deg

    def fits(r0, extra_delta_deg):
        mx, my, rot_deg = place_at(r0, extra_delta_deg)
        if _collides(mx, my, rot_deg, local_bbox, obstacles):
            return False
        corners = _true_corners(mx, my, rot_deg, local_corners)
        return all(_point_in_own_wedge(c, x, y) for x, y in corners)

    def largest_contiguous_run_midpoint(deltas_sorted):
        """deltas_sorted: ascending list of deltas that individually pass
        fits(). With real point obstacles (mics/caps) in the way, the
        valid set isn't necessarily one contiguous band -- there can be a
        clear window on each side of an obstacle with a blocked gap in
        between. Taking the midpoint of the overall min/max (fine when the
        only constraint was the two smooth boundary curves) can land
        straight in such a gap. Find the widest contiguous run instead and
        use its midpoint."""
        runs = []
        start = deltas_sorted[0]
        prev = start
        for d in deltas_sorted[1:]:
            if d - prev > _GAP_DELTA_STEP * 1.5:
                runs.append((start, prev))
                start = d
            prev = d
        runs.append((start, prev))
        lo, hi = max(runs, key=lambda run: run[1] - run[0])
        return (lo + hi) / 2.0

    n_r = int(round((_GAP_R_MAX_MM - _GAP_R_MIN_MM) / _GAP_R_STEP_MM))
    n_d = int(round(2 * _GAP_DELTA_DEG / _GAP_DELTA_STEP))
    for i in range(n_r + 1):
        r0 = _GAP_R_MIN_MM + i * _GAP_R_STEP_MM
        ok_deltas = [
            -_GAP_DELTA_DEG + j * _GAP_DELTA_STEP
            for j in range(n_d + 1)
            if fits(r0, -_GAP_DELTA_DEG + j * _GAP_DELTA_STEP)
        ]
        if ok_deltas:
            extra_delta_deg = largest_contiguous_run_midpoint(ok_deltas)
            # The midpoint is derived from the coarse scan grid, not
            # verified directly -- confirm it (it should always pass,
            # since it's built from two points that did, but the module
            # is a rigid shape so don't trust that blindly).
            if fits(r0, extra_delta_deg):
                mx, my, rot_deg = place_at(r0, extra_delta_deg)
                corners = _true_corners(mx, my, rot_deg, local_corners)
                max_r = max(math.hypot(x, y) for x, y in corners)
                return mx, my, rot_deg, max_r

    raise RuntimeError(f"cluster {c}: no collision-free placement found for the Cmod S7 module")


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


# ── hub board ────────────────────────────────────────────────────────────

def place_hub(board):
    # Simple rectangle, offset well clear of the 4-cluster assembly.
    hub_w, hub_h = 160.0, 90.0
    hub_cx, hub_cy = 0.0, -(R_BOARD_MAX_MM + 60.0 + hub_h / 2.0)
    x0, y0 = hub_cx - hub_w / 2.0, hub_cy - hub_h / 2.0
    add_outline(board, [
        (x0, y0), (x0 + hub_w, y0), (x0 + hub_w, y0 + hub_h), (x0, y0 + hub_h),
    ])

    # CMOD_A7_35T uses the same corner-origin DIP-48 footprint as the
    # cluster boards' Cmod S7 (pin 1 at the origin, body extending 58.42mm
    # in the rotated +Y direction -- see find_module_placement()'s comment).
    board.Add(load_fp(DIP48_LIB, DIP48_NAME, f"A{N_CLUSTERS + 1}",
                       hub_cx - 70.0, hub_cy, rot_deg=90.0))
    board.Add(load_fp(LOCAL_LIB, "FT232H_Breakout", f"A{N_CLUSTERS + 2}",
                       hub_cx + 18.0, hub_cy, rot_deg=90.0))
    board.Add(load_fp(LOCAL_LIB, "TCXO_Can", "Y1",
                       hub_cx + 50.0, hub_cy))


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
    # everything.
    board = pcbnew.CreateEmptyBoard()
    for c in range(N_CLUSTERS):
        build_cluster(board, c, cluster_mic_rows[c], placements[c])
    place_hub(board)

    os.makedirs(OUTDIR, exist_ok=True)
    board.Save(OUT_PCB)

    n_fp = len(board.GetFootprints())
    print(f"Saved {OUT_PCB}")
    print(f"Footprints placed: {n_fp} (expected {96 + 96 + N_CLUSTERS + 3})")
    print(f"R_BOARD_MAX_MM = {R_BOARD_MAX_MM:.2f}")


if __name__ == "__main__":
    main()
