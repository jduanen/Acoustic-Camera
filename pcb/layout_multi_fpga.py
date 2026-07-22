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

# Radial margin beyond R_MAX_MM given to each cluster board for its Cmod S7
# module + clearance (DIP-48 socket footprint spans ~58.4mm across its pad
# rows lengthwise, plus margin on both ends).
BOARD_OUTER_MARGIN_MM = 90.0
R_BOARD_MAX_MM = R_MAX_MM + BOARD_OUTER_MARGIN_MM

# ── Underbrink log-spiral helpers (mirrors notebooks/make_nb18.py exactly) ──

_B = 1.0 / math.tan(math.radians(SPIRAL_DEG))


def _t_at_r(r_mm):
    """Spiral parameter t (radians of sweep from r_min) at radius r_mm."""
    return math.log(r_mm / R_MIN_MM) / _B


def _r_at_t(t):
    return R_MIN_MM * math.exp(_B * t)


T_BOARD_MAX = _t_at_r(R_BOARD_MAX_MM)


def _cut_curve_points(offset_deg, n=24, t_max=T_BOARD_MAX):
    """Sample points (mm) along the boundary spiral rotated by offset_deg,
    from r=R_MIN_MM (t=0) out to r=R_BOARD_MAX_MM (t=t_max)."""
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


# ── cluster board ────────────────────────────────────────────────────────

def place_cluster(board, c, mic_rows):
    add_outline(board, cluster_outline_points(c))

    for row in mic_rows:
        mic_idx = int(row["mic_idx"])
        x, y = float(row["x_mm"]), float(row["y_mm"])
        u_ref = f"U{mic_idx + 1}"
        c_ref = f"C{mic_idx + 1}"
        board.Add(load_fp(MIC_FP_LIB, MIC_FP_NAME, u_ref, x, y))

        # Cap offset 3mm further out along the mic's own radial direction
        # from the array centre -- keeps it clear of the mic footprint
        # without needing per-mic hand placement.
        r = math.hypot(x, y)
        if r > 0:
            cx, cy = x + 3.0 * x / r, y + 3.0 * y / r
        else:
            cx, cy = x + 3.0, y
        board.Add(load_fp(CAP_FP_LIB, CAP_FP_NAME, c_ref, cx, cy))

    # Cmod S7: DIP-48 socket, on the cluster's angular bisector, long axis
    # (the footprint's local +Y, spanning its 24-pin row length) pointed
    # radially outward, pin-1 end (the footprint's local origin -- KiCad
    # places DIP parts with the origin at pin 1, not body centre) just past
    # the mic ring, body extending further out from there.
    #
    # SetOrientationDegrees(t) maps local +Y to world direction
    # (sin t, cos t) (confirmed empirically: t=0 keeps +Y -> world +Y;
    # t=90 maps +Y -> world +X). To land local +Y on world angle phi
    # (using this script's (cos phi, sin phi) convention, matching
    # underbrink_array()'s x=r*cos(t), y=r*sin(t)), solve
    # (sin t, cos t) = (cos phi, sin phi) -> t = 90 - phi.
    bisector_deg = 90.0 * c + 30.0
    module_r = R_MAX_MM + 5.0  # pin-1 end, just clear of the mic ring
    theta = math.radians(bisector_deg)
    mx, my = module_r * math.cos(theta), module_r * math.sin(theta)
    board.Add(load_fp(DIP48_LIB, DIP48_NAME, f"A{c + 1}", mx, my, rot_deg=90.0 - bisector_deg))


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
    # in the rotated +Y direction -- see place_cluster()'s comment).
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

    board = pcbnew.CreateEmptyBoard()

    for c in range(N_CLUSTERS):
        arms = range(c * ARMS_PER_CLUSTER, (c + 1) * ARMS_PER_CLUSTER)
        mic_rows = [r for r in rows if int(r["arm_idx"]) in arms]
        place_cluster(board, c, mic_rows)

    place_hub(board)

    os.makedirs(OUTDIR, exist_ok=True)
    board.Save(OUT_PCB)

    n_fp = len(board.GetFootprints())
    print(f"Saved {OUT_PCB}")
    print(f"Footprints placed: {n_fp} (expected {96 + 96 + N_CLUSTERS + 3})")


if __name__ == "__main__":
    main()
