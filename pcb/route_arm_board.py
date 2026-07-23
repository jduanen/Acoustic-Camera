#!/usr/bin/env python3
"""
route_arm_board.py -- assign nets to arm_board.kicad_pcb's pads (from
pcb/multi_fpga/cluster_00.kicad_sch's actual wiring, since arm_board is
built once at c=0 and its local U1-U24/C1-C24/A1 numbering coincides
exactly with cluster_00's own global numbering -- see
layout_multi_fpga.py's module docstring), then autoroute it via
Freerouting (Specctra DSN export -> external autorouter -> Specctra SES
import).

Net assignment mirrors pcb/make_schematic.py's make_arm() (per-mic wiring)
and pcb/make_schematic_multi_fpga.py's make_cluster() (Cmod S7 DIP header)
pad-for-pad:
  mic (IFX-PG-LLGA-5-4) pad 1=VDD->+1V8, 2=GND, 3=DATA->DATA_NN, 4=CLK->
    C0_PDM_CLK, 5=SEL->GND(L)/+1V8(R); unnamed pad = NPTH mechanical, no net.
  cap (C_0603_1608Metric) pad 1->+1V8, 2->GND.
  Cmod S7 (DIP-48, A1): pad N == Digilent's own "D{N}" pin numbering
    (confirmed: pad1 at one end of the left column, pad25 starts the right
    column, standard IC DIP numbering) -- pad1->C0_PDM_CLK, pads2-13->
    DATA_00..DATA_11, pad24 (VU)->+5V, pad25 (GND)->GND. Pads 14-23/26-48
    are real DIP pins with no net in this design (unused GPIO), left
    without a net -- not a routing gap, just unused hardware pins.

KNOWN GAP (not solved here, flagged instead): the schematic's spoke bus
(SPOKE0_D0-D5/STROBE/CLK/GND/VU) is wired to Cmod S7's Pmod JA header, but
the PCB layout only places a single 48-pin DIP footprint for Cmod S7 -- no
separate Pmod JA footprint/pads exist anywhere on this board. J1 (the
spoke header) therefore has no physically-modelled source pad to route
to; its pins are left netless here rather than inventing a connection
that isn't actually modelled in copper. Adding a Pmod JA footprint to the
Cmod S7 placement is a separate follow-up (mechanical placement, not a
routing-script concern).

Usage (from project root):
  python pcb/route_arm_board.py                 # assign nets + set up design rules, save
  python pcb/route_arm_board.py --dsn out.dsn    # + export Specctra DSN for Freerouting
  python pcb/route_arm_board.py --ses in.ses     # + import a routed Specctra session back
"""

import argparse
import os
import sys

import pcbnew

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "multi_fpga")
PCB_PATH = os.path.join(OUTDIR, "arm_board.kicad_pcb")

N_PER_ARM = 8
N_PAIRS = N_PER_ARM // 2  # 4
CLK_NET = "C0_PDM_CLK"    # cluster 0's forwarded PDM clock (see make_cluster(0))

# Track width / clearance -- reasonable defaults for this board's signals
# (slow decimated PDM-ish digital I/O, not high-speed/impedance-controlled):
# 0.25mm trace / 0.2mm clearance, comfortably inside what most fab houses
# offer at standard (non-premium) pricing.
TRACK_WIDTH_MM = 0.25
CLEARANCE_MM = 0.32  # > KiCad's default 0.25mm hole-clearance rule (with margin) --
                      # the mic footprint's own NPTH mounting hole sits close to
                      # neighbouring copper, and the first autoroute pass placed
                      # several new vias/traces inside that hole's clearance zone
VIA_DIA_MM = 0.6
VIA_DRILL_MM = 0.3


def get_or_create_net(board, netinfo_list, name):
    net = netinfo_list.GetNetItem(name)
    if net is not None:
        return net
    net = pcbnew.NETINFO_ITEM(board, name)
    board.Add(net)
    return net


def mic_nets(g):
    """g: 0-based global mic index (0-23, this cluster's own 3 arms)."""
    arm_idx = g // N_PER_ARM
    pair = (g % N_PER_ARM) // 2
    is_l = (g % 2 == 0)
    data = arm_idx * N_PAIRS + pair
    return {
        "1": "+1V8",
        "2": "GND",
        "3": f"DATA_{data:02d}",
        "4": CLK_NET,
        "5": "GND" if is_l else "+1V8",
    }


CMOD_S7_PAD_NETS = {1: CLK_NET}
CMOD_S7_PAD_NETS.update({2 + i: f"DATA_{i:02d}" for i in range(12)})
CMOD_S7_PAD_NETS[24] = "+5V"
CMOD_S7_PAD_NETS[25] = "GND"


def assign_nets(board):
    netinfo_list = board.GetNetInfo()
    assigned = 0
    skipped_j1 = 0
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref.startswith("U"):
            g = int(ref[1:]) - 1
            nets = mic_nets(g)
            for pad in fp.Pads():
                name = nets.get(pad.GetPadName())
                if name is None:
                    continue  # unnamed/NPTH mechanical pad
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref.startswith("C") and fp.GetPadCount() == 2:
            for pad in fp.Pads():
                name = "+1V8" if pad.GetPadName() == "1" else "GND"
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "A1":
            for pad in fp.Pads():
                pad_num = int(pad.GetPadName())
                name = CMOD_S7_PAD_NETS.get(pad_num)
                if name is None:
                    continue  # real DIP pin, unused in this design
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "J1":
            # Spoke header -- see module docstring's KNOWN GAP: no modelled
            # Pmod JA pads on the Cmod S7 footprint to route these to.
            skipped_j1 += fp.GetPadCount()
        # H1A/H1E1/H1E2 (mounting holes): mechanical only, no net -- untouched.
    return assigned, skipped_j1


def set_design_rules(board):
    ds = board.GetDesignSettings()
    dnc = ds.m_NetSettings.GetDefaultNetclass()
    dnc.SetTrackWidth(pcbnew.FromMM(TRACK_WIDTH_MM))
    dnc.SetClearance(pcbnew.FromMM(CLEARANCE_MM))
    dnc.SetViaDiameter(pcbnew.FromMM(VIA_DIA_MM))
    dnc.SetViaDrill(pcbnew.FromMM(VIA_DRILL_MM))
    ds.SetTrackWidthIndex(0)
    ds.SetViaSizeIndex(0)


def _board_outline_polygon(board):
    """The board's own Edge.Cuts polygon (as saved -- already page-shifted),
    reused directly as each pour zone's outline rather than recomputing the
    logical/unshifted shape, since a zone just needs some enclosing area."""
    for s in board.GetDrawings():
        if s.GetLayerName() == "Edge.Cuts" and s.GetShape() == pcbnew.SHAPE_T_POLY:
            outline = s.GetPolyShape().Outline(0)
            return [(outline.CPoint(i).x, outline.CPoint(i).y) for i in range(outline.PointCount())]
    raise RuntimeError("no Edge.Cuts polygon found")


def add_power_zones(board):
    """GND pour, covering the whole board outline -- mops up most of the
    point-to-point-unroutable many-to-many GND connections left over after
    autorouting.

    F.Cu, not B.Cu: the mic (IFX-PG-LLGA-5-4) and cap (C_0603_1608Metric)
    footprints are single-sided SMD -- their pads carry copper on F.Cu only
    (confirmed via each pad's GetLayerSet()), so a B.Cu pour would only
    reach A1's through-hole DIP48 pins and any pad a via happens to already
    land under, missing virtually every mic/cap GND pad. Only GND gets a
    pour: +1V8 needs the same F.Cu real estate (same single-sided SMD
    pads), and two full-board pours can't both occupy one layer without via
    stitching -- +1V8 is left to Freerouting's traces (see module docstring
    KNOWN GAP note in the routing summary)."""
    netinfo = board.GetNetInfo()
    outline_pts = _board_outline_polygon(board)
    zones = []
    for net_name, layer in [("GND", pcbnew.F_Cu)]:
        net = netinfo.GetNetItem(net_name)
        if net is None:
            raise RuntimeError(f"net {net_name!r} not found -- run net assignment first")
        zone = pcbnew.ZONE(board)
        poly = pcbnew.SHAPE_POLY_SET()
        poly.NewOutline()
        for x, y in outline_pts:
            poly.Append(pcbnew.VECTOR2I(x, y))
        zone.SetOutline(poly)
        poly.thisown = False  # ZONE::SetOutline() takes ownership of the raw
                               # pointer; without releasing SWIG's Python-side
                               # ownership too, `poly` going out of scope frees
                               # it a second time -- segfaults inside Fill()
                               # later (intermittently, since it depends on
                               # when the GC actually runs).
        zone.SetLayer(layer)
        zone.SetNetCode(net.GetNetCode())
        zone.SetZoneName(f"{net_name}_POUR")
        zone.SetIsFilled(False)
        board.Add(zone)
        zones.append(zone)
    for zone in zones:
        pcbnew.ZONE_FILLER(board).Fill([zone])
    return zones


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsn", help="export a Specctra DSN file for Freerouting after net assignment")
    ap.add_argument("--ses", help="import a routed Specctra SES file back onto the board")
    ap.add_argument("--zones", action="store_true", help="add GND (B.Cu) / +1V8 (F.Cu) pour zones and fill them")
    args = ap.parse_args()

    board = pcbnew.LoadBoard(PCB_PATH)

    if args.ses:
        ok = pcbnew.ImportSpecctraSES(board, args.ses)
        if not ok:
            print(f"ImportSpecctraSES failed for {args.ses}", file=sys.stderr)
            sys.exit(1)
        board.Save(PCB_PATH)
        print(f"Imported routed session {args.ses} -> {PCB_PATH}")
        return

    if args.zones:
        zones = add_power_zones(board)
        board.Save(PCB_PATH)
        print(f"Added + filled {len(zones)} pour zone(s) (GND on F.Cu).")
        return

    assigned, skipped_j1 = assign_nets(board)
    set_design_rules(board)
    board.Save(PCB_PATH)
    print(f"Assigned {assigned} pad nets, {board.GetNetInfo().GetNetCount() - 1} distinct nets "
          f"(excluding the default no-net).")
    print(f"J1 (spoke header): {skipped_j1} pads left netless -- no modelled Pmod JA pad on "
          f"Cmod S7 to route them to, see module docstring KNOWN GAP.")

    if args.dsn:
        ok = pcbnew.ExportSpecctraDSN(board, args.dsn)
        if not ok:
            print(f"ExportSpecctraDSN failed for {args.dsn}", file=sys.stderr)
            sys.exit(1)
        print(f"Exported {args.dsn}")


if __name__ == "__main__":
    main()
