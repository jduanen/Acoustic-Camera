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
and pcb/make_schematic_multi_fpga.py's make_cluster() (this cluster's own
Cmod A7-35T DIP header, power connector, LDO) pad-for-pad:
  mic (IFX-PG-LLGA-5-4) pad numbers match the REAL IM72D128V01 datasheet
    (Table 9: 1=DATA, 2=VDD, 3=CLOCK, 4=SELECT, 5=GND -- confirmed against
    both the datasheet and the vendor's own KiCad symbol,
    pcb/IM72D128/KiCad/IM72D128V.kicad_sym; an earlier revision of this
    project used a fabricated, wrong numbering (1=VDD,2=GND,3=DATA,4=CLK,
    5=SEL) that would have shorted VDD to GND and driven DC power onto the
    DATA pin if built -- see make_schematic.py's _lib_mic() for the fix):
    pad1->DATA_NN, pad2->C0_1V8, pad3->C0_PDM_CLK, pad4->GND(L)/C0_1V8(R),
    pad5->GND; unnamed pad = NPTH mechanical, no net.
  cap C1-C24 (mic decoupling, C_0603_1608Metric): placed close to that
    mic's own VDD (pad2)/GND (pad5) specifically (see
    layout_multi_fpga.py's _mic_and_cap_xy()) -- pad1->C0_1V8 (near VDD),
    pad2->GND (near GND, ~0.28mm away -- see fix_gnd_stub() for why this
    matters).
  Cmod A7-35T (DIP-48, A1): pad N == Digilent's own "D{N}" pin numbering
    (confirmed: pad1 at one end of the left column, pad25 starts the right
    column, standard IC DIP numbering) -- pad1->C0_PDM_CLK, pads2-13->
    DATA_00..DATA_11, pad24 (VU)->+5V, pad25 (GND)->GND, pads20-23/26-28/18->
    SPOKE0_D0-D5/STROBE/CLK (see SPOKE_SIGNAL_SUFFIX below -- taken directly
    from fpga/cluster/xdc/cluster_top.xdc's real, already place-and-routed
    pin constraints, cross-checked against Digilent's own CmodA7_Master.xdc;
    fixes the KNOWN GAP this used to describe, see below). All other DIP
    pins (excl. spoke pins) are real DIP pins with no net in this design
    (unused GPIO), left without a net -- not a routing gap, just unused
    hardware pins.
  J1 (spoke header, PinHeader_2x06, same Pmod-2x6 pad layout as the hub's
    J1-J4): positions 1,2,3,4,7,8,9,10 -> SPOKE0_D0-D5/STROBE/CLK (see
    J1_POS_TO_SUFFIX_IDX); 5,6,11,12 unused (would be the Cmod's own Pmod
    JA GND/VCC3V3 pins if this were still wired to the Pmod header -- it
    isn't, see below -- left unpopulated for mechanical/footprint-shape
    reasons only).
  J2 (power connector from hub, PinHeader_1x02): pad1->+5V, pad2->GND.
  VR1 (MCP1700-1802 LDO, SOT-23): pad1 (GND)->GND, pad2 (OUT)->C0_1V8,
    pad3 (IN)->+5V (see LDO_PINS in make_schematic_multi_fpga.py).
  C25/C26 (VR1's bypass caps): C25 (input side) pad1->+5V, pad2->GND;
    C26 (output side) pad1->C0_1V8, pad2->GND.

  C0_1V8 (not the single-FPGA design's shared "+1V8"): this cluster's own
  independent +1.8V rail, fed by its own local VR1 -- see
  pcb/make_schematic.py's make_arm() vdd_label parameter and
  SCHEMATIC_NOTES.md's power section for why (4 arm boards, 4 independent
  LDOs, can't share one global net).

FORMER KNOWN GAP, now fixed: the spoke bus used to be wired (in the
schematic) to the cluster's Cmod's Pmod JA header -- but the PCB layout
only ever placed a single 48-pin DIP footprint for it, no separate Pmod JA
footprint/pads, so J1 had no physically-modelled source pad to route to.
Worse, it turned out the Pmod JA pins aren't even reachable from the DIP
header at all on the real module (confirmed against Digilent's own
schematic: the 8 Pmod JA balls are electrically distinct from every ball
already broken out to the DIP header) -- so the old approach could never
have been fixed by just adding a Pmod footprint either. Fixed by moving
the spoke bus onto real DIP-connected, otherwise-unused GPIO pins instead
(20-23/26-28/18, see above) -- see cluster_00..03.kicad_sch's
CMOD_A7_35T_CLUSTER symbol and SCHEMATIC_NOTES.md.

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
VDD_NET = "C0_1V8"        # cluster 0's own independent +1.8V rail (see module docstring)

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
        "1": f"DATA_{data:02d}",
        "2": VDD_NET,
        "3": CLK_NET,
        "4": "GND" if is_l else VDD_NET,
        "5": "GND",
    }


CLUSTER_PAD_NETS = {1: CLK_NET}
CLUSTER_PAD_NETS.update({2 + i: f"DATA_{i:02d}" for i in range(12)})
CLUSTER_PAD_NETS[24] = "+5V"
CLUSTER_PAD_NETS[25] = "GND"

# Spoke bus, DIP pins 20, 21, 22, 23, 26, 27, 28, 18 (in D0..CLK order --
# not contiguous, unlike the PDM pins above) -- taken directly from
# fpga/cluster/xdc/cluster_top.xdc's real, already place-and-routed
# SPOKE_D0-5/SPOKE_STROBE/SPOKE_CLK pin constraints (same source
# pcb/make_schematic_multi_fpga.py's CLUSTER_SPOKE_DIP_PINS uses), not
# shared with anything else in this design; see cluster_00..03.kicad_sch's
# CMOD_A7_35T_CLUSTER symbol and SCHEMATIC_NOTES.md for the
# Pmod-JA-pins-aren't-on-the-DIP-header bug this replaced.
SPOKE_SIGNAL_SUFFIX = ["D0", "D1", "D2", "D3", "D4", "D5", "STROBE", "CLK"]
CLUSTER_SPOKE_DIP_PINS = [20, 21, 22, 23, 26, 27, 28, 18]
CLUSTER_PAD_NETS.update({dip: f"SPOKE0_{suffix}"
                          for dip, suffix in zip(CLUSTER_SPOKE_DIP_PINS, SPOKE_SIGNAL_SUFFIX)})

# J1 (spoke header, PinHeader_2x06 -- same Pmod-2x6 pad layout/position
# convention as the hub's J1-J4 spoke sockets): positions 1,2,3,4,7,8,9,10 ->
# SPOKE0_D0-D5/STROBE/CLK; 5,6,11,12 unused (see the Pmod JA GND/VCC3V3
# hard-wiring note above -- still true, just no longer relevant to J1's own
# wiring).
J1_POS_TO_SUFFIX_IDX = {1: 0, 2: 1, 3: 2, 4: 3, 7: 4, 8: 5, 9: 6, 10: 7}

# VR1 (LDO_1V8, MCP1700-1802): pad1=GND, pad2=OUT, pad3=IN -- see LDO_PINS
# in make_schematic_multi_fpga.py.
VR1_PAD_NETS = {"1": "GND", "2": VDD_NET, "3": "+5V"}


def assign_nets(board):
    netinfo_list = board.GetNetInfo()
    assigned = 0
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
        elif ref in ("C25", "C26") and fp.GetPadCount() == 2:
            # VR1's bypass caps -- C25 (input side, +5V/GND), C26 (output
            # side, C0_1V8/GND). Not a mic decoupling cap (see below).
            in_net = "+5V" if ref == "C25" else VDD_NET
            for pad in fp.Pads():
                name = in_net if pad.GetPadName() == "1" else "GND"
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref.startswith("C") and fp.GetPadCount() == 2:
            # Mic decoupling caps, C1-C24.
            for pad in fp.Pads():
                name = VDD_NET if pad.GetPadName() == "1" else "GND"
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "A1":
            for pad in fp.Pads():
                pad_num = int(pad.GetPadName())
                name = CLUSTER_PAD_NETS.get(pad_num)
                if name is None:
                    continue  # real DIP pin, unused in this design
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "VR1":
            for pad in fp.Pads():
                name = VR1_PAD_NETS.get(pad.GetPadName())
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "J2":
            # Power connector from the hub: pin1->+5V, pin2->GND.
            for pad in fp.Pads():
                name = "+5V" if pad.GetPadName() == "1" else "GND"
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        elif ref == "J1":
            # Spoke header -- see CLUSTER_PAD_NETS/J1_POS_TO_SUFFIX_IDX above;
            # module docstring's old KNOWN GAP (no modelled DIP pads to route
            # these to) is fixed now that the spoke bus lands on real DIP
            # pins instead of the Cmod's DIP-inaccessible Pmod JA.
            for pad in fp.Pads():
                pos = int(pad.GetPadName())
                if pos not in J1_POS_TO_SUFFIX_IDX:
                    continue
                name = f"SPOKE0_{SPOKE_SIGNAL_SUFFIX[J1_POS_TO_SUFFIX_IDX[pos]]}"
                pad.SetNet(get_or_create_net(board, netinfo_list, name))
                assigned += 1
        # H1A/H1E1/H1E2 (mounting holes): mechanical only, no net -- untouched.
    return assigned


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
    pour: C0_1V8 needs the same F.Cu real estate (same single-sided SMD
    pads), and two full-board pours can't both occupy one layer without via
    stitching -- C0_1V8 is left to Freerouting's traces.

    Removes any existing GND_POUR zone(s) first -- calling this on a board
    that already has one (e.g. re-running --zones) used to silently add a
    second, overlapping zone (DRC's "zones_intersect"), since ZONE objects
    aren't tracked/deduplicated by KiCad automatically."""
    for z in list(board.Zones()):
        if z.GetZoneName() == "GND_POUR":
            board.Remove(z)
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
        # Default 0.5mm thermal gap starves some mic GND pads (e.g. U17 pad
        # 4) of their 2nd required spoke -- the IM72D128 footprint packs a
        # different-net pad only ~0.25mm from GND on one side, less than
        # the gap KiCad needs to route a spoke through. 0.2mm clears that.
        zone.SetThermalReliefGap(pcbnew.FromMM(0.2))
        zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.25))
        zone.SetIsFilled(False)
        board.Add(zone)
        zones.append(zone)
    for zone in zones:
        pcbnew.ZONE_FILLER(board).Fill([zone])
    return zones


GND_STUB_WIDTH_MM = 0.2  # thinner than the board's normal 0.25mm track (this
                         # jumper stays entirely inside the mic/cap pair's
                         # own tiny footprint envelope) but not below the
                         # board's own 0.2mm minimum track width rule
                         # (m_TrackMinWidth).


def fix_gnd_stub(board):
    """Pad 5 (GND) on every one of the 24 mics sits too close to that same
    mic's own NPTH mounting hole for the GND pour or Freerouting's traces to
    legally approach it (confirmed as a footprint-geometry constraint
    hitting all 24 mics identically, not a fluke -- pad 5's position is
    fixed regardless of which pin function we assign it, so this is the
    same physical-clearance problem regardless of the pinout fix above).
    Fixed by hand here rather than via more autorouting: pad 5 connects
    directly to that same mic's own decoupling cap's pad 2 (also GND) --
    thanks to the cap now being placed right next to VDD/GND specifically
    (see layout_multi_fpga.py's _mic_and_cap_xy()), that pad sits only
    ~0.28mm away, and this path never has to get any closer to the NPTH
    hole than pad 5's own position already is."""
    fixed = 0
    fps_by_ref = {fp.GetReference(): fp for fp in board.GetFootprints()}
    for ref, fp in fps_by_ref.items():
        if not (ref.startswith("U") and ref[1:].isdigit()):
            continue
        cap_fp = fps_by_ref.get(f"C{ref[1:]}")
        if cap_fp is None:
            continue
        pad5 = fp.FindPadByNumber("5")
        target = cap_fp.FindPadByNumber("2")
        if pad5 is None or target is None:
            continue
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pad5.GetPosition())
        track.SetEnd(target.GetPosition())
        track.SetWidth(pcbnew.FromMM(GND_STUB_WIDTH_MM))
        track.SetLayer(pcbnew.F_Cu)
        track.SetNetCode(pad5.GetNetCode())
        board.Add(track)
        fixed += 1
    return fixed


def fix_select_stub(board):
    """Pad 4 (SELECT) is left unrouted by Freerouting on every one of the 12
    R-mics (SELECT=C0_1V8 for R, GND for L -- see mic_nets()), consistently
    across repeated autoroute attempts (not a fluke): with the corrected
    pinout, SELECT sits on the opposite side of the mic body (top, y=+0.85)
    from where the cap now lives (bottom, near VDD/GND), so there's no
    nearby same-net copper for it to reach, unlike pad 5 above. Fixed by
    hand: pad 2 (VDD, also C0_1V8 on R-mics) sits at the same local x
    (-0.39) as pad 4, directly above/below it with nothing else on that
    line -- a short straight stitch between them needs no detour."""
    fixed = 0
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if not (ref.startswith("U") and ref[1:].isdigit()):
            continue
        g = int(ref[1:]) - 1
        if g % 2 == 0:
            continue  # L-mic: pad4=GND, pad2=VDD -- different nets, don't stitch
        pad2 = fp.FindPadByNumber("2")
        pad4 = fp.FindPadByNumber("4")
        if pad2 is None or pad4 is None:
            continue
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pad2.GetPosition())
        track.SetEnd(pad4.GetPosition())
        track.SetWidth(pcbnew.FromMM(GND_STUB_WIDTH_MM))
        track.SetLayer(pcbnew.F_Cu)
        track.SetNetCode(pad2.GetNetCode())
        board.Add(track)
        fixed += 1
    return fixed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsn", help="export a Specctra DSN file for Freerouting after net assignment")
    ap.add_argument("--ses", help="import a routed Specctra SES file back onto the board")
    ap.add_argument("--zones", action="store_true", help="add + fill a GND pour zone (F.Cu)")
    ap.add_argument("--fix-gnd", action="store_true",
                     help="manually stitch each mic's GND pad (pad 5) to its own cap's GND pad, "
                          "and each R-mic's SELECT pad (pad 4) to its own VDD pad (pad 2); "
                          "see fix_gnd_stub()/fix_select_stub() docstrings for why this can't be "
                          "left to the autorouter/zone")
    args = ap.parse_args()

    board = pcbnew.LoadBoard(PCB_PATH)

    if args.fix_gnd:
        gnd_fixed = fix_gnd_stub(board)
        sel_fixed = fix_select_stub(board)
        zones = [z for z in board.Zones() if z.GetZoneName() == "GND_POUR"]
        for zone in zones:
            pcbnew.ZONE_FILLER(board).Fill([zone])
        board.Save(PCB_PATH)
        print(f"Stitched {gnd_fixed} mic GND pads to their own cap's GND pad, "
              f"{sel_fixed} R-mic SELECT pads to their own VDD pad, refilled {len(zones)} zone(s).")
        return

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

    assigned = assign_nets(board)
    set_design_rules(board)
    board.Save(PCB_PATH)
    print(f"Assigned {assigned} pad nets, {board.GetNetInfo().GetNetCount() - 1} distinct nets "
          f"(excluding the default no-net).")

    if args.dsn:
        ok = pcbnew.ExportSpecctraDSN(board, args.dsn)
        if not ok:
            print(f"ExportSpecctraDSN failed for {args.dsn}", file=sys.stderr)
            sys.exit(1)
        print(f"Exported {args.dsn}")


if __name__ == "__main__":
    main()
