#!/usr/bin/env python3
"""
route_hub_board.py -- assign nets to hub.kicad_pcb's pads (from
pcb/multi_fpga/hub.kicad_sch's actual wiring, see make_schematic_multi_fpga.py's
make_hub()), then autoroute via Freerouting, same pipeline as
route_arm_board.py (reused directly for the generic net/zone helpers).

Net assignment, pad-for-pad:
  A5 (CMOD_A7_35T, DIP-48): pad N == Digilent's own "D{N}" pin numbering
    (same convention confirmed on the arm board's Cmod S7). Which signal
    lands on which DIP pin comes straight from make_schematic_multi_fpga.py's
    CMOD_A7_35T_DIP list (imported directly, not retyped, so this can't drift
    out of sync with the schematic generator): entries 0-31 are the 4 spokes'
    8 signals each (SPOKE{cidx}_{suffix}), entries 32-44 are the FT232H/TCXO
    signals (USB_TCXO_NETS), pad 24 (VU) -> +5V, pad 25 (GND) -> GND.
  A6 (FT232H_Breakout): pads 1-8 -> USB_D0-7, 9-12 -> USB_RXF_N/TXE_N/RD_N/
    WR_N, 13 (VCCIO) -> +3V3, 14 (GND) -> GND.
  Y1 (TCXO_Can): pad 1 (VDD) -> +3V3, 2 (GND) -> GND, 3 (OE, tied high) ->
    +3V3, 4 (OUT) -> TCXO_CLK.
  J1-J4 (spoke sockets, one per cluster): pin positions 1,2,3,4,7,8,9,10 ->
    that cluster's SPOKE{cidx}_D0-D5/STROBE/CLK (same position convention as
    the arm board's mating header); positions 5/6/11/12 unused -- these are
    hard-wired on the real Cmod S7 to GND/VCC3V3 (that module's own onboard
    regulator output), not a usable power input, so this design doesn't
    wire anything to them (see datasheets/Cmod+S7_sch-public.pdf and
    SCHEMATIC_NOTES.md's power section for the full story).
  J5-J8 (power sockets, one per cluster, PinSocket_1x02): pin1 -> +5V,
    pin2 -> GND -- the actual path +5V/GND take to each arm board, since
    J1-J4 can't carry them.
  VR5 (MCP1700-3302 LDO, SOT-23): pad1 (GND) -> GND, pad2 (OUT) -> +3V3,
    pad3 (IN) -> +5V (same LDO_PINS convention as the arm board's VR1).
  C1/C2 (VR5's bypass caps): C1 (input side) pad1 -> +5V, pad2 -> GND;
    C2 (output side) pad1 -> +3V3, pad2 -> GND.

Unlike the arm board, the hub's own modules (A5 DIP-48, A6, spoke sockets)
are all through-hole (pads span every copper layer) -- only Y1 (TCXO_Can)
is single-sided SMD. No equivalent of the arm board's SEL-pin/NPTH-hole
problem is expected here, but the same verification steps still apply.

Usage (from project root):
  python pcb/route_hub_board.py                 # assign nets + design rules, save
  python pcb/route_hub_board.py --dsn out.dsn    # + export Specctra DSN for Freerouting
  python pcb/route_hub_board.py --ses in.ses     # + import a routed Specctra session back
  python pcb/route_hub_board.py --zones          # + add/fill a GND pour (F.Cu)
"""

import argparse
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_schematic_multi_fpga as SCH  # noqa: E402 -- reuse its pin/net tables directly
import route_arm_board as ARM  # noqa: E402 -- reuse generic net/zone/design-rule helpers

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "multi_fpga")
PCB_PATH = os.path.join(OUTDIR, "hub.kicad_pcb")


def _dip_pin_nets():
    """DIP pad number (1-48) -> net name, for every one of A5's 45 wired
    signals + VU/GND (pads with no entry here are real, unused GPIO)."""
    nets = {}
    for pin_i, (_fpga_pin, dip_num) in enumerate(SCH.CMOD_A7_35T_DIP):
        if pin_i < 32:
            cidx, j = divmod(pin_i, 8)
            net = f"SPOKE{cidx}_{SCH.SPOKE_SIGNAL_SUFFIX[j]}"
        else:
            net = SCH.USB_TCXO_NETS[pin_i - 32]
        nets[dip_num] = net
    nets[24] = "+5V"
    nets[25] = "GND"
    return nets


FT232H_PAD_NETS = {str(i + 1): f"USB_D{i}" for i in range(8)}
FT232H_PAD_NETS.update({
    "9": "USB_RXF_N", "10": "USB_TXE_N", "11": "USB_RD_N", "12": "USB_WR_N",
    "13": "+3V3", "14": "GND",
})

TCXO_PAD_NETS = {"1": "+3V3", "2": "GND", "3": "+3V3", "4": "TCXO_CLK"}

# VR5 (LDO_3V3, MCP1700-3302): pad1=GND, pad2=OUT, pad3=IN -- see LDO_PINS
# in make_schematic_multi_fpga.py (same convention as the arm board's VR1).
VR5_PAD_NETS = {"1": "GND", "2": "+3V3", "3": "+5V"}

# Spoke socket pin position -> index into SPOKE_SIGNAL_SUFFIX (matches the
# arm board header's CMOD_S7_PMOD_JA position convention: 1,2,3,4,7,8,9,10).
SOCKET_POS_TO_SUFFIX_IDX = {1: 0, 2: 1, 3: 2, 4: 3, 7: 4, 8: 5, 9: 6, 10: 7}


def assign_nets(board):
    netinfo = board.GetNetInfo()
    dip_nets = _dip_pin_nets()
    assigned = 0
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref == "A5":
            for pad in fp.Pads():
                name = dip_nets.get(int(pad.GetPadName()))
                if name is None:
                    continue
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref == "A6":
            for pad in fp.Pads():
                name = FT232H_PAD_NETS.get(pad.GetPadName())
                if name is None:
                    continue
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref == "Y1":
            for pad in fp.Pads():
                name = TCXO_PAD_NETS.get(pad.GetPadName())
                if name is None:
                    continue
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref.startswith("J") and ref[1:].isdigit() and fp.GetPadCount() == 12:
            # Spoke sockets J1-J4: positions 1,2,3,4,7,8,9,10 only -- see
            # module docstring for why 5/6/11/12 are left unwired.
            cidx = int(ref[1:]) - 1
            for pad in fp.Pads():
                pos = int(pad.GetPadName())
                if pos not in SOCKET_POS_TO_SUFFIX_IDX:
                    continue
                name = f"SPOKE{cidx}_{SCH.SPOKE_SIGNAL_SUFFIX[SOCKET_POS_TO_SUFFIX_IDX[pos]]}"
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref.startswith("J") and ref[1:].isdigit() and fp.GetPadCount() == 2:
            # Power sockets J5-J8: pin1 -> +5V, pin2 -> GND.
            for pad in fp.Pads():
                name = "+5V" if pad.GetPadName() == "1" else "GND"
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref == "VR5":  # N_CLUSTERS + 1, see make_hub()/place_hub()
            for pad in fp.Pads():
                name = VR5_PAD_NETS.get(pad.GetPadName())
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        elif ref in ("C1", "C2") and fp.GetPadCount() == 2:
            in_net = "+5V" if ref == "C1" else "+3V3"
            for pad in fp.Pads():
                name = in_net if pad.GetPadName() == "1" else "GND"
                pad.SetNet(ARM.get_or_create_net(board, netinfo, name))
                assigned += 1
        # H*B (standoffs), HCAM1/2, HPI1-4: mechanical only, no net.
    return assigned


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsn", help="export a Specctra DSN file for Freerouting after net assignment")
    ap.add_argument("--ses", help="import a routed Specctra SES file back onto the board")
    ap.add_argument("--zones", action="store_true", help="add + fill a GND pour zone (F.Cu)")
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
        zones = ARM.add_power_zones(board)
        board.Save(PCB_PATH)
        print(f"Added + filled {len(zones)} pour zone(s) (GND on F.Cu).")
        return

    assigned = assign_nets(board)
    ARM.set_design_rules(board)
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
