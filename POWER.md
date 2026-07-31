     Power distribution for the Multi-FPGA (Clustered) design: 5V from Pi 5, local 1.8V/3.3V LDOs, fixing the 
     spoke-connector power bug

     Context

     The user's power plan: an external 5V/4A+ supply feeds the Raspberry Pi 5 over its USB-C
     port; the Pi 5's own +5V rail is then tapped to power the hub board and all 4 arm/cluster
     boards. DESIGN.md's "Power Supply" section is currently just **TBD** — nothing about
     regulators, bypass caps, or rail distribution has been designed yet, even though the
     schematic already references +1V8/+3V3/+5V/GND nets throughout (currently all
     "externally supplied" via PWR_FLAG symbols, with no real regulator anywhere in the
     design — see pcb/multi_fpga/SCHEMATIC_NOTES.md's ERC section).

     A real bug was found while researching this: earlier this session, the spoke Pmod
     connector (CMOD_S7_PMOD_JA) was given 2 extra pins, SPOKE_GND/SPOKE_VU at Pmod
     positions 5/6, wired to the hub's GND/+5V nets — the idea being the hub sources +5V to
     each cluster board over the same connector as the spoke signals. Digilent's actual Cmod S7
     schematic (datasheets/Cmod+S7_sch-public.pdf, page 1 + page 6) shows this is not possible:
     Pmod JA's pins 5/6 (and 11/12) are hard-wired on the Cmod S7 itself to GND/VCC3V3 — its
     own onboard LTC3569 triple-buck regulator's 3.3V output, generated locally from that
     cluster's own VU input. There is no way to inject the hub's +5V into that pin; doing so
     would fight the Cmod's own regulator output. None of Cmod S7's internal rails (1.0V/1.8V/
     3.3V) are exposed anywhere else on the DIP header either — only VU (raw 5V in, pin 24) and
     GND (pin 25).

     This plan both fixes that bug (remove SPOKE_GND/SPOKE_VU, add a dedicated small power
     connector instead) and designs the actual regulator chain the user asked for.

     Power architecture

     - +5V (from Pi 5, externally supplied — stays a PWR_FLAG-satisfied net, no on-schematic
     source modeled, same as today): reaches the hub board directly (feeds A5/Cmod A7-35T's
     own VU pin, unchanged), and reaches each arm board over a new dedicated 2-pin power
     connector mounted next to the existing spoke connector — PinHeader_1x02_P2.54mm_Vertical
     on the arm board (J2), mating PinSocket_1x02_P2.54mm_Vertical on the hub, one per
     cluster (J5-J8, continuing after the existing spoke sockets J1-J4). Pin 1 = +5V,
     pin 2 = GND. This feeds that arm board's own A1/Cmod S7 VU pin (unchanged wiring) and
     the new local LDO below.
     - +1.8V (mic array supply, ~31mA per arm board at 24 mics × ~1.3mA typ. per
     docs/infineon-im72d128-datasheet-en.pdf): one LDO per arm board, local to that
     board's own 24 mics — VR1, Microchip MCP1700-1802 (SOT-23-3, 250mA max, stable with
     1uF ceramic in/out caps per its datasheet's typical application circuit — confirm exact
     pin-to-function mapping against the datasheet before finalizing the schematic symbol,
     same "flag before fab" confidence level this project already uses elsewhere). Input from
     the arm board's own +5V (from J2), output to +1V8.
     - +3.3V (FT232H + TCXO on the hub only, ~50mA total): one LDO on the hub, VR2,
     MCP1700-3302 (same family/package, different fixed output). Input from hub's +5V,
     output to +3V3.
     - Both are simple linear LDOs, not switching bucks — loads are tens of mA, heat is
     negligible, and this avoids inductor placement on boards that are already tight
     (especially the arm board, after this session's mic-footprint/NPTH-clearance fights).

     Net-naming consequence: +1V8 is currently a bare KiCad power symbol, which makes it
     one project-wide net regardless of hierarchy — fine when nothing drives it, but now each of
     the 4 arm boards gets its own independent VR1 output, so 4 regulators can't all share
     one global +1V8 net (that's 4 outputs shorted together). +1V8 needs to become a
     per-cluster-scoped net, the same way PDM_CLK already is (C{idx}_PDM_CLK, via
     make_arm()'s existing clk_label override parameter). +3V3/GND/+5V don't have this
     problem — +3V3 only exists once (hub-only), and GND/+5V are genuinely meant to be one
     global net everywhere.

     Implementation

     1. pcb/make_schematic.py — make_arm() gets a vdd_label override

     make_arm() is shared between the single-FPGA primary design (pcb/make_schematic.py's own
     caller, one shared +1V8 net across all 96 mics on one board — must keep working exactly as
     today) and the multi-FPGA alternate (pcb/make_schematic_multi_fpga.py, needs a distinct
     net per cluster). Add a vdd_label=None parameter alongside the existing clk_label:
     - None (default): keep today's behavior exactly — _pwr("power:+1V8", "+1V8", ...) at
     each mic/cap VDD pin. Single-FPGA design's calls stay unchanged.
     - given (multi-FPGA case, e.g. f"C{cluster_idx}_1V8"): replace those _pwr() calls with a
     wire + _glabel(vdd_label, ..., shape="input") at each VDD pin instead, mirroring exactly
     how DATA_nn/clk_label already wire a global label at a pin (same file, same function,
     a few lines away — reuse that pattern, don't invent a new one).

     2. pcb/make_schematic_multi_fpga.py

     - Remove the bug: delete SPOKE_GND/SPOKE_VU from CMOD_S7_PMOD_JA-related pin lists
     in both _all_lib_symbols() (~line 413) and make_cluster() (~line 484), and delete the
     2 _stub_and_pwr() calls that wired them (~lines 537-540) — back to the original 8-signal
     Pmod, matching physical reality (pins 5/6/11/12 exist on the real connector but this
     design doesn't use them for anything).
     - make_arm() calls in main(): pass vdd_label=f"C{cluster_idx}_1V8" alongside the
     existing clk_label argument.
     - New per-cluster power connector + LDO in make_cluster(): add J2
     (PinHeader_1x02_P2.54mm_Vertical, pins +5V/GND) and VR1 (MCP1700-1802) + 2 bypass
     caps (C25/C26, reusing the same C_0603_1608Metric-backed symbol already used for mic
     caps, values 1uF) wired: J2 pin1 → +5V (same stub-and-pwr pattern already used for
     A1's VU/GND), VR1 IN → +5V, GND → GND, OUT → C{idx}_1V8 (global label, not a
     power symbol — see above).
     - New hub-side power sockets + LDO in make_hub(): J5-J8
     (PinSocket_1x02_P2.54mm_Vertical, one per cluster, pin1 → +5V, pin2 → GND), and VR2
     (MCP1700-3302) + 2 bypass caps (C1/C2) wired +5V → IN, GND → GND, OUT → +3V3.
     - Remove now-redundant PWR_FLAGs: +1V8's flag
     (_inject_pwr_flag_onto_net(content, arm_uuid, "power:+1V8") in main(), ~line 797) and
     +3V3's entry in make_hub()'s _pwr_flag_pair loop (~line 677) — both nets now have a
     real on-schematic driver (VR1/VR2's output pin), so ERC's "power input not driven"
     rule is satisfied naturally; keep GND/+5V's flags (still genuinely externally supplied
     from the Pi 5, no on-schematic source).

     3. pcb/layout_multi_fpga.py

     - New footprints per arm board (built once at c=0, same as everything else on that board):
     VR1 (Package_TO_SOT_SMD.pretty/SOT-23), 2 caps (reuse CAP_FP_LIB/CAP_FP_NAME), J2
     (Connector_PinHeader_2.54mm.pretty/PinHeader_1x02_P2.54mm_Vertical) — placed in open
     space on the wedge board (visually confirmed there's room in the lower/outer region from
     this session's earlier renders), found via the same "compute obstacle AABBs, search for a
     clear spot" approach already used for the hub's axis-aligned module placement, verified
     with the same courtyard-corner/SAT collision check used throughout this file.
     - New footprints on the hub: VR2 + 2 caps, and 4× J5-J8
     (Connector_PinSocket_2.54mm.pretty/PinSocket_1x02_P2.54mm_Vertical, one near each
     existing spoke socket J1-J4) — hub has more open area, should be straightforward.
     - Reuse add_mounting_hole-style load_fp() calls and the existing verification pattern
     (courtyard SAT overlap check, containment-in-outline check) — nothing structurally new
     needed in this file beyond more calls to what's already there.

     4. Re-route both boards

     Both arm_board.kicad_pcb and hub.kicad_pcb were fully routed (with the arm board's
     GND pour + manual SEL-pin stitching, and the hub's GND pour) earlier this session. Adding
     new footprints/nets means redoing net assignment + Freerouting + pours for both, using the
     already-working pipelines: pcb/route_arm_board.py (re-run net assignment, including new
     nets for J2/VR1/caps and the renamed per-cluster C0_1V8; re-run the SEL-pin manual fix
     since it's independent of this change; re-add the GND pour) and pcb/route_hub_board.py
     (same idea, plus J5-J8/VR2/caps). Important: don't re-run
     pcb/layout_multi_fpga.py's main() casually once both boards have manual/generated
     routing state again — it regenerates both files from scratch and wipes routing (this
     happened once already this session; recover via git checkout from the user's last commit
     if it happens again).

     5. Documentation

     - pcb/multi_fpga/SCHEMATIC_NOTES.md: document the Pmod-VCC3V3 bug and fix, the new power
     architecture (rail-by-rail, matching the "Power architecture" section above), part numbers
     with the same confidence-flagging convention already used in this file (e.g. Cmod pin 16,
     Pi Camera lens diameter) — flag MCP1700's exact SOT-23 pin-to-function mapping and the
     mic/Cmod S7 current-draw estimates as needing datasheet confirmation before fab.
     Update the ERC section's PWR_FLAG list (remove +1V8/+3V3, note why).
     - DESIGN.md: replace "Power Supply: TBD" with a short summary of the above (source →
     hub/arm distribution → per-board LDOs), pointing at the schematic notes for detail.

     Verification

     - kicad-cli sch erc pcb/multi_fpga/top.kicad_sch --format json — still 0 error-severity
     violations (the established check throughout this session).
     - Footprint/net counts sanity-checked the same way route_arm_board.py/route_hub_board.py
     already print them (e.g. arm board: was 53 footprints, now +4 (VR1+2 caps+J2); hub: was
     17, now +4 for VR2+2 caps, +4 for J5-J8 = +8... adjust expected counts in both scripts).
     - Re-run the full routing pipeline on both boards (net assign → Freerouting → GND pour →
     arm board's SEL-pin fix) and confirm via kicad-cli pcb drc that violation counts don't
     regress beyond what's already accepted/documented (the pre-existing footprint-tightness
     baseline established this session) — 0 new unconnected items expected.
     - Visual SVG render of both boards (established pattern) to confirm the new parts land in
     open space with no overlaps.
     - Sanity-check the current budget: ~125mA (96 mics) + Cmod S7/A7-35T board draw (flag as
     needing Digilent's published figures, not directly in the schematic PDF) + FT232H/TCXO
     (~50mA) comfortably inside the user's stated 4A+ 5V supply once the Pi 5's own consumption
     is accounted for — call out as an estimate, not a hard guarantee, consistent with this
     project's existing confidence-flagging style.
