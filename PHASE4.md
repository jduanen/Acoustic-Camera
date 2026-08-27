# Phase 4 — Full Custom Array

96-mic Underbrink-spiral array + single-FPGA front-end (one ALINX AC7200 module, Xilinx
XC7A200T) doing the entire 96-channel CIC/FIR pipeline plus direct GbE/UDP packetization to
the host — no cluster/hub split, no USB bridge. Physically two mating boards: `mic_array`
(the 96-mic disc) and `front_end` (FPGA module, buck regulator, GbE PHY). RTL feasibility is
confirmed on real placed/routed numbers; PCB layout is in progress. The original Multi-FPGA
(Clustered) design (5× Xilinx 7-series FPGAs, USB bridge to a Raspberry Pi 5) is kept as a
proven, already-working alternate — its RTL is fully implemented and verified, and it
synthesizes/places/routes clean on real hardware.
Goal: full-performance acoustic camera meeting system requirements (200 Hz – 8 kHz, ±45° FoV,
~5° resolution @ 1 kHz).

---

## Hardware Design Decisions

### Microphone — Infineon IM72D128

Upgraded from the IM69D120 originally noted in DESIGN.md.

| Spec | IM69D120 (previous) | IM72D128 (chosen) |
|---|---|---|
| SNR | 69 dB(A) | **72 dB(A)** (+3 dB) |
| AOP | 120 dBSPL | **128 dBSPL** (+8 dB) |
| Sensitivity match | ±1 dB | ±1 dB |
| Phase match | ±2° | **±2°** (unchanged — factory calibrated) |
| Ingress protection | — | **IP57** (dust and moisture) |
| Interface | PDM | PDM |
| Approximate unit cost | ~$0.76 | ~$0.90–1.10 |

The phase matching spec is unchanged — critical for coherent array beamforming. The 3 dB SNR gain
improves weak-source detectability (dynamic range). IP57 is useful for outdoor / industrial use.
At 96 units the cost difference is approximately $14 total.

**Part number**: IM72D128V01XTMA1  
**Source**: DigiKey / Mouser / Future Electronics (standard lead times as of May 2026)

No strong competitor exists at this price point with documented ±2° factory phase matching.
TDK, STMicro, and Vesper PDM mics in this range do not publish comparable phase tolerance specs.

---

### FPGA — Single-FPGA (ALINX AC7200 Module)

One Xilinx XC7A200T does the entire 96-channel CIC + FIR pipeline plus direct GbE/UDP
packetization to the host, instead of splitting the front end across 5 smaller FPGAs (the
alternate Multi-FPGA design, described below). This was originally deferred behind two
structural costs: the XC7A200T ships only in a 484-pin BGA (bare-chip prototyping would need
a dev board, and custom-PCB assembly would need BGA rework), and all 48 direct PDM lines
would need routing the full ~300mm array span on one monolithic PCB. Both costs are resolved
by using the **ALINX AC7200** — a real, currently-sold XC7A200T System-on-Module (FPGA + 1GB
DDR3 + QSPI flash + clocks + full power delivery, fully assembled) — as the FPGA subsystem,
carried on a custom carrier board, rather than designing around the bare chip: no BGA
soldering or rework at any build stage, and the module's own 4× 80-pin/0.5mm board-to-board
connectors (195 usable GPIO across FPGA banks 13/14/15/16/34) comfortably cover the 48 PDM
DATA lines + PDM_CLK + RGMII/MDIO + JTAG this design needs.

Real quoted price (Aug 2026, CodeRobin/eBay): ~$299 — only ~$23 over the bare
XC7A200T-1FBG484C chip's own $275.99 DigiKey price for a complete, working, BGA-assembled
module. Compared to the alternate Multi-FPGA design's 5× Digilent Cmod A7-35T modules (~$520
for the FPGAs alone), this comes out roughly **$240/unit cheaper** at the component level,
with the real engineering risk of custom BGA power delivery and prototyping removed entirely.

#### Physical design: two mating boards

Unlike the original single-monolithic-PCB plan, the physical design splits into two boards
that stack together via board-to-board connectors:

- **`mic_array`** — the 96-mic disc itself: all 96 IM72D128 mics + decoupling caps, the
  2-tier PDM clock fan-out tree (see Supporting ICs below), and local power regulation.
- **`front_end`** — the ALINX AC7200 module, a buck pre-regulator (TLV62569DBV) feeding the
  module's own +5V input, and the GbE PHY (see Supporting ICs below).

Both boards are managed as independent KiCad projects (`mic_array.kicad_pro`,
`front_end.kicad_pro`) sharing one common project-local symbol/footprint library
(`pcb/libraries/`). PCB layout is in progress; not yet fabricated.

#### Device comparison

| Device | LUTs | DSP | BRAM | 96-ch headroom | 128-ch headroom | Notes |
|---|---|---|---|---|---|---|
| XC7A35T | 20,800 | 90 | 1.8 Mb | No | No | Too small |
| ECP5-25F | 25,500 | 56 | 1.67 Mb | No | No | Too small |
| ECP5-45F | 44,500 | 90 | 1.93 Mb | Tight (5%) | No | Open toolchain; no 128-ch path |
| XC7A100T | 63,400 | 240 | 4.86 Mb | 35% | 16% | Considered; rejected (tight at 128-ch) |
| **XC7A200T** | **134,600** | **740** | **13.1 Mb** | **~75%** | **~60%** | **Chosen** |

#### Why XC7A200T over XC7A100T

The XC7A100T has 35% LUT headroom for 96 channels but only 16% for 128 channels — tight for
a first build where HDL synthesis estimates are uncertain. The XC7A200T gives 75% headroom
at 96 channels and 60% at 128 channels, with room for future FPGA-side additions (octave-band
parallel beamforming, hardware PSF, etc.).

Its 740 DSP48E1 blocks cover all 96 FIR compensation chains in dedicated DSPs with zero LUT
cost for MAC operations. The XC7A100T's 240 DSPs could handle this too but leaves little
margin for any additional DSP-heavy logic.

Both use the same Artix-7 family: same Vivado flow, same ILA/VIO in-circuit debug tools.
(The GbE MAC is not Xilinx's own TEMAC IP — see below.)

#### RTL feasibility: real placed/routed numbers, not estimates

Confirmed on `xc7a200tfbg484-1` — the exact part the ALINX AC7200 carries — via full
`synth_design → opt_design → place_design → route_design`, never trusting a synth-stage or
naively-extrapolated LUT number (this project has been burned by that twice already, see the
alternate Multi-FPGA design's own "Superseded by real synthesis" note below):

| Stage | Slice LUTs | Utilization | Registers | DSP48E1 | Notes |
|---|---|---|---|---|---|
| Pipeline only (96ch CIC+FIR) | 58,232 / 133,800 | **43.52%** | 13.39% | 96/740 (12.97%) | Route clean, 0 errors |
| + GbE MAC + UDP packetizer | 64,406 / 133,800 | **48.14%** | 22.12% | 96/740 (12.97%) | Route clean, 0 errors |

Comfortable margin either way — the fully-parallel (no CIC-sharing) architecture fits with
real margin to spare. GbE MAC is `alexforencich/verilog-ethernet` (MIT license), not
Xilinx's TEMAC IP core — TEMAC requires a separate license Xilinx doesn't bundle by default,
so the design switched to this open-source RGMII MAC instead, verified to elaborate cleanly
against the real Xilinx IODDR/BUFR primitives. Full protocol (fixed-source/dest UDP,
transmit-only, no ARP/DHCP/ICMP, checksum disabled per RFC 768) is in
`fpga/single_fpga/GBE_FRAMING.md`; end-to-end RTL is bit-exact against a golden model,
including the real multi-tone PDM pipeline feeding the packetizer.

#### Build strategy

No dev board, no bare-chip BGA rework, no deferred-to-rev-2 custom hub PCB — the ALINX AC7200
module *is* the finished FPGA subsystem from day one, carried on the `front_end` carrier
board described above. Real connector part numbers for the module's own 4× 80-pin headers
still need sourcing from ALINX's manual before the carrier board's connector footprints are
finalized.

#### Alternate: Lattice ECP5-45F

Only if a fully open-source toolchain (Yosys + nextpnr, no Vivado) is a hard requirement.
Fits 96 channels with ~5% margin; does not fit 128 channels. GbE SerDes integration is
harder (~4–6 extra weeks). Suitable for a rev-2 board after the HDL is proven on Artix-7.

#### Considered and rejected: Zynq-7020

85,000 LUTs + dual-core ARM Cortex-A9. Potentially interesting for Config A (standalone)
because the ARM could replace the Pi 5, but rejected:
- Cortex-A9 @ 1 GHz is 4–5× slower than Pi 5's Cortex-A76 for NumPy/BLAS
- 96-ch D&S at 3°/pt: ~80–100 ms on Zynq ARM vs ~20 ms on Pi 5 (~5–10 fps vs 15–20 fps)
- 85k LUT fabric gives less headroom than XC7A200T
- PS+PL integration adds toolchain complexity (Vivado + Vitis)
- Pi 5 + XC7A200T is better performance at comparable total cost
- Worth revisiting for a future rev-2 if a single-board integrated design is desired

#### Considered and rejected: 128-mic array

Adding 32 mics (96 → 128, 8 arms × 16) was evaluated:
- Benefits: +1.3 dB array gain, ~1 dB sidelobe improvement — marginal
- Costs: FPGA headroom drops from 60% to ~45% on XC7A200T (still fine, but closer);
  ECP5-45F no longer fits; Pi 5 D&S grows from ~20 ms to ~36 ms at 3°/pt
- HPBW is aperture-limited, not mic-count-limited — adding mics does not change resolution
- Verdict: not worth it for the first board; revisit if Phase 4 data shows sidelobe-limited
  performance in a specific measured scenario

> Note: this tradeoff is specific to this design's fixed LUT budget. The alternate Multi-FPGA
> (Clustered) design (below) scales past 96 mics by adding a 5th cluster (each cluster tile's
> own per-FPGA cost stays fixed at 24ch/71% LUT regardless of cluster count, confirmed by real
> synthesis — see `fpga/README.md`) rather than upsizing one chip — this LUT-headroom squeeze
> is itself a reason to prefer that design if the mic count is likely to grow.

#### Considered and rejected: 350 mm aperture

Increasing aperture from 300 mm to 350 mm was evaluated:
- HPBW improvement: ~14% across all frequencies (e.g., 22° → 19° at 3 kHz) — modest
- Far-field distance increases: 4.2 m → 5.7 m at 8 kHz; at a typical 5 m working range the
  system would be operating in near-field at 8 kHz, introducing systematic DoA error
- Spatial Nyquist drops: scaling 96 mics over a larger aperture increases min spacing from
  ~21 mm to ~24.5 mm, dropping Nyquist from 8.2 kHz to ~7.0 kHz; restoring Nyquist requires
  ~131 mics → rounds to 128
- PCB is 17% larger; harder to mount at array center
- Verdict: the far-field regression outweighs the HPBW gain for the first board; revisit with
  a larger mic count after Phase 4 field data is available

---

### FPGA — Alternate: Multi-FPGA (Clustered)

Splits the front-end across **5 small Xilinx 7-series FPGAs**: 4 "cluster" FPGAs, each
handling one quadrant of the array, plus 1 "hub" FPGA that aggregates their output and
bridges it to a Raspberry Pi 5 over USB (see Host interface below) — the hub never speaks
Ethernet itself.

Kept as a proven, already-working alternate to the primary single-FPGA design (above).
Motivation (historical, i.e. why this was originally the primary design before the AC7200
module made the single-FPGA path practical): a single-FPGA design needs the XC7A200T,
which ships only in a 484-pin FBGA (`XC7A200T-1FBG484C`) — forcing a dev board at prototype
stage and BGA rework at custom-PCB stage — and routes all 48 direct PDM lines the full
~300mm span of the array on one monolithic PCB. Splitting the front end across several much
smaller FPGAs — each fitting a hand-assembly-friendly module — removes that constraint at
both the prototype and custom-PCB stage, and turns one 300mm PCB with 48 matched PDM traces
into several independently buildable, testable, and replaceable tiles.

#### Partition: 4 clusters of 3 arms each

96 mics = 12 arms × 8 mics (the chosen Underbrink spiral, see Array Geometry below). The
12 arms divide evenly into 4 symmetric 90° quadrants of 3 arms (24 mics) each — the natural
mechanical unit for this split.

Grouping was evaluated at several granularities. Two blocks — PDM capture control and the
serial-link framing logic — are largely fixed overhead per FPGA rather than scaling with
channel count, so grouping arms uses silicon more efficiently, not just fewer connectors:

| Grouping | FPGAs | Ch/FPGA | Est. LUT/FPGA | Headroom on XC7S25 (14,600 LUT) |
|---|---|---|---|---|
| 1 arm/FPGA | 12 | 8 | ~2,950 | ~80% unused |
| 2 arms/FPGA | 6 | 16 | ~5,350 | ~63% unused |
| **3 arms/FPGA (chosen)** | **4** | **24** | **~7,750** | **~47% unused** |
| 4 arms/FPGA | 3 | 32 | ~10,150 | ~30% unused |

(Model: CIC ≈ 250 LUT/ch and FIR ≈ 25 LUT/ch — both linear, matching the 96-channel
pipeline estimate above — plus PDM control ≈ 150 + 15/ch and link framing ≈ 400 + 10/ch as
fixed-ish terms. Rough estimates pending real HDL synthesis, same caveat as the 96ch
numbers above.)

4 clusters at 24ch each was chosen over 1-per-arm (12 FPGAs, mostly-idle silicon, 3× the
connectors) and over coarser groupings that start eating into headroom without much
further connector reduction.

> **Superseded by real synthesis**: the table above (and the XC7S25-based reasoning through
> this whole section) reflects pre-HDL, pre-synthesis estimates. Once the CIC/FIR pipeline
> was actually built, the chosen 3-arm/24ch grouping's *real* placed cost came back at
> **14,768 LUT** (see `fpga/multi_fpga/cluster/rtl/cluster_top.v`'s header comment) — 1.9× the ~7,750
> estimated above, driven by control-set packing overhead a pre-place estimate doesn't see.
> That's too big for the XC7S25 (14,600 LUT) this section chose, so the cluster tier moved to
> the same XC7A35T (20,800 LUT) the hub already used — landing on **71.00%** utilization
> there, not the ~47% "headroom" this table projected. A later exploration (this project's
> plan file, "Mark II re-partition") built the 4-arm/32ch row for real too: **19,662 LUT =
> 94.53%** of the XC7A35T, nowhere near this table's ~30%-unused projection — confirming the
> 3-arm/24ch grouping chosen here was the right call, just on a bigger, single-part-number
> chip than planned. See `fpga/README.md` for current numbers on both tiers.

#### Architecture: hub-and-spoke, single shared clock domain

- **4× cluster FPGA** (Xilinx Artix-7 **XC7A35T**, 20,800 LUT — originally planned as a
  Spartan-7 XC7S25, moved after real synthesis, see the note above): PDM clock fan-out to its
  24 local mics, per-cluster CIC decimation + FIR compensation, then frames the decimated
  48kHz PCM onto a parallel single-ended bus back to the hub (see Spoke link below — not
  true LVDS; the Cmod A7-35T's exposed I/O has no differential-capable pins). No GbE MAC, no
  PHY chip, no TCXO on the tile. Fits on the **Digilent Cmod A7-35T module** (~$99) — BGA
  pre-mounted, no hand rework needed even at prototype stage.
- **1× hub FPGA** (Xilinx Artix-7 **XC7A35T**, 20,800 LUT — same part as the clusters, see
  above): holds the single 12.288 MHz TCXO, generates the 3.072 MHz PDM clock and forwards it
  to all 4 clusters over the same spoke links — source-synchronous, one clock domain
  end-to-end (only a fixed, calibrate-once per-spoke cable-length skew, not drift).
  Reassembles the 4 incoming streams (96 channels total) and frames the result out over a
  **USB FIFO bridge to a Raspberry Pi 5** (see Host interface below) — no GbE MAC, no RGMII
  PHY chip, no Ethernet routing on the hub board at all. Available on the **Digilent Cmod
  A7-35T module** (~$99) — same compact 48-pin DIP breadboardable form factor as the cluster
  tiles (see "Why Cmod A7-35T, not Arty A7-35T" below); no on-board Ethernet PHY to leave
  unused, since this module doesn't have one at all.

Star topology, not a daisy-chain: each cluster connects directly to the hub over its own
short link (clusters already sit close to the center), rather than chaining cluster→
cluster→hub, which would add latency and let one broken link take out others downstream
of it.

| | Cluster FPGA (×4) | Hub FPGA (×1) |
|---|---|---|
| CIC decimation (24ch) | ~6,000 LUT | — (moved to clusters) |
| FIR compensation (24ch) | ~600 LUT | — |
| PDM capture/demux | ~500 LUT | — |
| Spoke bus framing/deframing | ~400 LUT | 4× deframer ≈ 1,000-1,200 LUT |
| USB FIFO bridge interface (sync FIFO ctrl + framing) | — (not needed) | ~300-400 LUT |
| Clock gen (PLL from TCXO) | — (receives forwarded clock) | ~500 LUT |
| **Total estimate** | **~7,500 LUT** | **~1,800-2,100 LUT** |

#### Spoke link: parallel single-ended bus, not LVDS

Both the cluster's and the hub's Cmod A7-35T modules route every exposed I/O pin (Pmod and
48-pin DIP header alike) through a 200-240Ω series protection resistor, capped at 25 MHz —
standard practice for a breadboard-friendly module on both boards, but it rules out true
differential LVDS: a series resistor at the connector breaks the controlled 100Ω
differential impedance a real LVDS receiver needs. Digilent only exposes genuine
shunt-configurable/high-speed differential pins on other boards (e.g. the Arty A7's JB/JC);
neither Cmod module has that option on any connector.

Instead, each spoke is an ordinary **parallel single-ended bus**: **6 data bits + 1 strobe +
1 forwarded PDM clock (in) = 8 signals**, one cable per spoke. 27.6 Mbps payload ÷ 6 bits ≈
4.6 MHz per wire — about 5× margin under the 25 MHz cap, comfortable for a short cable at
prototype stage. Cluster-side, each cluster carries its spoke on its own Cmod A7-35T's single
Pmod, JA. Hub-side, all 4 spokes land on the hub's DIP header, not its Pmod —
see "Why all-DIP, no Pmod" below.

#### Host interface: USB bridge to Raspberry Pi 5

This hub always talks to a co-located **Raspberry Pi 5** over USB, and the Pi 5 decides what
to do with the stream (unlike the primary single-FPGA design, whose hub drives GbE directly to a
network switch, received by either host in Configuration A or B — see Host Configurations,
below):

- **Hub → Pi 5**: one **FTDI FT232H** USB-to-FIFO bridge (~$5 chip, ~$15 breakout module),
  wired to the hub FPGA as a synchronous 245-mode 8-bit parallel FIFO (~12 signals: 8 data +
  RXF#/TXE#/RD#/WR#, no SerDes/GTP needed — a small enough pin count to fit on the hub's
  DIP header alongside the spoke links, see Spoke link above). USB 2.0
  Hi-Speed sync-FIFO mode sustains ~320 Mbps, comfortably over the 110 Mbps (96ch × 24-bit ×
  48kHz) payload, with headroom to spare if channel count or bit depth grows later. Talks to
  either of the Pi 5's USB 3.0 ports (USB2-speed device, backward compatible).
- **Standalone**: Pi 5 reads the stream over USB and runs beamforming/display locally
  (Configuration A — see Host Configurations, below).
- **Tethered**: Pi 5 relays the same stream out its own on-board Gigabit Ethernet port to an
  external GPU workstation (Configuration B — see Host Configurations, below). Unlike the
  primary single-FPGA design's direct hub→switch→workstation link, this path is
  hub→USB→Pi 5→GbE→workstation. The Pi 5's native GbE, otherwise idle in standalone mode, is
  reused for this rather than adding a second Ethernet interface.

This removes the hub's GbE MAC + RGMII PHY entirely (see the LUT table above and the
"Why XC7A35T" note below) at the cost of routing every byte through the Pi 5 even in
tethered mode — an extra hop, but a trivial one: 110 Mbps is ~11% of the Pi 5's own GbE
link, and Cortex-A76 UDP relay/forwarding overhead at this rate is not a real bottleneck.
The Pi 5 becomes a mandatory part of the BOM for both configurations, not just standalone.

#### Why XC7A35T for the hub, not XC7S25 like the clusters were originally planned

Historical reasoning (from when the clusters were still planned as XC7S25 — see the
"Superseded by real synthesis" note earlier in this section for what actually happened to
that plan): dropping GbE MAC/PHY from the hub, and replacing its LVDS deserializer with a
simpler parallel-bus deframer (see Spoke link and Host interface, above/below), shrinks its
LUT need well below the earlier GbE-hub estimate — the hub's ~1,800-2,100 LUT would leave
**~86-88% headroom on the same XC7S25** planned for the clusters at the time, an even
stronger case for one single part number (5× identical Cmod S7 modules) than before.

Chosen instead: **XC7A35T for the hub** (~90-91% headroom at this LUT count),
matching the primary single-FPGA design's reasoning for picking the bigger XC7A200T over XC7A100T —
headroom for future FPGA-side additions (octave-band parallel beamforming, hardware PSF
correction) — while still being a small, cheap part relative to the XC7A200T it replaces.
With GbE gone, LUT budget no longer drove this choice in either direction; it was purely a
bet on whether future hub-side additions were worth keeping open. All-XC7S25 was noted here
as a documented lower-cost/single-part-number fallback if that headroom wasn't needed —
**that fallback never happened, but the "one single part number" outcome did anyway**: once
real synthesis forced the clusters off XC7S25 (see above), they landed on this same XC7A35T,
so the hub's choice here turned out to set the part both tiers would eventually share.

#### Why Cmod A7-35T, not Arty A7-35T

The chip choice above (XC7A35T) is independent of which physical board carries it. The
original build used the **Arty A7-35T dev board** (~$130): 4 pluggable Pmods (one per spoke
— no DIP-header wiring needed) plus a shield connector roomy enough for the FT232H bridge,
at the cost of being a full-size dev board unlike the clusters' compact Cmod modules.

Chosen instead: the **Digilent Cmod A7-35T module** (~$99) — same XC7A35T die, but in the
identical 48-pin DIP breadboardable form factor as the cluster tiles, rather than a
different, larger board family for the one-off hub. Digilent's own reference design
(`Cmod-A7-35T-GPIO`) confirms 44 digital DIP GPIO pins across the header, enough on its own
for all 4 spokes (32 signals) + the FT232H bridge (12) + TCXO clock in (1) = 45 signals with
1 pin to spare, once the module's one Pmod is also folded into the DIP-wired scheme — see
"Why all-DIP, no Pmod" below.

The tradeoff: unlike the Arty's 4 identical pluggable Pmods, none of the hub's spoke cables
get a plug-and-play connector — all 4 spokes plus the FT232H breakout are point-to-point
wiring on the DIP header. A real, if modest, bring-up inconvenience relative to the Arty. In
exchange: one part family (Cmod, not Cmod + Arty) across all 5 tiles, a smaller/cheaper
board, and a form factor that fits the same rev-2 compact-tile ambition already planned for
the clusters (see Rev-2, in Hardware Sub-Tasks below) instead of needing a separate path for
the hub.

#### Why all-DIP, no Pmod

Cmod A7-35T does have one real Pmod (JA, 8 signals) — the first version of this hub kept
spoke 0 on it, matching how every cluster's own spoke already uses a pluggable Pmod cable,
and used the DIP header only for spokes 1-3 + FT232H + TCXO (37 of 44 DIP pins).

Moved everything to the DIP header instead, spoke 0 included: mixing connector types (1
Pmod cable + DIP wiring for the rest) is *more* bring-up complexity than a single consistent
scheme, not less — one cable type to keep track of, one set of DIP wiring for the whole
board, rather than "which spoke is the pluggable one" as a special case to remember. The
cost is one additional DIP pin: 4 full spokes + FT232H + TCXO = 45 signals, one more than
the 44 confirmed-digital DIP pins. Filled by DIP pin 16 — documented as an XADC auxiliary
analog input (`vaux12`) rather than plain GPIO, but 7-series aux-analog pins are ordinary
fabric I/O when not driven into analog mode (the same reasoning already applied to unused
analog-capable FMC pins on the primary single-FPGA design's mic-array connector, and this design
has no use for the XADC at all). This specific pin hasn't been confirmed against a
plain-GPIO-mode example from Digilent, unlike the other 44 — flag for verification before
ordering hardware.

#### Why this design satisfies the modularity/cost motivation

- **Clock coherence is preserved, not compromised.** All 96 channels must stay phase-locked
  to one 48kHz word clock for coherent beamforming (see Sample alignment in FPGA
  responsibilities, below). Giving each cluster its own TCXO would reintroduce exactly the
  drift problem TCXOs were chosen to prevent in the first place; keeping the single TCXO on
  the hub and forwarding its clock preserves the existing clock-plan numbers unchanged
  (12.288 MHz → ÷4 → 3.072 MHz PDM → ×4 PLL → 49.152 MHz → ÷1024 → 48.000 kHz WS) — just
  distributed over cable instead of PCB trace.
- **Both tiers are far smaller than the XC7A200T** (134,600 LUT) — under 6% of it either way.
- **PCB routing simplifies at both levels**: each cluster only routes matched PDM traces
  within its own 90° sector (short spans to 3 nearby arms) instead of the full 300mm span,
  and the hub routes 4 spoke links instead of 48 matched PDM lines — likely drops the
  6-layer PCB recommendation for the mic array board.
- **No RGMII PHY chip or GbE MAC on the hub at all** (see Host interface, above) — a USB
  FIFO bridge to the Pi 5 is a far simpler board-level interface than routing RGMII plus an
  external PHY, and sidesteps PHY part selection entirely.

#### Honest tradeoffs

- New engineering work not needed for the primary single-FPGA design: the cluster-to-hub
  synchronous link protocol (clock forwarding, framing, per-spoke cable-skew calibration) has
  to be designed from scratch — it replaces what would otherwise be an internal parallel bus.
- PDM routing within a cluster reaches 3 arms per board, not 1 — a modest but real routing
  exercise, worth a first-pass floorplan sketch during detailed design.
- More BOM line items and connector points than the primary single-FPGA design (5 FPGAs instead of
  1, plus 4 cluster-to-hub cables) — each individually simpler/cheaper and independently
  testable, but more total assembly points to design connector keying/strain-relief for.
- Cost comparison here is directional, not quoted, matching how the primary single-FPGA design
  hedges its own $ figures — confirm with current distributor quotes once package/qty are
  picked.

#### Considered and rejected

- **1 FPGA per arm (12 total)**: each tile would use only ~20% of even the smallest
  practical FPGA — maximizes connector/BOM count for no headroom benefit.
- **2 FPGAs per arm-pair (6 total)**: a reasonable middle ground, superseded by the further
  consolidation to 4×24ch, which also enabled the hub-unification option above.
- **Daisy-chain instead of star topology**: rejected — adds latency, and a single broken
  link takes out all downstream clusters.
- **Per-cluster GbE PHY (no hub, parallel topology)**: rejected — 4× GbE PHY chips + 4×
  TEMAC MAC logic costs strictly more silicon/parts than one shared hub MAC+PHY, for no
  benefit given the array needs one coherent clock domain regardless.
- **Hub drives GbE directly (own RGMII PHY, no Pi 5 relay)**: the first version of this
  design gave the hub its own GbE MAC + PHY, mirroring the primary single-FPGA design's approach
  exactly. Rejected in favor of the USB-to-Pi-5 bridge above: it removes an entire PHY
  part-selection problem and ~3,000 LUT of TEMAC/UDP gateware from the hub, at the cost of
  making the Pi 5 mandatory in every deployment (including tethered/GPU-host mode) rather
  than optional.
- **Lattice iCE40UP5K for the cluster tiles**: considered for its open toolchain
  (Yosys/nextpnr/IceStorm) and lower per-chip cost (~$5-8) — clusters never touch GbE, so
  the ECP5 alternate's GbE-SerDes objection (above) wouldn't apply here either. Rejected:
  its 5,280 LUT ceiling doesn't fit the chosen 24ch/cluster load (~7,750 LUT estimate; even
  16ch would leave near-zero margin), and its few small hard multipliers make DSP-based FIR
  compensation (the same approach used here and in the primary single-FPGA design) unreliable at
  this channel count. Would also reintroduce a second toolchain alongside the hub's Vivado flow.

---

### Supporting ICs

#### GbE PHY

Needed only by the primary single-FPGA design (above) — the alternate Multi-FPGA design's hub
talks USB to a Raspberry Pi 5 instead and has no GbE MAC/PHY at all (see Host interface). The
FPGA implements the MAC layer in fabric (`alexforencich/verilog-ethernet`'s 1G RGMII MAC, MIT
license — not Xilinx's TEMAC IP core, which turned out to need a separate license Xilinx
doesn't bundle by default); a separate PHY chip handles the analog physical layer and
provides the RGMII interface.

**Chosen: Microchip KSZ9031RNX** — 48-pin QFN (no BGA reflow needed, unlike the other
candidate below), real quoted price ~$3.70–5.96, RGMII, widely used in hobbyist designs.
Marvell 88E1111 was considered and rejected: industry-standard and well-documented, but ships
only in a 117-ball BGA package and has real, confirmed sourcing difficulty at low volume —
neither advantage outweighs the extra BGA-assembly risk on a board that otherwise has none.

#### Master Clock — 12.288 MHz TCXO

A temperature-compensated oscillator (TCXO) is required rather than a plain crystal because
sample-rate drift accumulates in the CSM over long captures and corrupts coherence.

The 12.288 MHz frequency divides cleanly to all required audio clocks:

| Derived clock | Division | Value |
|---|---|---|
| PDM clock to mics | ÷ 4 | **3.072 MHz** (exact) |
| FPGA PLL output | × 4 | **49.152 MHz** |
| PCM word select (WS) | ÷ 1024 | **48.000 kHz** (exact) |

Recommended: any 12.288 MHz TCXO in ±2.5 ppm or better (e.g., NDK NZ2520SD, TXC 7M series).

#### PDM Clock Distribution (single-FPGA mic array board)

96 mics is too much fan-out for a bare TCXO output or a single 1:10-class buffer, and driving
it directly would exceed the TCXO's own rated output load. Two-tier real fan-out tree, TI
CDCLVC11xx family throughout (same low-skew family, pin-compatible members):

```
TCXO (12.288 MHz) -> 1x CDCLVC1112 (1:12) -> 12x CDCLVC1108 (1:8, one per mic arm) -> 96 mics
```

- **Tier 0**: a single CDCLVC1112 (1:12 LVCMOS buffer) takes the TCXO output directly — its
  only load, well inside the TCXO's rated drive. One output per arm, so all 12 second-tier
  buffers see the identical upstream edge (no device-to-device skew at this hop at all, since
  there's only one Tier-0 chip).
- **Tier 1**: one CDCLVC1108 (1:8) per arm, driven by its own Tier-0 output, driving that
  arm's 8 mic CLK pins directly (star topology, not daisy-chained).
- **Real skew budget** (from the CDCLVC11xx datasheet, not estimated): same-device output
  skew 50ps max (mics sharing one arm's buffer) vs. part-to-part skew 0.5ns max (mics on
  different arms) — the cross-arm figure dominates by 10x and is a fixed silicon
  characteristic, not fixable by trace-length matching. If beamforming coherence ever needs
  tighter than 0.5ns across arms, the fix is a one-time per-channel calibration measurement,
  not tighter routing.
- 13 buffer ICs total (1 + 12), all one part family — single BOM line, easy sourcing.

#### Power Supply (single-FPGA mic array board)

Real combined 3.3V load, computed from actual datasheet current figures (not estimated):

| Load | Per-unit | Count | Total |
|---|---|---|---|
| IM72D128 mics @ 3.072 MHz PDM clock | 980µA typ / 1120µA max | 96 | 94–108mA |
| CDCLVC1112 + 12x CDCLVC1108 (static, near 3.3V) | ~6mA typ / ~10mA max each | 13 | 78–130mA |
| Clock-tree dynamic (C_PD x VDD x f, ~108 active outputs) | ~61µA/output | — | ~7mA |
| ECS-TXO-5032 TCXO | 6.0mA | 1 | 6mA |
| **Total** | | | **~185mA typ, ~250mA worst-case** |

(AC7200 module power is separate — it takes +5V directly and generates its own onboard rails.)

A single small linear regulator off +5V (originally an MCP1700T-3302E/TT, 250mA max) is not
adequate: the worst-case load sits right at that part's current ceiling with no margin, and
at a 1.7V dropout (5V in, 3.3V out) it would dissipate up to ~425mW in a SOT-23 package — a
real thermal problem, not just a current-margin one. Real fix, buck pre-regulator + split
linear post-regulation:

```
+5V -> TLV62569DBV (buck, adjustable, 2A) -> ~3.6V -> 2x MCP1700T-3302E/TT -> clean 3.3V
                                                        (VR5: mics, VR6: clock tree + TCXO)
```

- Buck output set to ~3.6V (R1=499k / R2=100k feedback divider, VOUT = 0.6V x (1 + R1/R2)) —
  enough headroom for clean LDO dropout without wasting buck efficiency.
- Splitting the two LDOs by load *type* (quiet analog mic rail vs. noisy digital clock-tree
  rail), not by mic arm — per-arm splitting buys nothing once the buck removes the thermal/
  current problem, since every mic already has its own local decoupling cap; digital/analog
  separation is the split that actually reduces correlated-noise risk on a 96-channel
  coherent beamformer.
- Each LDO now sees ≤143mA against a 250mA rating (43–57% of max) with a ~0.3V dropout instead
  of 1.7V — comfortable current and thermal margin on both rails.

---

## Array Geometry

Same as established in Phase 1 simulation (nb05–nb11):

| Parameter | Value |
|---|---|
| Mic count | 96 |
| Pattern | Underbrink multi-arm log-spiral |
| Arms × mics per arm | 12 × 8 (6 × 16 simulated as alternative in Phase 1) |
| Aperture | ~300 mm diameter |
| Min mic spacing | ~13 mm |
| Spatial Nyquist | ~13 kHz (no aliasing within 8 kHz operating range) |
| Far-field distance | 0.52 m @ 1 kHz · 1.6 m @ 3 kHz · 4.2 m @ 8 kHz |

The geometry was optimized in Phase 1 simulation. The Underbrink pattern is preferred over a
regular grid because its logarithmic spiral arm spacing suppresses grating lobes across the full
frequency range, giving better sidelobe performance at low mic count.

---

## Host Configurations

In the primary single-FPGA design, the front-end drives GbE/UDP directly to whichever host is
present — a Pi 5 in standalone mode, or a GPU workstation in tethered mode — with no
intermediate relay required (see Interface, below). Two operating modes share the same
`acoustic_camera_p4.py` script, selected via a `--backend {numpy,cupy}` flag. (The alternate
Multi-FPGA design instead talks USB to a co-located Raspberry Pi 5, which is present in every
deployment and, in tethered mode, relays the stream onward over its own GbE port — see the
blockquote in Interface, below.)

---

### Configuration A — Standalone (Raspberry Pi 5, 8 GB)

Self-contained field unit. Pi is mounted in the camera housing alongside the FPGA hub board.
Runs from a battery. Display via HDMI touchscreen or SSH + web UI.

#### Compute feasibility

The dominant operation is `R @ H` (96×96 CSM times 96×n_grid steering matrix). Scaling from
the Phase 3 UMA-16 benchmark (16-ch D&S at 1°/pt ≈ 7 ms on desktop Linux):

| Config | Grid points | Est. desktop | Est. Pi 5 |
|---|---|---|---|
| 16-ch D&S, 1°/pt (Phase 3 measured) | 5,551 | 7 ms | ~25 ms |
| 96-ch D&S, 1°/pt | 5,551 | ~250 ms | ~900 ms |
| 96-ch D&S, 3°/pt, H pre-computed | 651 | ~6 ms | **~20 ms** |
| 96-ch MVDR, 3°/pt | 651 | ~15 ms | **~50 ms** |
| 96-ch MUSIC, 3°/pt | 651 | ~15 ms | **~50 ms** |

Pi 5 Cortex-A76 @ 2.4 GHz with OpenBLAS is roughly 3–4× slower than a modern desktop CPU on
dense BLAS. **With a pre-computed steering matrix and a 3°/pt grid, D&S runs in ~20 ms —
feasible at 15–20 fps.** MVDR/MUSIC are workable at ~10 fps. CLEAN-SC: ≤ 8 iterations for
real-time; 20 for offline.

The CSM itself (~5 ms for 128 snapshots) is not the bottleneck.

**3°/pt is adequate for live display**: HPBW at 8 kHz ≈ 8° for a 300 mm aperture gives ~2.7
samples/lobe — sufficient for peak localization. Use 0.5°/pt for offline post-processing.

#### Why Pi 5 (not Pi 4 or CM4)

- **Cortex-A76** — ~1.5–2× faster than Pi 4's A72 for NumPy/BLAS
- **PCIe M.2 slot** — upgrade path to Hailo-8 NPU (~$70) for ML beamforming (Phase 5),
  without changing any other hardware
- **MIPI CSI connector** — Pi Camera Module 3 (IMX708, 12 MP) preferred over USB webcam;
  lower CPU overhead, native hardware sync between camera frames and audio timestamps
- **8 GB RAM** — fits steering matrix, CSM buffers, and video pipeline comfortably
- **Native GbE** — receives the front-end's UDP stream directly (see Interface, below); no
  adapter needed
- **USB 3.0** — unused by the primary design in standalone mode; the alternate Multi-FPGA
  design uses it instead, to receive its hub's stream over a synchronous FIFO bridge

#### Camera: Pi Camera Module 3 Wide

The 120° diagonal FoV matches a typical acoustic camera field of view. Mounts at array center.
Accessed via `picamera2` library rather than OpenCV `VideoCapture`.

---

### Configuration B — Tethered, GbE-attached Host with GPU

High-performance workstation or server connected directly to the front-end over a standard
network switch or direct GbE cable (see Interface, below) — no Pi 5 relay needed; the Pi 5 is
not present in this configuration at all for the primary design. Runs full-resolution
beamforming at 20+ fps using a CUDA GPU.

#### Compute feasibility with GPU

CuPy provides near-drop-in NumPy replacement on CUDA. The `R @ H` matrix multiply that costs
~250 ms on CPU (96-ch, 1°/pt) becomes a single cuBLAS ZGEMM call:

| Config | Grid points | CPU (desktop) | GPU (RTX 3060) |
|---|---|---|---|
| 96-ch D&S, 1°/pt | 5,551 | ~250 ms | **~3 ms** |
| 96-ch D&S, 0.5°/pt | 21,901 | ~1,000 ms | **~10 ms** |
| 96-ch MVDR, 1°/pt | 5,551 | ~260 ms | **~5 ms** |
| 96-ch MUSIC, 1°/pt | 5,551 | ~260 ms | **~5 ms** |
| 96-ch CLEAN-SC (20 iter), 1°/pt | 5,551 | ~5,000 ms | **~100 ms** |

All four algorithms at full 0.5°/pt resolution are real-time feasible on GPU.

#### GPU requirements

The steering matrix + CSM + working buffers fit in under 100 MB VRAM at full resolution
(96 × 21,901 × 16 bytes = 33 MB for complex128 steering matrix). Any CUDA-capable GPU works.
Minimum useful: GTX 1070 (8 GB VRAM, ~$100 used). Recommended: RTX 3060 (12 GB).

#### Camera: USB 3.0

Standard USB camera via OpenCV `VideoCapture`. No Pi-specific hardware needed.

---

### Interface: direct GbE, no relay

The FPGA hardware does not change between configurations — only which host is plugged into
the front-end's GbE port:

```
front_end (AC7200 module)  ──GbE/UDP──  [network switch or direct cable]  ──  Pi 5  (Config A)
                                                                            ──  GPU workstation  (Config B)
```

Either host (Pi 5 in Config A; the GPU workstation in Config B) receives the same UDP stream
directly off its own NIC — no relay hop, no intermediate device. A background thread on the
receiving host reads frames, checks sequence numbers, and pushes PCM frames into a thread-safe
deque. The main thread drains the deque to build the sliding audio buffer. Identical code path
for ingestion in both configurations. The Pi 5 is entirely optional in Configuration B — a GPU
workstation can run standalone with no Pi 5 present at all.

> **Alternate Multi-FPGA design**: USB to Pi 5, GbE relay for Configuration B. The FPGA
> hardware does not change between configurations — only what the Pi 5 does with the 110 Mbps
> stream it receives over USB:
> ```
> Cluster FPGAs ──spoke bus──> Hub FPGA ──USB──> Raspberry Pi 5 ──── Config A: compute locally
>                                                               └─GbE─ Config B: relay to GPU workstation
> ```
> In Configuration A, the Pi 5's own beamforming pipeline reads the stream directly off the USB
> FIFO. In Configuration B, a lightweight relay on the Pi 5 forwards the same packets out its
> GbE port unmodified — the GPU workstation's ingestion code is unaware the stream originated
> over USB rather than a NIC. Unlike the primary design, the Pi 5 is mandatory in every
> deployment of this alternate, since it's the only thing the hub ever talks to.

---

### Software Architecture: `--backend {numpy,cupy}`

A single script `src/acoustic_camera_p4.py` selects the compute backend at startup:

```python
if args.backend == 'cupy':
    import cupy as xp
    from cupy.linalg import inv, eigh
else:
    import numpy as xp
    from scipy.linalg import inv, eigh
```

All beamforming functions (`beamform_ds`, `beamform_mvdr`, `beamform_music`) operate on `xp`
arrays transparently. The steering matrix is pre-computed once on the selected device. Grid
resolution defaults: `--grid_deg 3.0` for `--backend numpy` (Pi 5), `--grid_deg 0.5` for
`--backend cupy` (GPU host).

CLEAN-SC is the one exception: its loop structure does not parallelize cleanly across grid
points, so GPU speedup is limited. Implement as CPU fallback even on the GPU host, or use
the Functional Beamforming approximation instead for real-time display.

---

## FPGA / Host Partition

The FPGA does only what a CPU cannot: real-time parallel PDM capture and decimation of 96
channels at 3 MHz. Everything flexible or algorithmically complex stays on the Pi in Python.

### FPGA responsibilities (hard real-time, parallel)

One FPGA (the AC7200 module) does the entire pipeline end-to-end in the primary single-FPGA
design:

| Block | Detail |
|---|---|
| **Master clock generation** | 12.288 MHz TCXO fanned out to the mic array (see PDM Clock Distribution, above) |
| **PDM capture** | 48 data input lines; data latched at each PDM clock edge |
| **L/R demux** | Each data line carries 2 mics (SEL low → even channel, SEL high → odd channel) |
| **CIC decimation** | 5-stage CIC, 64:1 per channel; 3.072 MHz → 48 kHz, all 96 channels fully parallel (no time-sharing) |
| **FIR compensation** | 32-tap linear-phase FIR per channel; corrects CIC passband droop |
| **Sample alignment** | All 96 PCM channels locked to the same 48 kHz word-select boundary |
| **GbE/UDP packetization** | `gbe_packetizer.v`: assemble 5 frames × 96 channels per packet, prepend sequence number + timestamp, hand off to the RGMII MAC → PHY; see `fpga/single_fpga/GBE_FRAMING.md` |

> **Alternate Multi-FPGA design**: split across the cluster and hub tiers instead of one chip —
>
> **Cluster FPGA (×4)**
>
> | Block | Detail |
> |---|---|
> | **PDM clock fan-out** | Forwarded PDM clock (from hub) fanned out to 24 local mics |
> | **PDM capture** | 12 data input lines; data latched at each PDM clock edge |
> | **L/R demux** | Each data line carries 2 mics (SEL low → even channel, SEL high → odd channel) |
> | **CIC decimation** | 5-stage CIC, 64:1 per channel; 3.072 MHz → 48 kHz |
> | **FIR compensation** | ~32-tap linear-phase FIR per channel; corrects CIC passband droop |
> | **Spoke bus framing** | Frame 24 channels' 48 kHz PCM onto the 6-bit parallel spoke bus (see Spoke link) |
>
> **Hub FPGA (×1)**
>
> | Block | Detail |
> |---|---|
> | **Master clock generation** | 12.288 MHz TCXO → PLL → 3.072 MHz; forwarded to all 4 clusters over their spoke links |
> | **Spoke bus deframing** | Reassemble the 4× 24-channel streams into 96 channels total |
> | **Sample alignment** | All 96 PCM channels locked to the same 48 kHz word-select boundary |
> | **USB FIFO framing** | Per-spoke tagged 76-byte records (sync + spoke_id + seq_num + 24 channels), sent independently over the synchronous FIFO to the FT232H bridge -- not a unified 96-channel frame; see `fpga/multi_fpga/USB_FRAMING.md` |
> | **PPS input** (optional) | 1 Hz GPIO for absolute time-tagging; enables future multi-unit synchronization |

### Host responsibilities (flexible, Python)

| Block | Detail |
|---|---|
| **UDP ingestion** | Python socket; check sequence numbers; log packet drops |
| **Audio ring buffer** | Thread-safe deque of incoming PCM frames |
| **Cross-spectral matrix** | Windowed FFT → outer product → running average over N snapshots |
| **Beamforming** | D&S, MVDR, CLEAN-SC, MUSIC (reuse `src/acoustic_camera_p3.py` code as base) |
| **Grid strategy** | 3°/pt for live display; 0.5°/pt for offline post-processing of recordings |
| **NPU acceleration** | Optional Hailo-8 via PCIe M.2 HAT for ML beamforming (Phase 5) |
| **Calibration** | Gain + phase correction vector applied to CSM: `R_cal = outer(1/e, conj(1/e)) * R` |
| **Camera capture** | Pi Camera Module 3 via MIPI CSI (preferred) or USB webcam fallback |
| **Heatmap rendering** | Power → dB → percentile normalize → colormap → resize to frame |
| **Video overlay + display** | `cv2.addWeighted`, crosshair, status label, frequency sliders |
| **Record / playback** | Write raw PCM packets + video frames; replay from file for offline analysis |

---

## Hardware Sub-Tasks

### Primary design — Single-FPGA (ALINX AC7200)

No dev-board stage at all — the ALINX AC7200 module skips it (see Build strategy, above).
Work splits into RTL (done) and the two-board PCB layout (in progress).

| Sub-task | Status | Detail |
|---|---|---|
| **RTL: pipeline** | Done | 96-channel CIC+FIR, fully parallel, placed/routed clean (43.52% LUT) |
| **RTL: GbE/UDP packetizer** | Done | `gbe_packetizer.v` + open-source RGMII MAC, placed/routed clean (48.14% LUT combined); bit-exact against golden model, both unit and end-to-end testbenches passing |
| **Library consolidation** | Done | Shared `pcb/libraries/` (AC7200, AXK connectors, CDCLVC clock buffers, IM72D128, etc.), used by both `mic_array` and `front_end` projects |
| **`mic_array` PCB layout** | In progress | 96× IM72D128 in the Underbrink spiral, PDM clock fan-out tree, local power regulation; footprints linked and synced from schematic, DRC-clean |
| **`front_end` PCB layout** | In progress | AC7200 module footprint + carrier connectors, TLV62569DBV buck pre-regulator, KSZ9031RNX GbE PHY; DRC-clean aside from pre-existing unrelated issues (no board outline defined yet) |
| **Connector part numbers** | Open | Real part numbers for the AC7200 module's 4× 80-pin/0.5mm board-to-board headers still need sourcing from ALINX's manual |
| **PCB fabrication + assembly** | Not started | Gated on layout completion |

### Alternate design — Multi-FPGA (Clustered)

Fully implemented and working on real hardware. Three parallel workstreams that merged at
integration.

#### Workstream 1 — Cluster tiles (Cmod A7-35T dev boards)

Originally planned around Cmod S7 (XC7S25) dev boards — moved to Cmod A7-35T after real
synthesis showed the CIC/FIR pipeline didn't fit the XC7S25 (see the "Superseded by real
synthesis" note earlier in this doc).

| Sub-task | Description | Dependency |
|---|---|---|
| **Procure 4× Cmod A7-35T** | Digilent ~$99 each (~$396 total); XC7A35T, BGA pre-mounted; includes Vivado WebPACK license | None |
| **HDL development** | CIC + FIR + spoke bus framing pipeline in Verilog/VHDL, 24ch; test on one Cmod A7-35T before duplicating to the other 3 | Cmod A7-35T in hand |
| **Spoke bus cabling** | Pmod cable per cluster (8 signals: 6 data + strobe + fwd clock, see Spoke link) — the cluster's Cmod A7-35T's Pmod JA to flying leads on the hub's DIP header, not Pmod-to-Pmod (see "Why all-DIP, no Pmod") | HDL ping-pong test passing |

#### Workstream 2 — Hub (Cmod A7-35T module)

| Sub-task | Description | Dependency |
|---|---|---|
| **Procure Cmod A7-35T** | Digilent ~$99; XC7A35T, 48-pin DIP + 1 Pmod; includes Vivado WebPACK license | None |
| **Procure FT232H module** | FTDI UM232H (~$25); USB 2.0 Hi-Speed sync FIFO bridge to Pi 5 -- exposes `CLKOUT`, which the required synchronous 245 FIFO mode needs (the originally-planned Adafruit #2264 breakout doesn't) | None |
| **Spoke + FT232H wiring** | All 4 spokes + FT232H bridge: point-to-point wiring on the DIP header, no Pmod cables (see "Why all-DIP, no Pmod", above) | Cmod A7-35T in hand |
| **HDL development** | Clock generation/forwarding + spoke deframing/reassembly + USB FIFO framing; test on Cmod A7-35T | Cmod A7-35T in hand |

#### Workstream 3 — Mic array PCB

| Sub-task | Description | Dependency |
|---|---|---|
| **Geometry finalization** | Confirm 8×12 Underbrink spiral from Phase 1 simulation; generate mic XY coordinates | Phase 1 data |
| **PCB design** | 96× IM72D128 in spiral, split into 4 quadrant sections (3 arms/24 mics each) matching the cluster partition; 12.288 MHz TCXO lives on the hub, not the array board; per-cluster PDM clock fan-out with matched traces; DIP-header cable to each cluster's Cmod A7-35T | Geometry final |
| **PCB fabrication** | Likely fewer layers than a monolithic array board — each quadrant only routes matched PDM traces to 3 nearby arms, not the full ~300mm span; confirm during layout | Layout complete |
| **Assembly** | IM72D128 is a small LGA; reflow oven or PCB assembly service | PCB received |

#### Integration & Software

| Sub-task | Description | Dependency |
|---|---|---|
| **First integration** | Connect mic array PCB quadrants to their Cmod A7-35T clusters, clusters to the Cmod A7-35T hub via spoke cables/DIP wiring, hub to Pi 5 via USB; verify all 96 PDM channels on ILA | All 3 workstreams complete |
| **Host software** | USB ingestion (Config A) / GbE relay (Config B) + 96-ch pipeline; extend `acoustic_camera_p3.py` → `acoustic_camera_p4.py` | Hub producing a valid USB stream |
| **Camera** | Pi Camera Module 3 Wide (Config A) or USB webcam (Config B) | Host software running |
| **Calibration** | Gain + phase estimation at 96-ch scale; extend nb17 approach | Hardware assembled |

#### Rev-2 (deferred)

Custom cluster PCBs (bare XC7A35T, one per quadrant) and a custom hub PCB (bare XC7A35T +
FT232H + 12.288 MHz TCXO) designed after the full pipeline is validated on the dev boards.
Eliminates the 5 dev boards and produces compact integrated tiles suitable for the Phase 4b
housing.

---

## PCB Bring-Up Checklist

*Populate as bring-up proceeds.*

- [ ] AC7200 module powers up on `front_end`; JTAG accessible
- [ ] 12.288 MHz TCXO (on `mic_array`) oscillating (verify with scope)
- [ ] PDM clock fanned out and present on the `mic_array` ↔ `front_end` board-to-board
      connector
- [ ] Single mic PDM data line shows valid 1-bit stream on ILA
- [ ] CIC output produces 48 kHz PCM samples (verify with ILA + known tone)
- [ ] FIR compensation: flat frequency response confirmed on single channel
- [ ] All 48 PDM data lines active; 96 channels valid
- [ ] GbE PHY link up (LED); RGMII MAC passes traffic
- [ ] UDP packets received on host, no sequence gaps
- [ ] Host pipeline (standalone): 96-channel CSM computed; beamform produces coherent energy map
- [ ] Host pipeline (tethered): GPU workstation receives the same UDP stream directly, no
      sequence gaps
- [ ] Calibration: cross-correlation gain/phase vectors captured and applied

> **Alternate Multi-FPGA design's checklist**: Hub Artix-7 powers up → JTAG accessible → 12.288
> MHz TCXO (on hub) oscillating → hub PLL locked, 3.072 MHz PDM clock forwarded to each spoke's
> CLK line → cluster 0 Spartan-7 powers up, receives forwarded PDM clock → single mic PDM
> stream valid on ILA → CIC/FIR verified on one channel → all 12 PDM data lines active, 24
> channels valid → spoke bus (cluster 0 ↔ hub) reads correctly on hub ILA → repeat for clusters
> 1–3 → hub reassembles all 4 spokes, 96 channels valid, sample-aligned → USB link up (FT232H
> enumerates on the Pi 5, sync FIFO transfers a known test pattern) → USB packets received on
> Pi 5, no sequence gaps → host pipeline (standalone) produces a coherent energy map → GbE
> relay (tethered): Pi 5 forwards the stream out its own port, GPU workstation receives it with
> no sequence gaps → calibration applied. 5 FPGAs, one bring-up pass per tile.

---

## Key References

- Ben Wang, "192-channel phased array microphone" (2023) — similar FPGA + GbE architecture
- Underbrink multi-arm log-spiral array patent — geometry basis
- Infineon IM72D128 datasheet — mic specs and PDM timing
- ALINX AC7200 user manual — module specs, connector pinouts, GPIO bank mapping (primary
  design's FPGA module)
- `alexforencich/verilog-ethernet` — open-source 1G RGMII GbE MAC (MIT license), used by the
  primary design in place of Xilinx's TEMAC IP core (see GbE PHY / RTL feasibility, above)
- FTDI FT232H datasheet + D2XX/D3XX driver docs — USB sync-FIFO bridge (alternate design's hub)
- Digilent Cmod A7-35T reference manual — connector pinouts, FPGA pin names
  (alternate design's dev-board modules)
- Xilinx TEMAC IP core product guide (PG051) — considered for the primary design's GbE MAC,
  rejected: requires a separate license Xilinx doesn't bundle by default (see RTL feasibility,
  above)
