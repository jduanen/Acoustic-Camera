# Phase 4 — Full Custom Array

96-mic Underbrink-spiral PCB + Multi-FPGA (Clustered) front-end (4× Spartan-7 cluster FPGAs +
1× Artix-7 hub) + USB bridge to a Raspberry Pi 5 (standalone, or relaying to a GbE-attached GPU
workstation when tethered). A single-FPGA (XC7A200T) design is kept as a documented alternate.
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

### FPGA — Multi-FPGA (Clustered)

Splits the front-end across **5 small Xilinx 7-series FPGAs**: 4 "cluster" FPGAs, each
handling one quadrant of the array, plus 1 "hub" FPGA that aggregates their output and
bridges it to a Raspberry Pi 5 over USB (see Host interface below) — the hub never speaks
Ethernet itself.

Motivation: a single-FPGA design (documented below as an alternate) needs the XC7A200T,
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

#### Architecture: hub-and-spoke, single shared clock domain

- **4× cluster FPGA** (Xilinx Spartan-7 **XC7S25**, 14,600 LUT): PDM clock fan-out to its
  24 local mics, per-cluster CIC decimation + FIR compensation, then frames the decimated
  48kHz PCM onto a parallel single-ended bus back to the hub (see Spoke link below — not
  true LVDS; the Cmod S7's exposed I/O has no differential-capable pins). No GbE MAC, no PHY
  chip, no TCXO on the tile. Fits on the **Digilent Cmod S7 module** (~$45) — BGA
  pre-mounted, no hand rework needed even at prototype stage.
- **1× hub FPGA** (Xilinx Artix-7 **XC7A35T**, 20,800 LUT): holds the single 12.288 MHz
  TCXO, generates the 3.072 MHz PDM clock and forwards it to all 4 clusters over the same
  spoke links — source-synchronous, one clock domain end-to-end (only a fixed,
  calibrate-once per-spoke cable-length skew, not drift). Reassembles the 4 incoming streams
  (96 channels total) and frames the result out over a **USB FIFO bridge to a
  Raspberry Pi 5** (see Host interface below) — no GbE MAC, no RGMII PHY chip, no Ethernet
  routing on the hub board at all. Available on the **Digilent Cmod A7-35T module** (~$99) —
  same compact 48-pin DIP breadboardable form factor as the cluster tiles' Cmod S7 (see "Why
  Cmod A7-35T, not Arty A7-35T" below); no on-board Ethernet PHY to leave unused, since this
  module doesn't have one at all.

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

Both the cluster's Cmod S7 and the hub's Cmod A7-35T route every exposed I/O pin (Pmod and
48-pin DIP header alike) through a 200-240Ω series protection resistor, capped at 25 MHz —
standard practice for a breadboard-friendly module on both boards, but it rules out true
differential LVDS: a series resistor at the connector breaks the controlled 100Ω
differential impedance a real LVDS receiver needs. Digilent only exposes genuine
shunt-configurable/high-speed differential pins on other boards (e.g. the Arty A7's JB/JC);
neither Cmod module has that option on any connector.

Instead, each spoke is an ordinary **parallel single-ended bus**: **6 data bits + 1 strobe +
1 forwarded PDM clock (in) = 8 signals**, one cable per spoke. 27.6 Mbps payload ÷ 6 bits ≈
4.6 MHz per wire — about 5× margin under the 25 MHz cap, comfortable for a short cable at
prototype stage. Cluster-side, all 4 clusters carry their spoke on the same connector (Cmod
S7's single Pmod, JA). Hub-side, all 4 spokes land on the hub's DIP header, not its Pmod —
see "Why all-DIP, no Pmod" below.

#### Host interface: USB bridge to Raspberry Pi 5

This hub always talks to a co-located **Raspberry Pi 5** over USB, and the Pi 5 decides what
to do with the stream (unlike the single-FPGA alternate, whose hub drives GbE directly to a
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
  single-FPGA alternate's direct hub→switch→workstation link, this path is
  hub→USB→Pi 5→GbE→workstation. The Pi 5's native GbE, otherwise idle in standalone mode, is
  reused for this rather than adding a second Ethernet interface.

This removes the hub's GbE MAC + RGMII PHY entirely (see the LUT table above and the
"Why XC7A35T" note below) at the cost of routing every byte through the Pi 5 even in
tethered mode — an extra hop, but a trivial one: 110 Mbps is ~11% of the Pi 5's own GbE
link, and Cortex-A76 UDP relay/forwarding overhead at this rate is not a real bottleneck.
The Pi 5 becomes a mandatory part of the BOM for both configurations, not just standalone.

#### Why XC7A35T for the hub, not XC7S25 like the clusters

Dropping GbE MAC/PHY from the hub, and replacing its LVDS deserializer with a simpler
parallel-bus deframer (see Spoke link and Host interface, above/below), shrinks its LUT
need well below the earlier GbE-hub estimate — the hub's ~1,800-2,100 LUT would leave
**~86-88% headroom on the same XC7S25** used for the clusters, an even stronger case for
one single part number (5× identical Cmod S7 modules) than before.

Chosen instead: **XC7A35T for the hub** (~90-91% headroom at this LUT count),
matching the single-FPGA alternate's reasoning for picking the bigger XC7A200T over XC7A100T —
headroom for future FPGA-side additions (octave-band parallel beamforming, hardware PSF
correction) — while still being a small, cheap part relative to the XC7A200T it replaces.
With GbE gone, LUT budget no longer drives this choice at all in either direction; it's
purely a bet on whether future hub-side additions are worth keeping open. All-XC7S25
remains a documented lower-cost/single-part-number fallback if that headroom isn't needed.

#### Why Cmod A7-35T, not Arty A7-35T

The chip choice above (XC7A35T) is independent of which physical board carries it. The
original build used the **Arty A7-35T dev board** (~$130): 4 pluggable Pmods (one per spoke
— no DIP-header wiring needed) plus a shield connector roomy enough for the FT232H bridge,
at the cost of being a full-size dev board unlike the clusters' compact Cmod S7 modules.

Chosen instead: the **Digilent Cmod A7-35T module** (~$99) — same XC7A35T die, but in the
identical 48-pin DIP breadboardable form factor as the cluster tiles' Cmod S7, rather than a
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
analog-capable FMC pins on the single-FPGA alternate's mic-array connector, and this design
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

- New engineering work not needed for the single-FPGA alternate: the cluster-to-hub
  synchronous link protocol (clock forwarding, framing, per-spoke cable-skew calibration) has
  to be designed from scratch — it replaces what would otherwise be an internal parallel bus.
- PDM routing within a cluster reaches 3 arms per board, not 1 — a modest but real routing
  exercise, worth a first-pass floorplan sketch during detailed design.
- More BOM line items and connector points than the single-FPGA alternate (5 FPGAs instead of
  1, plus 4 cluster-to-hub cables) — each individually simpler/cheaper and independently
  testable, but more total assembly points to design connector keying/strain-relief for.
- Cost comparison here is directional, not quoted, matching how the single-FPGA alternate
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
  design gave the hub its own GbE MAC + PHY, mirroring the single-FPGA alternate's approach
  exactly. Rejected in favor of the USB-to-Pi-5 bridge above: it removes an entire PHY
  part-selection problem and ~3,000 LUT of TEMAC/UDP gateware from the hub, at the cost of
  making the Pi 5 mandatory in every deployment (including tethered/GPU-host mode) rather
  than optional.
- **Lattice iCE40UP5K for the cluster tiles**: considered for its open toolchain
  (Yosys/nextpnr/IceStorm) and lower per-chip cost (~$5-8) — clusters never touch GbE, so
  the ECP5 alternate's GbE-SerDes objection (above) wouldn't apply here either. Rejected:
  its 5,280 LUT ceiling doesn't fit the chosen 24ch/cluster load (~7,750 LUT estimate; even
  16ch would leave near-zero margin), and its few small hard multipliers make DSP-based FIR
  compensation (the same approach used here and in the single-FPGA alternate) unreliable at
  this channel count. Would also reintroduce a second toolchain alongside the hub's Vivado flow.

---

### FPGA — Alternate: Single-FPGA (XC7A200T)

The primary design (above) splits the front-end across 5 small FPGAs specifically to avoid
this design's two structural costs: the XC7A200T ships only in a 484-pin BGA (forcing a dev
board at prototype stage, and BGA rework at custom-PCB stage), and every one of its 48 direct
PDM lines has to be routed the full ~300mm span of the array on one monolithic PCB. This
single-chip design remains documented here as the simpler alternative if those two costs are
judged acceptable — one FPGA, one PCB, no cluster-to-hub link protocol to design, and a
direct GbE connection to the host rather than a USB bridge through a Raspberry Pi 5.

#### Pipeline resource estimate

| Block | LUT estimate |
|---|---|
| CIC decimation, 96 ch × ~250 LUTs | ~24,000 |
| FIR compensation, 96 ch (DSP48, minimal LUTs) | ~2,400 |
| GbE MAC + UDP stack | ~3,000 |
| PDM capture, L/R demux, control | ~2,000 |
| **Total** | **~31,400** (with DSP-based FIR) |

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

Both use the same Artix-7 family: same Vivado flow, same TEMAC GbE IP, same ILA/VIO
in-circuit debug tools. The price difference is ~$90–120 at single-unit quantities.

**Part number**: XC7A200T-1FBG484C (484-pin FBGA)

**Build strategy**: Use a **Nexys Video dev board** (~$500, Digilent) as the FPGA
hub. The Nexys Video uses the same XC7A200T and adds an FMC LPC connector exposing 68 I/O —
enough for all 48 PDM DATA lines + 1 CLK. The mic array is a separate custom PCB that
connects via an FMC LPC cable. No BGA soldering required until rev-2.

> **Why not Nexys A7-200T ($350)?**  The Nexys A7-200T only exposes 32 I/O pins (4× Pmod) —
> 17 short of the 49 needed for the full 96-channel array. The Nexys Video adds FMC LPC
> (68 I/O) for $25 less, making it the better choice for this application.

The custom FPGA hub PCB (bare XC7A200T) remains deferred to rev-2 until the full pipeline
is validated end-to-end on the dev board.

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

> Note: this tradeoff is specific to this single-FPGA alternate's fixed LUT budget. The
> primary Multi-FPGA (Clustered) design (above) scales past 96 mics by adding a 5th cluster
> (still comfortably-sized XC7S25 tiles) rather than upsizing one chip — this LUT-headroom
> squeeze is itself a reason to prefer the primary design if the mic count is likely to grow.

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

### Supporting ICs

#### GbE PHY

Needed only by the single-FPGA alternate (above) — the primary Multi-FPGA design's hub talks
USB to a Raspberry Pi 5 instead and has no GbE MAC/PHY at all (see Host interface). The FPGA
implements the MAC layer (via TEMAC IP); a separate PHY chip handles the analog physical
layer and provides the RGMII interface.

Recommended options (both well-supported in Xilinx reference designs):
- **Marvell 88E1111** — industry standard, extensive documentation, RGMII
- **Microchip KSZ9031RNX** — lower cost, also RGMII, widely used in hobbyist designs

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

In the primary Multi-FPGA design, the hub only ever talks to a co-located Raspberry Pi 5 over
USB (see Host interface, above) — the Pi 5 is present in every deployment, not optional. Two
operating modes share that hardware and the same `acoustic_camera_p4.py` script, selected via
a `--backend {numpy,cupy}` flag. (The single-FPGA alternate instead drives GbE directly from
the hub to either host, with no Pi 5 relay required — see Interface, below.)

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
- **USB 3.0** — receives the hub's stream directly over a synchronous FIFO bridge (see Host
  interface, above); no adapter
- **Native GbE** — unused in standalone mode; relays the stream to an external host in
  Configuration B instead of receiving it (see Interface, below)

#### Camera: Pi Camera Module 3 Wide

The 120° diagonal FoV matches a typical acoustic camera field of view. Mounts at array center.
Accessed via `picamera2` library rather than OpenCV `VideoCapture`.

---

### Configuration B — Tethered, GbE-attached Host with GPU

High-performance workstation or server connected via a standard network switch or direct GbE
cable — not to the FPGA hub directly, but to the Raspberry Pi 5's on-board GbE port. The Pi 5
relays the USB stream it receives from the hub straight out that port (see Host interface,
above, and Interface, below); the Pi 5 is present in this configuration too, just relaying
rather than computing. Runs full-resolution beamforming at 20+ fps using a CUDA GPU.

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

### Interface: USB to Pi 5, GbE relay for Configuration B

The FPGA hardware does not change between configurations — only what the Pi 5 does with the
110 Mbps stream it receives over USB:

```
Cluster FPGAs ──spoke bus──> Hub FPGA ──USB──> Raspberry Pi 5 ──── Config A: compute locally
                                                              └─GbE─ Config B: relay to GPU workstation
```

In Configuration A, the Pi 5's own beamforming pipeline reads the stream directly off the USB
FIFO. In Configuration B, a lightweight relay on the Pi 5 forwards the same packets out its
GbE port unmodified — the GPU workstation's ingestion code is unaware the stream originated
over USB rather than a NIC.

On the receiving host (Pi 5 in Config A; the GPU workstation in Config B), a background thread
receives frames, checks sequence numbers, and pushes PCM frames into a thread-safe deque. The
main thread drains the deque to build the sliding audio buffer. Identical code path for
ingestion in both configurations.

> **Single-FPGA alternate**: no Pi 5 relay — the hub drives GbE directly to a network switch,
> and either host (Pi 5 or GPU workstation) receives the same UDP stream directly:
> ```
> FPGA hub  ──GbE──  [network switch or direct cable]  ──  Pi 5  (Config A)
>                                                       ──  GPU workstation  (Config B)
> ```
> The Pi 5 is optional in that design's Configuration B (a GPU workstation can run standalone
> with no Pi 5 present at all), unlike the primary design where it's mandatory in every
> deployment.

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

Split across the cluster and hub tiers in the primary Multi-FPGA design:

#### Cluster FPGA (×4)

| Block | Detail |
|---|---|
| **PDM clock fan-out** | Forwarded PDM clock (from hub) fanned out to 24 local mics |
| **PDM capture** | 12 data input lines; data latched at each PDM clock edge |
| **L/R demux** | Each data line carries 2 mics (SEL low → even channel, SEL high → odd channel) |
| **CIC decimation** | 5-stage CIC, 64:1 per channel; 3.072 MHz → 48 kHz |
| **FIR compensation** | ~32-tap linear-phase FIR per channel; corrects CIC passband droop |
| **Spoke bus framing** | Frame 24 channels' 48 kHz PCM onto the 6-bit parallel spoke bus (see Spoke link) |

#### Hub FPGA (×1)

| Block | Detail |
|---|---|
| **Master clock generation** | 12.288 MHz TCXO → PLL → 3.072 MHz; forwarded to all 4 clusters over their spoke links |
| **Spoke bus deframing** | Reassemble the 4× 24-channel streams into 96 channels total |
| **Sample alignment** | All 96 PCM channels locked to the same 48 kHz word-select boundary |
| **USB FIFO framing** | Assemble N frames × 96 channels; prepend sequence number + timestamp; send over synchronous FIFO to the FT232H bridge |
| **PPS input** (optional) | 1 Hz GPIO for absolute time-tagging; enables future multi-unit synchronization |

> **Single-FPGA alternate**: one FPGA does the entire pipeline above end-to-end — PDM clock
> generation, 48-line PDM capture, L/R demux, CIC/FIR, sample alignment, and GbE/UDP
> packetization (assemble N frames × 96 channels, prepend sequence number + timestamp, send
> over RGMII to PHY) — rather than splitting capture/decimation (clusters) from
> aggregation/host-bridging (hub).

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

Phase 4 is split into three parallel workstreams that merge at integration.

### Workstream 1 — Cluster tiles (Cmod S7 dev boards)

| Sub-task | Description | Dependency |
|---|---|---|
| **Procure 4× Cmod S7** | Digilent ~$45 each (~$180 total); XC7S25, BGA pre-mounted; includes Vivado WebPACK license | None |
| **HDL development** | CIC + FIR + spoke bus framing pipeline in Verilog/VHDL, 24ch; test on one Cmod S7 before duplicating to the other 3 | Cmod S7 in hand |
| **Spoke bus cabling** | Pmod cable per cluster (8 signals: 6 data + strobe + fwd clock, see Spoke link) — Cmod S7's Pmod JA to flying leads on the hub's DIP header, not Pmod-to-Pmod (see "Why all-DIP, no Pmod") | HDL ping-pong test passing |

### Workstream 2 — Hub (Cmod A7-35T module)

| Sub-task | Description | Dependency |
|---|---|---|
| **Procure Cmod A7-35T** | Digilent ~$99; XC7A35T, 48-pin DIP + 1 Pmod; includes Vivado WebPACK license | None |
| **Procure FT232H breakout** | ~$15 (e.g. Adafruit #2264); USB 2.0 Hi-Speed sync FIFO bridge to Pi 5 | None |
| **Spoke + FT232H wiring** | All 4 spokes + FT232H bridge: point-to-point wiring on the DIP header, no Pmod cables (see "Why all-DIP, no Pmod", above) | Cmod A7-35T in hand |
| **HDL development** | Clock generation/forwarding + spoke deframing/reassembly + USB FIFO framing; test on Cmod A7-35T | Cmod A7-35T in hand |

### Workstream 3 — Mic array PCB

| Sub-task | Description | Dependency |
|---|---|---|
| **Geometry finalization** | Confirm 8×12 Underbrink spiral from Phase 1 simulation; generate mic XY coordinates | Phase 1 data |
| **PCB design** | 96× IM72D128 in spiral, split into 4 quadrant sections (3 arms/24 mics each) matching the cluster partition; 12.288 MHz TCXO lives on the hub, not the array board; per-cluster PDM clock fan-out with matched traces; DIP-header cable to each cluster's Cmod S7 | Geometry final |
| **PCB fabrication** | Likely fewer layers than the single-FPGA design's 6-layer recommendation — each quadrant only routes matched PDM traces to 3 nearby arms, not the full 300mm span; confirm during layout | Layout complete |
| **Assembly** | IM72D128 is a small LGA; reflow oven or PCB assembly service | PCB received |

### Integration & Software

| Sub-task | Description | Dependency |
|---|---|---|
| **First integration** | Connect mic array PCB quadrants to their Cmod S7 clusters, clusters to the Cmod A7-35T hub via spoke cables/DIP wiring, hub to Pi 5 via USB; verify all 96 PDM channels on ILA | All 3 workstreams complete |
| **Host software** | USB ingestion (Config A) / GbE relay (Config B) + 96-ch pipeline; extend `acoustic_camera_p3.py` → `acoustic_camera_p4.py` | Hub producing a valid USB stream |
| **Camera** | Pi Camera Module 3 Wide (Config A) or USB webcam (Config B) | Host software running |
| **Calibration** | Gain + phase estimation at 96-ch scale; extend nb17 approach | Hardware assembled |

### Rev-2 (deferred)

Custom cluster PCBs (bare XC7S25, one per quadrant) and a custom hub PCB (bare XC7A35T +
FT232H + 12.288 MHz TCXO) designed after the full pipeline is validated on the dev boards.
Eliminates the 5 dev boards and produces compact integrated tiles suitable for the Phase 4b
housing.

> **Single-FPGA alternate's workstream**: a single Nexys Video dev board (~$500, XC7A200T +
> FMC LPC) replaces Workstreams 1 and 2 above; the mic array PCB is one monolithic board with
> a 48-line ribbon cable to the Nexys's FMC breakout instead of 4 quadrant sections; rev-2 is
> a single custom hub PCB (bare XC7A200T + 88E1111 PHY + TCXO) instead of 5 tiles.

---

## PCB Bring-Up Checklist

*Populate as bring-up proceeds.*

- [ ] Hub Artix-7 powers up; JTAG accessible
- [ ] 12.288 MHz TCXO (on hub) oscillating (verify with scope)
- [ ] Hub PLL locked; 3.072 MHz PDM clock forwarded to each spoke's CLK line
- [ ] Cluster 0 Spartan-7 powers up; JTAG accessible; receives forwarded PDM clock
- [ ] Single mic (cluster 0) connected; PDM data line shows valid 1-bit stream on ILA
- [ ] Cluster 0 CIC output produces 48 kHz PCM samples (verify with ILA + known tone)
- [ ] Cluster 0 FIR compensation: flat frequency response confirmed on single channel
- [ ] Cluster 0: all 12 PDM data lines active; 24 channels valid
- [ ] Spoke bus (cluster 0 ↔ hub): 6 data bits + strobe read correctly on hub ILA
- [ ] Repeat cluster power-up/PDM/CIC/FIR/spoke checks for clusters 1, 2, 3
- [ ] Hub reassembles all 4 spokes; 96 channels valid, sample-aligned
- [ ] USB link up: FT232H enumerates on the Pi 5; sync FIFO transfers a known test pattern
- [ ] USB packets received on Pi 5; no sequence gaps
- [ ] Host pipeline (standalone): 96-channel CSM computed; beamform produces coherent energy map
- [ ] GbE relay (tethered): Pi 5 forwards the stream out its own port; GPU workstation receives
      it with no sequence gaps
- [ ] Calibration: cross-correlation gain/phase vectors captured and applied

> **Single-FPGA alternate's checklist**: FPGA powers up → TCXO oscillating → PLL locked, PDM
> clock present on header pin → single mic PDM stream valid on ILA → CIC/FIR verified on one
> channel → all 48 data lines active, 96 channels valid → GbE PHY link up (LED) → UDP packets
> received on host, no sequence gaps → host pipeline produces a coherent energy map →
> calibration applied. No spoke bus, USB, or per-cluster steps — one FPGA, one bring-up pass.

---

## Key References

- Ben Wang, "192-channel phased array microphone" (2023) — similar FPGA + GbE architecture
- Underbrink multi-arm log-spiral array patent — geometry basis
- Infineon IM72D128 datasheet — mic specs and PDM timing
- FTDI FT232H datasheet + D2XX/D3XX driver docs — USB sync-FIFO bridge (primary design's hub)
- Digilent Cmod S7 / Cmod A7-35T reference manuals — connector pinouts, FPGA pin names
  (primary design's dev-board modules)
- Xilinx TEMAC IP core product guide (PG051) — GbE MAC integration (single-FPGA alternate)
- `alexforencich/verilog-ethernet` — open-source GbE MAC, alternative to TEMAC (single-FPGA
  alternate)
