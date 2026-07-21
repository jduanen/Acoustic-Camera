# Phase 4 — Full Custom Array

96-mic Underbrink-spiral PCB + Artix-7 XC7A200T FPGA front-end + GbE to host (Pi 5 standalone
or GPU workstation). Goal: full-performance acoustic camera meeting system requirements
(200 Hz – 8 kHz, ±45° FoV, ~5° resolution @ 1 kHz).

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

### FPGA — Xilinx Artix-7 XC7A200T

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

**Phase 4 build strategy**: Use a **Nexys Video dev board** (~$500, Digilent) as the FPGA
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

> Note: this tradeoff is specific to the single-FPGA design's fixed LUT budget. Under the
> Multi-FPGA (Clustered) Alternative below, scaling past 96 mics means adding a 5th cluster
> (still comfortably-sized XC7S25 tiles) rather than upsizing one chip — worth revisiting
> alongside this section if a clustered build is pursued.

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

### FPGA — Multi-FPGA (Clustered) Alternative

The primary design above puts all 96 channels through one XC7A200T. An alternative —
preferred for a build prioritizing modularity and per-unit cost over a single-chip design —
splits the front-end across **5 small Xilinx 7-series FPGAs**: 4 "cluster" FPGAs, each
handling one quadrant of the array, plus 1 "hub" FPGA that aggregates their output and
bridges it to a Raspberry Pi 5 over USB (see Host interface below) — unlike the primary
design's single FPGA, the hub never speaks Ethernet itself.

Motivation: the XC7A200T ships only in a 484-pin FBGA (`XC7A200T-1FBG484C`), which is why
the primary design defers a custom PCB to rev-2 and prototypes on a $500 Nexys Video dev
board instead. Splitting the front end across several much smaller FPGAs — each fitting a
hand-assembly-friendly module — removes that constraint at both the prototype and
custom-PCB stage, and turns one 300mm PCB with 48 matched PDM traces into several
independently buildable, testable, and replaceable tiles.

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
  routing on the hub board at all. Available on the **Arty A7-35T dev board** (~$130) — the
  primary design's FMC LPC connector was needed for 48 direct PDM lines; a hub handling
  only 4 spoke links plus a USB bridge doesn't need it, so a cheaper board works. (The
  Arty A7-35T's own on-board 10/100 Ethernet PHY goes unused here — irrelevant now that the
  hub doesn't speak Ethernet at all, see Host interface below.)

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

The Cmod S7's exposed I/O (its one Pmod, and its 48-pin DIP header) routes every pin through
a 200-240Ω series protection resistor, capped at 25 MHz — standard practice for a
breadboard-friendly module, but it rules out true differential LVDS: a series resistor at
the connector breaks the controlled 100Ω differential impedance a real LVDS receiver needs.
Digilent only exposes genuine shunt-configurable/high-speed differential pins on other
boards (e.g. the Arty's JB/JC); the Cmod S7 has no such option on either connector.

Instead, each spoke is an ordinary **parallel single-ended bus** on the Cmod S7's single
8-signal Pmod: **6 data bits + 1 strobe + 1 forwarded PDM clock (in) = 8 signals**, one
Pmod-to-Pmod cable per spoke. 27.6 Mbps payload ÷ 6 bits ≈ 4.6 MHz per wire — about 5×
margin under the 25 MHz cap, comfortable for a short cable at prototype stage. The hub side
needs no differential-capable connector either; any of the Arty A7's 4 Pmods work
identically for single-ended signaling, so the earlier "high-speed Pmod" distinction on the
hub board is moot.

#### Host interface: USB bridge to Raspberry Pi 5

Unlike the primary design (hub drives GbE directly to a network switch, received by either
host in Configuration A or B — see Host Configurations, below), the clustered hub always
talks to a co-located **Raspberry Pi 5** over USB, and the Pi 5 decides what to do with the
stream:

- **Hub → Pi 5**: one **FTDI FT232H** USB-to-FIFO bridge (~$5 chip, ~$15 breakout module),
  wired to the hub FPGA as a synchronous 245-mode 8-bit parallel FIFO (~12 signals: 8 data +
  RXF#/TXE#/RD#/WR#, no SerDes/GTP needed — fits a single Pmod-style header). USB 2.0
  Hi-Speed sync-FIFO mode sustains ~320 Mbps, comfortably over the 110 Mbps (96ch × 24-bit ×
  48kHz) payload, with headroom to spare if channel count or bit depth grows later. Talks to
  either of the Pi 5's USB 3.0 ports (USB2-speed device, backward compatible).
- **Standalone**: Pi 5 reads the stream over USB and runs beamforming/display locally —
  functionally identical to Configuration A in the primary design, just fed over USB instead
  of GbE.
- **Tethered**: Pi 5 relays the same stream out its own on-board Gigabit Ethernet port to an
  external GPU workstation — replacing Configuration B's direct hub→switch→workstation link
  with a hub→USB→Pi 5→GbE→workstation path. The Pi 5's native GbE (already used in
  Configuration A) is reused for this rather than adding a second Ethernet interface.

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
matching the primary design's reasoning for picking the bigger XC7A200T over XC7A100T —
headroom for future FPGA-side additions (octave-band parallel beamforming, hardware PSF
correction) — while still being a small, cheap part relative to the XC7A200T it replaces.
With GbE gone, LUT budget no longer drives this choice at all in either direction; it's
purely a bet on whether future hub-side additions are worth keeping open. All-XC7S25
remains a documented lower-cost/single-part-number fallback if that headroom isn't needed.

#### Why this satisfies the modularity/cost motivation better than the primary design

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

- New engineering work not needed for the primary design: the cluster-to-hub synchronous
  link protocol (clock forwarding, framing, per-spoke cable-skew calibration) has to be
  designed from scratch — it replaces what used to be an internal parallel bus.
- PDM routing within a cluster reaches 3 arms per board, not 1 — a modest but real routing
  exercise, worth a first-pass floorplan sketch during detailed design.
- More BOM line items and connector points than the primary design (5 FPGAs instead of 1,
  plus 4 cluster-to-hub cables) — each individually simpler/cheaper and independently
  testable, but more total assembly points to design connector keying/strain-relief for.
- Cost comparison here is directional, not quoted, matching how the primary design hedges
  its own $ figures — confirm with current distributor quotes once package/qty are picked.

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
  design gave the hub its own GbE MAC + PHY, mirroring the primary design exactly. Rejected
  in favor of the USB-to-Pi-5 bridge above: it removes an entire PHY part-selection problem
  and ~3,000 LUT of TEMAC/UDP gateware from the hub, at the cost of making the Pi 5
  mandatory in every deployment (including tethered/GPU-host mode) rather than optional.
- **Lattice iCE40UP5K for the cluster tiles**: considered for its open toolchain
  (Yosys/nextpnr/IceStorm) and lower per-chip cost (~$5-8) — clusters never touch GbE, so
  the ECP5 alternate's GbE-SerDes objection (above) wouldn't apply here either. Rejected:
  its 5,280 LUT ceiling doesn't fit the chosen 24ch/cluster load (~7,750 LUT estimate; even
  16ch would leave near-zero margin), and its few small hard multipliers make DSP-based FIR
  compensation (the primary design's approach) unreliable at this channel count. Would also
  reintroduce a second toolchain alongside the hub's Vivado flow.

---

### Supporting ICs

#### GbE PHY

The FPGA implements the MAC layer (via TEMAC IP); a separate PHY chip handles the analog
physical layer and provides the RGMII interface.

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

Two deployment targets share the same FPGA hardware and the same GbE UDP stream. Only the
receiving host and compute backend differ. A single `acoustic_camera_p4.py` script supports
both via a `--backend {numpy,cupy}` flag.

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
- **Native GbE** — receives FPGA UDP stream directly; no adapter

#### Camera: Pi Camera Module 3 Wide

The 120° diagonal FoV matches a typical acoustic camera field of view. Mounts at array center.
Accessed via `picamera2` library rather than OpenCV `VideoCapture`.

---

### Configuration B — GbE-attached Host with GPU

High-performance workstation or server connected to the FPGA hub via a standard network switch
or direct GbE cable. Runs full-resolution beamforming at 20+ fps using a CUDA GPU.

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

### Interface: GbE — identical for both configurations

The FPGA design does not change. Both hosts receive the same 110 Mbps UDP stream.

```
FPGA hub  ──GbE──  [network switch or direct cable]  ──  Pi 5  (Config A)
                                                      ──  GPU workstation  (Config B)
```

On both hosts, a background thread receives UDP packets via Python `socket`, checks sequence
numbers, and pushes PCM frames into a thread-safe deque. The main thread drains the deque to
build the sliding audio buffer. Identical code path for ingestion.

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

| Block | Detail |
|---|---|
| **PDM clock generation** | 12.288 MHz TCXO → PLL → 3.072 MHz; fanned out to all 96 mics with matched-length traces |
| **PDM capture** | 48 data input lines; data latched at each PDM clock edge |
| **L/R demux** | Each data line carries 2 mics (SEL low → even channel, SEL high → odd channel) |
| **CIC decimation** | 5-stage CIC, 64:1 per channel; 3.072 MHz → 48 kHz |
| **FIR compensation** | ~32-tap linear-phase FIR per channel; corrects CIC passband droop |
| **Sample alignment** | All 96 PCM outputs locked to the same 48 kHz word-select boundary |
| **GbE/UDP packetization** | Assemble N frames × 96 channels; prepend sequence number + timestamp; send over RGMII to PHY |
| **PPS input** (optional) | 1 Hz GPIO for absolute time-tagging; enables future multi-unit synchronization |

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

Phase 4 is split into two parallel workstreams that merge at integration.

### Workstream 1 — FPGA hub (Nexys Video dev board)

| Sub-task | Description | Dependency |
|---|---|---|
| **Procure Nexys Video** | Digilent ~$500; XC7A200T + FMC LPC; includes Vivado WebPACK license | None |
| **HDL development** | CIC + FIR + GbE/UDP pipeline in Verilog/VHDL; test on Nexys Video | Nexys in hand |
| **FMC breakout adapter** | Passive adapter: FMC LPC plug on one end, 50-pin IDC headers on other; carries 48 DATA + 1 CLK | HDL ping-pong test passing |

### Workstream 2 — Mic array PCB

| Sub-task | Description | Dependency |
|---|---|---|
| **Geometry finalization** | Confirm 8×12 Underbrink spiral from Phase 1 simulation; generate mic XY coordinates | Phase 1 data |
| **PCB design** | 96× IM72D128 in spiral; 12.288 MHz TCXO; PDM clock fan-out with matched traces; 48-line ribbon cable header to Nexys | Geometry final |
| **PCB fabrication** | 6-layer recommended (PDM ground plane, clock shielding); JLCPCB / PCBWay | Layout complete |
| **Assembly** | IM72D128 is a small LGA; reflow oven or PCB assembly service | PCB received |

### Integration & Software

| Sub-task | Description | Dependency |
|---|---|---|
| **First integration** | Connect mic array PCB to Nexys; verify all 96 PDM channels on ILA | Both workstreams complete |
| **Host software** | UDP ingestion + 96-ch pipeline; extend `acoustic_camera_p3.py` → `acoustic_camera_p4.py` | Nexys producing valid UDP |
| **Camera** | Pi Camera Module 3 Wide (Config A) or USB webcam (Config B) | Host software running |
| **Calibration** | Gain + phase estimation at 96-ch scale; extend nb17 approach | Hardware assembled |

### Rev-2 (deferred)

Custom FPGA hub PCB (bare XC7A200T + 88E1111 PHY + 12.288 MHz TCXO) designed after the
full pipeline is validated on the Nexys. Eliminates the dev board and produces a compact
integrated unit suitable for the Phase 4b housing.

---

## PCB Bring-Up Checklist

*Populate as bring-up proceeds.*

- [ ] FPGA Artix-7 powers up; JTAG accessible
- [ ] 12.288 MHz TCXO oscillating (verify with scope)
- [ ] PLL locked; 3.072 MHz PDM clock present on header pin
- [ ] Single mic connected; PDM data line shows valid 1-bit stream on ILA
- [ ] CIC output produces 48 kHz PCM samples (verify with ILA + known tone)
- [ ] FIR compensation: flat frequency response confirmed on single channel
- [ ] All 48 data lines active; 96 channels valid
- [ ] GbE PHY link up (LED)
- [ ] UDP packets received on host; no sequence gaps
- [ ] Host pipeline: 96-channel CSM computed; beamform produces coherent energy map
- [ ] Calibration: cross-correlation gain/phase vectors captured and applied

---

## Key References

- Ben Wang, "192-channel phased array microphone" (2023) — similar FPGA + GbE architecture
- Underbrink multi-arm log-spiral array patent — geometry basis
- Xilinx TEMAC IP core product guide (PG051) — GbE MAC integration
- Infineon IM72D128 datasheet — mic specs and PDM timing
- `alexforencich/verilog-ethernet` — open-source GbE MAC (alternative to TEMAC)
