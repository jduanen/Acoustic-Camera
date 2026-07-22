# Acoustic Camera Design Target

## Table of Contents

* Hardware
  - Microphone Array
  - FPGA Front-End
    * FPGA Responsibilities (hard real-time, parallel)
    * FPGA Choice
      - Primary: Multi-FPGA (Clustered)
      - Alternate: Single-FPGA (Xilinx Artix-7 XC7A200T)
      - Alternate: Lattice ECP5-45F
      - Considered and rejected: Zynq-7020
  - Supporting ICs
  - Touchscreen Display
  - Power Supply
* Host Configurations
  - Config A — Standalone (Raspberry Pi 5, 8 GB)
  - Config B — Tethered, GbE-attached Host with GPU
  - Interface: USB to Pi 5, GbE relay for Config B
* Video
* Software
* Device Functions
  - Core Functions
  - Extended Functions
* Interface Design
  - Phase 2: Host-tethered (USB)
  - Phase 3: Standalone (Pi 5, 5" touchscreen, battery-powered)
  - Phase 4: Standalone Field Use (Pi 5, 7" touchscreen, battery-powered)
  - Phase 5: **TBD**

---

## Hardware

### Microphone Array

* **96× Infineon IM72D128** PDM MEMS mics
  - 72 dB(A) SNR, 128 dBSPL AOP, IP57 ingress protection
  - Factory-calibrated: ±1 dB sensitivity match, ±2° phase match
  - Part: IM72D128V01XTMA1 (DigiKey / Mouser)
* **Underbrink multi-arm log-spiral** pattern
  - 12 arms × 8 mics (6 × 16 alternative simulated in Phase 1; 12 × 8 chosen)
* **~300 mm diameter aperture; ~13 mm minimum mic spacing**
  - Spatial Nyquist ~13 kHz — no aliasing within 8 kHz operating range
  - Far-field distance: 0.52 m @ 1 kHz · 1.6 m @ 3 kHz · 4.2 m @ 8 kHz
* **Custom PCB(s)**
  - Primary design: 4 quadrant boards (3 arms/24 mics each), paired L/R on data lines →
    12 DATA + 1 CLK per board, to that quadrant's cluster FPGA
  - Single-FPGA alternate: one monolithic board, 48 DATA + 1 CLK to the single hub FPGA

### FPGA Front-End

#### FPGA responsibilities (hard real-time, parallel)

Split across cluster and hub tiers in the primary Multi-FPGA design (full split in
[`PHASE4.md`](./PHASE4.md#fpga-responsibilities-hard-real-time-parallel)):

| Block | Cluster (×4) | Hub (×1) |
|---|---|---|
| PDM clock | fan-out to 24 local mics | generate from 12.288 MHz TCXO; forward to all 4 clusters |
| PDM capture / L/R demux | 12 local data lines | — |
| CIC decimation + FIR compensation | per-channel, 24ch | — |
| Framing | onto spoke bus, 24ch | reassemble 4×24ch = 96ch; USB FIFO out to Pi 5 |
| Sample alignment | — | all 96 channels locked to one 48 kHz word-select boundary |
| PPS input (optional) | — | 1 Hz GPIO for absolute time-tagging; enables multi-unit sync |

Single-FPGA alternate: same responsibilities in one FPGA, plus direct GbE/UDP packetization
(N frames × 96 ch + sequence number + timestamp → RGMII → PHY) instead of USB framing.

#### FPGA Choice

##### Primary: Multi-FPGA (Clustered)

Splits the front end across 4 small Xilinx Spartan-7 (XC7S25) "cluster" FPGAs — one per
90° quadrant of 3 arms / 24 mics each, doing local PDM capture + CIC/FIR — plus 1 Xilinx
Artix-7 (XC7A35T) hub FPGA that aggregates their output over a parallel single-ended bus
(not true LVDS — the Cmod S7's exposed I/O has no differential-capable pins). The hub never
speaks Ethernet: it bridges the aggregated stream over USB (FTDI FT232H sync FIFO) to a
co-located Raspberry Pi 5, which either runs beamforming locally (standalone) or relays the
stream out its own on-board GbE port to an external host (tethered) — no RGMII PHY chip or
GbE MAC anywhere in this design. Both FPGA tiers are far smaller than a single-chip XC7A200T
design (under 6% of its LUTs each), all on hand-assembly-friendly modules (Digilent Cmod S7 /
Arty A7-35T), no BGA rework needed even at prototype stage.

Chosen over the single-chip alternate (below) because the XC7A200T ships only in a 484-pin
BGA — forcing a dev board at prototype stage and BGA rework at custom-PCB stage — and needs
all 48 PDM lines routed the full ~300mm array span on one monolithic PCB. Splitting into
several independently buildable, testable, replaceable tiles avoids both costs.

Full reasoning, LUT/pin/bandwidth budgets, and rejected alternatives (finer/coarser
grouping, daisy-chain topology, Lattice iCE40 tiles, hub-direct GbE) are in
[`PHASE4.md`](./PHASE4.md#fpga--multi-fpga-clustered).

##### Alternate: Single-FPGA (Xilinx Artix-7 XC7A200T)

Simpler single-chip design: one FPGA does the entire 96-channel CIC + FIR + GbE pipeline
(~40,000–43,000 LUTs) rather than splitting across cluster and hub tiers.

| Device | LUTs | DSP | BRAM | 96-ch headroom | 128-ch headroom | Notes |
|---|---|---|---|---|---|---|
| XC7A35T | 20,800 | 90 | 1.8 Mb | No | No | Too small |
| ECP5-25F | 25,500 | 56 | 1.67 Mb | No | No | Too small |
| ECP5-45F | 44,500 | 90 | 1.93 Mb | Tight (5%) | No | Open toolchain only |
| XC7A100T | 63,400 | 240 | 4.86 Mb | 35% | 16% | Considered; headroom tight for expansion |
| **XC7A200T** | **134,600** | **740** | **13.1 Mb** | **70%** | **61%** | **Chosen for this alternate** |

XC7A200T chosen over XC7A100T: 70% LUT headroom vs 35% (comfortable margin for future
additions), 61% vs 16% at a 128-channel upgrade, 740 DSP48E1 blocks (all FIR chains in
dedicated DSPs, zero LUT MAC cost), same Artix-7 Vivado/TEMAC/ILA-VIO flow. Ships only in a
484-pin BGA (XC7A200T-1FBG484C) — **Nexys Video** dev board (~$550) used as the FPGA hub at
prototype stage (same chip + FMC LPC connector for all 48 PDM DATA lines + CLK); bare chip on
custom PCB deferred to rev-2.

Full reasoning and device comparison detail are in
[`PHASE4.md`](./PHASE4.md#fpga--alternate-single-fpga-xc7a200t).

##### Alternate: Lattice ECP5-45F

Applies to the single-FPGA alternate above (not the primary Multi-FPGA design, whose cluster
tiles never touch GbE). Use only if a fully open-source toolchain (Yosys + nextpnr, no
Vivado) is a hard requirement. Fits 96 channels with ~5% LUT margin; does **not** fit 128
channels. GbE SerDes integration on the ECP5 open tools is harder (~4–6 weeks extra).
Suitable for a rev-2 board.

##### Considered and rejected: Zynq-7020

The XC7Z020 combines 85,000 LUTs of FPGA fabric with a dual-core ARM Cortex-A9 processor on
the same die. Potentially interesting for Config A (standalone) because the ARM could replace
the Pi 5. Rejected for the first board because:
- Cortex-A9 @ 1 GHz is 4–5× slower than Pi 5's Cortex-A76 @ 2.4 GHz for NumPy/BLAS;
  96-ch D&S at 3°/pt would run at ~5–10 fps rather than ~15–20 fps
- 85,000 LUT fabric gives only ~50% headroom for 96-ch; tighter than the single-FPGA
  alternate's XC7A200T
- Added complexity of PS+PL integration (Vitis toolchain in addition to Vivado)
- Pi 5 + either FPGA front-end gives better standalone performance at comparable cost
- Worth revisiting if a future rev integrates everything into one board

### Supporting ICs

* **GbE PHY**: needed only by the single-FPGA alternate — Marvell 88E1111 or Microchip
  KSZ9031RNX (RGMII ↔ FPGA TEMAC); the primary design's hub has no GbE MAC/PHY at all
* **Master clock**: 12.288 MHz TCXO (temperature-compensated oscillator)
  - ÷ 4 = 3.072 MHz PDM clock (exact)
  - × 4 via PLL = 49.152 MHz FPGA master; ÷ 1024 = 48.000 kHz word select (exact)
  - TCXO required (not plain crystal) — sample-rate drift accumulates in the CSM over long captures

### Touchscreen Display

**TBD**

### Power Supply

**TBD**

---

## Host Configurations

In the primary Multi-FPGA design, the hub only ever talks to a co-located Raspberry Pi 5 over
USB — the Pi 5 is present in every deployment, not optional. Two operating modes share that
hardware. (The single-FPGA alternate instead drives GbE directly from the hub to either host,
with no Pi 5 relay required.)

### Config A — Standalone (Raspberry Pi 5, 8 GB)

Self-contained field unit. Pi mounts in the camera housing alongside the FPGA hub board;
receives the hub's stream directly over USB (FT232H sync FIFO bridge).

* **Compute**: NumPy + OpenBLAS (no GPU)
  - 3°/pt grid for live display: D&S ~20 ms (~20 fps), MVDR/MUSIC ~50 ms (~10 fps)
  - 0.5°/pt used for offline post-processing of recordings
* **Camera**: Pi Camera Module 3 Wide (IMX708, 120° FoV) via MIPI CSI
* **Upgrade path**: PCIe M.2 slot → Hailo-8 NPU (~$70) for Phase 5 ML beamforming
* **Display**: HDMI touchscreen or SSH + web UI for field use

### Config B — Tethered, GbE-attached Host with GPU

High-performance workstation connected via GbE — not to the FPGA hub directly, but to the
Raspberry Pi 5's on-board GbE port, which relays the same USB stream it receives from the hub
(Pi 5 is present here too, relaying rather than computing). Full-resolution real-time
beamforming.

* **Compute**: CuPy on CUDA GPU
  - 0.5°/pt grid: D&S ~10 ms, MVDR/MUSIC ~5 ms, all at 20+ fps
  - VRAM required: ~33 MB for steering matrix (complex128, 96 × 21,901) — any GPU fits
  - Minimum GPU: GTX 1070 (8 GB). Recommended: RTX 3060 (12 GB)
* **Camera**: USB 3.0 webcam via OpenCV `VideoCapture`

### Interface: USB to Pi 5, GbE relay for Config B

Data rate: 96 ch × 48 kHz × 24 b = 110 Mbps — well within the USB 2.0 Hi-Speed sync-FIFO
bridge (~320 Mbps) and within 1 GbE for the Config B relay hop.

Frames carry: sequence number, timestamp, N frames × 96 channels PCM.
Host ingestion: background thread → thread-safe deque → sliding audio buffer — identical code
path whether frames arrive over USB (Config A) or via the Config B GbE relay.

> Single-FPGA alternate: hub drives GbE directly to a switch; either host receives the same
> UDP stream directly, no Pi 5 relay — the Pi 5 is optional in that design's Config B.

## Video

* **Config A**: Pi Camera Module 3 Wide (IMX708) via MIPI CSI; accessed via `picamera2`
* **Config B**: USB 3.0 camera via OpenCV `VideoCapture`
* Co-located at array center; FoV matched to array aperture at expected working distance
* Real-time overlay of energy map on video stream

## Software

* Custom Python pipeline (no Acoular dependency for Phase 4)
  - Reuses beamforming functions from `src/acoustic_camera_p3.py`
  - Entry point: `src/acoustic_camera_p4.py`
* Backend selection via `--backend {numpy,cupy}`:

```python
if args.backend == 'cupy':
    import cupy as xp
    from cupy.linalg import inv, eigh
else:
    import numpy as xp
    from scipy.linalg import inv, eigh
```

All beamforming functions operate on `xp` arrays transparently. Grid resolution default:
`--grid_deg 3.0` for numpy (Pi 5), `--grid_deg 0.5` for cupy (GPU host).

* CLEAN-SC does not parallelize well across grid points; use Functional Beamforming
  (D&S power map raised to exponent ν) for real-time GPU display. CLEAN-SC for offline.
* Algorithm progression: D&S → MVDR → CLEAN-SC → Functional BF → ML (Phase 5)

## Device Functions

### Core Functions

Every commercial acoustic camera supports these:

* Real-time energy map: beamformed sound field rendered as a 2D heatmap image
* Video + acoustic overlay: live video with energy map overlaid (colormap on camera image)
* Selectable frequency range: bandpass filter to focus on frequencies of interest (octave or 1/3 octave bands)
* Dynamic range control: adjustable display floor and ceiling (analogous to brightness/contrast for the acoustic image)
* Field of view selection: selectable FoV (e.g., 90° or 60°); must match video camera optics
* Image persistence/temporal averaging: controls the time window over which energy is averaged
  - fast = tracks transients; slow = reveals weak stationary sources
  - range: 10 ms to 10 s
* Record/playback: capture synchronized audio, video, and energy map sequences; replay offline

### Extended Functions

Features that differentiate higher-end products; implement progressively:

* Algorithm selector: choose beamforming algorithm at runtime: D&S → MVDR → CLEAN-SC (and later ML-based)
* Frequency-resolved maps: display energy maps at multiple octave bands simultaneously (side-by-side or toggled)
* Source tracking: lock on and follow the loudest detected source across frames
* Near-field/far-field mode: toggle assumption about wave curvature; affects steering delay computation
* Depth/range estimation: estimate distance to source using a co-located depth camera (e.g., Intel RealSense); enables 3D source localization
* Calibration mode: guided workflow to measure and store mic-to-mic sensitivity, phase, and position corrections

## Interface Design

### Phase 2: Host-tethered (USB)

Follow the ACAM_64 (Convergence Instruments) model: USB audio streaming to host PC, open protocol, Python-based desktop GUI.

GUI elements:
* Live video window with energy map overlay (adjustable colormap, opacity)
* Frequency band selector (octave and 1/3 octave buttons or slider)
* Dynamic range sliders (floor/ceiling dB)
* Algorithm selector dropdown (D&S, MVDR, CLEAN-SC)
* Persistence/averaging time slider
* FOV selector (if multiple optics supported)
* Record/Stop button and session file naming
* Real-time SPL meter and peak-hold indicator
* Status bar: sample rate, latency, mic count, connection state

### Phase 3: Standalone (Pi 5, 5" touchscreen, battery-powered)

Same GUI elements as Phase 2/3, running on Pi 5. Display via HDMI touchscreen attached to
housing, or web UI served over WiFi for phone/tablet access.

Full-resolution energy maps at all algorithms. GUI runs on workstation display.
Additional elements:
* GbE packet ingestion status (sequence gaps, drop rate)
* Calibration workflow UI (guided mic position / sensitivity estimation)
* Offline playback and post-processing mode (load recorded audio, re-beamform at 0.5°/pt)

### Phase 4: Standalone Field Use (Pi 5, 7" touchscreen, battery-powered)

* Embedded web UI (served from Pi, accessed via phone or tablet over WiFi)
* Small integrated touchscreen (7" is the commercial standard: HEAD VMA V pattern)
* Physical record/stop button(s) on the housing
* Battery-powered operation with charge indicator

### Phase 5: TBD

**TBD**
