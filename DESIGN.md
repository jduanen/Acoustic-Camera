# Acoustic Camera Design Target

## Table of Contents

* Hardware
  - Microphone Array
  - FPGA Front-End
    * FPGA Responsibilities (hard real-time, parallel)
    * FPGA Choice
      - Primary: Single-FPGA (ALINX AC7200 Module)
      - Alternate: Multi-FPGA (Clustered)
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
  - Primary design: two mating boards, `mic_array` (96-mic disc, 48 DATA + 1 CLK to the
    board-to-board connector) and `front_end` (ALINX AC7200 module, buck regulator, GbE PHY)
  - Alternate Multi-FPGA design: 4 quadrant boards (3 arms/24 mics each), paired L/R on data
    lines → 12 DATA + 1 CLK per board, to that quadrant's cluster FPGA

### FPGA Front-End

#### FPGA responsibilities (hard real-time, parallel)

One FPGA (the AC7200 module) does the entire pipeline end-to-end in the primary design (full
detail in [`PHASE4.md`](./PHASE4.md#fpga-responsibilities-hard-real-time-parallel)): PDM clock
generation, 48-line PDM capture + L/R demux, CIC decimation + FIR compensation (96ch, fully
parallel), sample alignment, and direct GbE/UDP packetization (5 frames × 96ch + sequence
number + timestamp → RGMII → PHY).

The alternate Multi-FPGA design splits the same responsibilities across cluster and hub tiers
instead:

| Block | Cluster (×4) | Hub (×1) |
|---|---|---|
| PDM clock | fan-out to 24 local mics | generate from 12.288 MHz TCXO; forward to all 4 clusters |
| PDM capture / L/R demux | 12 local data lines | — |
| CIC decimation + FIR compensation | per-channel, 24ch | — |
| Framing | onto spoke bus, 24ch | reassemble 4×24ch = 96ch; USB FIFO out to Pi 5 |
| Sample alignment | — | all 96 channels locked to one 48 kHz word-select boundary |
| PPS input (optional) | — | 1 Hz GPIO for absolute time-tagging; enables multi-unit sync |

#### FPGA Choice

##### Primary: Single-FPGA (ALINX AC7200 Module)

One Xilinx XC7A200T does the entire 96-channel CIC + FIR pipeline plus direct GbE/UDP
packetization, carried on a real, currently-sold **ALINX AC7200** System-on-Module (FPGA +
1GB DDR3 + QSPI flash + clocks + full power delivery, fully assembled — ~$299) rather than a
from-scratch bare-chip board. This avoids the XC7A200T's two structural costs (484-pin BGA
needing a dev board at prototype stage and BGA rework at custom-PCB stage; 48 PDM lines
needing to route the full ~300mm array span on one monolithic PCB) the same way the alternate
design avoids them — by using a pre-assembled module — without needing to split the pipeline
across 5 chips: no BGA soldering or rework at any build stage, and the module's own 4× 80-pin
board-to-board connectors (195 usable GPIO) comfortably cover the 48 PDM lines + GbE + JTAG
this design needs.

Physically two mating boards: `mic_array` (96-mic disc: mics, PDM clock fan-out, local power)
and `front_end` (AC7200 module, buck pre-regulator, GbE PHY), sharing one project-local KiCad
library (`pcb/libraries/`).

| Device | LUTs | DSP | BRAM | 96-ch headroom | 128-ch headroom | Notes |
|---|---|---|---|---|---|---|
| XC7A35T | 20,800 | 90 | 1.8 Mb | No | No | Too small |
| ECP5-25F | 25,500 | 56 | 1.67 Mb | No | No | Too small |
| ECP5-45F | 44,500 | 90 | 1.93 Mb | Tight (5%) | No | Open toolchain only |
| XC7A100T | 63,400 | 240 | 4.86 Mb | 35% | 16% | Considered; headroom tight for expansion |
| **XC7A200T** | **134,600** | **740** | **13.1 Mb** | **~75%** | **~60%** | **Chosen** |

XC7A200T chosen over XC7A100T: ~75% LUT headroom vs 35% (comfortable margin for future
additions), ~60% vs 16% at a 128-channel upgrade, 740 DSP48E1 blocks (all FIR chains in
dedicated DSPs, zero LUT MAC cost), same Artix-7 Vivado/ILA-VIO flow. Real placed/routed RTL
results on `xc7a200tfbg484-1`: 43.52% LUT for the pipeline alone, 48.14% combined with the
GbE MAC + UDP packetizer (`alexforencich/verilog-ethernet`, open-source, used instead of
Xilinx's TEMAC IP core, which needs a separate license Xilinx doesn't bundle by default).

Full reasoning, real RTL numbers, and rejected alternatives are in
[`PHASE4.md`](./PHASE4.md#fpga--single-fpga-alinx-ac7200-module).

##### Alternate: Multi-FPGA (Clustered)

Splits the front end across 4 small "cluster" FPGAs — one per 90° quadrant of 3 arms / 24
mics each, doing local PDM capture + CIC/FIR — plus 1 hub FPGA that aggregates their output
over a parallel single-ended bus (not true LVDS — the Cmod A7-35T's exposed I/O has no
differential-capable pins). Both tiers use the same part, Xilinx Artix-7 XC7A35T: the cluster
tier originally targeted a smaller Spartan-7 XC7S25, but that part couldn't fit the CIC/FIR
pipeline even after sharing optimizations (see `fpga/multi_fpga/cluster/rtl/cluster_top.v`'s header
comment), so it moved to the same XC7A35T the hub already used — landing on one part number
for all 5 modules instead of two. The hub never speaks Ethernet: it bridges the aggregated
stream over USB (FTDI FT232H sync FIFO) to a co-located Raspberry Pi 5, which either runs
beamforming locally (standalone) or relays the stream out its own on-board GbE port to an
external host (tethered) — no RGMII PHY chip or GbE MAC anywhere in this design. Both FPGA
tiers are far smaller than the primary single-chip design (the hub uses under 10% of its LUTs,
the cluster ~71% — see `fpga/README.md` for current synthesis numbers), all on
hand-assembly-friendly Digilent Cmod A7-35T modules, no BGA rework needed even at prototype
stage.

Kept as a proven, already-working alternate — fully implemented, synthesizes/places/routes
clean on real hardware. Splitting the front end into several independently buildable,
testable, replaceable tiles was the original motivation before the AC7200 module made the
single-chip path practical without paying its BGA/routing costs.

Full reasoning, LUT/pin/bandwidth budgets, and rejected alternatives (finer/coarser
grouping, daisy-chain topology, Lattice iCE40 tiles, hub-direct GbE) are in
[`PHASE4.md`](./PHASE4.md#fpga--alternate-multi-fpga-clustered).

##### Alternate: Lattice ECP5-45F

Applies to the primary single-FPGA design above (not the alternate Multi-FPGA design, whose
cluster tiles never touch GbE). Use only if a fully open-source toolchain (Yosys + nextpnr,
no Vivado) is a hard requirement. Fits 96 channels with ~5% LUT margin; does **not** fit 128
channels. GbE SerDes integration on the ECP5 open tools is harder (~4–6 weeks extra).
Suitable for a rev-2 board.

##### Considered and rejected: Zynq-7020

The XC7Z020 combines 85,000 LUTs of FPGA fabric with a dual-core ARM Cortex-A9 processor on
the same die. Potentially interesting for Config A (standalone) because the ARM could replace
the Pi 5. Rejected for the first board because:
- Cortex-A9 @ 1 GHz is 4–5× slower than Pi 5's Cortex-A76 @ 2.4 GHz for NumPy/BLAS;
  96-ch D&S at 3°/pt would run at ~5–10 fps rather than ~15–20 fps
- 85,000 LUT fabric gives only ~50% headroom for 96-ch; tighter than the primary design's
  XC7A200T
- Added complexity of PS+PL integration (Vitis toolchain in addition to Vivado)
- Pi 5 + either FPGA front-end gives better standalone performance at comparable cost
- Worth revisiting if a future rev integrates everything into one board

### Supporting ICs

* **GbE PHY**: needed only by the primary single-FPGA design — chosen: Microchip KSZ9031RNX
  (48-QFN, no BGA, ~$4–6, RGMII ↔ FPGA MAC); Marvell 88E1111 considered and rejected (117-ball
  BGA, real sourcing difficulty at low volume). The alternate design's hub has no GbE MAC/PHY
  at all
* **Master clock**: 12.288 MHz TCXO (temperature-compensated oscillator)
  - ÷ 4 = 3.072 MHz PDM clock (exact)
  - × 4 via PLL = 49.152 MHz FPGA master; ÷ 1024 = 48.000 kHz word select (exact)
  - TCXO required (not plain crystal) — sample-rate drift accumulates in the CSM over long captures

### Touchscreen Display

**TBD**

### Power Supply

External 5V/4A+ supply → Raspberry Pi 5's USB-C input. The Pi 5's own +5V
rail is tapped to power the hub board directly and each of the 4 arm
boards over a dedicated 2-pin connector (separate from the spoke data
bus — see `pcb/multi_fpga/SCHEMATIC_NOTES.md`'s "Power architecture" for
why). Each arm board has its own local +1.8V LDO for its 24 mics
(independent per board, not shared); the hub has its own +3.3V LDO for
the FT232H USB bridge and TCXO. Full rail-by-rail detail, part numbers,
and confidence flags: `pcb/multi_fpga/SCHEMATIC_NOTES.md`.

---

## Host Configurations

In the primary single-FPGA design, the front-end drives GbE/UDP directly to whichever host is
present — no relay required. Two operating modes share the same hardware. (The alternate
Multi-FPGA design instead talks USB to a co-located Raspberry Pi 5, which is present in every
deployment and, in tethered mode, relays the stream onward over its own GbE port.)

### Config A — Standalone (Raspberry Pi 5, 8 GB)

Self-contained field unit. Pi mounts in the camera housing alongside the front-end board;
receives the front-end's UDP stream directly over its native GbE port.

* **Compute**: NumPy + OpenBLAS (no GPU)
  - 3°/pt grid for live display: D&S ~20 ms (~20 fps), MVDR/MUSIC ~50 ms (~10 fps)
  - 0.5°/pt used for offline post-processing of recordings
* **Camera**: Pi Camera Module 3 Wide (IMX708, 120° FoV) via MIPI CSI
* **Upgrade path**: PCIe M.2 slot → Hailo-8 NPU (~$70) for Phase 5 ML beamforming
* **Display**: HDMI touchscreen or SSH + web UI for field use

### Config B — Tethered, GbE-attached Host with GPU

High-performance workstation connected directly to the front-end via GbE — no Pi 5 relay
needed; the Pi 5 is not present in this configuration at all. Full-resolution real-time
beamforming.

* **Compute**: CuPy on CUDA GPU
  - 0.5°/pt grid: D&S ~10 ms, MVDR/MUSIC ~5 ms, all at 20+ fps
  - VRAM required: ~33 MB for steering matrix (complex128, 96 × 21,901) — any GPU fits
  - Minimum GPU: GTX 1070 (8 GB). Recommended: RTX 3060 (12 GB)
* **Camera**: USB 3.0 webcam via OpenCV `VideoCapture`

### Interface: direct GbE, no relay

Data rate: 96 ch × 48 kHz × 24 b = 110 Mbps — well within 1 GbE.

Frames carry: sequence number, timestamp, 5 frames × 96 channels PCM. Host ingestion:
background thread → thread-safe deque → sliding audio buffer — identical code path for
Config A or Config B, since both receive the same UDP stream directly off their own NIC.

> Alternate Multi-FPGA design: hub bridges USB to a co-located Pi 5 instead; in tethered mode
> the Pi 5 relays the same stream out its own GbE port to the GPU workstation — the Pi 5 is
> mandatory in every deployment of that design, unlike the primary design where it's optional
> in Config B.

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
