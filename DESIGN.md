# Acoustic Camera Design Target

## Microphone Array

* **96× Infineon IM72D128** PDM MEMS mics
  - 72 dB(A) SNR, 128 dBSPL AOP, IP57 ingress protection
  - Factory-calibrated: ±1 dB sensitivity match, ±2° phase match
  - Part: IM72D128V01XTMA1 (DigiKey / Mouser)
* **Underbrink multi-arm log-spiral** pattern
  - 8 arms × 12 mics (simulate 6 × 16 as alternative in Phase 1; geometry already optimized)
* **~300 mm diameter aperture; ~21 mm minimum mic spacing**
  - Spatial Nyquist at 8 kHz: λ/2 = 343/(2×8000) ≈ 21.4 mm
* **Custom PCB(s)**
  - Mics share PDM clock, paired L/R on data lines → 48 DATA + 1 CLK to FPGA

## FPGA Front-End

### Primary: Xilinx Artix-7 XC7A100T

The 96-channel CIC + FIR + GbE pipeline requires ~38,000–40,000 LUTs. The XC7A35T (20,800
LUTs) and ECP5-25F (25,500 LUTs) do not fit.

| Device | LUTs | DSP | BRAM | Fits? |
|---|---|---|---|---|
| XC7A35T | 20,800 | 90 | 1.8 Mb | No |
| ECP5-25F | 25,500 | 56 | 1.67 Mb | No |
| ECP5-45F | 44,500 | 90 | 1.93 Mb | Yes (tight) |
| **XC7A100T** | **63,400** | **240** | **4.86 Mb** | **Yes (~35% headroom)** |

Reasons for XC7A100T:
- TEMAC (Tri-Mode Ethernet MAC) IP included in Vivado — production-grade GbE
- ILA / VIO in-circuit debug — essential for PCB bring-up
- 240 DSP48E1 blocks — efficient FIR MAC without burning LUTs
- Dev board for HDL development: Arty A7-100T ($149), same chip
- Part: XC7A100T-1FTG256C

### Alternate: Lattice ECP5-45F

Use if a fully open-source toolchain (Yosys + nextpnr, no Vivado) is required. GbE SerDes
integration on the ECP5 open tools is harder (~4–6 weeks extra). Suitable for a rev-2 board.

### Supporting ICs

* **GbE PHY**: Marvell 88E1111 or Microchip KSZ9031RNX (RGMII ↔ FPGA TEMAC)
* **Master clock**: 12.288 MHz TCXO (temperature-compensated oscillator)
  - ÷ 4 = 3.072 MHz PDM clock (exact)
  - × 4 via PLL = 49.152 MHz FPGA master; ÷ 1024 = 48.000 kHz word select (exact)
  - TCXO required (not plain crystal) — sample-rate drift accumulates in the CSM over long captures

### FPGA responsibilities (hard real-time, parallel)

| Block | Detail |
|---|---|
| PDM clock generation | 12.288 MHz TCXO → PLL → 3.072 MHz, fanned out to all 96 mics |
| PDM capture | 48 input lines, latched at PDM clock edges |
| L/R demux | Each line carries 2 mics; SEL low → even channel, SEL high → odd channel |
| CIC decimation | 5-stage, 64:1 per channel; 3.072 MHz → 48 kHz |
| FIR compensation | ~32-tap per channel; corrects CIC passband droop |
| Sample alignment | All 96 PCM outputs locked to the same 48 kHz word-select boundary |
| GbE/UDP packetization | N frames × 96 ch + sequence number + timestamp → RGMII → PHY |
| PPS input (optional) | 1 Hz GPIO for absolute time-tagging; enables multi-unit sync |

## Host Configurations

Two deployment targets share the same FPGA hardware and GbE UDP stream. Interface is
identical; only the receiving host and compute backend differ.

### Config A — Standalone (Raspberry Pi 5, 8 GB)

Self-contained field unit. Pi mounts in the camera housing alongside the FPGA hub board.

* **Compute**: NumPy + OpenBLAS (no GPU)
  - 3°/pt grid for live display: D&S ~20 ms (~20 fps), MVDR/MUSIC ~50 ms (~10 fps)
  - 0.5°/pt used for offline post-processing of recordings
* **Camera**: Pi Camera Module 3 Wide (IMX708, 120° FoV) via MIPI CSI
* **Upgrade path**: PCIe M.2 slot → Hailo-8 NPU (~$70) for Phase 5 ML beamforming
* **Display**: HDMI touchscreen or SSH + web UI for field use

### Config B — GbE-attached Host with GPU

High-performance workstation connected via GbE. Full-resolution real-time beamforming.

* **Compute**: CuPy on CUDA GPU
  - 0.5°/pt grid: D&S ~10 ms, MVDR/MUSIC ~5 ms, all at 20+ fps
  - VRAM required: ~33 MB for steering matrix (complex128, 96 × 21,901) — any GPU fits
  - Minimum GPU: GTX 1070 (8 GB). Recommended: RTX 3060 (12 GB, ~$300)
* **Camera**: USB 3.0 webcam via OpenCV `VideoCapture`

### GbE interface (same for both)

Data rate: 96 ch × 48 kHz × 24 b = 110 Mbps — fits within 1 GbE.

UDP packets carry: sequence number, timestamp, N frames × 96 channels PCM.
Host ingestion: background thread → thread-safe deque → sliding audio buffer.

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

## Video

* **Config A**: Pi Camera Module 3 Wide (IMX708) via MIPI CSI; accessed via `picamera2`
* **Config B**: USB 3.0 camera via OpenCV `VideoCapture`
* Co-located at array center; FoV matched to array aperture at expected working distance
* Real-time overlay of energy map on video stream

## Device Functions & Interface

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

### Interface Design

#### Phase 2 / Phase 3: Host-tethered (USB)

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

#### Phase 4A: Standalone (Pi 5)

Same GUI elements as Phase 2/3, running on Pi 5. Display via HDMI touchscreen attached to
housing, or web UI served over WiFi for phone/tablet access.

#### Phase 4B: GbE + GPU workstation

Full-resolution energy maps at all algorithms. GUI runs on workstation display.
Additional elements:
* GbE packet ingestion status (sequence gaps, drop rate)
* Calibration workflow UI (guided mic position / sensitivity estimation)
* Offline playback and post-processing mode (load recorded audio, re-beamform at 0.5°/pt)

#### Phase 4 Standalone Field Use

* Embedded web UI (served from Pi, accessed via phone or tablet over WiFi)
* Small integrated touchscreen (7" is the commercial standard: HEAD VMA V pattern)
* Physical record/stop button(s) on the housing
* Battery-powered operation with charge indicator
