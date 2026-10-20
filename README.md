# Acoustic Camera — Project Summary

A low-cost, high-performance acoustic camera: a microphone array computes a power map of the
sound sources in a video camera's field of view and overlays it on the video in real time.
Uses include locating vibration, gas leaks and electrical breakdown, and as a hearing aid for
people who can no longer localize sound with one ear. Condensed from [README.md](./README.md);
see the linked documents for detail.

## How it works

The array samples the sound field at every mic at once. For each candidate direction
(azimuth, elevation), a **steering vector** describes the expected phase pattern across the
mics; a **beamformer** scores each direction against the measured **cross-spectral matrix
(CSM)** and the scores form the energy map overlaid on the video.

Three array properties set performance:

| Property | Controls | Rule |
|---|---|---|
| Aperture *D* | Beam width (resolution) and low-frequency limit | HPBW ≈ 58° × λ/*D*; useful directionality needs *f* > *c*/*D* |
| Minimum mic spacing *d* | High-frequency limit (grating lobes above it) | *f*<sub>Nyq</sub> = *c* / (2*d*) |
| Mic layout and count | Sidelobe level (MSL), SNR | Irregular spacing suppresses aliasing; more mics lower sidelobes |

Other key ideas: far field begins at *r* > 2*D*²/λ; the **point spread function** is how the
array smears a point source; commercial cameras average power incoherently across an octave
band to cut noise. Details: [BACKGROUND.md](./BACKGROUND.md), [MIC_ARRAYS.md](./MIC_ARRAYS.md).

**Algorithms explored** (one per family): Delay-and-Sum (simple, robust, wide beam), MVDR
(adaptive, suppresses interferers), CLEAN-SC (deconvolution, removes PSF artifacts), MUSIC
(subspace, super-resolution when the source count is known). ML methods come in Phase 5.

## Requirements and key design choices

| Parameter | Target |
|---|---|
| Frequency range | 200 Hz – 8 kHz (useful imaging from ~2 kHz) |
| Resolution (HPBW) | ~9.4° @ 8 kHz, ~19° @ 4 kHz, ~82° @ 1 kHz (simulated, Phase 4 array) |
| Distance | ~0.5 m (far-field limit) to ~10 m |
| Field of view | ±45° H, ±30° V, matched to the camera |
| Array | ~300 mm aperture, 96 MEMS mics, Underbrink 12 × 8 log-spiral, 12.94 mm min spacing (Nyquist ≈ 13.3 kHz) |

Why these numbers ([TRADEOFFS.md](./TRADEOFFS.md)):
- **300 mm aperture** is the practical sweet spot: portable, yet ~9° at 8 kHz. 5° at 1 kHz
  would need a ~4 m aperture.
- **96 mics** matches commercial practice (gfai Mikado uses 96 in the same pattern). Mic count
  grows as (aperture/spacing)², so going much smaller in spacing costs ~400+ mics.
- **Underbrink spiral** gave the best sidelobe suppression of all patterns simulated
  (−24.4 dB @ 4 kHz), with circular symmetry.
- **Beamforming grid**: 2–3 points per HPBW is enough. Finer grids cost memory and time
  (the steering matrix dominates) without improving resolution; the 96-mic array needs a
  finer grid than the UMA-16's 0.5°–1°.

## Development phases

Each phase delivers a working end-to-end system ([PLAN.md](./PLAN.md)).

| Phase | Hardware | Goal | Status |
|---|---|---|---|
| 1 | Simulation | Validate algorithms and array geometry | Complete |
| 2 | ReSpeaker XVF3800 (4 mics, 90 mm) | First end-to-end pipeline: capture → beamform → overlay | Complete |
| 3 | miniDSP UMA-16 v2 (16 mics, 126 mm) | 2D (az × el) beamforming, calibration, touch UI | Complete |
| 4 | Custom PCBs (96 mics, 300 mm, FPGA) | Full-performance system | In progress |
| 5 | Phase 3/4 hardware | ML beamformer (PILOT / CRNN) vs CLEAN-SC | Not started |

### Phase 1 — simulation ([PHASE1.md](./PHASE1.md))
- Recommended array: Underbrink 12 × 8, α = 22°, 96 mics, 300 mm, 12.9 mm minimum spacing, alias-free to ~13 kHz.
- HPBW: >180° below 500 Hz, ~82° @ 1 kHz, ~37° @ 2 kHz, ~19° @ 4 kHz, ~9° @ 8 kHz.
- Algorithm choice: CLEAN-SC for real-time single-source display; MVDR for multiple or weak
  sources; MUSIC for closely spaced sources with known count; D&S for lowest latency.
  256 snapshots per CSM.
- Works well: industrial source hunting at 2–8 kHz, multi-source scenes, normal rooms,
  near field at 0.5–3 m. Not suited: below 1 kHz, very reverberant spaces (DRR < 3 dB),
  separating sources on the same bearing, MUSIC at SNR < 10 dB.

### Phase 2 — ReSpeaker smoke test ([PHASE2.md](./PHASE2.md))
- Raw mics are USB channels 2–5; gain imbalance up to ~3.6× motivated calibration.
- D&S, MVDR and CLEAN-SC agreed within 5°. Live script: `src/acoustic_camera_p2.py`.

### Phase 3 — UMA-16 ([PHASE3.md](./PHASE3.md))
- 16 channels at 48 kHz, 2D beamforming with full-frame overlay on webcam or Pi camera.
- Live script `src/acoustic_camera_p3.py` with a touch UI: frequency sliders and spectrum
  pan, energy threshold (auto/manual), zoom/recenter/dwell, power indicator, saved settings.
- Shared math in `src/beamforming.py` (pytest suite in `tests/`); `src/benchmark_algos.py`
  measures per-algorithm cost. A 1°/pt grid brings all four algorithms within 20 fps.
- Lesson: the UMA-16's usable window is only ~2–4 kHz, bounded by aperture below and
  spatial Nyquist above.

### Phase 4 — custom hardware ([PHASE4.md](./PHASE4.md))
96-mic Underbrink spiral, 12 arms × 8 mics, with PDM → CIC/FIR decimation in an FPGA and
data streamed to a host (96 ch × 48 kHz × 24 bit ≈ 110 Mbit/s, within 1 GbE). A Pi Camera
Module 3 Wide mounts through the centre of the array.

- **Single-FPGA design (primary).** One ALINX AC7200 module (Xilinx XC7A200T) runs all 96
  channels plus a GbE/UDP link. RTL is done and fits (43.5% LUTs for the pipeline, 48.1% with
  the GbE MAC and packetizer). Two boards:
  - `mic_array`: Ø320 mm, 96 Infineon IM72D128 mics, TCXO and clock tree, local 3.3 V LDOs.
  - `front_end`: AC7200 module, power regulation, DP83867 GbE PHY.
  - Joined by Panasonic P5KS board-to-board connectors (9 mm mated height).
  - Status: the current Phase 4 design; PCBs being laid out and prepared for fabrication. See
    [pcb/SCHEMATIC_NOTES.md](./pcb/SCHEMATIC_NOTES.md) for the current schematic, power and
    connector details.
- **Multi-FPGA alternate.** Four Cmod A7-35T cluster boards (24 mics each) feed a hub board
  over a shared-clock spoke bus; the hub relays to a Raspberry Pi 5 over USB. Breadboarded
  and laid out as PCBs, but the boards were never built.

### Phase 5 — ML
Needs real Phase 3/4 data to train and validate models.

## Landscape

- **Commercial products** ([PRODUCTS.md](./PRODUCTS.md)): 64–96 MEMS mics is the sweet spot
  for handheld far-field use; USB for portable, PoE for fixed installs; a co-located video
  camera at the array centre is universal.
- **Open-source projects** ([PROJECTS.md](./PROJECTS.md)): FPGA → GbE → GPU host is the
  established pattern above ~96 mics; cross-correlation calibration with GPU gradient descent
  (Ben Wang) recovers mic positions and speed of sound. This project uses **Acoular** (Python)
  for parts of its beamforming work.

## Open questions and TODOs (from the README)

- Match camera optics to the array's FOV and resolution.
- Partition processing between FPGA and host; choose final processing/display hardware.
- Packaging; low-noise audio front-end layout.
- Add-on arms to extend the aperture for better low-frequency resolution.
