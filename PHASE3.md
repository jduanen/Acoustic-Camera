# Phase 3 — UMA-16 v2 Intermediate Array Test

End-to-end pipeline validation with a 16-mic 4×4 URA. Goals: validate 2D beamforming, calibration,
and video overlay on real hardware at higher resolution than Phase 2, measure the resources required for each algorithm, and gain experience with practical application in the real-world environment.

Hardware: **miniDSP UMA-16 v2** (16 mics, 126 mm aperture, 48 kHz, driverless USB)
- Knowles SPH1668LM4H-1 MEMS mics, 65.5 dB SNR, 122 dB AOP
- XMOS Xcore200 PDM→PCM, UAC2, 16-ch 24-bit at up to 48 kHz
- USB device index 12; `UMA16v2: USB Audio (hw:4,0)`

---

## Array Geometry

| Parameter | Value | Notes |
|---|---|---|
| Mic count | 16 | 4×4 Uniform Rectangular Array |
| Grid spacing | 42 mm | All rows and columns |
| Aperture | 126 mm × 126 mm | 3 × 42 mm in each axis |
| Spatial Nyquist | ~4.1 kHz | c / (2 × 0.042) = 343/0.084 |
| Effective ceiling | ~3.7 kHz | 10% aliasing margin |
| Far-field distance | ~0.19 m @ 4 kHz | r_FF = 2D²/λ, D=0.126 m |
| Board dimensions | 132 × 202 × 18 mm | Total PCB; mic array = 132 × 132 mm |

Mic positions (center-referenced, USB channel → MIC label → (x, y) in mm):

| USB ch | MIC | x (mm) | y (mm) |
|---|---|---|---|
| 0 | MIC1 | −21 | −63 |
| 1 | MIC2 | −63 | −63 |
| 2 | MIC3 | −21 | −21 |
| 3 | MIC4 | −63 | −21 |
| 4 | MIC5 | −21 | +21 |
| 5 | MIC6 | −63 | +21 |
| 6 | MIC7 | −21 | +63 |
| 7 | MIC8 | −63 | +63 |
| 8 | MIC9 | +63 | +63 |
| 9 | MIC10 | +21 | +63 |
| 10 | MIC11 | +63 | +21 |
| 11 | MIC12 | +21 | +21 |
| 12 | MIC13 | +63 | −21 |
| 13 | MIC14 | +21 | −21 |
| 14 | MIC15 | +63 | −63 |
| 15 | MIC16 | +21 | −63 |

Channel ordering follows PDM L/R multiplexing pairs (each pair shares one data line).

**Coordinate system note (verified):** Manual Figure 1 is drawn from the sound-source side.
The beamformer and camera use the observer-side (camera-side) frame, which is x-mirrored.
Fix: `MIC_X = -_xy[:, 0]` (negate x). Confirmed on hardware; the cross-hair tracks correctly.

Expected HPBW vs frequency (horizontal axis, 126 mm aperture):

| Freq (Hz) | HPBW (approx) |
|---|---|
| 1000 | ~156° (near omni) |
| 2000 | ~78° |
| 3000 | ~52° |
| 3700 | ~42° (Nyquist ceiling) |

---

## Notebooks

### nb15 — Array Characterization (`notebooks/15_uma16_array.ipynb`)
*Offline simulation — no hardware required*

Analogous to nb12 for Phase 2. Establishes quantitative expectations:
- 4×4 URA geometry definition and visualization
- PSF / HPBW sweep 500 Hz - 4 kHz
- Spatial Nyquist verification (42 mm spacing → ~4.1 kHz)
- Algorithm comparison (D&S, MVDR, CLEAN-SC, MUSIC): with N=16, MVDR/MUSIC provide
  meaningful super-resolution benefit over D&S (vs. Phase 2 where N=4 showed no benefit)
- Snapshot convergence
- 2D PSF plots (azimuth × elevation)
- Comparison: Phase 2 (4-mic, 90 mm) vs Phase 3 (16-mic, 126 mm) vs Phase 4 (96-mic, 300 mm)

### nb15 Results

**Array Geometry (verified)**

| Parameter | Simulated value |
|---|---|
| Aperture | 126 mm × 126 mm |
| Spatial Nyquist | 4083 Hz |
| Far-field distance @ 4 kHz | 0.37 m |

**D&S: HPBW vs Frequency** (1D azimuth PSF, source at 30°, N_SNAP = 512)

| Freq (Hz) | HPBW |
|---|---|
| 500 | 180° (near-omni) |
| 1000 | 115.6° |
| 2000 | 73.2° |
| 3000 | 43.3° |
| 4000 | 31.6° |
| 4500 | 28.0° (above Nyquist — aliases present) |

Practical operating ceiling: **3000-3700 Hz** gives 43-37° HPBW with good localization
and no aliasing.

**Snapshot Convergence** (D&S / MVDR / MUSIC, 3000 Hz, SNR = 20 dB, 50 trials; mean DoA error in degrees)

| N_SNAP | D&S | MVDR | MUSIC |
|---|---|---|---|
| 4 | 0.86° | 0.97° | 0.85° |
| 8 | 0.53° | 0.74° | 0.53° |
| 16 | 0.43° | 0.64° | 0.43° |
| 32 | 0.25° | 0.32° | 0.25° |
| 64 | 0.16° | 0.17° | 0.16° |
| **128** | **0.11°** | **0.13°** | **0.11°** |
| 256 | 0.09° | 0.09° | 0.09° |
| 512 | 0.05° | 0.07° | 0.05° |

With N=16 mics, the CSM is full-rank at N_SNAP ≥ 16. **N_SNAP = 128** (the live script
default) gives sub-0.15° mean error at 20 dB SNR, which is well within one display pixel. Diminishing
returns beyond 256 snapshots. At this array size and SNR, MVDR and MUSIC offer no meaningful
accuracy advantage over D&S; the super-resolution benefit only appears for closely-spaced
two-source separation (see section 5 of nb15).

### nb16 — Audio Capture & Offline Beamforming (`notebooks/16_uma16_capture.ipynb`)
*Requires UMA-16 connected via USB*

Analogous to nb13:
1. Device discovery and channel verification
2. Record 5-second clip, known-angle source
3. Compute CSM, beamform, compare to nb15 predictions
4. Verify channel-to-mic mapping (tone from boresight)
5. 2D energy map plot

### nb16 Results

**Device**

| Parameter | Value |
|---|---|
| Device index | 12 |
| Name | `UMA16v2: USB Audio (hw:4,0)` |
| Channels | 16 |
| Sample rate | 48 kHz |
| Input latency | 8.7 ms (low) |

**Channel balance (ambient RMS)**

All 16 channels live and well-matched. Max/min RMS ratio: **1.22×** (ch14 loudest, ch13 quietest).
Far better than Phase 2 ReSpeaker (3.6×). The UMA-16's factory-matched MEMS mics need no calibration for gain.

**Beamforming at 3000 Hz** (233 Welch blocks, 2048-sample window, 50% overlap; off-axis source)

| Algorithm | Peak azimuth | Notes |
|---|---|---|
| D&S | −48.6° | |
| MVDR | −50.1° | |
| CLEAN-SC | −48.6° | |

All three agree within **1.5°** -- **PASS**.
With a single dominant source, CLEAN-SC converges with D&S (contrast with ambient run where CLEAN-SC diverged after subtracting the dominant source and finding residual noise).

The 2D peak was reported at Az=−45°, el=+5°. The Az is clipped at the ±45° grid boundary; the true source is ~3–5° beyond the edge. The El estimate (+5°) reflects the actual elevation offset of the source.

**Frequency Sweep: peak azimuth vs. frequency**

| Freq (Hz) | D&S peak | Notes |
|---|---|---|
| 500 | −6.2° | Near-omni; no directionality |
| 750 | −8.7° | Near-omni |
| 1000 | −8.4° | Near-omni |
| 1500 | −24.5° | Beginning to resolve source direction |
| 2000 | −20.9° | Resolving but wide beam (~73° HPBW) |
| 2500 | −42.0° | Locking in |
| 3000 | −48.6° | Stable — good localization |
| 3500 | −46.8° | Stable |
| 4000 | −20.6° | Near-Nyquist instability begins |
| 4500 | −15.6° | Above Nyquist — aliasing |
| 5000 | −31.6° | Above Nyquist — aliasing |

Matches nb15 predictions closely: near-omni below ~1.5 kHz, progressive locking from 1.5-2.5 kHz, stable localization 2.5-3.5 kHz, aliasing above the 4083 Hz spatial Nyquist. The practical operating window is **2.5–3.7 kHz**.

### nb17 — Calibration (`notebooks/17_uma16_calibration.ipynb`)
*Requires UMA-16 connected via USB*

Analogous to nb14. Cross-correlation gain+phase calibration.
Run while playing 1 kHz tone from boresight. Saves `test/UMA16/cal.npy`.

**N.B. Have to re-run this, too much background noise when calibrating**

### nb17 Results

**Measured Per-Channel Delays** (relative to ch0, 1 kHz boresight tone)

| ch | Delay (µs) | Samples | Gain (dB) |
|---|---|---|---|
| 0 | 0.0 | 0.0 | 0.00 |
| 1 | −41.7 | −2.0 | −0.75 |
| 2 | +62.5 | +3.0 | +1.91 |
| 3 | +62.5 | +3.0 | +0.15 |
| 4 | +125.0 | +6.0 | +2.10 |
| 5 | +145.8 | +7.0 | +1.13 |
| 6 | +229.2 | +11.0 | +2.60 |
| 7 | +250.0 | +12.0 | +1.13 |
| 8 | +208.3 | +10.0 | +3.62 |
| 9 | +208.3 | +10.0 | +3.49 |
| 10 | +145.8 | +7.0 | +3.50 |
| 11 | +125.0 | +6.0 | +3.27 |
| 12 | +104.2 | +5.0 | +2.64 |
| 13 | +83.3 | +4.0 | +2.60 |
| 14 | +83.3 | +4.0 | +0.86 |
| 15 | +41.7 | +2.0 | +0.67 |

Gain spread: **4.37 dB** (ch1 lowest at -0.75 dB, ch8 highest at +3.62 dB).

**DoA Before vs. After Calibration (boresight recording, true = 0°)**

| Algorithm | Uncalibrated | Calibrated | Change |
|---|---|---|---|
| D&S | +3.9° | +11.3° | -7.4° (worse) |
| MVDR | +2.4° | +4.6° | -2.2° (worse) |

Calibration was saved to `test/UMA16/cal.npy` but **made DoA worse on both algorithms**.

**==> Need to re-run calibration**

**Diagnosis**

The measured delays (up to +250 µs / 12 integer samples) are far too large and too systematic
to be hardware phase offsets — true mic-to-mic hardware delays are typically sub-sample
(< 20 µs). Instead, the delays encode the actual acoustic propagation delay from the source
to each mic, meaning the source was **not at true boresight** (or was in the near field).

The cross-correlation approach conflates two things:
- Hardware phase offsets (small, random, what we want to correct)
- Propagation delays from the source position (large, spatially structured, geometry-dependent)

Applying the resulting vector as a hardware correction steers the array *away* from boresight,
which is exactly what was observed (error grew from 3.9° to 11.3°).

The 4.37 dB **gain spread** is real and worth correcting independently; the UMA-16's
factory-matched MEMS mics should show < 1 dB spread under ideal conditions, so this likely
reflects near-field acoustic non-uniformity during the recording rather than hardware sensitivity
mismatch.

**For a valid phase calibration**: the boresight tone must be at **far field** (r > 2D²/λ ≈ 0.37 m
at 4 kHz; at 1 kHz r > 0.09 m is sufficient) and aimed precisely at 0° azimuth / 0° elevation.
With the 126 mm aperture, even a few degrees of source offset produce multi-sample differential
delays at 48 kHz that overwhelm the actual hardware offsets.

---

## Live Script (`src/acoustic_camera_p3.py`)

Real-time two-thread pipeline identical in structure to Phase 2, but with:
- 16 channels at 48 kHz (vs 4 channels at 16 kHz)
- 2D azimuth × elevation beamforming (vs 1D azimuth strip)
- Full-frame acoustic heatmap overlay with JET colormap (vs bottom-strip only)
- Cross-hair peak marker (az, el)
- Real-time frequency spectrum strip with interactive frequency range sliders

```bash
python src/acoustic_camera_p3.py                              # D&S, default range 500–4000 Hz
python src/acoustic_camera_p3.py --algo mvdr                  # MVDR beamformer
python src/acoustic_camera_p3.py --algo music --nsrc 2        # MUSIC, 2 sources
python src/acoustic_camera_p3.py --cal test/UMA16/cal.npy     # with calibration
python src/acoustic_camera_p3.py --alpha 0.7 --video 4        # higher opacity, camera 4
```

Key CLI options: `--algo {ds,mvdr,clean,music}`, `--snap N` (default 128),
`--smooth 0-1` (default 0.7), `--nsrc N` (MUSIC only), `--device idx`, `--video idx` (default 4),
`--cal path`, `--az_fov deg` (default 90), `--el_fov deg` (default 60), `--alpha 0-1` (default 0.5)

### Display Layout

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│   Webcam frame + 2D acoustic heatmap (JET colormap)     │
│   Green cross-hair at beamformer peak (az, el)          │
│   Status label: algo / freq / az / el / fps             │
│                                                         │
│ ░░░░░░░░░░ Spectrum strip (90 px) ░░░░░░░░░░░░░░░░░░░░ │
│ Green bars · White line = beamform freq · Blue = Nyquist│
│ 500  1k    2k    3k   4k   │ tick labels update live    │
├─────────────────────────────────────────────────────────┤
│ F lo: 500 Hz  ────●──────────────────────  (drag)       │
│ F hi: 4000 Hz ──────────────────────●────  (drag)       │
└─────────────────────────────────────────────────────────┘
```

### Heatmap Overlay

Per-frame percentile stretch (10th-100th percentile → full colormap range) with `COLORMAP_JET`.
The background collapses to blue; the acoustic hot spot appears red/yellow regardless of absolute
level. Opacity controlled by `--alpha` (default 0.5).

### Frequency Spectrum Strip

A 90 px strip immediately below the video frame shows the incoherent average power spectrum
(mean PSD across all 16 channels) computed from the most recent 2048-sample window:

- **Green bars**: 64 bands spanning the selected frequency range, normalized over a 40 dB window
- **White vertical line**: beamforming frequency (midpoint of the selected range)
- **Blue vertical line**: spatial Nyquist (~4.1 kHz); only shown when it falls within the range
- **Tick labels**: round-number frequency labels on the x-axis, spacing auto-selected to give 3-6 ticks

### Frequency range sliders

Two mouse-draggable sliders appended below the spectrum strip set the frequency range of interest:

| Slider | Control | Effect |
|---|---|---|
| **F lo** (green handle) | Left edge of display range | Lower bound in Hz (min 100 Hz) |
| **F hi** (orange handle) | Right edge of display range | Upper bound in Hz (min F lo + 100 Hz) |

The beamforming frequency is automatically set to the midpoint `(F lo + F hi) / 2` and updates
every frame. Drag either handle left/right; the heatmap, spectrum strip, and status label all
respond immediately.

**Workflow**: watch the spectrum strip for a peak below the blue Nyquist line, then drag the
sliders to bracket that peak. The beamformer auto-targets the center of the selected band.

---

## Dependencies

Same as Phase 2: `sounddevice`, `opencv-python`, `scipy`. No new packages required.

---

## Phase 3 Summary

*Populate after running hardware experiments.*

### Hardware findings

- USB device index 12; 16 channels; 48 kHz; `UMA16v2: USB Audio (hw:4,0)`
- All 16 USB channels are raw mic data; RMS levels balanced (~5e-5 ambient noise floor)
- Channel ordering: PDM L/R pairs per data line; mapping derived from manual Figure 1
- **x-axis mirror confirmed**: Figure 1 is drawn from sound-source side → negate MIC_X for camera frame; `MIC_X = -_xy[:, 0]`

### Pipeline validation

- Cross-hair tracks sound source correctly after x-axis fix -- **PASS**

### Calibration results

*Run nb17 with deliberate 1 kHz boresight tone.*

### Issues surfaced

- **Overlay color indistinct** with fixed 30 dB window + INFERNO: D&S sidelobes are within ~5–10 dB
  of the peak, so most pixels mapped to the bright end of the scale and looked uniform.
  Fix: per-frame percentile stretch (10th–100th percentile → full colormap range) + `COLORMAP_JET`.
  This ensures background collapses to blue and the hot spot always appears red/yellow regardless of
  absolute dynamic range.
