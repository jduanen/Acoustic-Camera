# Phase 3 — UMA-16 v2 Smoke Test

End-to-end pipeline validation with a 16-mic 4×4 URA. Goal: validate 2D beamforming, calibration,
and video overlay on real hardware at higher resolution than Phase 2.

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
Fix: `MIC_X = -_xy[:, 0]` (negate x). Confirmed on hardware — cross-hair tracks correctly.

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
- PSF / HPBW sweep 500 Hz – 4 kHz
- Spatial Nyquist verification (42 mm spacing → ~4.1 kHz)
- Algorithm comparison (D&S, MVDR, CLEAN-SC, MUSIC) — with N=16, MVDR/MUSIC provide
  meaningful super-resolution benefit over D&S (vs Phase 2 where N=4 showed no benefit)
- Snapshot convergence
- 2D PSF plots (azimuth × elevation)
- Comparison: Phase 2 (4-mic, 90 mm) vs Phase 3 (16-mic, 126 mm) vs Phase 4 (96-mic, 300 mm)

### nb15 Results

*Run nb15 and populate this section.*

### nb16 — Audio Capture & Offline Beamforming (`notebooks/16_uma16_capture.ipynb`)
*Requires UMA-16 connected via USB*

Analogous to nb13:
1. Device discovery and channel verification
2. Record 5-second clip, known-angle source
3. Compute CSM, beamform, compare to nb15 predictions
4. Verify channel-to-mic mapping (tone from boresight)
5. 2D energy map plot

### nb16 Results

*Run nb16 and populate this section.*

### nb17 — Calibration (`notebooks/17_uma16_calibration.ipynb`)
*Requires UMA-16 connected via USB*

Analogous to nb14. Cross-correlation gain+phase calibration.
Run while playing 1 kHz tone from boresight. Saves `test/UMA16/cal.npy`.

### nb17 Results

*Run nb17 and populate this section.*

---

## Live Script (`src/acoustic_camera_p3.py`)

Real-time two-thread pipeline identical in structure to Phase 2, but with:
- 16 channels at 48 kHz (vs 4 channels at 16 kHz)
- 2D azimuth × elevation beamforming (vs 1D azimuth strip)
- Full-frame acoustic overlay (vs bottom-strip only)
- Cross-hair peak marker (az, el)
- Real-time frequency spectrum strip at the bottom of the frame

```bash
python src/acoustic_camera_p3.py                              # D&S, 3000 Hz
python src/acoustic_camera_p3.py --algo mvdr --freq 3000      # MVDR, 3 kHz
python src/acoustic_camera_p3.py --algo music --nsrc 2        # MUSIC, 2 sources
python src/acoustic_camera_p3.py --cal test/UMA16/cal.npy     # with calibration
python src/acoustic_camera_p3.py --alpha 0.7 --video 4        # higher opacity, camera 4
```

Key CLI options: `--algo {ds,mvdr,clean,music}`, `--freq Hz` (default 3000), `--snap N` (default 128),
`--smooth 0-1` (default 0.7), `--nsrc N` (MUSIC only), `--device idx`, `--video idx` (default 4),
`--cal path`, `--az_fov deg` (default 90), `--el_fov deg` (default 60), `--alpha 0-1` (default 0.5)

### Frequency spectrum strip

A 90-pixel strip at the bottom of the frame shows the incoherent average power spectrum
(mean across all 16 channels) updated every frame:

- **Green bars** — per-band power, 64 bars spanning 0–6 kHz, normalized over a 40 dB window
- **White line** — current beamforming frequency (`--freq`)
- **Blue line** — spatial Nyquist (~4.1 kHz); above this the heatmap aliases

The strip uses a per-frame max-relative normalization (40 dB window) so it always fills the
display regardless of absolute level. Use it to pick an effective `--freq` — the beamformer
works best where the spectrum has a clear peak below the blue Nyquist line.

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

- Cross-hair tracks sound source correctly after x-axis fix — **PASS**

### Calibration results

*Run nb17 with deliberate 1 kHz boresight tone.*

### Issues surfaced

- **Overlay color indistinct** with fixed 30 dB window + INFERNO: D&S sidelobes are within ~5–10 dB
  of the peak, so most pixels mapped to the bright end of the scale and looked uniform.
  Fix: per-frame percentile stretch (10th–100th percentile → full colormap range) + `COLORMAP_JET`.
  This ensures background collapses to blue and the hot spot always appears red/yellow regardless of
  absolute dynamic range.
