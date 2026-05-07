# Phase 2 — ReSpeaker Smoke Test

End-to-end pipeline validation on real hardware. The goal to create a fully functional (not high performance) pipeline: audio capture → beamforming → energy map → video overlay.

Hardware: **ReSpeaker XVF3800 USB 4-Mic Array** (already in hand)
- 4-mic circular array, 90mm diameter, XMOS XVF3800 onboard DSP
- USB interface, driverless on Linux/macOS (UAC)
- Onboard: AEC, AGC, DoA, beamforming, VAD, noise suppression, de-reverberation
  * not planning on using these features, just use it as a simple mic array
- USB control: `python_control/xvf_host.py` (pyusb)

---

## Array Geometry

| Parameter | Value | Notes |
|---|---|---|
| Mic count | 4 | Circular, equal 90° spacing |
| Aperture | 90mm | vs. 300mm for Phase 4 target |
| Radius | ~45mm | Exact positions from `AEC_MIC_ARRAY_GEO` |
| Min chord spacing | ~63.6mm | 2×45×sin(45°); adjacent mics at 90° |
| Spatial Nyquist | ~2.7kHz | c/(2×0.0636); effective range ≤ ~2.5kHz |
| Far-field distance | ~0.19m @ 4kHz | r_FF = 2D²/λ; always far field in practice |
| Expected HPBW | ~3.3× Phase 4 | Scales as aperture ratio: 300/90 |

HPBW vs frequency (from nb12 simulation):

| Freq (Hz) | Phase 2 HPBW (4-mic) | Phase 4 HPBW (96-mic) |
|---|---|---|
| 500 | 180° (omni) | >180° |
| 1000 | 180° (omni) | 82° |
| 1500 | 135° | — |
| 2000 | 88° | 38° |
| 2500 | 67° | — |

At ≤1 kHz the array is omnidirectional. At 2–2.5 kHz there is enough directionality to be
useful for smoke-test purposes. This is acceptable — the goal is pipeline validation, not
spatial resolution.

**No near-field correction needed:** r_FF < 0.2m at all frequencies in the target range.
Far-field steering vectors are correct for all practical operating distances.

---

## Notebooks

Notebooks follow Phase 1 conventions: `make_nbXX.py` generator scripts, same beamformer function
signatures (`beamform_ds`, `beamform_mvdr`, `clean_sc`, `steering_matrix`), same plotting style.

### nb12 — Array Characterization (`notebooks/12_respeaker_array.ipynb`)
*Offline simulation — no hardware required*

Establishes quantitative expectations before any hardware experiments:
- 4-mic circular array geometry definition and visualization
- PSF/HPBW sweep 500Hz – 2.5kHz
- Spatial Nyquist verification
- Algorithm comparison (D&S, MVDR, CLEAN-SC, MUSIC) — note reduced benefit of MUSIC with N=4
  (noise subspace dimension = N−k = 3 at best; eigenvalue gap is small relative to noise)
- Snapshot count sweep: with N=4, CSM is theoretically full-rank at N_SNAP=4; convergence expected
  to be fast but accuracy limited by low spatial diversity
- Expected DoA accuracy and minimum resolvable source separation
- Comparison table: Phase 2 (4-mic, 90mm) vs Phase 4 (96-mic, 300mm) expected performance

### nb12 Results

**Spatial Nyquist:** 2695 Hz (chord = 63.6 mm). Effective ceiling ~2425 Hz (10% margin).

**HPBW:**

| Freq (Hz) | HPBW |
|---|---|
| ≤ 1000 | 180° (omni) |
| 1500 | 135° |
| 2000 | 88° |
| 2500 | 67° |

**Algorithm comparison:** D&S, MVDR, and MUSIC produce nearly identical DoA accuracy with N=4
mics. The super-resolution benefit of MVDR/MUSIC requires many more mics than signal subspace
dimensions; with N=4 there is too little diversity. D&S is as accurate as MVDR here.

**SNR sensitivity:** at SNR=20 dB, mean DoA error ≈ 0.2° for all algorithms. SNR floor for
reliable DoA (error < 1°) is approximately 15 dB — 5 dB higher than the 96-mic Phase 4 array,
consistent with ~6 dB array gain (10·log10(4)) vs ~20 dB for 96 mics.

**Snapshot convergence:**

| N_SNAP | D&S | MVDR | MUSIC |
|---|---|---|---|
| 4 | 1.33° | 2.38° | 1.33° |
| 16 | 0.71° | 0.71° | 0.71° |
| 64 | 0.28° | 0.28° | 0.28° |
| 256 | 0.22° | 0.21° | 0.22° |
| 512 | 0.12° | 0.11° | 0.12° |

Full-rank CSM at N_SNAP=4; convergence continues improving beyond that because each snapshot
adds noise-averaging benefit. Recommend N_SNAP=256 (same as Phase 1) for consistency.

**Spatial aliasing:** at regular 90° spacing, aliasing above Nyquist produces a single strong
grating lobe (vs many weak lobes for Underbrink). Stay below ~2400 Hz to avoid ambiguity.

---

### nb13 — Audio Capture & Offline Beamforming (`notebooks/13_respeaker_capture.ipynb`)
*Requires ReSpeaker connected via USB*

Validates the full audio capture → CSM → beamforming pipeline on real data:

1. **Device discovery** — `sounddevice.query_devices()`: find 4-channel input, print sample rate
   and latency. Document device index for live script.
2. **Raw mic output mode** — if needed, set `AEC_ASROUTONOFF=0` via USB control transfer to get
   one channel per mic (AEC-residual, not truly raw, but usable for beamforming).
3. **Record 5-second test clip** — single source (tone or clap) at a measured azimuth angle.
   Save to `test/ReSpeaker/capture_nb13.wav`.
4. **CSM computation** — STFT of 4-channel WAV, cross-spectral matrix at source frequency.
5. **Beamforming** — D&S and MVDR energy maps; compare to expected HPBW from nb12.
6. **DoA accuracy** — compare estimated vs measured source angle.
7. **Cross-check** — read onboard `DOA_VALUE` from device; compare to our estimate.
8. **Hardware issues** — document USB latency, clock drift, channel ordering, noise floor.

### nb13 Results

**Device:** index 12, `reSpeaker XVF3800 4-Mic Array: USB Audio (hw:4,0)`, 6 channels, 16 kHz, 23.9 ms latency

**Channel mapping (confirmed from SeeedStudio documentation + power level analysis):**

| USB ch | RMS (output.wav) | RMS (live) | Identity |
|---|---|---|---|
| 0 | 0.01599 | 0.02292 | Conference processed audio (AGC, ~6-10× higher power) |
| 1 | 0.00275 | 0.00301 | ASR processed audio |
| 2 | 0.00278 | 0.00191 | **Mic 0 raw** |
| 3 | 0.00260 | 0.00183 | **Mic 1 raw** |
| 4 | 0.00257 | 0.00191 | **Mic 2 raw** |
| 5 | 0.00000 | 0.00197 | **Mic 3 raw** |

**Action:** use USB channels 2–5 for beamforming. Notebooks and live script updated accordingly.

**CSM (channels 2–5, 1500 Hz, live recording):** diagonal = [0.000665, 0.000250, 0.000544, 0.000893] — mic1 (ch3) is ~3.6× lower than mic3; coherence matrix 0.734–0.949 for all off-diagonal pairs (higher than ambient-only prior run, consistent with a more coherent scene). Mic1 imbalance motivates nb14 calibration.

**Beamformer peaks (channels 2–5, 1500 Hz, ambient audio):**
- D&S: 2.7°, MVDR: 7.5°, CLEAN-SC: 2.7° — all near boresight (~0°), consistent with dominant frontal source. All three agree within 5° (PASS), well within the 135° HPBW at 1500 Hz.

**USB control interface (AEC_MIC_ARRAY_GEO, DOA_VALUE):** requires udev rule for non-root access. To enable:
```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="001a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-respeaker.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
```
Once applied, nb13 can be re-run to read actual mic XYZ positions from `AEC_MIC_ARRAY_GEO` and onboard DoA from `DOA_VALUE`.

---

### nb14 — Calibration (`notebooks/14_respeaker_calibration.ipynb`)
*Requires ReSpeaker connected via USB*

Implements cross-correlation-based gain and phase calibration:

1. **Reference recording** — 1 kHz sine tone from boresight (0°), 5 seconds.
2. **Delay estimation** — cross-correlate each mic pair with mic 0:
   `τ_n = argmax(|IFFT(conj(F_0) · F_n)|) / fs`
3. **Phase correction** — `φ_n = 2π·f·τ_n` at calibration frequency
4. **Gain estimation** — `g_n = RMS(mic_n) / RMS(mic_0)`
5. **Calibration vector** — `e = g · exp(jφ)` (shape: (4,) complex)
6. **Apply correction** — `R_cal = outer(1/e, 1/e*) ⊙ R_uncal`
7. **Validation** — DoA error before vs after on an off-axis test source.
8. **Save** — `np.save('test/ReSpeaker/cal.npy', e)`

Expected: calibration matters more with 4 mics than with 96 (less spatial averaging to suppress
per-mic errors). Per Phase 1 nb08: 96-mic spatial averaging suppresses mic errors by ~20 dB;
4-mic array provides only ~6 dB of spatial averaging → calibration has measurable benefit.

*Results will be documented below after nb14 runs.*

---

## Live Script (`src/acoustic_camera_p2.py`)

Real-time two-thread pipeline:
- **Audio thread**: `sounddevice.InputStream` (4-ch, 16 kHz) → `collections.deque`
- **Main thread**: accumulate sliding-window CSM → beamform → OpenCV heatmap overlay → display

Usage:
```
python src/acoustic_camera_p2.py --algo ds --freq 1000
python src/acoustic_camera_p2.py --algo mvdr --freq 2000 --cal test/ReSpeaker/cal.npy
```

Key CLI options: `--algo {ds,mvdr,clean}`, `--freq Hz`, `--snap N` (default 256),
`--device idx`, `--cal path`, `--fov deg` (default 180)

---

## Dependencies Added (Phase 2)

```
sounddevice      # USB audio capture
opencv-python    # video capture and energy map overlay
pyusb            # XVF3800 USB control (AEC_ASROUTONOFF configuration)
libusb-package   # pyusb backend
```

---

## Phase 2 Summary

*To be filled in after all three notebooks complete.*

### Hardware findings
*TBD*

### Pipeline validation
*TBD*

### Calibration results
*TBD*

### Issues surfaced
*TBD*

### What Phase 2 does NOT cover

- Multi-source resolution (4x mics, wide HPBW — not the goal of this phase)
- Frequency range above ~2.5kHz (spatial aliasing above Nyquist)
- Near-field range estimation (not needed; r_FF < 0.2 m)
- GPU acceleration (CPU is sufficient for 4-mic real-time pipeline)
- 2D azimuth × elevation maps (1D azimuth sweep is sufficient for smoke test)
- Production-quality GUI (command-line + OpenCV imshow is the deliverable)
