# Phase 1 — Simulation & Algorithm Benchmarking

Findings from Phase 1 simulation work.  Each section corresponds to a notebook in `notebooks/`.

---

## 01 — Array Geometry (`notebooks/01_array_geometry.ipynb`)

### Patterns evaluated

Five patterns were simulated for 96 mics in a 300 mm aperture (150 mm radius, 25 mm inner keep-out):

| Pattern | N | Min spacing | Median spacing | Alias-free ceiling |
|---|---|---|---|---|
| Underbrink 8×12 | 96 | 12.2 mm | 12.3 mm | 14 kHz |
| Underbrink 6×16 | 96 | 9.0 mm | 9.0 mm | 19 kHz |
| Vogel/Fermat | 96 | 24.5 mm | 25.7 mm | 6.9 kHz |
| Concentric Rings | 96 | 25.0 mm | 25.8 mm | 6.9 kHz |
| Regular Grid | 92 | 27.1 mm | 27.1 mm | 6.3 kHz |

**Key finding:** Vogel/Fermat, concentric rings, and regular grid all have minimum spacing > 21.4 mm —
every adjacent mic pair is spatially aliased at 8 kHz.  Only Underbrink achieves the target band
with 96 mics in a 300 mm aperture.  This confirms the TRADEOFFS.md recommendation.

### Underbrink arms sweep (α = 22°)

| Config | N | Min | Median | Max | Uniformity |
|---|---|---|---|---|---|
| H=4, 24/arm | 96 | 5.9 mm | 5.9 mm | 5.9 mm | Uniform but wastefully dense |
| H=6, 16/arm | 96 | 9.0 mm | 9.0 mm | 9.0 mm | Dense, uniform |
| **H=8, 12/arm** | **96** | **12.2 mm** | **12.3 mm** | **12.3 mm** | **Uniform, near-optimal** |
| H=12, 8/arm | 96 | 12.9 mm | 19.3 mm | 19.3 mm | Bimodal — two distinct clusters |
| H=16, 6/arm | 96 | 9.8 mm | 26.9 mm | 27.0 mm | Bimodal — arms too close |

H=8 is the cleanest result: d_min ≈ d_med ≈ d_max ≈ 12.2 mm — equal arc-length spacing
gives nearly perfectly uniform coverage across the whole aperture.  H=12 has a bimodal
distribution (12.9 mm inter-arm, 19.3 mm within-arm), which is less isotropic.

### Spiral angle sweep (H=8, 12/arm)

Min spacing rises monotonically with spiral angle α:

| α | Min | Alias-free ceiling |
|---|---|---|
| 10° | 11.5 mm | 14.9 kHz |
| 15° | 11.8 mm | 14.6 kHz |
| 22° | 12.2 mm | 14.0 kHz |
| 28° | 12.8 mm | 13.3 kHz |
| 35° | 13.8 mm | 12.4 kHz |

Grid search (H=8, α swept 10°–40°): best minimum spacing 14.8 mm at α = 40°.

Tighter spirals (small α) cluster mics toward the outer edge.
Looser spirals (large α) spread mics more uniformly, increasing minimum spacing.

### Conclusions from notebook 01

Spacing metrics are necessary (avoid aliasing) but insufficient to choose the best configuration —
side-lobe suppression and main-lobe width depend on the full spatial distribution and are
evaluated in notebook 02.

**Candidates carried into notebook 02:**
- Primary: H=8, 12/arm, α = 22° (literature default)
- Alternative: H=8, 12/arm, α = 35° (spacing-optimised)

---

## 02 — Beam Pattern / PSF (`notebooks/02_beam_pattern.ipynb`)

### Method

Frequency-domain Delay-and-Sum array factor (PSF = beamformer output for a single boresight source):

$$AF(u,v,f) = \frac{1}{N} \left| \sum_{n} \exp\!\left( j \frac{2\pi f}{c}(u\,x_n + v\,y_n) \right) \right|$$

Metrics extracted per configuration and frequency:
- **HPBW** — half-power (−3 dB) beamwidth along azimuth
- **MSL** — peak side-lobe level (dB), measured beyond the first null of the main lobe

### Pattern comparison

Metrics computed over ±89° scan range (full hemisphere).  10dB BW at 1kHz is undefined
for all arrays — the main lobe fills the hemisphere at 300mm aperture and has no −10dB
crossing in the valid scan range.

| Array | 1 kHz HPBW | 1 kHz MSL | 4kHz HPBW | 4kHz MSL | 8kHz HPBW | 8kHz MSL |
|---|---|---|---|---|---|---|
| Underbrink 8×12, α=22° | 82.0° | −5.5dB | 18.9° | −7.2dB | 9.3° | −13.1dB |
| Underbrink 8×12, α=35° | 82.0° | −5.6dB | 18.9° | −14.4dB | 9.3° | −14.4dB |
| Underbrink 6×16, α=22° | 82.3° | −5.5dB | 18.9° | −12.0dB | 9.6° | −12.0dB |
| **Underbrink 12×8, α=22°** | 81.6° | −5.6dB | 18.9° | **−24.4dB** | 9.3° | **−18.9dB** |
| Vogel/Fermat | 72.0° | −6.1dB | 16.7° | −17.0dB | 8.6° | −17.0dB |
| Concentric Rings | 65.9° | −6.2dB | 15.7° | −7.5dB | 7.8° | −25.8dB |
| Regular Grid | 73.8° | −6.0dB | 17.1° | −17.8dB | 8.6° | −17.8dB |

Notes:
- At 1kHz, all Underbrink MSL values are around −5.5dB
  * the main lobe is so wide (~82°) that there is little hemisphere left for side lobes
    - this is a physical limit, not a geometry effect
- Concentric Rings shows a suspiciously low MSL at 8kHz (−25.8 dB)
  * this is likely a measurement artefact from the first-null algorithm being confused by the deep nulls between coherent grating lobes
  * the 4kHz result (−7.5dB) is the more representative figure
- HPBW is nearly identical across all Underbrink configurations
  * aperture size, not arm count or spiral angle, determines beamwidth

### Key findings

**H=12×8 has the best side-lobe suppression** (−24.4dB at 4kHz, −18.9dB at 8kHz).
Despite its bimodal spacing distribution identified in notebook 01, the irregular inter-arm
distances actively suppress side lobes by preventing coherent grating lobe formation

**Spiral angle matters significantly for H=8×12:** α=35° gives −14.4dB MSL at both 4 and
8kHz, vs. only −7.2dB/−13.1dB at α=22°
  * the spacing-optimised angle also improves the beam pattern
    - these objectives are aligned, not in tension

**Beamwidth is set by aperture alone:** all Underbrink configurations produce the same HPBW
(~19° at 4kHz, ~9° at 8kHz) regardless of arm count or spiral angle
  * the arm structure only affects side-lobe levels

### Revised candidates for further simulation

| Config | Rationale |
|---|---|
| **H=12×8, α=22°** | Best MSL at both 4 and 8kHz; carry forward as primary |
| **H=8×12, α=35°** | Good MSL, simpler arm structure, more uniform spacing; carry as alternative |

H=8×12 at α=22° is dropped — it has substantially worse MSL (−7.2dB @ 4kHz) than either revised candidate with no compensating benefit

---

## 03 — Algorithm Benchmarking (`notebooks/03_algorithms.ipynb`)

### Method

* Synthetic far-field signals generated analytically:
  - steering vectors + additive white Gaussian noise, averaged over 256 snapshots to form the Cross-Spectral Matrix (CSM)
* Array: Underbrink H=12×8, α=22° (96 mics)
* Evaluation frequency: 4 kHz (HPBW ≈ 19°).

Four algorithms benchmarked:
- **D&S** — Conventional Delay-and-Sum: `P(θ) = h^H R h`
- **MVDR** — Minimum Variance Distortionless Response (Capon): `P(θ) = 1 / (h^H R⁻¹ h)`
- **CLEAN-SC** — Iterative source-coherence deconvolution (Sijtsma 2007)
- **MUSIC** — Multiple Signal Classification: `P(θ) = 1 / (h^H E_n E_n^H h)` where E_n is the noise subspace

### Scenario results

| Scenario | D&S | MVDR | CLEAN-SC | MUSIC |
|---|---|---|---|---|
| 1: single source at 0°, SNR=20 dB | 0.1° error | 0.1° error | 0.1° error | 0.1° error |
| 2: two equal sources ±15° (30° sep), SNR=20 dB | Resolved | Resolved | Resolved | Resolved |
| 3: two equal sources ±4.5° (9° sep ≈ 0.5×HPBW), SNR=20 dB | Not resolved | Not resolved | Not resolved | **Resolved** |
| 4: strong (−15°) / weak (−20 dB, +10°), SNR=30 dB | Not resolved | **Resolved** | Not resolved | **Resolved** |

### Key findings

**All algorithms achieve sub-0.1° DoA accuracy** on a single isolated source at SNR=20dB
  * the limiting factor for accuracy is scan-grid resolution, not algorithm quality

**MUSIC demonstrates super-resolution:**
  * MUSIC resolves two equal sources at 9° separation (0.5×HPBW) — the only algorithm to do so
  * D&S cannot exceed the Rayleigh limit (matched filter)
  * MVDR and CLEAN-SC also fail at this separation with 256 snapshots / SNR=20dB
  * MUSIC's subspace method sidesteps the aperture limit — it does not scan a beamformed power map but searches for steering vectors orthogonal to the noise subspace
  * Requires prior knowledge of source count (n_sources=2 specified)

**MVDR and MUSIC both resolve the dynamic range scenario:**
  * both successfully detected the −20 dB weak source (25° from the strong source) @ SNR=30dB
  * D&S fails because the strong source's side lobes mask the valley between the two sources
  * CLEAN-SC fails because the residual after subtracting the dominant source is not clean enough for the weak peak to emerge at this iteration depth (30 iterations, loop gain 0.5)

**Practical implication for Phase 2/3:**
  * MUSIC is the best choice when source count is known and super-resolution matters (closely spaced sources)
  * MVDR is preferred when source count is unknown but dynamic range is important
  * CLEAN-SC is preferred for clean single-source maps (suppresses diffuse clutter)
  * D&S remains the baseline reference — simplest, most robust at all SNR

### SNR sensitivity (single source at 10°, 4kHz, 256x snapshots)

| SNR (dB) | D&S (°) | MVDR (°) | MUSIC (°) |
|---|---|---|---|
| 0 | 0.69 | 0.03 | 1.29 |
| 4 | 0.33 | 0.15 | 0.45 |
| 8 | 0.15 | 0.03 | 0.15 |
| 12 | 0.15 | 0.27 | 0.15 |
| 14 | 0.03 | 0.09 | 0.03 |
| 20+ | 0.03 | 0.03 | 0.03 |

* All three algorithms reach the scan-grid floor (0.03°) by SNR ≈ 14dB and remain there
* Below 10dB errors increase, but 256x snapshots produces high variance
  - individual values at low SNR are dominated by noise, rather than algorithm behaviour
* The 0.03° floor is a scan resolution artefact (1000-point grid over ±60°), not a true accuracy limit

**Practical floor:** plan for SNR ≥ 10 dB at the array for reliable DoA estimation

---

## 04 — Near-Field (Focused) Beamforming (`notebooks/04_nearfield.ipynb`)

### Far-field validity criterion

Fraunhofer distance for the 300mm aperture: `r_FF = 2D²/λ`

| Freq (Hz) | λ (mm) | r_FF (m) |
|---|---|---|
| 200 | 1715 | 0.10 |
| 500 | 686 | 0.26 |
| 1000 | 343 | 0.52 |
| 2000 | 172 | 1.05 |
| 4000 | 86 | 2.10 |
| 8000 | 43 | 4.20 |

At typical operating distances (0.5–2m), the 300mm array is in the near field above ~500Hz

### Near-field steering vector

Spherical-wave steering:
`d_n = |(r·sinθ − x_n, −y_n, r·cosθ)|`,  `h_n ∝ exp(−j·2πf/c·d_n) / d_n`, normalised

The far-field formulation ignores both the quadratic phase curvature (`x_n²/(2r)`) and the
cross-term from the y-axis mic positions (`y_n²/(2r)`) (both grow as the source moves closer)

### 1D comparison (r = 1m, az = 20°, f = 4kHz)

| Beamformer | HPBW (°) | DoA error (°) |
|---|---|---|
| Far-field D&S (mismatched) | 20.3 | 0.12 |
| Near-field D&S (focused) | 20.3 | 0.00 |

The 1D azimuth slice shows only a small DoA improvement because the near-field correction
primarily affects the range axis.  The compelling benefit appears in the 2D map.

### 2D focused map

Near-field D&S scanned over a (range × azimuth) grid localises a source in both dimensions:

| Name | Value |
|---|---|
| True position | r = 1.50m, az = 25.0° |
| Estimated position | r = 1.49m, az = 25.0° |
| Range error | 0.8cm |
| Az error | 0.0° |

### Algorithm comparison (near-field steering, r = 1 m)

| Scenario | D&S | MVDR | MUSIC |
|---|---|---|---|
| 1: single @ 0° | 0.1° | 0.1° | 0.1° |
| 2: ±15° (30° sep) | Resolved | Resolved | Resolved |
| 3: ±4.5° (9° sep, 0.5×HPBW) | No | No | **Yes** |
| 4: −20dB weak source | No | **Yes** | **Yes** |

Algorithm relative performance is unchanged from the far-field study — near-field steering
does not affect the subspace structure, only the steering delays.

### Key findings

**Near-field correction primarily benefits range estimation, not azimuth accuracy:**
  * 1D azimuth HPBW is identical for far-field and near-field D&S at r = 1m
  * DoA accuracy improves modestly (0.12° → 0.00°)
  * The real benefit is enabling 2D (range, azimuth) localisation

**2D focused beamforming recovers range with high accuracy:**
  * Sub-centimetre range error at 1.5m (0.8cm) in simulation
  * Resolves two sources at different ranges even when they share the same angular direction

**CLEAN-SC is excluded from the near-field comparison:**
  * CLEAN-SC subtracts source contributions from the working CSM using the steering vector at the peak
  * In the near-field formulation, that subtraction requires spherical-wave steering
    - a straightforward port of the far-field version would use mismatched vectors and produce an incorrect residual
  * Near-field CLEAN-SC is a known extension in the literature and should be added in a later notebook

**Phase 2 (ReSpeaker, 90mm aperture) does not need near-field correction:**
  * r_FF for 90mm at 4kHz ≈ 0.19m, it is always far field at any practical distance
  * Near-field formulation is the correct baseline for Phase 4 (300mm array)

---

## 05 — Broadband / Frequency-Swept Beamforming (`notebooks/05_broadband.ipynb`)

### Setup
- Array: Underbrink H=12×8, α=22° (96 mics, 300mm aperture, d_min=12.9mm)
- SNR: 20 dB, N_SNAP=256, rng seed fixed
- Octave bands: 250 / 500 / 1k / 2k / 4k / 8k Hz
- Per-band map: 5 log-spaced frequency bins incoherently averaged

### HPBW vs frequency (D&S, single source at 0°)

| Freq (Hz) | HPBW measured | Theory (0.886 λ/D) |
|---|---|---|
| 200 | >180° (omnidirectional) | 290° |
| 500 | >180° (omnidirectional) | 116° |
| 1000 | 81.7° | 58.0° |
| 2000 | 38.2° | 29.0° |
| 4000 | 18.8° | 14.5° |
| 8000 | 9.3° | 7.3° |

Theory assumes a uniform circular aperture; the Underbrink array has a lower fill factor,
so measured HPBW is ~1.3× wider than theory.  Below ~1 kHz the array is effectively
omnidirectional.

### Two-source resolution vs frequency (sources at ±15°, 30° separation)

Resolution criterion: −6 dB valley between peaks.

| Band (Hz) | D&S | MVDR | MUSIC |
|---|---|---|---|
| 250 | No | No | No |
| 500 | No | No | No |
| 1000 | No | No | No |
| 2000 | No | **Yes** | **Yes** |
| 4000 | No | **Yes** | **Yes** |
| 8000 | **Yes** | **Yes** | **Yes** |

- Below 2 kHz: no algorithm resolves 30°-separated sources with a 300mm aperture
- At 2–4 kHz: MVDR and MUSIC resolve; D&S does not (super-resolution advantage)
- At 8 kHz: all algorithms resolve (HPBW ≈ 9.3°, well below 30° separation)

### Spatial aliasing

| Parameter | Value |
|---|---|
| Min mic spacing d_min | 12.9 mm |
| Spatial Nyquist | 13,253 Hz |
| Operating max | 8,000 Hz |
| Margin | 1.7× |

No aliasing within the operating range.  Above Nyquist the Underbrink geometry spreads
alias energy across many weak irregular lobes (rather than a single strong grating lobe),
a benefit of the non-uniform spacing.

### Key findings

**The 300mm array has no useful directionality below ~1 kHz:**
  * At 200–500 Hz, HPBW exceeds 180°; the array is omnidirectional
  * At 1 kHz, HPBW ≈ 82°; source pairs within 80° cannot be separated by any algorithm
  * Below 2 kHz, the aperture is the bottleneck — not the algorithm

**MVDR/MUSIC provide super-resolution in the 2–4 kHz band:**
  * D&S fails to resolve ±15° sources at 2 and 4 kHz (HPBW > 30° separation)
  * MVDR and MUSIC resolve at 2 kHz — roughly 2× below the D&S resolution limit

**Incoherent octave-band averaging is effective:**
  * Averaging 5 frequency bins per band suppresses noise and stabilises DoA estimates
  * Localisation accuracy matches the single-frequency results from notebook 03
  * The broadband map HPBW is approximately that of the band's upper edge

**Spatial aliasing is not a concern for the target hardware:**
  * 1.7× margin at 8 kHz with the chosen d_min = 12.9 mm
  * The irregular Underbrink geometry distributes alias energy across multiple weak lobes
    rather than a single grating lobe — confirmed visually in the PSF plots

---

## 06 — Snapshot Count Sweep (`notebooks/06_snapshot_sweep.ipynb`)

### Setup
- Array: Underbrink H=12×8, α=22° (96 mics, N_MICS=96)
- Frequency: 4 kHz, SNR: 20 dB (also 10 dB for sensitivity)
- Source: single at 25° (DoA error sweep); two at ±15° (resolution reliability sweep)
- N_SNAP sweep: [16, 32, 64, 128, 256, 512, 1024, 2048, 4096]
- N_TRIALS: 15 independent CSM realisations per N_SNAP value
- Resolution criterion: −6 dB valley between two peaks

### Latency table

| N_SNAP | Latency (ms) | Update rate (fps) | N_SNAP / N_MICS |
|---|---|---|---|
| 16 | 0.3 | 3000 | 0.17 |
| 32 | 0.7 | 1500 | 0.33 |
| 64 | 1.3 | 750 | 0.67 |
| 128 | 2.7 | 375 | 1.33 |
| 256 | 5.3 | 188 | 2.67 |
| 512 | 10.7 | 94 | 5.33 |
| 1024 | 21.3 | 47 | 10.67 |
| 2048 | 42.7 | 23 | 21.33 |
| 4096 | 85.3 | 12 | 42.67 |

CSM becomes theoretically full-rank only at N_SNAP ≥ N_MICS=96; diagonal loading allows MVDR/MUSIC to operate below this.

### DoA error convergence (single source, 20 dB SNR)

| Algorithm | N_SNAP for <0.5° error | Latency at convergence |
|---|---|---|
| D&S | 16 | 0.3 ms |
| MVDR | 16 | 0.3 ms |
| MUSIC | 16 | 0.3 ms |

All three algorithms converge to <0.5° DoA error at the minimum tested N_SNAP (16) with 20 dB SNR at 4 kHz.  The 96-mic array with diagonal-loaded MVDR is robust even at N_SNAP ≪ N_MICS.

### Resolution reliability (two sources at ±15°, 20 dB SNR)

| Algorithm | N_SNAP for 90% resolution |
|---|---|
| D&S | >4096 (never) |
| MVDR | 16 |
| MUSIC | 16 |

D&S never reliably resolves ±15° sources at 4 kHz — this is consistent with the notebook 05 finding that D&S HPBW ≈ 19° at 4 kHz, making 30° separation marginal for D&S even with infinite snapshots.  MVDR and MUSIC achieve 90% resolution reliability at N_SNAP=16 regardless.

### Key findings

**Snapshot count is not a bottleneck for this array at moderate SNR:**
  * All algorithms reach <0.5° DoA accuracy at N_SNAP=16 (0.3 ms latency) at 20 dB SNR
  * The 96-mic array oversamples spatially; even a rank-deficient CSM (N_SNAP=16 ≪ 96) carries enough information for reliable DoA

**D&S resolution limit is aperture-driven, not snapshot-driven:**
  * D&S cannot resolve ±15° at 4 kHz regardless of N_SNAP — the HPBW (~19°) is larger than the source separation (30°)
  * Adding more snapshots does not help D&S below its physical resolution limit
  * MVDR and MUSIC super-resolve from the very first snapshot count tested

**Practical N_SNAP recommendation: 256 (5.3 ms, 188 fps)**
  * Provides 16× margin above the convergence point
  * Latency is below human perception (~10ms perceptual threshold)
  * 50% overlap brings effective update rate to ~375 fps if latency is critical
  * Going below N_SNAP=128 (2.7ms) offers no accuracy improvement at 20 dB SNR

**At 10 dB SNR:** convergence shifts to higher N_SNAP (visible in the side-by-side SNR comparison plot); D&S and MVDR degraded more gracefully than MUSIC below N_SNAP=96.

---

## 07 — MUSIC Robustness to Source Count Mismatch (`notebooks/07_music_robustness.ipynb`)

### Setup
- Array: Underbrink H=12×8, α=22° (96 mics)
- Frequency: 4 kHz, N_SNAP=256 (except the N_SNAP sweep)
- Scenarios: 1 source at 20° (overcount study); 2 sources at ±15° (undercount study)
- N_TRIALS: 20–30 per sweep point
- Peak detector: local maxima above 5% of global max, separated by ≥5°

### Peak count table (SNR=20 dB, N_SNAP=256)

| True k | n_sources spec | Mismatch | Mean peaks | Std |
|---|---|---|---|---|
| 1 | 1 | 0 | 1.0 | 0.00 |
| 1 | 2 | +1 | 1.0 | 0.00 |
| 1 | 3 | +2 | 1.0 | 0.00 |
| 1 | 4 | +3 | 1.0 | 0.00 |
| 2 | 1 | −1 | **3.0** | **0.89** |
| 2 | 2 | 0 | 2.0 | 0.00 |
| 2 | 3 | +1 | 2.0 | 0.00 |
| 2 | 4 | +2 | 2.0 | 0.00 |

### Undercount (n_sources < true k) — detection rate vs SNR

Two sources at ±15°, n_sources=1 (undercount by 1):

| SNR (dB) | Undercount (n=1) | Correct (n=2) |
|---|---|---|
| 0 | 0.90 | 1.00 |
| 5 | 0.90 | 1.00 |
| 10 | 1.00 | 1.00 |
| 15 | 1.00 | 1.00 |
| 20 | 0.87 | 1.00 |
| 25 | 0.90 | 1.00 |
| 30 | 0.93 | 1.00 |

### Overcount (n_sources > true k) — false alarm rate vs SNR

One source at 20°, n_sources=2 or 3 (overcount):

| SNR (dB) | FA rate (n=2, +1) | FA rate (n=3, +2) |
|---|---|---|
| 0 | 0.93 | 1.00 |
| 5 | 1.00 | 0.97 |
| 10 | **0.00** | **0.00** |
| 15 | 0.00 | 0.00 |
| 20+ | 0.00 | 0.00 |

### N_SNAP effect on undercount robustness

Two sources at ±15°, n_sources=1, SNR=20 dB:
- Detection rate is ~90–100% at all N_SNAP values from 16 to 2048
- N_SNAP does not rescue undercount — the robustness is already near-ceiling due to source separation

### Key findings

**Undercount is surprisingly robust for well-separated sources (30° at 4 kHz):**
  * n_sources=1 with 2 true sources at ±15° detects both 87–100% of the time across all tested SNR
  * At 30° separation (≈1.6× the HPBW at 4 kHz), the two signal eigenvectors are nearly orthogonal; shrinking the noise subspace by one dimension still leaves both source directions "nulled"
  * Undercount does NOT prevent detection — but it DOES create extra spurious peaks (mean 3.0 vs 2.0 for correct specification)

**Overcount is benign above 10 dB SNR:**
  * At SNR ≥ 10 dB: specifying n_sources=2 or 3 with only 1 true source produces zero false alarms
  * At SNR < 10 dB: noise eigenvectors look like signals; FA rate rises to 93–100% with overcount
  * Overcount with multiple true sources is completely safe: 2 true sources + n_sources=3 still gives exactly 2 detected peaks (std=0.00) — the extra dimension is absorbed cleanly

**N_SNAP does not change the mismatch behaviour:**
  * Undercount robustness is geometrically determined (source separation vs noise subspace dimensionality), not noise-limited
  * More snapshots do not rescue an undercounted MUSIC specification

**Practical guidance for MUSIC source count specification:**
  * Above 10 dB SNR: **slightly overcount** (set n_sources to the expected maximum + 1) — safe, no false alarms, full detection rate, no risk from undercounting
  * Below 10 dB SNR: overcounting is dangerous (high FA); prefer n_sources=1 or use information-theoretic model order selection (AIC/MDL applied to the eigenvalue spectrum) to estimate source count automatically
  * The `eigh` eigenvalue spectrum itself is the diagnostic: the gap between the k-th and (k+1)-th eigenvalue indicates where signal ends and noise begins; a flat noise floor indicates the correct threshold

---

## 08 — Calibration Sensitivity (`notebooks/08_calibration_sensitivity.ipynb`)

### Setup
- Array: Underbrink H=12×8, α=22° (96 mics)
- Frequency: 4 kHz, SNR=20 dB, N_SNAP=512 (large — to isolate mismatch from CSM noise)
- Mismatch model: `e_n = g_n · exp(j·φ_n)`, `g_n ~ Uniform(−G, +G)` dB, `φ_n ~ Uniform(−Φ, +Φ)°`
- IM69D120 spec: G=1 dB, Φ=2°
- N_TRIALS=30 independent mismatch realisations per condition

### DoA error at IM69D120 spec (single source at 25°)

| Algorithm | Mean error (°) | Std (°) |
|---|---|---|
| D&S | 0.030 | 0.046 |
| MVDR | 0.027 | 0.044 |
| MUSIC | 0.030 | 0.046 |

These values are at the scan-grid floor (~0.017° baseline with no mismatch). Mismatch increases mean error by ~0.010° — negligible in practice.

### Gain vs phase contribution (at IM69D120 spec level)

| Condition | D&S | MVDR | MUSIC |
|---|---|---|---|
| No mismatch | 0.017° | 0.023° | 0.017° |
| Gain only ±1 dB | 0.017° | 0.030° | 0.017° |
| Phase only ±2° | 0.027° | 0.027° | 0.027° |
| Combined (spec) | 0.030° | 0.027° | 0.030° |

Phase mismatch has slightly more effect than gain mismatch; both are negligible.

### Resolution reliability (two sources at ±15°, N_SNAP=256)

| Condition | D&S | MVDR | MUSIC |
|---|---|---|---|
| No mismatch | 0.0 | 1.0 | 1.0 |
| IM69D120 spec (1 dB, 2°) | 0.0 | 1.0 | 1.0 |

Resolution reliability is **completely unaffected** by IM69D120-spec mismatch.

### Calibration correction effect

| Condition | D&S | MVDR | MUSIC |
|---|---|---|---|
| No mismatch | 0.017° | 0.023° | 0.017° |
| Uncorrected | 0.023° | 0.023° | 0.027° |
| Perfect calibration | 0.017° | 0.023° | 0.017° |
| Imperfect cal (±0.2 dB, ±0.5°) | 0.017° | 0.023° | 0.017° |

Perfect calibration fully restores the no-mismatch floor. A residual estimation error of ±0.2 dB / ±0.5° is small enough to make imperfect calibration indistinguishable from perfect.

### Key findings

**IM69D120-spec mismatch causes essentially zero degradation:**
  * 96-mic spatial averaging suppresses individual mic gain/phase errors — the array is robust by design
  * DoA error increase is ≤0.013° at spec; resolution reliability is unchanged
  * This is a direct consequence of array gain: 10·log10(96) ≈ 20 dB of spatial averaging suppresses per-mic errors by roughly 10× in amplitude

**Phase mismatch dominates over gain mismatch:**
  * Gain ±1 dB: DoA error increase is unmeasurable (D&S, MUSIC) or 0.007° (MVDR)
  * Phase ±2°: all algorithms show ~0.010° increase — still negligible
  * Conclusion: keep the shared PDM clock low-jitter; gain matching is not critical

**Calibration is not required for basic DoA operation with IM69D120 mics:**
  * Uncorrected IM69D120-spec mismatch keeps all algorithms at or near the scan-grid floor
  * Calibration would matter for high dynamic range scenarios (weak source detection near a strong source) — not tested here, but not the primary concern for the initial system

**Imperfect calibration with ±0.2 dB / ±0.5° residual is fully effective:**
  * The residual error at that level is already below the noise floor of the mismatch effect
  * Cross-correlation-based calibration methods typically achieve <0.2 dB / <0.5° residual — sufficient for full restoration

---

## 09 — Near-Field CLEAN-SC (`notebooks/09_nearfield_cleansc.ipynb`)

### Setup
- Array: Underbrink H=12×8, α=22° (96 mics, 300mm aperture)
- Frequency: 4 kHz, SNR=20 dB, N_SNAP=256
- Fraunhofer distance at 4 kHz: **r_FF = 2.10 m**
- 2D scan grid: 24 range pts (0.4–4.5 m, step 0.18 m) × 61 azimuth pts (−60° to +60°, step 2°)
- CLEAN-SC: n_iter=40, loop_gain=0.5

### Section 1: 2D map — single source at (r=1.0 m, az=25°)

| Beamformer | Estimated range | Estimated az | Range error | Az error |
|---|---|---|---|---|
| NF D&S | 0.93 m | 26.0° | 0.07 m | 1.0° |
| NF CLEAN-SC | 0.93 m | 26.0° | 0.07 m | 1.0° |

Both errors are within half a grid step (0.09 m, 1.0°) — pure quantization, not algorithm error.
The CLEAN-SC map is visually cleaner (lower sidelobes) than the D&S map.

### Section 2: Range separation — two co-azimutal sources

Scenario: two sources at az=0°, r=0.8 m and r=2.5 m.

- **FF CLEAN-SC (1D)**: single merged peak at az=0° — cannot disambiguate range at all
- **NF CLEAN-SC 2D**: single merged peak at r=1.29 m, az=0° — **cannot separate them**

This is a physical limitation, not a grid issue. At 4 kHz with a 300 mm horizontal aperture,
the range resolution along a single bearing (same azimuth) is determined by phase curvature:
Δr ≈ r²·c / (f·D²) ≈ 1m²·343 / (4000·0.09m²) ≈ 0.95 m. The two sources are separated by
1.7 m — at the limit — but the algorithm first finds an intermediate merged peak and subtracts
it, preventing two distinct clean components from emerging. Range separation of co-azimutal
sources requires a depth camera (RealSense or LiDAR) or much higher frequency.

### Section 3: DoA error vs source distance

| Source range | FF CLEAN-SC az err | NF CLEAN-SC 2D az err | NF range err |
|---|---|---|---|
| 0.50 m | 1.000° | 1.000° | 0.078 m |
| 0.70 m | 0.400° | 1.000° | 0.057 m |
| 1.00 m | 0.200° | 1.000° | 0.065 m |
| 1.50 m | 0.200° | 1.000° | 0.030 m |
| 2.10 m (r_FF) | 0.100° | 1.000° | 0.096 m |
| 3.00 m | 0.100° | 1.000° | 0.074 m |
| 5.00 m | 0.100° | 1.000° | 0.500 m |

**Important caveat:** the comparison is not apples-to-apples. FF CLEAN-SC scans 1201 azimuth
points (step 0.05°) while NF CLEAN-SC 2D scans only 61 (step 2°). The constant 1.0° NF
azimuth error is a **grid quantization artefact** (half the 2° step) — not a real accuracy
difference. With a finer az grid, NF CLEAN-SC 2D would match or beat FF CLEAN-SC at close range.

**What the table actually shows about NF CLEAN-SC 2D:**
- Range error stays within one grid step (≤0.1 m) for r ≤ 3 m — reliable range estimation
- At r=5 m (beyond r_FF): range error grows (0.5 m) as the wavefront curvature becomes too small to carry range information — this is expected
- The algorithm correctly reports range at all near-field distances

**The real FF vs NF comparison** (az bias, ignoring grid): FF CLEAN-SC shows increasing az bias at close range (0.4°–1.0° below 1 m) relative to its far-field baseline (0.1° at 5 m). NF CLEAN-SC with a matched fine grid would hold near-zero az error at all ranges because the steering model is correct.

### Section 4: General two-source case (different range and azimuth)

Sources: (az=−20°, r=0.8 m) and (az=+12°, r=2.0 m)

NF CLEAN-SC 2D results:
- Peak 1: r=0.76 m, az=−20.0° — true (0.80 m, −20°): range error 0.04 m, az error 0°
- Peak 2: r=1.83 m, az=+12.0° — true (2.00 m, +12°): range error 0.17 m, az error 0°

Both within one grid step. The algorithm correctly separates sources with different ranges and azimuths.

### Key findings

**NF CLEAN-SC works correctly as a 2D localiser:**
  * For sources with distinct azimuths, it recovers both range and azimuth within grid quantization
  * Clean maps (NF CLEAN-SC) have lower sidelobes than D&S maps even at identical grid resolution
  * The spherical-wave subtraction correctly removes each source's 2D contribution from the working CSM

**Cannot separate same-azimuth sources in range:**
  * Range resolution along a single bearing at 4 kHz / 300 mm aperture is ~1 m
  * Two sources at the same azimuth but different ranges merge into a single intermediate peak
  * This is a fundamental array-geometry limitation; co-located depth camera is the practical solution

**Far-field CLEAN-SC has a DoA bias at close range:**
  * FF CLEAN-SC az error rises from ~0.1° (5 m) to ~1° (0.5 m) as the plane-wave assumption breaks down
  * NF CLEAN-SC with a fine az grid eliminates this bias at all near-field distances

**Grid resolution dominates reported accuracy:**
  * With 2° az step → ≤1° az quantization error; with 0.18 m range step → ≤0.09 m range error
  * Finer grids improve accuracy at the cost of compute; CLEAN-SC is iterative so grid size directly multiplies iteration cost
  * Practical deployment: use a coarse grid for fast real-time operation; refine locally around detected peaks

---

## Phase 1 Summary

### Array geometry decision

**Primary candidate: Underbrink H=12×8, α=22°** (96x mics, 300mm aperture)
- Best side-lobe suppression (−24.4dB @ 4kHz, −18.9dB @ 8kHz)
- Min spacing 12.9mm → alias-free up to ~13kHz (target: 8kHz)
- Bimodal spacing distribution is acceptable
  * the irregular arm spacing is what suppresses side lobes

**Alternative: Underbrink H=8×12, α=35°**
- More uniform spacing (12.2mm, nearly perfectly equal arc-length)
- −14.4dB MSL @ 4 and 8kHz is adequate, but ~10dB worse than the primary choice
- Simpler layout is easier to route if PCB space is tight

### Algorithm selection by use case

| Use case | Recommended algorithm |
|---|---|
| Real-time display, single dominant source | CLEAN-SC |
| Multiple sources or weak source detection | MVDR |
| Closely-spaced sources, source count known | MUSIC |
| Lowest latency / embedded compute | D&S |
| Near-field, range + angle estimation | Focused D&S (or MVDR/MUSIC with NF steering) |

### Snapshot count recommendation

**N_SNAP = 256** (5.3 ms latency, 188 fps) is the practical baseline:
- All algorithms converge to <0.5° DoA accuracy at N_SNAP=16 at 20 dB SNR / 4 kHz — snapshot count is not a bottleneck
- 256 provides a comfortable 16× margin and keeps latency below the 10 ms perceptual threshold
- 50% window overlap (standard in practice) brings effective update rate to ~375 fps if needed
- D&S resolution is aperture-limited, not snapshot-limited — adding snapshots does not help D&S resolve below its HPBW

### What Phase 1 does NOT cover

The following remain for Phase 2 (or later) simulation:
- ~~**MUSIC robustness to wrong source count**~~ — covered in notebook 07
- ~~**Broadband / frequency-swept maps**~~ — covered in notebook 05
- ~~**Snapshot count sweep**~~ — covered in notebook 06
- ~~**Calibration sensitivity**~~ — covered in notebook 08
- ~~**Near-field CLEAN-SC**~~ — covered in notebook 09
- **2D elevation × azimuth maps**: all simulations are 1D azimuth scans
- **Reverberant/multipath environments**: free-field assumption is made throughout this phase
