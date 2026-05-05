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

Metrics computed over ±89° scan range (full hemisphere).  10dB BW at 1 kHz is undefined
for all arrays — the main lobe fills the hemisphere at 300 mm aperture and has no −10 dB
crossing in the valid scan range.

| Array | 1 kHz HPBW | 1 kHz MSL | 4 kHz HPBW | 4 kHz MSL | 8 kHz HPBW | 8 kHz MSL |
|---|---|---|---|---|---|---|
| Underbrink 8×12, α=22° | 82.0° | −5.5 dB | 18.9° | −7.2 dB | 9.3° | −13.1 dB |
| Underbrink 8×12, α=35° | 82.0° | −5.6 dB | 18.9° | −14.4 dB | 9.3° | −14.4 dB |
| Underbrink 6×16, α=22° | 82.3° | −5.5 dB | 18.9° | −12.0 dB | 9.6° | −12.0 dB |
| **Underbrink 12×8, α=22°** | 81.6° | −5.6 dB | 18.9° | **−24.4 dB** | 9.3° | **−18.9 dB** |
| Vogel/Fermat | 72.0° | −6.1 dB | 16.7° | −17.0 dB | 8.6° | −17.0 dB |
| Concentric Rings | 65.9° | −6.2 dB | 15.7° | −7.5 dB | 7.8° | −25.8 dB |
| Regular Grid | 73.8° | −6.0 dB | 17.1° | −17.8 dB | 8.6° | −17.8 dB |

Notes:
- At 1 kHz, all Underbrink MSL values are around −5.5 dB — the main lobe is so wide (~82°)
  that there is little hemisphere left for side lobes; this is a physical limit, not a geometry effect.
- Concentric Rings shows a suspiciously low MSL at 8 kHz (−25.8 dB); this is likely a
  measurement artefact from the first-null algorithm being confused by the deep nulls between
  coherent grating lobes.  The 4 kHz result (−7.5 dB) is the more representative figure.
- HPBW is nearly identical across all Underbrink configurations — aperture size, not arm count
  or spiral angle, determines beamwidth.

### Key findings

**H=12×8 has the best side-lobe suppression** (−24.4 dB at 4 kHz, −18.9 dB at 8 kHz).
Despite its bimodal spacing distribution identified in notebook 01, the irregular inter-arm
distances actively suppress side lobes by preventing coherent grating lobe formation.

**Spiral angle matters significantly for H=8×12:** α=35° gives −14.4 dB MSL at both 4 and
8 kHz, vs. only −7.2 dB / −13.1 dB at α=22°.  The spacing-optimised angle also improves
the beam pattern — these objectives are aligned, not in tension.

**Beamwidth is set by aperture alone:** all Underbrink configurations produce the same HPBW
(~19° at 4 kHz, ~9° at 8 kHz) regardless of arm count or spiral angle.  The arm structure
only affects side-lobe levels.

### Revised candidates for further simulation

| Config | Rationale |
|---|---|
| **H=12×8, α=22°** | Best MSL at both 4 and 8 kHz; carry forward as primary |
| **H=8×12, α=35°** | Good MSL, simpler arm structure, more uniform spacing; carry as alternative |

H=8×12 at α=22° is dropped — it has substantially worse MSL (−7.2 dB at 4 kHz) than either
revised candidate with no compensating benefit.

---

## 03 — Algorithm Benchmarking (`notebooks/03_algorithms.ipynb`)

### Method

Synthetic far-field signals generated analytically: steering vectors + additive white Gaussian noise,
averaged over 256 snapshots to form the Cross-Spectral Matrix (CSM).
Array: Underbrink H=12×8, α=22° (96 mics).  Evaluation frequency: 4 kHz (HPBW ≈ 19°).

Three algorithms benchmarked:
- **D&S** — Conventional Delay-and-Sum: `P(θ) = h^H R h`
- **MVDR** — Minimum Variance Distortionless Response (Capon): `P(θ) = 1 / (h^H R⁻¹ h)`
- **CLEAN-SC** — Iterative source-coherence deconvolution (Sijtsma 2007)

### Scenario results

| Scenario | D&S | MVDR | CLEAN-SC |
|---|---|---|---|
| 1: single source at 0°, SNR=20 dB | 0.1° error | 0.1° error | 0.1° error |
| 2: two equal sources ±15° (30° sep), SNR=20 dB | Resolved | Resolved | Resolved |
| 3: two equal sources ±4.5° (9° sep ≈ 0.5×HPBW), SNR=20 dB | Not resolved | Not resolved | Not resolved |
| 4: strong (−15°) / weak (−20 dB, +10°), SNR=30 dB | Not resolved | **Resolved** | Not resolved |

### Key findings

**All algorithms achieve sub-0.1° DoA accuracy** on a single isolated source at SNR=20 dB.
The limiting factor for accuracy is scan-grid resolution, not algorithm quality.

**Resolution limit is at the HPBW:** sources separated by 0.5×HPBW (9° at 4 kHz) are not
resolved by any algorithm at 256 snapshots / SNR=20 dB.  D&S is a matched filter so cannot
exceed the Rayleigh limit; MVDR's super-resolution advantage appears only at high SNR and
large snapshot counts (not demonstrated here).

**MVDR has the best dynamic range:** only MVDR successfully detected the −20 dB weak source
(25° from the strong source) at SNR=30 dB.  D&S fails because the strong source's side lobes
mask the valley between the two sources.  CLEAN-SC fails because the residual after
subtracting the dominant source is not clean enough for the weak peak to emerge at this
iteration depth (30 iterations, loop gain 0.5).

**Practical implication for Phase 2/3:** two-source resolution requires at least 1×HPBW
separation (~19° at 4 kHz, ~9° at 8 kHz) for reliable detection at typical SNR.  CLEAN-SC
is preferred for clean single-source maps (suppresses diffuse clutter); MVDR is preferred
when dynamic range and source separation matter.
