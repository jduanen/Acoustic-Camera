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

| Array | 4 kHz HPBW | 4 kHz MSL | 8 kHz HPBW | 8 kHz MSL |
|---|---|---|---|---|
| Underbrink 8×12, α=22° | 19.0° | −7.1 dB | 9.4° | −17.6 dB |
| Underbrink 8×12, α=35° | 19.0° | **−17.1 dB** | 9.4° | −14.4 dB |
| Underbrink 6×16, α=22° | 19.0° | −12.0 dB | 9.4° | −12.0 dB |
| **Underbrink 12×8, α=22°** | 18.7° | **−24.4 dB** | 9.4° | **−19.4 dB** |
| Vogel/Fermat | 16.8° | −17.0 dB | 8.4° | −17.0 dB |
| Concentric Rings | 15.6° | −7.5 dB | 7.9° | −7.7 dB |
| Regular Grid | 17.3° | −17.8 dB | 8.6° | −17.8 dB |

Notes:
- 1 kHz MSL is undefined for all Underbrink configs — the main lobe (~82°) fills the ±60°
  scan window and no first null is visible.  This is a physical limit of a 300 mm aperture at 1 kHz,
  not a measurement artefact.
- Concentric Rings is consistently poor (−7 to −8 dB MSL) due to coherent grating lobes
  from the regular angular spacing.
- HPBW is nearly identical across all Underbrink configurations — aperture size, not arm count
  or spiral angle, determines beamwidth.

### Key findings

**H=12×8 has the best side-lobe suppression** (−24.4 dB at 4 kHz, −19.4 dB at 8 kHz).
Despite its bimodal spacing distribution identified in notebook 01, the irregular inter-arm
distances actively suppress side lobes by preventing coherent grating lobe formation.

**Spiral angle matters significantly for H=8×12:** α=35° gives −17.1 dB MSL at 4 kHz vs.
only −7.1 dB at α=22°.  The spacing-optimised angle also improves the beam pattern —
these objectives are aligned, not in tension.

**Beamwidth is set by aperture alone:** all Underbrink configurations produce the same HPBW
(~19° at 4 kHz, ~9° at 8 kHz) regardless of arm count or spiral angle.  The arm structure
only affects side-lobe levels.

### Revised candidates for further simulation

| Config | Rationale |
|---|---|
| **H=12×8, α=22°** | Best MSL at both 4 and 8 kHz; carry forward as primary |
| **H=8×12, α=35°** | Good MSL, simpler arm structure, more uniform spacing; carry as alternative |

H=8×12 at α=22° is dropped — it has substantially worse MSL (−7.1 dB at 4 kHz) than either
revised candidate with no compensating benefit.

---

## 03 — Algorithm Benchmarking (`notebooks/03_algorithms.ipynb`)

*Planned.*
