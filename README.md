# Acoustic-Camera
The design and implementation of a low-cost, high-performance, acoustic camera

**WIP**

An Acoustic Camera computes a power map of the sound scene that exists within a video camera's field of view and overlays it onto the video in real time.
Capturing, visualizing, and tracking the position of sound sources is useful in a variety of different use cases, including the detection and localization of sources of vibration, gas leaks, electrical breakdown, etc. 
Acoustic Cameras can also serve as a useful prosthetic device for people who have lost hearing in one ear and can no longer localize sound sources.

## Acoustic Camera Products Overview
[PRODUCTS](./PRODUCTS.md)

## Beamforming Projects
[PROJECTS](./PROJECTS.md)

## Technical Background
[BACKGROUND](./BACKGROUND.md)

## Microphone Array Configurations
[MIC_ARRAYS](./MIC_ARRAYS.md)

## Design Trade-offs
[TRADEOFFS](./TRADEOFFS.md)

## System Requirements

| Parameter | Value | Notes |
|---|---|---|
| Min Distance | ~0.5 m | Far-field criterion r > 2D²/λ; 2×0.3²/0.343 ≈ 0.5 m for 300 mm array at 1 kHz |
| Max Distance | ~10 m | Practical limit for compact array at 200 Hz |
| Resolution | ~8° @ 8 kHz, ~17° @ 4 kHz, ~66° @ 1 kHz | Scales as λ/D; HPBW ≈ 58° × λ/D for 300 mm aperture |
| FOV | ±45° H, ±30° V | Matched to co-located video camera field of view |
| Mic Array Diameter | ~300 mm | 96 mics in Underbrink spiral, ~21 mm min spacing (Nyquist at 8 kHz) |
| Frequency Range | 200 Hz – 8 kHz | Broadband; mic spacing ≤21 mm avoids spatial aliasing at 8 kHz |
| Environment | General-purpose | Indoor/outdoor, low-to-moderate reverberation, single/multiple sources |

## Phase 1 Simulation Findings

Full results and methodology: [PHASE1](./PHASE1.md)

### Recommended array configuration

**Underbrink H=12×8, α=22°** — 96 mics, 300 mm aperture, 12.9 mm min spacing

- Best side-lobe suppression of all patterns tested: −24.4 dB @ 4 kHz, −18.9 dB @ 8 kHz
- Alias-free to ~13 kHz (target ceiling: 8 kHz, 1.7× margin)
- Perfect circular symmetry — azimuth and elevation HPBW are identical at all frequencies

Alternative: H=8×12, α=35° (−14.4 dB MSL, simpler layout, more uniform spacing — good if PCB routing is constrained)

### Beamwidth vs frequency

| Frequency | HPBW | Practical meaning |
|---|---|---|
| 200–500 Hz | > 180° | Omnidirectional — no spatial selectivity |
| 1 kHz | ~82° | Barely directional; cannot resolve sources within ~80° |
| 2 kHz | ~37° | Minimum frequency for useful imaging |
| 4 kHz | ~19° | Clear maps; commercial acoustic cameras primarily operate here |
| 8 kHz | ~9° | High-resolution; closely-spaced sources resolvable |

### Algorithm selection

| Use case | Recommended |
|---|---|
| Real-time display, single dominant source | CLEAN-SC |
| Multiple sources or weak source detection | MVDR |
| Closely-spaced sources, source count known | MUSIC |
| Lowest latency / embedded compute | D&S |
| Near-field range + angle estimation | Focused D&S or MVDR/MUSIC with NF steering |

Recommended snapshot count: **N_SNAP = 256** (5.3 ms, 188 fps). All algorithms converge at N_SNAP = 16
(0.3 ms); 256 provides 16× margin while remaining below the ~10 ms perceptual latency threshold.

### Where this system works well

- **Mechanical/industrial source hunting at 2–8 kHz** — the primary sweet spot. Fan noise, gear mesh,
  bearing defect harmonics, structural resonances. MVDR/MUSIC resolve sources separated by 10–15° that
  D&S cannot.
- **Multi-source scenes** — MVDR and MUSIC maintain 100% resolution reliability down to DRR = −3 dB
  (reverb exceeding direct power). MUSIC handles sub-HPBW separations when source count is known.
- **Typical indoor rooms and labs (DRR ≥ 10 dB)** — 96 channels provide ~20 dB array gain; DoA error
  at typical office DRR is indistinguishable from anechoic (~0.035°). Mild reverberation is essentially free.
- **Near-field operation at 0.5–3 m** — spherical-wave steering recovers range to within ~10 cm and
  azimuth within grid quantization. Sub-centimetre range error at 1.5 m in simulation.
- **Outdoors or in treated spaces** — consistent strong performance; no special mitigation needed.

### Where this system is NOT well-suited

- **Below 1 kHz** — the array is omnidirectional at 200–500 Hz and barely directional at 1 kHz.
  A dedicated low-frequency array would need ~1.7 m aperture for 10° resolution at 1 kHz.
- **Highly reverberant industrial environments (DRR < 3 dB)** — concrete rooms, large metal enclosures,
  live rooms. Spatial pre-filtering (WPE) is needed before beamforming.
- **Same-azimuth, different-range source separation** — range resolution along a single bearing at
  4 kHz / 300 mm aperture is ~1 m. Co-azimutal sources closer than ~1 m merge into a single peak.
  A co-located depth camera is the practical solution.
- **Low SNR (< 10 dB) with MUSIC** — overcounting n_sources produces 93–100% false alarm rate below
  10 dB SNR. D&S and MVDR degrade more gracefully; use AIC/MDL model-order selection or conservative
  n_sources at low SNR.
- **High dynamic range (weak source near a strong source) with D&S or CLEAN-SC** — a −20 dB weak source
  25° from a strong source is reliably detected only by MVDR and MUSIC.
- **Sources beyond ~3–4 m** — range estimation degrades sharply beyond the Fraunhofer distance (2.1 m
  at 4 kHz). Far-field azimuth-only mode still works at long range; range information is unavailable.

## Target Design
[DESIGN](./DESIGN.md)

## Development Plan
[PLAN](./PLAN.md)

## Implementation Details
[IMPLEMENTATION](./IMPLEMENTATION.md)

---

# Notes

* Outstanding Questions
  - what camera/optics are needed to match the mic array's FOV and resolution?
  - ?

* Phase 1
  - Tasks
    * array geometry visualization
    * a D&S beamforming demo
    * Underbrink vs. other patterns comparison
      - H=12x8 has best side-lobe suppression
      - spiral angle matters, a=35deg and a=22deg improve beam pattern a lot
      - beamwidth is set by aperture only
        * at 1kHz and below, the beam width is greater than the FOV, so can't localize
          - is this a problem?
          - where's the knee of the curve when it comes to beam width vs. aperture at lower frequencies?
      - ?

* TODOs
  - match camera and mic array resolution
  - match audio input path dynamic range to ADCs
  - careful layout of audio front-end to ensure low noise
  - get ADCs that share common clock and (if multiple in a package) sample at the same point
  - look at creating add-on arms to increase the aperture to improve resolution at low frequencies


* Advantages of low SNR mic in beamforming applications
  - https://audioxpress.com/article/microphone-array-beamforming-with-optical-mems-microphones
  - a better SNR mic can do better than multiple lesser SNR mics
  - lower SNR mic means more compact array is possible
    * 42mm mic spacing for a BW of  4KHz requires 17dB gain @ 100Hz -> min mic SNR = 65dBA
    * 21mm mic spacing for a BW of  8KHz requires 22dB gain @ 100Hz -> min mic SNR = 70dBA
    *  7mm mic spacing for a BW of 24KHz requires 32dB gain @ 100Hz -> min mic SNR = 80dBA
* I2S PCM MEMS mics comparisons
  - I have, and tested
    * INMP441 (441 NEO447)
    * MSM261S4030H0 (Xiao Sense?)
  - Amazon
    * SPH0645LM4H-B (Knowles?)
    * MSM261S4030H0
  - others
    * InvenSense ICS-52000 -- not I2S, TDM
    * InvenSense ICS-43434
    * InvenSense ICS-43432
* reSpeaker
  - ST MP34DT01TR-M
    * 4x PDM Omni MEMS mics
    * 90mm diameter, 33mm inter-mic spacing
    * SNR: 61 dB
    * Sensitivity: -26 dBFS
    * Overload: 120 dBSPL
    * Max sample-rate: 16 kHz (limited by onboard XMOS firmware, not the mic hardware)
  - micro USB interface, USB 2.0 (UAC1.0)
  - Linux/macOS driverless; Windows requires driver

---

# Docs

* 000405.pdf: On the Design of a MEMS Microphone Array for a Mobile Beamforming Application
  - built a mic array for the back of a smartphone
  - simulated different mic array configurations
  - used a genetic algorithm to optimize the placement of mics on a spiral
  - used XMOS dev board and 16x mics
    * Infineon’s IM69D130 digital MEMS mic
* 000468.pdf: Optimal planar microphone array arrangements [2015]
  - simulated lots of array arrangements for Vogel's Spiral with different H and V values
  - showed Underbrink Spiral is best trade-off between beam width and side-lobe levels
* 0080015889.pdf
  - ?
* 20080015889.pdf: A Deconvolution Approach for the Mapping of Acoustic Sources (DAMAS) Determined from Phased Microphone Arrays [2004]
  - ?
* 2021_12_De+Lucia.pdf: Implementation of a low-cost acoustic camera using arrays of MEMS microphones [2020]
  - ?
* 254.pdf: Comparison of Different Beamforming-based Approaches for Sound Source Separation of Multiple Heavy Equipment at Construction Job Sites
  - ?
* 373.pdf: Acoular Workshop: Accessible and Reproducible Microphone Array Signal Processing with Python
  - history and overview of Acoular
* BeBeC-2016-S4.pdf: A Generic Approach to Synthesize Optimal Array Microphone Arrangements [2016]
  - same material as 000468.pdf
* BeBeC-2022-D06.pdf
  - ?
* BeBeC-2022-S07.pdf
  - ?
* bp2144.pdf
  - ?
* Chakravarthula_Seeing_With_Sound_Long-range_Acoustic_Beamforming_for_Multimodal_Scene_Understanding_CVPR_2023_paper.pdf [2023]
  - ?
* EN-AVT-287-04.pdf: Fundamentals of Acoustic Beamforming [?]
  - ?
* GB2438259A.pdf: Audio recording system utilising a logarithmic spiral array [2006]
  - UK Patent application
  - ?
* IN-Pflug-Krischker-AspectsOfTheUseOfMemsMicrophones-2017.pdf: Aspects of the Use of MEMS Microphones in Phased Array Systems [2017]
  - compared MEMS mics to electrec capsule mics in 2015 and again in 2017
    * while MEMS got better in 2017, they still were worse than electret mics
  - MEMS mic EIN levels were worse than analog electrec in 2017, but cheaper per channel
    * can use more mics and effectively reduce the noise
  - also MEMS mic freq response was not as good in 2017
    * can do digital equalization filters to improve this for MEMS mics
* 'Lecture 4 - Microphone Arrays.pdf'
* mccormack2017parametric.pdf: Parametric Acoustic Camera for Real-Time Sound Capture, Analysis and Tracking [2017]
  - ?
* microphone_array.pdf: Microphone Arrays : A Tutorial [2001]
  - ?
* Mon-1-2-5.pdf
* noise-and-vibration-image-brochure-2024-gfaitech.pdf
  - ?
* p453.pdf: Design and Calibration of a Small Aeroacoustic Beamformer [2010]
  - ?
* p5.pdf: A comparison of popular beamforming arrays [2013]
  - ?
* s13272-019-00383-4.pdf
  - ?
* 'Schumacher - 2022 - Evaluation of microphone array methods for aircraf.pdf'
  - ?
* Sound_Localization_and_Speech_Enhancement_Algorith.pdf
  - ?
* time-domain-beamforming-3d-micarray-doebler-heilmann-meyer-2008-bebec.pdf
  - ?
* TP-2007-345.pdf
  - ?
* Wang_2023_J._Phys. Conf._Ser._2479_012026.pdf: 
Research on multi-sound source localization performance
based on leaf-shaped microphone array [2022]
  - ?
  

