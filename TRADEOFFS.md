# Acoustic Camera Design Tradeoffs

## General Concepts

* key characteristics
  - beam width (measured at 10dB from peak of main lobe)
  - side-lobe levels (measured as difference between main lobe peak and highest side-lobe)
* design objectives: narrow beam, low side-lobe levels
  - the larger number of mics, the lower the side-lobe levels
  - the larger the effective array size, the narrower the beam
* mic array characteristics
  - aperture response
  - spatial extent of array
  - type and number of transducers
  - transducer physical orientation and arrangement
* general array characteristics
  - larger mic arrays
    * larger arrays prioritize low-frequency captures and wide-area coverage
    * pros
      - improved spatial resolution
      - enhanced low-frequency (<1KHz) detection
        * larger mic spacing captures longer wavelengths
        * good for industrial noise
      - higher dynamic range
      - better noise suppression
      - increased localization accuracy
      - better in open/outdoor environments
        * due to better low-frequency directionality
    * cons
      - cumbersome, less portable, larger, heavier
      - more complex calibration
      - limits to effectiveness in broadband applications
        * due to problems with high-frequency grating lobes
  - more mics in the array
    * arrays with more mics optimize high-frequency detail and adaptive beamforming
      - but comes with increased computation and power costs
    * pros
      - enhances spatial resolution and high-frequency accuracy
        * reduced spatial aliasing
        * sharper beamforming and better noise rejection >=5KHz
      - better SNR
        * through constructive interference
      - works better in indoor/reverberant spaces
        * leverages phase differences for more precision source localization
    * cons
      - low-freq performance limited by mic spacing in smaller array size
      - more processing power and interconnect required
      - more power required
      - higher cost
  - frequency response and directionality
    * closer mic spacing improves high-frequency capture ability
      - provides wider BW before spatial aliasing occurs
        * spatial aliasing is an artifact of undersampling
    * wider spacing of mics enhances directionality at low freq
      - makes array more effective at blocking or localizing low-frequency sounds
  - spatial aliasing
    * if mics are spaced too far apart, high-frequency sounds can cause spatial aliasing
      - array can't distinguish between real and ghost sources
    * mic spacing should generally be less than half the wavelength of the highest freq of interest
  - SNR
    * optimal spacing can maximize SNR improvements
      - too wide or too narrow spacing can reduce an array's effectiveness across different frequencies
    * BW
      - narrow spacing provides broader usable freq range
      - wide spacing limits the array's BW -- only optimal for narrow freq range
  - mic spacing balances freq response, directionality, and spatial aliasing
* sound wavelength
  - a function of pressure and temperature
  - low freq sound (20-250Hz) wavelengths range from 18m to 1.4m
  - in human speech, low freq is where the energy is and higher freqs are where the information is located
* localizing sound in the 100-1000Hz range
  - ideal mic spacing: 170mm
  - spacing of 100-200mm balances aliasing risk and low-frequency coverage
* a dense array with 30+ mics and 100mm spacing provides good broadband coverage
  - but comes with cost in processing power and complexity
* a sparse array with 500mm spacing does well at low freq but fails above ~350Hz
* industrial leak detection centers around 500Hz
* broadband acoustic cameras typically use 100-150mm spacing and cover 200-1000Hz
  - they give up low-frequency precision for aliasing-free operation
* appears like many commercial products use arrays with spiral arms
  - many are 20-40cm in diameter
* processing
  - target algorithm progression: Delay-and-Sum → MVDR → CLEAN-SC → Functional Beamforming → ML (see [DESIGN](./DESIGN.md))
  - CLEAN-SC is the primary production target: good balance of resolution, speed, and robustness
  - Functional Beamforming as a fast real-time alternative (no matrix inversion; single power operation on D&S map)
  - required resources: GPU-accelerated host for CLEAN-SC and above; D&S/Functional BF can run on CPU
* video overlay of sound energy
  - the amount of time needed to create the energy map will dictate the effective frame rate of the Acoustic Camera
  - do we need to capture a frame of video at the same time (start/end of block?) that the audio is captured and overlay on that frame?
    * should we try to keep time consistent between audio overlay and video display
  - need fast frame rate to capture transients -- means more compute demands

## Aperture Size

Reducing aperture while holding mic count and system requirements constant involves the following tradeoffs.
Resolution (beamwidth) scales as λ/D, i.e., halving the aperture doubles the beamwidth at every frequency.

### Resolution vs. Aperture

For a circular aperture, HPBW ≈ 58° × λ/D (degrees):

| Aperture | Resolution@1kHz (λ=343mm) | Resolution@4kHz (λ=86mm) | Resolution@8kHz (λ=43mm) |
|---|---|---|---|
| 500mm | ~40° | ~10° | ~5° |
| 350mm | ~57° | ~14° | ~7° |
| 300mm | ~66° | ~17° | ~8° |
| 250mm | ~80° | ~20° | ~10° |
| 200mm | ~100° | ~25° | ~12° |
| 150mm | >90° | ~33° | ~17° |

* 5° resolution is achievable at 6–8kHz with a 400–500mm aperture (but not at 1kHz as one might initially assume)
  - achieving 5° @ 1kHz requires D ≈ 58° × 343mm / 5° ≈ 4 m
    * which is impractical for a hand-held device
* low-frequency performance suffers most when aperture is reduced
  - 200Hz is where a compact array is weakest regardless; reducing D makes this worse proportionally
* deconvolution methods (e.g., CLEAN-SC, Functional BF, DAMAS) can recover some resolution beyond the physical aperture limit
  - but there is a hard floor set by the aperture; algorithms improve it, not overcome it

### Side-Lobe Levels (improve with smaller aperture)

* with the same mic count packed into a smaller aperture, average mic density increases
* more uniform spatial sampling → lower side-lobe-to-main-lobe ratio
* the Underbrink pattern's irregular spacing benefits are enhanced with denser coverage
* this is a genuine benefit of reducing aperture when mic count is held constant

### Spatial Aliasing (aliasing-free ceiling rises)

* smaller aperture → smaller minimum inter-mic spacing → aliasing-free to a higher frequency
  - at 400mm with ~21mm min spacing: aliasing-free to ~8kHz (matches target band ceiling)
  - at 200mm with ~10mm min spacing: aliasing-free to ~17kHz (well above target band)
* for a target band topping at 8kHz, a moderately reduced aperture gives headroom to spare

### Near-Field Boundary (moves closer)

* far-field criterion: r > 2D²/λ
* smaller D → far-field condition met at shorter distances
  - D=400mm @ 1 kHz: far-field beyond ~0.9m
  - D=200mm @ 1 kHz: far-field beyond ~0.2m
* beneficial if sources need to be localized at close range (< 1 m)

### Physical (benefit)

* smaller array: lighter, cheaper PCB, easier to mount and point
* more practical for portable or hand-held configurations (Phase 4b)

### Summary

* the current system requirements target an aperture of 400–500mm
  - for increased portability, 350mm is the defensible minimum without fundamentally compromising the design goals
* there are benefits to having a smaller array, but the biggest issue drawback is lower spatial resolution
* size overview
  - 400–500mm is the sweet spot for resolution
    * ~5° @ 8kHz, ~10° @ 4kHz, ~40° @ 1kHz
  - 300–350mm is a reasonable compromise
    * loses ~1.5–2× resolution but gains portability, better side-lobe levels, and aliasing headroom well above 8 kHz
  - 350mm gets you to ~7° at 8 kHz with only a modest reduction in size
  - 300mm gives ~8° at 8 kHz and ~17° at 4 kHz, which is a reasonable compromise trading resolution for portability
  - below 300mm, the low-frequency performance (200–500Hz) degrades to the point where the array is mostly useful only above ~2–3 kHz
  - 200mm starts to look marginal
    * ~100° at 1kHz means essentially no directional discrimination at low frequencies

## Array Pattern Selection (assuming 96 mics, 300mm aperture)

### Geometry Feasibility

For 96 mics with 21mm minimum spacing (Nyquist at 8kHz) in a 300mm aperture:
  * available area: π × 150² ≈ 70,686mm²
  * exclusion area per mic (10.5mm radius): ≈ 346mm² each
  * total required: 96 × 346 ≈ 33,250mm² → **47% packing** — comfortable
  * average spacing: √(70,686 / 96) ≈ **27mm** — well above the 21mm minimum

At 300mm, all 96 mics fit with the 8kHz Nyquist constraint satisfied without pushing packing limits

### Pattern Comparison

| Pattern | Side-lobes | Aliasing | Isotropy | Notes |
|---|---|---|---|---|
| **Underbrink multi-arm log-spiral** | Excellent | Excellent | Excellent | Pareto-optimal; confirmed in [1]; used by gfai Mikado (96 mics) |
| Vogel/Fermat (golden angle) spiral | Very good | Good | Excellent | Simpler to parameterize; good simulation baseline |
| Concentric rings | Poor–Fair | Poor | Good | Regular spacing → grating lobes |
| Cross/star arms | Fair | Good | Poor | Angular bias in side-lobe structure |
| GA-optimized random | Potentially best | Excellent | Excellent | Requires Phase 1 simulation to design |

### Recommended Pattern: Underbrink Multi-Arm Log-Spiral

**8 arms × 12 mics** (alternatively 6 arms × 16 mics, I will simulate both)

* Rationale:
  - [1] (in docs/) directly confirms Underbrink outperforms all alternatives tested in both resolution and MSL
  - gfai Mikado uses exactly 96 mics in this pattern commercially at similar aperture
  - logarithmic spacing naturally samples multiple spatial scales — well-suited for the 200Hz–8kHz wideband target
  - tunable via H (arms) and V (spiral angle) parameters for simulation optimization in Phase 1

* Recommended parameters for 300mm:
  - inner radius r₀ ≈ 20–30mm (leaves center clear for camera mount)
  - outer radius = 150mm
  - H = 8–12 arms (more arms than at 400–500mm; needed to maintain uniform coverage at higher mic density)
  - spiral angle V: sweep in Phase 1 simulation
    * 300mm compression shifts the optimum vs. larger apertures

### Benchmark Alternative: Vogel/Fermat Spiral

* Place mics at r = √(n / 96) × 150mm, θ = n × 137.5° for n = 1…96
  - single-parameter design (golden angle is fixed), naturally uniform density, no preferred direction
  - will not beat optimized Underbrink but is a useful reference point in Phase 1 simulation

*[0]: Sarradj, E. & Herold, G. (2016). "A generic approach to synthesize optimal array microphone arrangements." BeBeC-2016-S4.*

*[1]: Zebb Prime and Con Doolan (2013). "A comparison of popular beamforming arrays." Proceedings of ACOUSTICS 2013—Victor Harbor.*
