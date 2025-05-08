# Acoustic-Camera
Low-cost, High-performance, Acoustic Camera

**WIP**

# System Requirements

* Min/Max Distance (m)
* Resolution (deg/auxel)
* FOV (+/- horizontal/vertical degs)
* Size of Mic Array (mm)
* Frequency Range (Hz)

# Target Design

**TBD**

# Beamforming Products

* Audio Technica
  - https://www.audio-technica.com/en-us/atnd1061
  - ATND1061 beamforming ceiling array microphone
  - ~$3,000
  - on-board DSP: AEC, noise reduction, AGC, 4-band EQ
  - 32 customizable zones across six audio output channels
* Brüel & Kjær
  - many microphone arrays
    * spherical, wheel, half-wheel, grid, sliced wheel, 2D robot, pentangular
  - ?
* Clockworks Signal Processing
  - A^2B Channel Microphone Array
  - ADAU7118 PDM to I2S interface controlled over A2B bus
  - uses AD2428W A2B device
  - up to 32x 24b 48KHz audio channels
    * over twisted pair, between multiple nodes, up to 15m
  - three boards (36mm stack):
    * mics (PDM out)
      - 8x Infineon IM68D130
      - SNR: 69dBA @ 48KHz, <1% THD up to 128dBSPL
      - Freq Range: 28Hz-20KHz (+/- 3dB)
    * PDM to I2S
      - ADAU7118
      - default config
        * 4x stereo PDM inputs (64x default)
        * Sample Rate: 48KHz
        * single, 8x channel, 32b TDM output to A2B
    * A2B
      - AD2428W
      - 32kB EEPROM
      - A2B config: locally-powered slave
  - 100mm diameter, mics spaced evenly every 45degs (90mm apart)
  - 50mA (typ) @ 9VDC + LED power
* Convergence Instruments
  - https://convergenceinstruments.com/product/acam_64/
  - ACAM-64, $2,100, 32x32, 20Hz-8KHz
  - FOV: 90/60 deg
  - image persistence: 10ms-10s
  - <= 1 img/ms
  - digital MEMS mics
  - saturation level (typ @ 1KHz): 120dB SPL
  - USB interface, 16KHz audio sampling rate
  - data formats: 16b PCM mono, or FP32
  - 187x182x16mm, .24kg
  - open protocol
  - optional camera
* miniDSP
  - UMA-8 v2
  - UMA-16 v2
    * 16 channels (4x4 grid), uniform rectangular array (URA)
    * Knowles SPH1668LM4H MEMS mics
      - Saturation: 120 dBSPL
      - SNR: 65dB
      - RF shielded
    * Resolution: 24b
    * Sample Rate: 8, 11.025, 12, 16, 32, 44.1, or 48KHz
    * center hole for optional USB camera
    * MCHStreamer Lite USB interface
    * XMOS Xcore200
      - PDM to PCM conversion
      - asynch USB audio up to 48KHz
    * 132x202x18mm, 42x42mm mic array
    * powered by USB
* Shure
  - Microflex Advance MXA920 Ceiling Array Microphones
  - programmable configurations
  - 325 (315?) mics, wide-angle camera
  - uses Audioscope SW
  - 10.1W, PoE
  - Sensitivity @ 1KHz: -1.74dBFS/Pa
  - Saturation(Max SPL relative to 0 dBFS overload): 95.74 dBSPL
  - SNR (@ 1KHz): 75.76 dB A-weighted
  - Latency: 15.9-26.6ms
  - Noise: 18.24 dB SPL-A
  - Dynamic Range: 77.5 dB
  - Freq Response: 125Hz-20KHz (+/- 10dB)
  - Weight: 5.8kg
  - 54.69x603.8x10.5mm
  - ~$5,000
* ?

# Microphones

* sensiBel
  - SBM100
    * optical MEMS
    * SNR (20-20KHz): 80 dBA
      - >10dB (8x) better SNR than capacitive MEMS mics
    * Saturation: 146 dBSPL
    * THD: <0.5%
    * Sensitivity: -46 dBFS (typ @ 1KHz)
    * Dynamic Range: 132 dB
    * Sample Rate: up to 192KHz
    * Clock freq: 1.5-12MHz
    * Digital Interface: PDM, I2S, 8-channel TDM
* ?

# Other Projects

* Ben Wang
  - https://benwang.dev/2023/02/26/Phased-Array-Microphone.html
* CMU Sonicam
  - https://hackaday.com/2020/08/10/acoustic-camera-uses-many-many-microphones/
  - https://hackaday.io/project/172053-acoustic-camera
  - https://course.ece.cmu.edu/~ece500/projects/s20-teame0/
* acoular Python library
  - https://github.com/acoular/acoular
  - example of 64 mic beamforming map
* ?

# Notes

* TODOs
  - match camera and mic array resolution
  - match audio input path dynamic range to ADCs
  - careful layout of audio front-end to ensure low noise
  - get ADCs that share common clock and (if multiple in a package) sample at the same point
* types of Beamforming
  - Delay-and-Sum (aka Phased-Array Processing)
    * each mic has a given delay and all mics are summed together to steer the beam (on-axis response)
    * typically done with linear array where 0deg angle is perpendicular to the array
    * can increase gain of on-axis signals
    * can attenuate off-axis signals (but very frequency-dependent)
    * symmetric, 0deg and 180deg are equivalent
    * can increase system SNR (AGWN sums to zero)
      - e.g., can get 3dB SNR gain with every additional mic used
        * however, lower SNR mics can get same SNR with fewer mics, or better SNR with same number of mics
  - Differential
    * subtract rear-facing mic from forward-facing mic
    * high off-axis rejection
  - Maximum Variance Distortionless Response (MVDR)
    * more complex processing than simple D&S or Differential
    * individual amplification and delay provided for each mic in the array
      - preserves on-axis sensitivity and maximizes off-axis attenuation
    * typically amplification and dealy are applied per-frequency at each mic
      - either with an FIR filter or frequency-domain processing
    * steer beam in Direction-of-Arrival (DoA)
      - sensitive to errors in DoA estimation and multi-path signals
    * higher SNR mics provide greater directionality than lower SNR ones
  - Linear Constrain Minimum Variance (LCMV)
  - Generalized Sidelobe Canceler (GSC)
  - Frost Beamforming
* Advantages of low SNR mic in beamforming applications
  - https://audioxpress.com/article/microphone-array-beamforming-with-optical-mems-microphones
  - a better SNR mic can do better than multiple lesser SNR mics
  - lower SNR mic means more compact array is possible
    * 42mm mic spacing for a BW of  4KHz requires 17dB gain @ 100Hz -> min mic SNR = 65dBA
    * 21mm mic spacing for a BW of  8KHz requires 22dB gain @ 100Hz -> min mic SNR = 70dBA
    *  7mm mic spacing for a BW of 24KHz requires 32dB gain @ 100Hz -> min mic SNR = 80dBA
* ?
