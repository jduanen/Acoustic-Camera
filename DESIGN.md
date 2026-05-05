# Acoustic Camera Design Target

## Microphone Array

* 96× Infineon IM69D120 PDM MEMS mics
  - 69 dBA SNR, factory-calibrated, ±1 dB sensitivity match, ±2° phase match
* Underbrink multi-arm logarithmic spiral pattern
  - 8x arms with 12x mics or 6x arms with 16x mics (to be optimized in Phase 1 simulation)
* ~21 mm min mic spacing
  - satisfies Nyquist at 8 kHz: λ/2 = 343/(2×8000) ≈ 21.4 mm
* ~300 mm diameter aperture
* Custom PCB(s), FPGA
  - mics share PDM clock, paired on data lines via L/R select → 48 DATA + 1 CLK to FPGA

## Interface & Compute

* PDM mics → FPGA hub → GbE → host PC
* FPGA responsibilities
  - PDM clock distribution: one shared clock fan-out to all 96x mics (careful layout for skew control)
  - PDM decimation: CIC + FIR filter chain per channel, 3.072 MHz PDM → 48 kHz 24-bit PCM
  - Synchronous sampling: all 96x channels share a common WS (word select) derived from the FPGA, guaranteeing sample-aligned capture
  - GbE packetization: assembled audio frames with sequence numbers and timestamps sent over UDP to host
  - Pin count: 48x DATA lines (2x mics per line via L/R select) + 1x CLK (manageable on mid-range FPGA)
  - Inspired by Ben Wang's 192-mic FPGA design (50 MHz FPGA, GbE to GPU host)
* Data rate: 96ch × 48kHz × 24b ≈ 110 Mbps (which fits comfortably within 1 GbE)
* FPGA candidates
  - Lattice ECP5: preferred for open-source toolchain (Yosys/nextpnr)
  - Xilinx Artix-7 (XC7A100T): preferred for resource headroom and ecosystem maturity (101K LUTs, abundant I/O)
  - Intel Cyclone 10: alternative
* Host: Linux PC with GPU for accelerated beamforming (PyTorch/CuPy)

## Software Stack

* Python pipeline: Acoular for beamforming core + custom extensions
* Algorithm progression: Delay-and-Sum → MVDR → CLEAN-SC → ML exploration
* GPU acceleration via PyTorch or CuPy for frequency-domain beamforming

## Video

* USB camera co-located at array center
* Field of view matched to array aperture and target distance range
* Real-time overlay of energy map on video stream using OpenCV

## Device Functions & Interface

### Core Functions

Every commercial acoustic camera supports these:

* Real-time energy map: beamformed sound field rendered as a 2D heatmap image
* Video + acoustic overlay: live video with energy map overlaid (colormap on camera image)
* Selectable frequency range: bandpass filter to focus on frequencies of interest (octave or 1/3 octave bands)
* Dynamic range control: adjustable display floor and ceiling (analogous to brightness/contrast for the acoustic image)
* Field of view selection: selectable FoV (e.g., 90° or 60°); must match video camera optics
* Image persistence/temporal averaging: controls the time window over which energy is averaged
  - fast = tracks transients; slow = reveals weak stationary sources
  - range: 10 ms to 10 s
* Record/playback: capture synchronized audio, video, and energy map sequences; replay offline

### Extended Functions

Features that differentiate higher-end products; implement progressively:

* Algorithm selector: choose beamforming algorithm at runtime: D&S → MVDR → CLEAN-SC (and later ML-based)
* Frequency-resolved maps: display energy maps at multiple octave bands simultaneously (side-by-side or toggled)
* Source tracking: lock on and follow the loudest detected source across frames
* Near-field/far-field mode: toggle assumption about wave curvature; affects steering delay computation
* Depth/range estimation: estimate distance to source using a co-located depth camera (e.g., Intel RealSense); enables 3D source localization
* Calibration mode: guided workflow to measure and store mic-to-mic sensitivity, phase, and position corrections

### Interface Design

#### Phase 1: ?sim?

?

#### Phase 2 / Phase 3: Host-tethered (USB)

Follow the ACAM_64 (Convergence Instruments) model: USB audio streaming to host PC, open protocol, Python-based desktop GUI.

GUI elements:
* Live video window with energy map overlay (adjustable colormap, opacity)
* Frequency band selector (octave and 1/3 octave buttons or slider)
* Dynamic range sliders (floor/ceiling dB)
* Algorithm selector dropdown (D&S, MVDR, CLEAN-SC)
* Persistence/averaging time slider
* FOV selector (if multiple optics supported)
* Record/Stop button and session file naming
* Real-time SPL meter and peak-hold indicator
* Status bar: sample rate, latency, mic count, connection state

Reference implementations:
* ACAM_64 open USB CDC protocol (virtual COM port + USB audio streaming)
* UMA-16 v2: raw 16-ch USB audio, host does all processing (which is a good scaffold for Phase 3 work)
* SpectAcoular (GUI layer on top of Acoular): open source, Python/Bokeh

#### Phase 4: FPGA + GbE (full custom array)

The FPGA hub sends 96-channel PCM audio over GbE (UDP packets)

Host GUI additions:
* GbE packet ingestion with sequence-number tracking and drop detection
* Calibration workflow UI (guided mic position/sensitivity estimation)
* Full algorithm suite (CLEAN-SC, MVDR, Functional Beamforming)

#### Phase 4b: Standalone/Field Use

As GbE tethering to a laptop becomes inconvenient for field use, optionally add:
* Embedded web UI (served from onboard SBC, accessed via phone or tablet browser over WiFi)
* Small integrated touchscreen (7" is the commercial standard: HEAD VMA V pattern)
* Physical record/stop button(s) on the housing
* Battery-powered operation with charge indicator
