# Acoustic-Camera
Low-cost, High-performance, Acoustic Camera

**WIP**

An Acoustic Camera computes a power map of the sound scene that exists within a video camera's field of view and overlays it onto the video in real time.
Capturing, visualizing, and tracking the position of sound sources is useful in a variety of different use cases, including the detection and localization of sources of vibration, gas leaks, electrical breakdown, etc. 
Acoustic Cameras can also serve as a useful prosthetic device for people who have lost hearing in one ear and can no longer localize sound sources.

# System Requirements

**TBD**

* Min/Max Distance (m)
* Resolution (deg/auxel)
* FOV (+/- horizontal/vertical degs)
* Size of Mic Array (mm)
* Frequency Range (Hz)
* Environment: anechoic, enclosed/reverberant, random/coherent noise, single/multiple sources of interest
* ?

# Target Design

**TBD**

# Existing Systems

To get some ideas of what's possible, existing products and projects are examined with respect to their targeted use cases, performance specifications, types of hardware used, cost, size, etc.

## Commercial Beamforming Products

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
    * 8x Infineon IM68D130 mics (PDM out),
    * PDM to I2S: ADAU7118
    * A2B: AD2428W
  - 100mm diameter, mics spaced evenly every 45degs (90mm apart)
  - 50mA (typ) @ 9VDC + LED power

* Convergence Instruments
  - https://convergenceinstruments.com/product/acam_64/
  - ACAM-64, $2,100, 32x32, 20Hz-8KHz
  - FOV: 90/60 deg
  - digital MEMS mics
  - USB interface, 16KHz audio sampling rate
  - data formats: 16b PCM mono, or FP32
  - 187x182x16mm, .24kg

* gfai Tech
  - https://www.gfaitech.com/
  - offer two types of acoustic cameras
    * Microphone Arrays
      - Mikado: hexagon, 96x MEMS mics
      - Octagon: octagon, 192x MEMS mics
      - Fibonacci AC Pro: spiral arms, ?
      - Ring32/48/72: ring, 32/48/72x mics
      - Star48: 48x electret mics, three radial arms, 3.4m diameter, 7.4kg
      - EVO AC Pro: 96-168 electret mics, random distribution, 2.4x2.5m planar, 70-90kg, for wind-tunnels

* Head acoustics
  - https://www.head-acoustics.com/products/sound-source-localization
  - Head Visor VMA V
    * two sizes
      - 60 MEMS mics, 0.3m diameter
        * has peculiar mic placement on 30cm disk (looks like pairs and odd spacing)
        * depth camera in the middle
      - 120 MEMS mics, 1m diameter
        * add 10x arms to double the mics (and triple the size)

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

* Sorama
  - https://sorama.eu/
  - CAM64
    * 64x MEMS PDM mics, 160x160mm, 0.42kg, POE
    * Beamforming (Far-Field): 1.2KHz-15KHz
    * near-field acoustic holography: 25Hz-23.4KHz?
    * open Pyramid-shaped with handle at apex
  - CAM1K
    * 1024 microphones, 640x640mm array, integrated HD video camera, POE
    * 640x785x130mm, 5.5kg
    * Far Field beamforming (optimal conditions): 300Hz-15kHz
    * Near Field Acoustic Holography frequency range: 25Hz-23.4?Hz
    * big version of CAM64
    * looks like a BBQ grill

## Beamforming Projects

* Ben Wang (2023)
  - https://benwang.dev/2023/02/26/Phased-Array-Microphone.html
  - https://github.com/kingoflolz/mic_host
  - 192-channel MEMS mic array
    * 24x radial arms with linear array of 8x mics, exponentially-spaced
    * 8x concentric circles
  - three types of HW: 24x mics boards, hub, and host
  - frequency-domain beamforming
  - two beamforming visualizations are implemented
    * 3D Near-Field Beamformer
    * 2D Far-Field Beamformer
  - converts PDM from MEMS mics and packetizes samples with an FPGA (50MHz)
    * sends packets to host (with GPU) via GBE
  - calibration
    * move noise source in front of the array
    * calculate cross-correlation between all pairs of mics (18K unique pairs)
      - FFT pair -> complex multiply together -> IFFT -> find peak
        * fused compute on GPU -- don't write intermediate results to memory
    * used gradient descent to optimize the positions of the source at each time step
      - PyTorch on GPU
      - loss function minimizes difference between the measured and ideal correlations
        * do this while trying to minimize the deviation of the mic positions from the inital positions
        * and minimize the jerk of the source trajectory
      - speed of sound is a parameter which is optimized to get the best model
        * speed of sound is a function of temperature and pressure
      - converged to reasonable solution for source and mic positions (and speed of sound)
        * took a few hundred iterations (a few secs) on the GPU

* Sonicam (2020)
  - https://hackaday.com/2020/08/10/acoustic-camera-uses-many-many-microphones/
  - https://hackaday.io/project/172053-acoustic-camera
  - https://course.ece.cmu.edu/~ece500/projects/s20-teame0/
  - Senior ECE project at CMU in 2020
  - 6x panels of 4x4 rectangular grids of MEMS mics
  - 96x mic array, FPGA, Host
  - TDK Invesense mics
  - FPGA with mic array, ethernet to the Host
  - [rather large]

* acoular Python library
  - https://github.com/acoular/acoular
  - example of 64 mic beamforming map

* Kickstarter: The First Handheld Sound Camera for Everyone (2018)
  - https://www.kickstarter.com/projects/351002836/the-first-handheld-sound-camera-for-everyone
  - 64x MEMS mics, camera, display, ~4K euro
  - mic arrays
    * three concentric rings: ~20 mics per ring
  - quad-core ARM53 @ 1.2GHz, Linux, 1GB DRAM
  - Display: 800x480, 7", capacitive touch screen
  - Range: 10-24KHz
  - Size: 340x340x95mm
  - Sample Rate: 48KHz
  - Resolution: 24b
  - Camera: 320x240 @ 50fps or 640x480 @ 16fps, FOV = +/-38deg, global shutter
  - battery powered
  - up to 100 fps acoustic, up to 50 fps optical
  - recording, replay, export sounds/images/distances

* 1024 “Pixel” Sound Camera Treats Eyes To Real-Time Audio (2016)
  - https://hackaday.com/2016/07/01/1024-pixel-sound-camera-treats-eyes-to-real-time-audio/
  - 32x32 MEMS mic array, made of 4x 8x8 arrays

* ?

# Beamforming Algorithms

* Delay-and-Sum
  - most fundamental and widely used technique for beamforming
  - delays the signal from each mic by an amount defined by the desired "look direction"
  - pros
    * easy to implement in both time and frequency domains
    * low computation/memory cost, can be run in real time
    * can dynamically steer the beam
  - cons
    * limited ability to distinguish between multiple closely-spaced sources
      - especially with a small array aperture or small number of mics
    * produces side lobe artifacts that can mask weaker sources

* Plane Wave Decomposition (PWD)
  - signal-independent beamformer for spherical mic arrays
  - most commonly used in spherical harmonic domain
    * decompose sound scene into individual spatial components -- aka Spherical Harmonics
  - assumes plane-waves, so far-field only

* Minimum-Variance Distortionless Response (MVDR)
  - signal-dependent beamforming technique
  - uses inter-channel dependencies between mic array signals to enhance beamforming
    * place nulls on the interferers
  - sensitive to high background noise or reverberation in the sound scene
  - improve by pre-processing to separate direct components from the diffuse field
    * subspace separation has been shown to be effective

* Multi-Signal Classification (MUSIC)
  - subspace-based beamforming approach
  - used primarily for estimating Direction-of-Arrival (DoA) of multiple signal sources
  - approach
    * Signal Model: create Spatial Covariance Matrix from input signals
    * Subspace Decomposition: decompose the Covariance Matrix into two orthogonal subspaces using eigenvalue decomposition
      - Signal Subspace: spanned by eigenvectors corresponding to the largest eigenvectors (related to the sources)
      - Noise Subspace: spanned by eigenvectors with the smallest eigenvalues (related to noise)
    * Spatial Spectrum Search: scans possible source directions
      - for each direction, compute projection of the array's steering vector onto the Noise Subspace
        * the projection is minimized when the steering vector aligns with the true source direction
    * DoA Estimation: directions corresponding to peaks are the estimated DoAs of the sources
  - can resolve multiple closely-spaced sources
  - provides high accuracy in low-noise and low-reverb environments

* Clean-based on Source Coherence (CLEAR-SC)
  - deconvolution approach to improve spatial resolution of beamformers
    * widely used in noise source localization tasks
    * good robustness and speed
    * reveals secondary sources by suppressing dominant source artifacts
  - starts with "dirty" map created by conventional (D&S) beamforming
    * often contains blurred sources and artifacts from the point-spread function (PSF)
  - iterates
    * identifies the location with max source power in the dirty map
    * estimates contribution of this dominant source and measures its coherence with other locations
    * subtracts estimated contribution of this source (including its side lobes) from the Cross-Spectral Matrix (CSM), and updates the map
    * stops when the largest signal is below a threshold
  - result is a "clean" map with improved spatial resolution and reduced side-lobe artifacts
    * more accurate localization and quantification of the actual sources
  - iterative deconvolution: iteratively removes sources and refines the map
  - uses sources' spatial coherence to separate overlapping sources

* Cross-Pattern Coherence (CroPaC)
  - spatial-filtering beamforming technique
  - measures correlation between coincident beamformers
    * post-filters to suppress noise, interferers, and reverberation
  - unlike other spatial filtering approaches, doesn't require direct estimation of mic noise
  - extended to spherical harmonic domain for arbitrary combinations of beamformers

# Microphone Elements

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

* Knowles SPH0645LM4H-B
  - digital MEMS
  - Power Supply: 1.8V (typ) <3.6V (max) @ 600uA (typ), <100mA (max)
  - Interface: I2S
  - Bottom Ported
  - onboard SigmaDelta converter and filter
  - Clock Freq: 3.072MHz (typ)
  - Sensitivity: 94dBSPL @ 1KHz: -26dBFS
  - SNR: 94dBSPL @ 1KHz A: 65 dBA
  - THD:
    * 94dBSPL @ 1KHz: 0.2-1%
    * 1% THD: 94-110 dBSPL
  - AOP: 10% THD @ 1KHZ, 120dBSPL
  - PSSR: 200mVpp sine @ 1KHz: -86dBA

* InvenSense ICS-52000
  - digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b TDM interface
  - up to 16x per TDM link, with synchronous sampling
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 50-20KHz
  - PSSR: -89dBFS
  - AOP: 117dBSPL
  - Interface: SCK, SD, WS, WSO
  - Vdd: -0.3 to 3.63V (max)
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Interface: 24b, twos complement, MSB first
    * each mic in daisy-chain emits in subsequent 32b slots

* InvenSense ICS-43434
  - digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b I2S interface
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 60-20KHz
  - PSR (217Hz 100mVpp square wave): -100dBFS
  - AOP (10% THD): 120dBSPL
  - Sample Rate: 23-51.6KHz
  - Vdd: -0.3 to 3.63V (max) @ 490uA
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Interface: I2S (and L/R control)
    * 24b, twos complement, MSB first
    * two mics per shared link (stereo)
    * must be 64 SCK cycles in each WS stereo frame
    * LR pin selects whether data is output on rising/falling edge of WS
  - Noise Floor (20-20KHz, A-weighted RMS): -90 dBFS
  - Group Delay (sound input to digital output): 2/fs secs

* InvenSense ICS-43432
  - Digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b I2S interface
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 50-20KHz
  - AOP (10% THD): 120dBSPL
  - EIN: 29dBSPL
  - Dynamic Range
    * (EIN to AOP): 87dB
    * (EIN to FS): 91dB
  - PSR
    * 217Hz 100mVpp square wave: -80dBFS
    * 1KHz sine wave: -90dBFS
  - Noise Floor (20-20KHz A-weighted RMS): -91dBFS
  - Interface
    * 24b, twos complement, MSB first
    * two mics per shared link (stereo)
    * must be 64 SCK cycles in each WS stereo frame
    * LR pin selects whether data is output on rising/falling edge of WS
  - SCK Freq: 3.379MHz (max), 3.072MHz (typ)
  - WS Freq: 52.8KHz (max)
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Vdd: -0.3 to 3.63V (max) @ 490uA
  - Group Delay (sound input to digital output): 2/fs secs
  - synchronize mics via WS signal

* Infineon IM69D120
  - Digital MEMS
  - Dynamic Range: 95dBA
  - Distorion: <=1% @ up to 118dBSPL
  - Freq Range: 28-?
  - close matching for arrays
    * phase matching: +/-2deg
    * sensitivity matching: +/-1dB
  - on-board LNA, SigmaDelta ADC
  - factory calibrated
  - Latency: 6usec @ 1KHz
  - Interface: PDM, (Data, Select, Clock)
  - Pattern: omni
  - Sensitivity (1KHz, 94dBSPL): -26dBFS (typ), -27dBFS (min)
  - AOP (THD=10%): 120dBSPL
  - SNR (A-weighted, Fclk=3.072MHz): 69dBA
  - EIN (A-weighted, Fclk=3.072MHz): 25dBSPL
  - THD (1KHz): 94dBSPL=0.5% ... 120dBSPL=10:0%
  - Group Delay: 25Hz=70usec, 4KHz=1usec
  - Phase Response: 75Hz=19deg ... 3KHz=-1deg
  - Freq Range (3dB): 25-11KHz?
  - Power Supply
    * Vdd: 1.62-3.6V
    * Idd @ 3.072MHz: .98mA (typ), 1.3mA (max)
  - Fclk: 3.072MHz (typ)
  - startup time: 50ms

* Solid State System 3SM222KMT1KA-P (aka 3SM222)
  - Digital MEMS
  - on-board pre-amp, ADC
  - Top Ported
  - Interface: PDM
    * supports two mics on single link
    * L/R, Clock, Data
    * polarity: increasing sound pressure = increasing density of 1's
  - Sensitivity deviation: +/-1dB
  - Pattern: omni
  - Freq Range: 50-10Hz?
  - Power Supply
    * Vdd: 1.6-3.6V
    * Isb: 550uA @ 1.8V, 850uA @ 3.6V
  - Sensitivity (1KHz 94dBSPL): -27 to -25dBFS
  - SNR: 64dBA
  - EIN: 30dBA
  - THD: <0.2% @ 94dBSPL, 1% @ 110dBSPL
  - AOP (10% THD @ 1KHz): 120dBSPL
  - Clock Freq: 1-4.8MHz
  - PSSR (1KHz 200mVpp sine wave): 60dBV/FS
  - PSR+N: -80dBFS(A)
  - has low-power mode (different specs)

* ST Micro MP23DB01HP
  - Digital MEMS
  - Bottom Ported
  - Pattern: omni
  - modes: sleep, low-power, performance
  - Power Supply:
    * Vdd: 1.6-3.6V, 1.8V (typ)
    * Idd: 880uA (perf mode @ Fc=3.072MHz)
  - sensitivity matched
  - Interface: PDM (L/R channel selector)
  - wakeup time: 10ms
  - Freq Response: 35-?Hz
  - Sensitivity (94dBSPL @ 1KHz): -42 to -40dBFS
  - SNR (94dBSPL @ 1KHz A-weighted): 65.5dBA (3.072MHz)
  - THD (94dBSPL @ 1KHz): 0.2%
  - AOP (94dBSPL @ 1KHz): 135dBSPL
  - PSR (100mVpp sine wave): -95dBFS

* ST Micro MP34DT05-A
  - Digital MEMS
  - Top Ported
  - Pattern: omni
  - Interface: PDM (L/R channel selector)
  - Power Supply:
    * Vdd: 1.6-3.6V, 1.8V (typ)
    * Idd: 650uA (typ)
  - AOP: 122.5dBSPL
  - Sensitivity: -29 to -23dBFS
  - SNR (94dBSPL @ 1KHz A-weighted): 64dBA
  - PSR (100mVpp sine wave): -90dBFS
  - Fclk: 1.2-3.25Mhz
  - wakeup time: 10ms
  - THD (1KHz): 94dBSPL=0.2%, 110dBSPL=0.7%, 120dBSPL=6% (all typ)
  - Freq Response: 35-10KHz?

* PUI Audio AMM-3738-B-R
  - Analog MEMS
  - Pattern: omni
  - on-board pre-amp
  - Sensitivity (1KHz): -38dB +/-1
  - Power Supply
    * Vdd: 1.5-3.6V, 2V (typ)
    * Idd @ 2V: 90uA
  - SNR (1KHz 94dB A-weighted): 62dB
  - Freq Range: 20-20KHz
  - TDH (94dB 1KHz): 0.5%
  - AOP (1KHz 10% THD): 125dB

* InvenSense (TDK) T3902
  - Digital MEMS
  - Bottom Ported
  - Pattern: omni
  - on-board ADC, PDM modulator
  - Interface: PDM (clk, data, select)
  - Power Supply
    * Vdd: 1.65-3.63V
    * Idd (Vdd=1.8V): 650uA (typ), 750uA (max)
  - Sensitivity (1KHz 94dBSPL): -33 to -31dBFS
  - SNR (20KHz BW, A-weighted): 64dBA
  - EIN (20KHz BW, A-weighted): 30dBASPL
  - Dynamic Range (EIN to AOP): 96dB
  - Freq Range: 36-?KHz?
  - THD (105dBSPL): 0.2-1.0%
  - PSR
    * 100mVpp square wave A-weighted: -94dBFS
    * 1KHz sine: -100dBFS
  - AOP (10% THD): 126dBSPL
  - FS Acoustic Level (0dBFS output): 126dBSPL
  - standard and low-power modes (different specs)
  - wakeup time: 20ms
  - Fclk (std): 1-3.3MHz

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
  - in human speech, low freq is where the energy is and higher freqs are where the information is
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
* ?