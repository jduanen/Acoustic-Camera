# Acoustic-Camera
Low-cost, High-performance, Acoustic Camera

**WIP**

An Acoustic Camera computes a power map of the sound scene that exists within a video camera's field of view and overlays it onto the video in real time.
Capturing, visualizing, and tracking the position of sound sources is useful in a variety of different use cases, including the detection and localization of sources of vibration, gas leaks, electrical breakdown, etc. 
Acoustic Cameras can also serve as a useful prosthetic device for people who have lost hearing in one ear and can no longer localize sound sources.

## Use of Beamforming in Acoustic Cameras

To create an Acoustic Camera, we want to compute the power map of the sources in the video camera's Field of View (FoV).
The camera's FoV is partitioned into a grid of rectangles corresponding to a set of one or more adjacent pixels. The grid can be defined in terms of Cartesian or polar coordinates (i.e., (x, y) or (az, el) tuples).
The size (and shape) of the camera's FoV, as well as the number or size of the grid nodes are constrained by the number and spacing of the mics used in the array.
The video sensor (and its associated optics) used by the camera is selected such that its FoV and resolution is a good match for the characteristics of the mic array.

The Acoustic Camera processes the inputs from its microphone array to determine the amount of sound energy (in the frequency range(s) of interest) which comes from the a region in space defined by a pyramid who's apex is at the camera's focal point and is bounded by a grid rectangle.
Various methods exist to provide additional information about the distance from the camera of the noise source detected within a pyramid.

An important aspect of Acoustic Camera design is the choice as to whether the sources are to be considered Far-Field (i.e., the sound waves interact with the mic array as plane waves) or Near-Field (in which case the curvature of the arriving sound waves must be taken into account).
Idealized sounds sources are point-emitters which create sounds waves which radiate outwards in a spherical pattern.
For this work, we will initially assume that the sources will all be Far-Field in nature. Which is to say, the sources are sufficiently far from the mic array that the curvature sound waves at the mic array is negligible.
The actual distance required for a source to be considered Far-Field is a function of the wavelengths of the sound, the size of the mic array, and the spacing between mics.

One approach to determining the amount of energy that emanates from a region of space is to use beamforming and steer the main beam to the desired point in space.
Beamforming provides a means of measuring the energy coming from a region, while suppressing energy coming from other regions.

A common beamforming-based approach steers the beam (i.e., the main lobe) to the center point of each grid, computes the energy coming from that region, assembling values for all of the grid nodes into a map, and then overlaying that on the video output.
The array of energy values resulting from scanning the beam over all the regions is known as an energy map, which can be overlaid on the video to provide a visualization of the sources of sound energy.

Both the beamforming algorithm used and the characteristics of the mic array (i.e., the number of mics, their arrangement, and the total size of the array) define the characteristics of the resulting beam.
The key characteristics of any beamforming approach are the width of the main beam (typically measured at the point at which it is 3dB down from the peak) and the levels of the side-lobes (typically measured by the delta from the largest side-lobe's peak to the peak of the main beam).
The choice of beamforming algorithm and mic array define the key characteristics of the Acoustic Camera.
For example, the distance at which sounds sources can be localized, the accuracy with which this localization can be done, the number and type of sound sources that can be simultaneously localized, the frequency range in which localization can be done, the dynamic range of sounds over which the localization is possible, etc.

Beamforming is also considered to be a type of spatial filtering, i.e., the effect of beamforming is to increase the gain in a given direction and attenuate signals from all other directions.

There are a wide variety of different types of beamforming algorithms in the literature, all with different performance characteristics and resource demands.

### Delay-and-Sum Beamforming

Delay-and-Sum is a simple and commonly used method for beamforming with a microphone array.
The beam is formed and steered by setting time delay values for each of the array's microphones.
The delay values required for each mic in the array to steer the beam in a given direction can be generated analytically (based on the location of the mics in the array), or can be generated as a result of a calibration process.
By cross-correlating the signals received from each pair of mics in the array, the calibration process can, for all grid nodes in the camera's FoV, generate the set of delays for each mic to steer the beam to the desired point in space.
The resulting time delay matrices can be used at runtime to steer the beam to the desired grid locations.

#### Time-Domain Approach

The conceptually simplest (and computationally fastest) approach to implementing this type of beamforming is to delay and sum all of the inputs in the time domain.
The sound energy coming from a region can be calculated by capturing a sequence of audio samples from each mic in the array, and each sample is delayed by the appropriate amount for the desired beam location. The samples are then summed and the result divided by the number of mics in the array to give the sound energy emanating from the chosen grid node.

The result of this delaying and summing of the mic inputs is to amplify (by constructive interference) the signal from the desired grid node, while attenuating (through destructive interference) signals from all other sources.
Note that uncorrelated noise sums to zero and therefore the effective SNR of the mic array is increased.

#### Frequency-Domain Approach

By operating on the mic inputs in the frequency domain, more sophisticated processing can be performed that can result in substantially better performance of the Acoustic Camera.

The sound energy coming from a region can be calculated by first capturing a sequence of audio samples from each mic in the array (where each sample is captured at the same time, typically by using a common clock for all of the audio ADCs)
The number of audio samples in each acquisition block is given by the number of mics in the array, and the number of time steps over which samples are taken.

The sample block can be converted into the frequency domain by way of an FFT, where the resulting output matrix represents the energy and phase (represented as a complex number) of the captured sounds at different frequencies.
A steering matrix can be defined for each grid node, that contains the time delays for each mic.
These values can be used in the FFT computation to direct the beam to a given grid node, so that the resulting output matrix reflects the energy coming from that location in space.

The summation of the magnitude (i.e., real) values from each of the (complex) elements in the FFT output blocks provides the energy present at the chosen grid node (and at different frequencies), over the period of time that the input block was captured.
An energy map can be created by applying steering vectors for all of the grid nodes to the FFTs of the input block of samples, and assembling all of the calculated energies for all of the grid nodes into a matrix.
The energy map can then be overlaid on the Acoustic Camera's video display to indicate the position and magnitude of different sound sources.

Working with inputs in the frequency domain allows more advanced processing methods to be applied to the problem of beamforming.

### Deconvolutional Beamforming

Complex audio scenes can have multiple sources in proximity to each other in the camera's FoV.
Conventional beamforming techniques have difficulty in resolving individual sources that are close to each other in space. This is due to the system's spatial resolution limitations and the fact that the beamforming itself creates pseudo noise sources (i.e., sources that don't actually exist but appear as relatively high energy points in the energy map due to spatial aliasing). Pseudo noise sources occur due to the side-lobes that appear in mic array's response pattern after beamforming.

Spatial aliasing is inherent in practical acoustic cameras due to the finite size of the mic array and the discontinuous distribution of mics.
One method of reducing the problem of spatial aliasing is to ensure that the minimum distance between mics in the array do not exceed half of the acoustic wavelength of the frequency of interest.  Furthermore, the number and arrangement of mics in the array result in differing levels of side-lobes (as well as the width of the main beam).

There are practical limits (e.g., size, weight, power, data rate, cost, etc.) to the number of mics that can added to an array. Similarly, only so much improvement can be achieved through the careful arrangement of mics.
In the optical application domain, Deconvolution methods are used to improve the effective spatial resolution of imaging sensors. Similarly, deconvolution can be used to improve the performance of acoustic cameras.
The basic assumption here is that the captured audio signals are the convolution of the actual source signal and the transfer function of the mic array (i.e., the array's PSF).

The Point-Spread Function (PSF) provides a representation of how a mic array responds to an idealized (point monopole) noise source placed in front of the array. 
Essentially, the PSF is the transfer function of the mic array for a given grid point.
The PSF of a mic array provides information on the key characteristics of the mic array, i.e., the beam width and the ratio of side-lobes to the main lobe.





### Eigenvector/Eigenvalue methods of beamforming

**TBD**

### ML-based Beamforming

* shown to be better than conventional SVD-based methods, particularly in dynamic or complex environments
* more adaptable to varying signal environments
* Classification-Based Transfer Learning (CBTL) and Denoising-Based Transfer Learning (DBTL) are better than or equal to traditional blind beamforming across diverse conditions
* ML methods like RL can speed up beam selection and reduce computational overhead
* ML models (including CNNs) have high beam tracking accuracy, outperforming other approaches in realtime applications
* generalize well after training with synthetic data and transfer learning

* Models
  - ?
...


## A Sampling of Beamforming Algorithms

Below is a survey of some commonly used beamforming algorithms.

* Differential
  - subtract rear-facing mic from forward-facing mic
  - high off-axis rejection
  - easy to implement, but not steerable

* Delay-and-Sum
  - most fundamental and widely used technique for beamforming
  - delay the signal from each mic by an amount defined by the desired "look direction"
    * each mic has a given delay and all mics are summed together to steer the beam (on-axis response)
  - typically done with linear array where 0deg angle is perpendicular to the array
  - can increase gain of on-axis signals
  - can attenuate off-axis signals (but very frequency-dependent)
  - symmetric, 0deg and 180deg are equivalent
  - can increase system SNR (AGWN sums to zero)
    * e.g., can get 3dB SNR gain with every additional mic used
    * however, lower SNR mics can get same SNR with fewer mics, or better SNR with same number of mics
  - pros
    * easy to implement in both time and frequency domains
    * low computation/memory cost, can be run in real time
    * can dynamically steer the beam
  - cons
    * limited ability to distinguish between multiple closely-spaced sources
      - especially with a small array aperture or small number of mics
    * produces side lobe artifacts that can mask weaker sources
  - this describes the time-domain version, this can also be done in the frequency domain

* Minimum-Variance Distortionless Response (MVDR)
  - signal-dependent beamforming technique
  - uses inter-channel dependencies between mic array signals to enhance beamforming
    * place nulls on the interferers
  - sensitive to high background noise or reverberation in the sound scene
  - improve by pre-processing to separate direct components from the diffuse field
    * subspace separation has been shown to be effective
  - more complex processing than simple D&S or Differential
  - individual amplification and delay provided for each mic in the array
    * preserves on-axis sensitivity and maximizes off-axis attenuation
  - typically amplification and dealy are applied per-frequency at each mic
    * either with an FIR filter or frequency-domain processing
  - steer beam in Direction-of-Arrival (DoA)
    * sensitive to errors in DoA estimation and multi-path signals

* Linear Constraint Minimum Variance (LCMV)
  - widely used spatial filtering approach
  - passes signals from specific directions or with certain properties
    * minimizes interference and noise from all other directions
  - objective is to minimize output power (variance) of the array signal
    * subject to a set of linear constraints that
      - preserve the desired signals
      - suppress interference from specific directions
  - linear constraints ensure that the beamformer maintains
    * a specified response in terms of gain and phase
      - for certain directions or for certain signals
    * e.g., unity gain for the desired direction and null known interferers
  - optimization problem
    * constraints defined desired beam pattern
      - e.g., main lobe direction, direction of nulls
    * Covariance Matrix is estimated from sensor array input data
    * computes the weights that minimize output power, while satisfying constraints
  - can impose multiple constraints for complex beam patterns
    * e.g., multiple main lobes/nulls
  - minimizes unwanted signals while preserving desired sources

* Generalized Side-Lobe Canceller (GSC)
  - adaptive beamforming approach
  - simpler to implement than LCMV, similar performance/function
    * doesn't require matrix inversion
  - splits input signals from array into two main paths
    * Fixed (Quiescent) Beamformer Path
      - pre-steer the array toward the desired signal direction (e.g., with D&S)
    * Side-Lobe Cancelling Path
      - use blocking matrix to remove the desired single, leaving the interference and noise
      - remove this with an adaptive filter (e.g., LMS or RLS)
  - subtracts the two paths
    * cancels interference and noise, while preserving the signal from the main lobe
  - better interference rejection than D&S
  - computationally efficient for large arrays
  - flexible and robust in dynamic noise environments

* Frost Beamforming
  - adaptive beamforming technique to enhance signal in the "look directions", whil minimizing noise and interference from other directions
  - filter-and-sum: each mic has a delay stage (to steer the beam) followed by an FIR filter
  - outputs of all FIR filters is summed to create the output
  - adaptive FIR filters' weights are updated using a Constrained Least Mean Squares (CLMS) approach
    * minimizes the MSE (output power)
    * subject to the constraint that the signal from the look direction is unmodified
  - works for wideband signals because the FIR filter can shape the frequency response for each channel
  - better SNR and interference ratio (SNIR) than D&S

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

* Plane Wave Decomposition (PWD)
  - signal-independent beamformer for spherical mic arrays
  - most commonly used in spherical harmonic domain
    * decompose sound scene into individual spatial components -- aka Spherical Harmonics
  - assumes plane-waves, so far-field only
  - represents complex wave field as a sum (or superposition) of plane waves characterized by a direction and amplitude
  - used for reconstructing and manipulating sound fields measured by mic arrays
  - Plane Wave: wave whose value at any instant is constant across any plane perpendicular to its direction of propagation
  - Decomposition: any measured acoustic field can be expressed as the some of multiple plane waves arriving from different directions
    * each plane wave is defined by its angle of incidence and frequency of content
  - measured data from mic array is modeled as a linear combination of plane waves
    * this is an inverse problem: given the array data, estimate the amplitudes and directions of the constituent plane waves
  - decomposition algorithms include:
    * D&S Beamforming: aligns and sums signals for different directions, scanning for plane wave components
    * Regularized Inversion: uses constraints (e.g., L2 or L1 norms) to stabilize the solution and suppress noise/artifacts
    * Spherical/Cylindrical Harmonics: for 3D analysis, field can be expanded in terms of spherical or cylindrical harmonics, which are related to plane wave decomposition
  - result is a map or spectrum showing amplitude (and sometimes, phase) of plane waves as a function of direction and frequency
  - accuracy and resolution of plane wave decomposition depend on array geometry and chosen regularization methods chosen to solve the inverse problem
  - sparse decomposition methods (using L1 norm) can provide cleaner results with fewer artifacts
    * especially in noisy or underdetermined scenarios

* Cross-Pattern Coherence (CroPaC)
  - spatial-filtering beamforming technique
  - measures correlation between coincident beamformers
    * post-filters to suppress noise, interferers, and reverberation
  - unlike other spatial filtering approaches, doesn't require direct estimation of mic noise
  - extended to spherical harmonic domain for arbitrary combinations of beamformers
  - ?

* CLEAN-PSF
  - building block to understand CLEAN-SC
  - deconvolution technique from astronomy, for where less bright stars are near very bright ones
  - considers the dirty map represents a convolution between the actual source distribution and the array's PSF
  - approach
    * iteratively finds the highest source value in the dirty map
    * subtracts its contribution from the dirty map
    * adds the source it finds to a separate clean map
    * keep going until all sources are removed from the dirty map
  - assumes sources behave like monopoles with negligible directivity effects
    * not always true -- e.g., wind tunnels, aircraft fly-over, etc.

* HR-CLEAN_SC
  - ?

* COMET2
  - gridless version of (HR-)CLEAN-SC are not as accurate as COMET2

* Deconvolution Approach for the Mapping of Acoustic Sources (DAMAS)
  - this approach models the CSM considering a distribution of statistically independent sources in the grid
    * modeled CSM is compared to the measured CSM, resulting in a linear system of equations
      - modeled CSM uses a single source placed at each grid location
    * solution of this system of equations provides source intensity and location
  - uses linear system of equations, makes no assumptions
    * N (equal to the number of grid points) equations and N unknowns
  - full rank equations are solved with an iterative method
  - noise radiated from a region of interest is determined by summing the mean-squared values over that region
  - algorithm
    * beamform over the source region to get the "dirty" map
      - dirty map is measured sound power, blurred by the array's PSF
    * model dirty map as a convolution of the true source distribution with the array's PSF
      - dirty map (power measured at each grid point) = PSF matrix (how source at a location affects measurement at other locations) * True source strengths (unknowns to be solved for)
    * solve this system of equations with an iterative non-negative least squares approach
      - at each step, estimates source strengths that best explain the observed dirty map, given the array's PSF
    * result is a "clean" map with improved spatial resolution and reduced side-lobes
  - output map provides explicit source strength values
    * not just relative levels like other methods
  - computationally intensive, especially for large grids
    * but provides superior performance for complex sound fields
  - developed after CLEAN deconvolution approach
  - converges (very slowly) to the solution of CMF
  - techniques have been developed to reduce the computation costs
    * e.g., DAMAS2 and compressed grids
  - DAMAS2 computes the PSF for each grid node in the frequency domain
    * can use shift-invariant PSF to simplify the computations
      - depends only on distance between grid node and source
      - sacrifices spatial resolution

* Covariance Matrix Fitting (CMF)
  - used to estimate source locations and strengths by fitting a modeled covariance matrix to the measured Cross-Spectral Matrix (CSM) obtained from the mic array
  - assumes model for the spatial distribution of sources and computes theoretical covariance matrix for this model
  - adjusts model parameters (e.g., source positions and powers) so that the modeled covariance matrix best matches the measured one
    * typically done by minimizing a fitting error
  - good for source location and quantification in low-/mid-frequencies
  - comparable to methods like DAMAS and eigenvalue-based approaches
  - particularly effective when the number of sources is limited and the array is high quality
  - frequently used with regularization techniques like NNLS or LassoLars to improve robustness

# Design Tradeoffs

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
* processing
  - algorithm: MUSIC, DAMAS, ?
  - required resources: compute, memory, ?

* video overlay of sound energy
  - the amount of time needed to create the energy map will dictate the effective frame rate of the Acoustic Camera
  - do we need to capture a frame of video at the same time (start/end of block?) that the audio is captured and overlay on that frame?
    * should we try to keep time consistent between audio overlay and video display
  - need fast frame rate to capture transients -- means more compute demands

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

* 16SoundsUSB
  - https://github.com/introlab/16SoundsUSB
  - also 8x mic version
  - open HW, based on XMOS xCORE-200
  - 2x Cirrus Logic CS5368 ADC
  - CS2100 clock generator and clock muliplier/jitter-reduced freq synthesizer
  - 8-96KHz sampling rates
  - USB powered
  - electret and MEMS mics connected to shielded RJ11 connectors

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

# Open-Source Acoustic Beamforming Software

* List of tools for acoustic beamforming
  - https://github.com/eac-ufsm/beamforming-tools

* ACOSOLO: Acoustical Source Localization with Optimization Methods
  - https://github.com/gilleschardon/acosolo
  - optimization-based source localization implementations
    * includes: CMF, Gridless, etc.
  - implementations of optimization-/sparsity-based methods for source location
    * Beamforming: maximum likelihood for one source
    * CMF: optimized implementation
    * Gridless methods: for conditional and unconditional models
      - Conditional: estimation of powers of random sources
      - Unconditional: estimation of amplitudes of deterministic sources
    * Greedy localization of correlated sources
    * maximum likelihood for localization of a source with asynchronous arrays
  - several demos
    * beamforming with syncrhonous/asynchronous arrays, using simulated data
    * CMF with experimental data
    * gridless source localization with experimental data
    * localization of correlated sources with simulated data

* Acoular: python package for acoustic beamforming
  - https://github.com/acoular/acoular
  - provides multiple algorithms, including deconvolution
  - FOSS, started in 2006 as library for wind tunnel testing using mic arrays
  - funded by DFG in 2024
  - multi-channel signal processing
  - simulation and analysis of moving audio sources
  - generates synthetic data and supports training of ML models through Acoupipe
  - GUI-based apps with SpectAcoular
  - extensible, ~100 classes
  - data acquisition, data import, time-domain linear/nonlinear filtering, spectrum estimation, array layout and mapping grids definition, spatial filtering (beamforming), time-/frequency-based deconvolution, inverse estimation methods, fixed/moving source data synthesis
  - uses lazy execution (only compute results that are needed, when they are needed)
    * important for pipelined operations on big datasets

* AcouPipe: Python toolkit for acoustical source localization and characterization
  - https://github.com/adku1173/acoupipe
  - https://adku1173.github.io/acoupipe/
  - used for training ML models
  - started in 2021
  - toolbox for generating unique acoustical source localization and characterization datasets
    * for training of ML models
  - supports distributed computation via Ray
    * https://docs.ray.io/en/master/
  - comes with two default datasets
  - two default classes to generate mic array datasets
    * DatasetSynthetic: fast and simple method using white noise, spatially stationary, anechoic conditions
    * DatasetMIRACLE: large set of measured spatial room impulse responses in anechoic chamber
      - from TU Berlin MIRACLE dataset and synthetic signals
      - realistic and quasi-infinite dataset
  - stores only input features for ML (not raw time-series data)

* Arlpy: a few beamforming algorithms for underwater applications
  - https://github.com/org-arl/arlpy
  - https://arlpy.readthedocs.io/en/latest/
  - tools for DSP, comms, beamforming and array processing, plotting, etc.

* Augen: Amiet-Acoular Integration Module in Python
  - https://github.com/eac-ufsm/augen
  - https://www.researchgate.net/publication/363031873_Integracao_de_multiplas_toolboxes_para_aplicacao_em_beamforming_e_aeroacustica
  - integration between Acoular and Amiet (aeroacoustics) Tools
  - examples of spiral array response

* Beamlib: library of acoustic beamforming algorithms from signals obtained from mic array
  - https://gitlab.isae-supaero.fr/acoustic-beamforming/beamlib
  - part of POLA3 project
  - generation and evaluation of mic arrays
  - includes implementations of D&S, DAMAS, and CLEAN-PSF/-SC methods
  - implemented mic arrays: Regular Grid, Archimedean Spiral, Dougherty log-Spiral, Arcondoulis Array, Underbrink

* Beamforming30: >30 beamforming algorithms in python
  - https://github.com/huangzhenyu/beamforming/tree/master
  - Chinese
  - frequency-domain beamforming
  - time-domain beamforming
  - plots of comparisons of different methods

* Fast beamforming in Python: fast and efficient beamforming notebooks in python
  - https://github.com/schipp/fast_beamforming
  - direct approach to cross-correlation beamforming fails for large problems
    * especially when matrices become too large for memory
  - use 'dask' to distribute computation across multiple machines

* MUSIC: C package for acoustic source localization
  - https://github.com/VarunPwr/Hydrophone
  - ?

* Open Embedded Audition System (ODAS)
  - https://github.com/introlab/odas
  - https://github.com/introlab/odas/wiki
  - replaces ManyEars
  - C library for sound source localization, tracking, separation, and post-filtering
  - has GUI for visualization
  - open source 8x and 16x USB arrays
    * https://github.com/introlab/16SoundsUSB
  - ?

* Pyroomacoustics: python package for room acoustics and audio
  - https://github.com/LCAV/pyroomacoustics
  - https://www.researchgate.net/publication/320344643_Pyroomacoustics_A_Python_Package_for_Audio_Room_Simulation_and_Array_Processing_Algorithms
  - supports DoA, DSB, simulation
  - package for rapid development and testing of audio array processing algorithms
  - main parts:
    * OO python interface to simulation scenarios involving multiple sources and mics in 2D and 3D rooms
    * fast C++ implementation of image source model and ray tracing for general polyhedral rooms to generate room impulse responses and simulation propagation between sources and mics
    * reference implementations of popular algorithms for STFT, beamforming, direction finding, adaptive filtering, source separation, and single-channel denoising
  - includes datasets from CMU ARCTIC, TIMIT, Google Speech Commands Dataset
  - includes several examples

* SpectAcoular: GUI on top of Acoular
  - https://github.com/acoular/spectacoular
  - started in 2019
  - ?

* vBeam: fast and differentiable beamformer for optimizing ultrasound imaging
  - https://github.com/magnusdk/vbeam
  - ?

# Microphone Array Patterns

The pattern of the microphone array used in acoustic camera applications can be 2D or 3D.

The distribution of mics in an array influences the achievable spatial resolution (aka the beamwidth) and the Maximum Side-Lobe Levels (MSL) (which is a measure of the ability of the array to reject sources that are off-beam).  Deconvolution methods (e.g., DAMAS) attempt to remove the properties of the array from beamforming results, but in practice, the array properties remain important in the quality of the results that can be provided.

One study (https://www.acoustics.asn.au/conference_proceedings/AAS2013/papers/p5.pdf) demonstrated that the Underbrink pattern outperforms other array patterns in both resolution and MSL (over the tested frequencies). This study showed that arrays based on multiple arms, with mics more evenly distributed over the array's area, tended to get the best resolution (with adequate MSLs for most of the area). Also, arrays with a high density of mics in the center of the array tended to get the best MSLs directly below the array, at the expense of array resolution and performance over a larger array area.

In general, for a given pattern, more mics improve noise rejection and directionality, providing better spatial resolution and better SNR. However, there are diminishing returns for doubling the number of mics in an array, and there's an increase in cost that grows linearly with the number of mics.

Irregular spacing is preferred to avoid spatial aliasing or side-lobe artifacts that come with uniform mic spacing.

Mics have to be closely matched in sensitivity and frequency response. Mismatched mics can degrade beam performance and reduce the directionality of the array.

TBD: look into spherical arrays

### 2D Array Patterns

* Regular Grid
  - 2D grid with mics at intersections
  - uniform spacing leads to predictable, but higher, side-lobe levels (artifacts)
    * this is due to periodic gaps in spatial sampling
    * this can mask weaker sources or create false/ghost sources
  - beam width is consistent across directions but wider compared to spiral arrays
    * this reduces resolution for closely spaced sources
  - prone to spatial aliasing at high frequencies
    * if mic spacing exceeds half of the wavelength -- violating Nyquist criterion
  - best suited for narrowband or mid-frequency applications that need uniform coverage
  - easier to construct than other geometries
  - better suited for planar (as opposed to 3D) measurements

* Archimedean Spiral
  - radius increases linearly with angle
    * 'r(theta) = a + b*theta'
  - parameters: min radius, max radius, number of turns, number of mics
  - mics are spaced so angular positions are evenly distributed along the whole spirale angle, and their radii increase linearly from the center outward
  - simple, single-arm spiral layout, relatively even distribution of mics from center outward

* Dougherty log-Spiral
  - logarithmic spiral, where radius increases exponentially with the angle
    * 'r(theta) = r_0*e^(y*theta)' where y is related to the spiral angle
  - mics are placed at equal arc-lengths along the spiral
    * not at equal angular intervals
  - arc length between mics is kept constant
    * results in denser distribution near the center and sparser at the edge
  - better spatial sampling at the center
    * improved performance for central sources and wideband applications
  - provides >10dB side-lobe suppression at high frequencies

* Arcondoulis Array
  - modified spiral array that adjusts the "squash" of the spiral in the x and y directions
    allows for elliptical or non-circular spiral shapes
  - mic density can be increased at the center by adjusting design parameters
  - parameters: min/max radii (r_0, r_max), spiral angle, number of mics, squash factors (e_x, e_y)
    * squash factors control the stretching of the spiral in x and y dims
  - can choose parameters that put more mics near the center, if so desired
  - flexible design that can emphasize central density or overall coverage
  - often used to improve main-/side-lobe characteristics for sources near the center of the array

* Underbrink
  - patented
  - multi-arm logarithmic spiral array, several arms radiating from the center
  - each arm is a log-spiral and mics are distributed along each arm
  - parameters: min/max radii (r_0, r_max), spiral angle, number of mics, number of arms
  - mics rotated equally around the origin
    * more uniform distribution of mics across the array aperture -- especially the outer regions
  - high spatial resolution and good side-lobe performance across a wide area
    * suitable for both central and off-axis source localization
  - considered best all-around performance among spiral arrays

* Bruel&Kjaer Spiral
  - patented
  - designed to be easily assembled/disassembled
  - two concentric hoops with mics located on spokes between the hoops
  - spokes are placed at an angle to the inner hoop, and mics have a non-uniform spacing along the spoke
  - parameters: number of spokes, number of mics per spoke, spoke angle, and distribution of mic locations along the spoke
  - looks like spokes come off the inner hoop at a tangent 
  - mics are distributed with the same spacing on each spoke

### 3D Array Patterns

# Microphone Elements

* general characteristics
  - higher SNR mics provide greater directionality than lower SNR ones
  - ?

### sensiBel SBM100
  - optical MEMS
  - SNR (20-20KHz): 80 dBA
    * >10dB (8x) better SNR than capacitive MEMS mics
  - Saturation: 146 dBSPL
  - THD: <0.5%
  - Sensitivity: -46 dBFS (typ @ 1KHz)
  - Dynamic Range: 132 dB
  - Sample Rate: up to 192KHz
  - Clock freq: 1.5-12MHz
  - Digital Interface: PDM, I2S, 8-channel TDM

### Knowles SPH0645LM4H-B
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

### InvenSense ICS-52000
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

### InvenSense ICS-43434
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

### InvenSense ICS-43432
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

### Infineon IM69D120
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

### Solid State System 3SM222KMT1KA-P (aka 3SM222)
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

### ST Micro MP23DB01HP
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

### ST Micro MP34DT05-A
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

### PUI Audio AMM-3738-B-R
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

### InvenSense (TDK) T3902
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
* Advantages of low SNR mic in beamforming applications
  - https://audioxpress.com/article/microphone-array-beamforming-with-optical-mems-microphones
  - a better SNR mic can do better than multiple lesser SNR mics
  - lower SNR mic means more compact array is possible
    * 42mm mic spacing for a BW of  4KHz requires 17dB gain @ 100Hz -> min mic SNR = 65dBA
    * 21mm mic spacing for a BW of  8KHz requires 22dB gain @ 100Hz -> min mic SNR = 70dBA
    *  7mm mic spacing for a BW of 24KHz requires 32dB gain @ 100Hz -> min mic SNR = 80dBA
* ?

# Docs

* 000405.pdf: On the Design of a MEMS Microphone Array for a Mobile Beamforming Application
  - built a mic array for the back of a smartphone
  - simulated different mic array configurations
  - used a genetic algorithm to optimize the placement of mics on a spiral
  - used XMOS dev board and 16x mics
    * Infineon’s IM69D130 digital MEMS mic
* 000468.pdf: Optimal planar microphone array arrangements [2015]
  - simulated lots of array arrangements for Vogel's Spiral with different H and V values
  - showed Underbrink Spiral is best tradeoff between beam width and side-lobe levels
* 0080015889.pdf
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
* BeBeC-2022-S07.pdf
* bp2144.pdf
* Chakravarthula_Seeing_With_Sound_Long-range_Acoustic_Beamforming_for_Multimodal_Scene_Understanding_CVPR_2023_paper.pdf [2023]
  - 
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
* p453.pdf: Design and Calibration of a Small Aeroacoustic Beamformer [2010]
  - ?
* p5.pdf: A comparison of popular beamforming arrays [2013]
  - ?
* s13272-019-00383-4.pdf
* 'Schumacher - 2022 - Evaluation of microphone array methods for aircraf.pdf'
* Sound_Localization_and_Speech_Enhancement_Algorith.pdf
* time-domain-beamforming-3d-micarray-doebler-heilmann-meyer-2008-bebec.pdf
* TP-2007-345.pdf
* Wang_2023_J._Phys.__Conf._Ser._2479_012026.pdf: 
Research on multi-sound source localization performance
based on leaf-shaped microphone array [2022]
  - ?
  

