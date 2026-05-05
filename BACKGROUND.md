# Background Technical Information

---

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

---

## Approaches to Beamforming

There are a wide variety of different types of beamforming algorithms in the literature, all with different performance characteristics and resource demands.

### Delay-and-Sum Beamforming

Delay-and-Sum is a simple and commonly used method for beamforming with a microphone array.
The beam is formed and steered by setting time delay values for each of the array's microphones.
The delay values required for each mic in the array to steer the beam in a given direction can be generated analytically (based on the location of the mics in the array), or can be generated as a result of a calibration process.
By cross-correlating the signals received from each pair of mics in the array, the calibration process can, for all grid nodes in the camera's FoV, generate the set of delays for each mic to steer the beam to the desired point in space.
The resulting time delay matrices can be used at runtime to steer the beam to the desired grid locations.

#### Time-Domain Beamforming

The conceptually simplest (and computationally fastest) approach to implementing this type of beamforming is to delay and sum all of the inputs in the time domain.
The sound energy coming from a region can be calculated by capturing a sequence of audio samples from each mic in the array, and each sample is delayed by the appropriate amount for the desired beam location. The samples are then summed and the result divided by the number of mics in the array to give the sound energy emanating from the chosen grid node.

The result of this delaying and summing of the mic inputs is to amplify (by constructive interference) the signal from the desired grid node, while attenuating (through destructive interference) signals from all other sources.
Note that uncorrelated noise sums to zero and therefore the effective SNR of the mic array is increased.

#### Frequency-Domain Beamforming

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

### Eigenvector/Eigenvalue Beamforming

Eigenvector/eigenvalue-based methods decompose the array's Spatial Covariance Matrix (SCM) into signal and noise subspaces via eigenvalue decomposition. They provide higher resolution DoA estimation than Delay-and-Sum, especially for closely spaced sources.

* MUSIC (Multiple Signal Classification) — see "A Sampling of Beamforming Algorithms" below; searches noise subspace for steering vector orthogonality
* Root-MUSIC — polynomial form of MUSIC; avoids spectral peak search → cheaper computation
* ESPRIT (Estimation of Signal Parameters via Rotational Invariance Techniques)
  - exploits rotational invariance in the array signal model between two displaced sub-arrays
  - returns DoA estimates directly without scanning a spatial spectrum
  - lower computational cost than MUSIC for the same resolution
  - requires arrays with specific geometric structure (pairs of displaced sub-arrays)
  - *Ref: Roy, R. & Kailath, T. (1989). "ESPRIT — Estimation of Signal Parameters via Rotational Invariance Techniques." IEEE Trans. Acoust. Speech Signal Process. 37(7):984–995. doi:10.1109/29.32276*
* CSSM (Coherent Signal Subspace Method)
  - extends subspace methods to wideband signals
  - aligns frequency-bin covariance matrices into a common subspace before eigen decomposition
* TR-MUSIC (Time-Reversal MUSIC)
  - exploits time-reversal invariance of the wave equation to build a time-reversal operator
  - decomposes operator into signal/noise subspaces as in standard MUSIC
  - improves DoA accuracy in reverberant and multipath environments where standard MUSIC degrades

### ML-based Beamforming

* overview
  - shown to be better than conventional SVD-based methods, particularly in dynamic or complex environments
  - more adaptable to varying signal environments
  - Classification-Based Transfer Learning (CBTL) and Denoising-Based Transfer Learning (DBTL) are better than or equal to traditional blind beamforming across diverse conditions
  - ML methods like RL can speed up beam selection and reduce computational overhead
  - ML models (including CNNs) have high beam tracking accuracy, outperforming other approaches in realtime applications
  - generalize well after training with synthetic data and transfer learning

* Models (2019–2025 literature)
  - CNN / CRNN-based DoA — classifies angular bins from GCC-PHAT or STFT magnitude features; fast inference, good generalization to unseen rooms
  - Transformer (self-attention) — captures long-range mic-to-mic dependencies; top performer on DCASE SELD benchmarks (e.g., SELD-Conformer, 2022–2024)
  - EIN-V2 / SELD-Conformer — combined Sound Event Detection + Localization (SELD); joint model for simultaneously answering "what" and "where"; consistent top-3 on DCASE 2022–2024 leaderboard
  - PILOT (2022) — Physics-Informed Learning for DoA with Transformers; embeds array steering vectors directly into the attention mechanism; strong generalization across array geometries
  - Deep MUSIC — neural network replaces or augments the eigendecomposition step; end-to-end trainable
  - NN-MVDR / FasNet — neural network estimates MVDR filter weights directly from raw waveforms; primarily speech enhancement but applicable to source separation in acoustic cameras
  - Physics-informed NNs — embed steering vector or wave propagation constraints into loss or network structure; improves generalization to out-of-distribution conditions
  - Note on CBTL/DBTL (referenced above): Classification-Based and Denoising-Based Transfer Learning originate in beam management literature (5G/mmWave); direct application to acoustic cameras is an open research area
* Training data
  - AcouPipe DatasetSynthetic or DatasetMIRACLE for generating labeled mic array datasets matched to a specific array geometry
  - DCASE 2022–2024 datasets for SELD training and benchmarking

---

## A Sampling of Beamforming Algorithms

Below is a overview of some commonly used beamforming algorithms.

### Differential
  - subtract rear-facing mic from forward-facing mic
  - high off-axis rejection
  - easy to implement, but not steerable

### Delay-and-Sum
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

### Minimum-Variance Distortionless Response (MVDR)
  - signal-dependent beamforming technique
  - uses inter-channel dependencies between mic array signals to enhance beamforming
    * place nulls on the interferers
  - sensitive to high background noise or reverberation in the sound scene
  - improve by pre-processing to separate direct components from the diffuse field
    * subspace separation has been shown to be effective
  - more complex processing than simple D&S or Differential
  - individual amplification and delay provided for each mic in the array
    * preserves on-axis sensitivity and maximizes off-axis attenuation
  - typically amplification and delay are applied per-frequency at each mic
    * either with an FIR filter or frequency-domain processing
  - steer beam in Direction-of-Arrival (DoA)
    * sensitive to errors in DoA estimation and multi-path signals
  - *Ref: Capon, J. (1969). "High-resolution frequency-wavenumber spectrum analysis." Proc. IEEE 57(8):1408–1418. doi:10.1109/PROC.1969.7278*

### Linear Constraint Minimum Variance (LCMV)
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

### Generalized Side-Lobe Canceller (GSC)
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

### Frost Beamforming
  - adaptive beamforming technique to enhance signal in the "look directions", while minimizing noise and interference from other directions
  - filter-and-sum: each mic has a delay stage (to steer the beam) followed by an FIR filter
  - outputs of all FIR filters is summed to create the output
  - adaptive FIR filters' weights are updated using a Constrained Least Mean Squares (CLMS) approach
    * minimizes the MSE (output power)
    * subject to the constraint that the signal from the look direction is unmodified
  - works for wideband signals because the FIR filter can shape the frequency response for each channel
  - better SNR and interference ratio (SNIR) than D&S

### Multi-Signal Classification (MUSIC)
  - subspace-based beamforming approach
  - used primarily for estimating Direction-of-Arrival (DoA) of multiple signal sources
  - approach
    * Signal Model: create Spatial Covariance Matrix from input signals
    * Subspace Decomposition: decompose the Covariance Matrix into two orthogonal subspaces using eigenvalue decomposition
      - Signal Subspace: spanned by eigenvectors corresponding to the largest eigenvalues (related to the sources)
      - Noise Subspace: spanned by eigenvectors with the smallest eigenvalues (related to noise)
    * Spatial Spectrum Search: scans possible source directions
      - for each direction, compute projection of the array's steering vector onto the Noise Subspace
        * the projection is minimized when the steering vector aligns with the true source direction
    * DoA Estimation: directions corresponding to peaks are the estimated DoAs of the sources
  - can resolve multiple closely-spaced sources
  - provides high accuracy in low-noise and low-reverb environments
  - *Ref: Schmidt, R.O. (1986). "Multiple emitter location and signal parameter estimation." IEEE Trans. Antennas Propagat. 34(3):276–280. doi:10.1109/TAP.1986.1143830*

### CLEAN-based on Source Coherence (CLEAN-SC)
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
  - *Ref: Sijtsma, P. (2007). "CLEAN Based on Spatial Source Coherence." Int. J. Aeroacoustics 6(4):357–374. doi:10.1260/147547207783359459*

#### CLEAN-PSF
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

#### HR-CLEAN_SC
  - ?

### COMET2
  - gridless version of (HR-)CLEAN-SC are not as accurate as COMET2
  - ?

### Plane Wave Decomposition (PWD)
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

### Cross-Pattern Coherence (CroPaC)
  - spatial-filtering beamforming technique
  - measures correlation between coincident beamformers
    * post-filters to suppress noise, interferers, and reverberation
  - unlike other spatial filtering approaches, doesn't require direct estimation of mic noise
  - extended to spherical harmonic domain for arbitrary combinations of beamformers
  - ?

### Deconvolution Approach for the Mapping of Acoustic Sources (DAMAS)
  - *Ref: Brooks, T.F. & Humphreys, W.M. (2006). "A deconvolution approach for the mapping of acoustic sources (DAMAS) determined from phased microphone arrays." J. Sound Vib. 294(4-5):856–879. doi:10.1016/j.jsv.2005.12.046*
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

### Covariance Matrix Fitting (CMF)
  - used to estimate source locations and strengths by fitting a modeled covariance matrix to the measured Cross-Spectral Matrix (CSM) obtained from the mic array
  - assumes model for the spatial distribution of sources and computes theoretical covariance matrix for this model
  - adjusts model parameters (e.g., source positions and powers) so that the modeled covariance matrix best matches the measured one
    * typically done by minimizing a fitting error
  - good for source location and quantification in low-/mid-frequencies
  - comparable to methods like DAMAS and eigenvalue-based approaches
  - particularly effective when the number of sources is limited and the array is high quality
  - frequently used with regularization techniques like NNLS or LassoLars to improve robustness

### Functional Beamforming (FB)
  - power map from conventional D&S beamforming is raised to a high integer power ν before computing the map
    * sharpens peaks and suppresses side-lobes without iterative deconvolution
  - simple extension of D&S: cost is a single power operation on the map, no matrix inversion
  - dramatically improves dynamic range vs. standard D&S at low computational cost
  - widely used in recent literature as a fast alternative to CLEAN-SC for real-time systems
  - resolution and dynamic range improve with ν, but optimal ν is problem-dependent
  - *Ref: Dougherty, R.P. (2014). "Functional Beamforming." Berlin Beamforming Conference (BeBeC) 2014, paper BeBeC-2014-01.*

### Sparse Bayesian Learning (SBL) / Atomic Norm Minimization
  - treats source localization as a sparse recovery (compressed sensing) problem
    * true sources are sparse in the angular/spatial domain
  - SBL: places sparsity-inducing priors over source powers and infers source locations via EM
    * automatically estimates source count and noise variance
    * superior resolution for closely-spaced sources vs. MUSIC/ESPRIT at low SNR
  - Atomic norm minimization: gridless CS approach; represents source directions as atoms in a continuous dictionary
    * avoids grid quantization error inherent in D&S/MUSIC
    * solved via semi-definite programming (SDP); computationally expensive but exact
  - active research area 2015–present; SBL implementations available in Acoular ecosystem

---

## Important Beamforming Concepts

### Cross-Spectral Matrix (CSM)

A frequency-domain matrix that stores the cross-power between every pair of mics at a given frequency.
The diagonal entries of this matrix are the auto-power spectra of each mic, and its off-diagonal entries capture how strongly pairs of sensors are correlated in phase and amplitude.
The CSM is used to estimate how signals and noise are distributed over the mic array.

The CSM captures spatial correlations between mic signals at each frequency and contains the information needed to differentiate between coherent source energy and uncorrelated noise.

In beamforming, the CSM is useful because it summarizes the spatial and spectral information needed to estimate where a sound source is located.
A common approach combines the CSM with a steering model to form a map of likely source locations.
The CSM is the input snapshot that an adaptive beamformer reads to decide how to steer, focus, and suppress unwanted signals.

The CSM reflects the current sound field and so adaptive beamformers can respond to changing conditions better than when using fixed weights.
