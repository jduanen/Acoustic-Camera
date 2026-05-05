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
  - *Ref: Sarradj, E. & Herold, G. (2017). "A Python framework for microphone array data processing." Applied Acoustics 116:50–58. doi:10.1016/j.apacoust.2016.09.015*

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
  - real-time pipeline: SSL (localization) → SST (tracking) → SSS (separation) → post-filter
  - designed for embedded deployment (Raspberry Pi, Jetson) as well as desktop
  - companion ODAS Studio GUI for real-time visualization of tracked sources
  - designed for use with 8x and 16x USB mic arrays (16SoundsUSB)
    * https://github.com/introlab/16SoundsUSB
  - *Ref: Grondin, F. et al. (2022). "ODAS: Open and Distributed Audition System." Frontiers in Robotics and AI. doi:10.3389/frobt.2022.854444*

* Pyroomacoustics: python package for room acoustics and audio
  - https://github.com/LCAV/pyroomacoustics
  - supports DoA, DSB, simulation
  - package for rapid development and testing of audio array processing algorithms
  - main parts:
    * OO python interface to simulation scenarios involving multiple sources and mics in 2D and 3D rooms
    * fast C++ implementation of image source model and ray tracing for general polyhedral rooms to generate room impulse responses and simulation propagation between sources and mics
    * reference implementations of popular algorithms for STFT, beamforming, direction finding, adaptive filtering, source separation, and single-channel denoising
  - includes datasets from CMU ARCTIC, TIMIT, Google Speech Commands Dataset
  - includes several examples
  - *Ref: Scheibler, R., Bezzam, E. & Dokmanić, I. (2018). "Pyroomacoustics: A Python Package for Audio Room Simulation and Array Processing Algorithms." ICASSP 2018. doi:10.1109/ICASSP.2018.8461310*

* SpectAcoular: browser-based GUI on top of Acoular
  - https://github.com/acoular/spectacoular
  - started in 2019
  - interactive beamforming analysis without writing code; built on Bokeh/Panel
  - supports live visualization of energy maps, array geometry, and spectral data
  - runs as a local web server; access from any browser on the same machine or network

* vBeam: fast and differentiable beamformer for optimizing ultrasound imaging
  - https://github.com/magnusdk/vbeam
  - JAX-based; differentiable end-to-end beamforming pipeline designed for gradient-based optimization
  - targeted at ultrasound imaging, but the differentiable beamforming approach is transferable to acoustic cameras
  - useful reference for GPU-accelerated and ML-optimized beamforming implementations
