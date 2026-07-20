# Overview of Acoustic Camera Products

To get some ideas of what's possible, existing products and projects are examined with respect to their targeted use cases, performance specifications, types of hardware used, cost, size, etc.

## Commercial Beamforming Products

* 16SoundsUSB
  - https://github.com/introlab/16SoundsUSB
  - also 8x mic version
  - open HW, based on XMOS xCORE-200
  - 2x Cirrus Logic CS5368 ADC
  - CS2100 clock generator and clock multiplier/jitter-reduced freq synthesizer
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

* Eigenmike (mh Acoustics)
  - https://mhacoustics.com/products
  - em32: 32-capsule spherical array, 4th-order Ambisonics
  - em64: 64-capsule spherical array, 7th-order Ambisonics
  - records full 3D sound field; outputs raw capsule audio + Ambisonics B-format
  - gold-plated condenser capsules, 20 Hz – 20 kHz
  - USB or Dante network audio interface

* Fluke
  - https://www.fluke.com/en-us/product/industrial-imaging/sonic-industrial-imager-ii900
  - https://www.fluke.com/en-us/product/industrial-imaging/precision-acoustic-imager-ii910
  - ii900 (discontinued, replaced by ii905): air/gas/vacuum leak detection
    * 64x digital MEMS mics
    * 2KHz-52KHz
    * range: 0.5->70m, detects 0.005 CFM leak @ 100PSI from 32.8ft
    * FOV: 63deg +-5deg (visual camera matches, 1.2MP, 3x digital zoom)
    * 7" 1280x800 capacitive touchscreen, sunlight readable
    * 186x322x68mm, 1.8kg w/ battery, 6hr Li-ion (spare included)
  - ii910 (discontinued, replaced by ii915): adds partial discharge (PD) detection
    * mic count not published; 2KHz-100KHz, 25FPS
    * range: 0.5->120m
    * FOV: 63deg +-5deg; 5MP visual camera, 3x digital zoom
    * PDQ Mode: on-device PD classification + PRPD plot
    * 186x322x68mm, 2.15kg w/ battery, 2x 6hr Li-ion
    * same display as ii900

* Fotric
  - https://www.fotric.com/h-series-high-performance-acoustic-imaging-camera
  - https://us.fotric.com/products/fotric-td2e-acoustic-imager-camera
  - TD-series: handheld leak detection
    * TD2e: 64x MEMS mics, 200KHz sample rate, 2KHz-100KHz, range 0.3-60m
      - FOV 58x45deg, 5MP camera, 3.5" 640x480 IPS touchscreen
      - IP54, 2m drop, 25g shock, 3.6V 5000mAh Li-ion (>=3hr), $999
    * TD3-LD: 96x MEMS mics, rapid leak detection variant
  - H-series: higher-end, adds partial discharge detection
    * H4 / H4mini / H6 / H7 / H-Flex
    * up to 162x MEMS mics (spiral array, 6.6" diameter), FOV 66x52deg, 13MP camera
    * H7: 345x MEMS mics, range up to 300m, NaviPdM AI assistant
      - on-device PD type recognition, leak severity scoring, auto report gen
    * freq range/weight/price not published for H-series
  - widely used reference platform for HOA research and spatial audio production

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
