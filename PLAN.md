# Phased Development Plan

Each phase produces a working end-to-end system, just with reduced features or performance.
My goal is to never have a "not yet working" state for more than one phase at a time.

## Phase 1 — Simulation & Algorithm Benchmarking
*No hardware required*

* Simulate virtual mic arrays and synthetic sound sources using Acoular + Pyroomacoustics
* Validate D&S, MVDR, and CLEAN-SC against known ground-truth DoA
* Compare array geometries (Underbrink vs. grid vs. simple ring) before committing to PCB fabrication
* Tune array parameters: number of mics, spacing, aperture, number of spiral arms
* Generate labeled datasets for later ML training using AcouPipe (DatasetSynthetic or DatasetMIRACLE)
* Deliverable: Python notebooks demonstrating energy maps and DoA accuracy for each algorithm

## Phase 2 — Smoke Test with ReSpeaker Mic Array v2.0
*Hardware already in hand*

* 4-mic circular array, 90 mm diameter, USB interface — limited spatial resolution but immediate availability
* Goals: validate the full software pipeline end-to-end: audio capture → beamforming → energy map → video overlay
* Surface real-world issues early: USB latency, clock drift, mic-to-mic sensitivity mismatch, background noise floor
* Implement calibration workflow (cross-correlation-based, inspired by Ben Wang's approach)
* Deliverable: live video with overlaid energy map; not high performance, but fully functional pipeline

## Phase 3 — Intermediate Array (16–32 mics)
*Validate scaling before custom PCB commitment*

Two options:
* Option A (fast): miniDSP UMA-16 — off-the-shelf, 16-ch, USB, XMOS interface; no PCB work required
* Option B (more representative): small custom PCB with 16–24 mics in a simplified Underbrink pattern

Goals:
* Validate PDM/TDM synchronization and data aggregation at scale
* Evaluate CLEAN-SC and MVDR on real multi-mic data vs. simulation predictions
* Exercise the calibration workflow at scale
* Confirm chosen mic (IM69D120) has adequate sensitivity and phase matching in practice
* Deliverable: calibrated beamforming demo with measurably better spatial resolution than Phase 2

## Phase 4 — Full Custom Array (96 mics, Underbrink spiral, FPGA hub)
*Only after Phase 3 pipeline is proven*

Hardware sub-tasks (can be parallelized):
* Mic array PCB(s): 96× Infineon IM69D120 in Underbrink spiral (geometry optimized in Phase 1 simulation); ~400–500 mm aperture, ~21 mm min spacing; careful analog layout and shielding
* FPGA hub board: PDM clock distribution, 96-channel CIC+FIR decimation (PDM→PCM), synchronous WS, GbE packetization; candidate: Lattice ECP5 or Xilinx Artix-7; open-source HDL (VHDL or SystemVerilog)
* Co-located video camera: USB camera mounted at array center, field of view matched to array aperture and target distance range

Software sub-tasks:
* Update host pipeline to ingest GbE audio packets, handle sequence/drop detection
* Full calibration workflow: cross-correlation-based mic position and sensitivity estimation (PyTorch gradient descent, as in Ben Wang's design)
* PSF measurement and array characterization

Deliverable: full-performance acoustic camera meeting System Requirements (200 Hz–8 kHz, ±45° FoV, ~5° resolution @ 1 kHz)

## Phase 5 — ML Enhancement
*After real data is available from Phase 3/4*

* Use AcouPipe to generate synthetic datasets matched to the actual array geometry
* Train PILOT or CRNN-based DoA model; benchmark against CLEAN-SC
* Explore real-time inference on GPU; profile latency vs. frame rate trade-off
* Deliverable: ML-based beamformer with comparable or better accuracy than CLEAN-SC at lower latency
