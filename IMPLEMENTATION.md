# Implementation Details

## Hardware

### Microphone Elements
[MICS](./MICS.md)

### Mic Array

* Infineon IM69D120V01XTSA1 digital MEMS microphone
  - Newark: $0.755

### FPGAs and FPGA Development Boards

Target use: PDM clock distribution, 96-channel CIC+FIR decimation, synchronous sampling, GbE packetization. See [DESIGN](./DESIGN.md) for details.

#### Candidate Devices

* Lattice ECP5
  - preferred for open-source toolchain (Yosys / nextpnr / openFPGALoader)
  - ECP5-85F: 84K LUTs, 3.4Mb BRAM, abundant I/O
  - low power; actively supported by open-source community
  - dev boards: OrangeCrab, ULX3S, Versa ECP5

* Xilinx Artix-7
  - preferred for resource headroom and mature ecosystem (Vivado)
  - XC7A100T: 101K LUTs, 4.8Mb BRAM, up to 210 user I/O
  - large library of IP cores (GbE MAC, PCIe, etc.)
  - dev boards: Digilent Arty A7-100T, Nexys A7

* Intel Cyclone 10 LP
  - alternative; good balance of cost and I/O count
  - dev boards: Intel Cyclone 10 LP Evaluation Kit

## Software

## Open-Source Acoustic Beamforming Software
[FOSS](./FOSS.md)

## Python Packages

* acoular: Beamforming core (D&S, MVDR, CLEAN-SC, array geometry)
* pyroomacoustics: Room acoustics simulation, synthetic source data
* acoupipe: ML training dataset generation (from GitHub) 
* numpy: Array math
* scipy: Signal processing
* matplotlib: Plotting
* h5py: HDF5 I/O (Acoular data format)
* jupyterlab: Notebooks (Phase 1 deliverable) 
* pandas: Results tabulation
* seaborn: Visualization
* tqdm: Progress bars
