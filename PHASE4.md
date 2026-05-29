# Phase 4 — Full Custom Array

96-mic Underbrink-spiral PCB + Artix-7 FPGA front-end + GbE to Linux host. Goal: full-performance
acoustic camera meeting system requirements (200 Hz – 8 kHz, ±45° FoV, ~5° resolution @ 1 kHz).

---

## Hardware Design Decisions

### Microphone — Infineon IM72D128

Upgraded from the IM69D120 originally noted in DESIGN.md.

| Spec | IM69D120 (previous) | IM72D128 (chosen) |
|---|---|---|
| SNR | 69 dB(A) | **72 dB(A)** (+3 dB) |
| AOP | 120 dBSPL | **128 dBSPL** (+8 dB) |
| Sensitivity match | ±1 dB | ±1 dB |
| Phase match | ±2° | **±2°** (unchanged — factory calibrated) |
| Ingress protection | — | **IP57** (dust and moisture) |
| Interface | PDM | PDM |
| Approximate unit cost | ~$0.76 | ~$0.90–1.10 |

The phase matching spec is unchanged — critical for coherent array beamforming. The 3 dB SNR gain
improves weak-source detectability (dynamic range). IP57 is useful for outdoor / industrial use.
At 96 units the cost difference is approximately $14 total.

**Part number**: IM72D128V01XTMA1  
**Source**: DigiKey / Mouser / Future Electronics (standard lead times as of May 2026)

No strong competitor exists at this price point with documented ±2° factory phase matching.
TDK, STMicro, and Vesper PDM mics in this range do not publish comparable phase tolerance specs.

---

### FPGA — Xilinx Artix-7 XC7A100T

#### Why not smaller devices

The 96-channel CIC + FIR + GbE pipeline requires approximately 38,000–40,000 LUTs:

| Block | LUT estimate |
|---|---|
| CIC decimation, 96 ch × ~250 LUTs | ~24,000 |
| FIR compensation, 96 ch × ~150 LUTs | ~14,400 |
| GbE MAC + UDP stack | ~3,000 |
| PDM capture, L/R demux, control | ~2,000 |
| **Total** | **~43,000** |

| Device | LUTs available | Fits? |
|---|---|---|
| Xilinx XC7A35T | 20,800 | No |
| Lattice ECP5-25F | 25,500 | No |
| Lattice ECP5-45F | 44,500 | Tight (~5% margin) |
| **Xilinx XC7A100T** | **63,400** | **Yes (~35% headroom)** |

#### Primary recommendation: Artix-7 XC7A100T

- **63,400 LUTs** — adequate headroom after the full audio pipeline
- **240 DSP48E1 blocks** — efficient FIR MAC without burning LUTs
- **4.86 Mb BRAM** — adequate for filter state and GbE packet buffers
- **TEMAC** (Tri-Mode Ethernet MAC) — production-grade GbE IP, included in Vivado evaluation
- **ILA / VIO** in-circuit logic analyzer — essential for PCB bring-up debugging
- **Vivado** free WebPACK tier covers all IP needed for this design
- **Dev board for HDL development**: Arty A7-100T ($149) uses the same chip and package

**Part number**: XC7A100T-1FTG256C (256-pin FTBGA, 1.0 speed grade)

#### Alternate: Lattice ECP5-45F

If a fully open-source toolchain (Yosys + nextpnr, no Vivado) is a hard requirement:
- 44,500 LUTs — fits with minimal headroom
- GbE SerDes integration on the ECP5 open toolchain is significantly harder than Vivado TEMAC;
  expect 4–6 additional weeks of integration effort
- Suitable for a rev-2 board once the HDL design is proven on Artix-7

**Verdict**: Use XC7A100T for the first PCB.

---

### Supporting ICs

#### GbE PHY

The FPGA implements the MAC layer (via TEMAC IP); a separate PHY chip handles the analog
physical layer and provides the RGMII interface.

Recommended options (both well-supported in Xilinx reference designs):
- **Marvell 88E1111** — industry standard, extensive documentation, RGMII
- **Microchip KSZ9031RNX** — lower cost, also RGMII, widely used in hobbyist designs

#### Master Clock — 12.288 MHz TCXO

A temperature-compensated oscillator (TCXO) is required rather than a plain crystal because
sample-rate drift accumulates in the CSM over long captures and corrupts coherence.

The 12.288 MHz frequency divides cleanly to all required audio clocks:

| Derived clock | Division | Value |
|---|---|---|
| PDM clock to mics | ÷ 4 | **3.072 MHz** (exact) |
| FPGA PLL output | × 4 | **49.152 MHz** |
| PCM word select (WS) | ÷ 1024 | **48.000 kHz** (exact) |

Recommended: any 12.288 MHz TCXO in ±2.5 ppm or better (e.g., NDK NZ2520SD, TXC 7M series).

---

## Array Geometry

Same as established in Phase 1 simulation (nb05–nb11):

| Parameter | Value |
|---|---|
| Mic count | 96 |
| Pattern | Underbrink multi-arm log-spiral |
| Arms × mics per arm | 8 × 12 (simulate 6 × 16 as alternative) |
| Aperture | ~300 mm diameter |
| Min mic spacing | ~21 mm |
| Spatial Nyquist | ~8.2 kHz (c / 2×0.021) |
| Far-field distance | ~0.37 m @ 8 kHz |

The geometry was optimized in Phase 1 simulation. The Underbrink pattern is preferred over a
regular grid because its logarithmic spiral arm spacing suppresses grating lobes across the full
frequency range, giving better sidelobe performance at low mic count.

---

## FPGA / Host Partition

The FPGA does only what a CPU cannot: real-time parallel PDM capture and decimation of 96
channels at 3 MHz. Everything flexible or algorithmically complex stays on the host in Python.

### FPGA responsibilities (hard real-time, parallel)

| Block | Detail |
|---|---|
| **PDM clock generation** | 12.288 MHz TCXO → PLL → 3.072 MHz; fanned out to all 96 mics with matched-length traces |
| **PDM capture** | 48 data input lines; data latched at each PDM clock edge |
| **L/R demux** | Each data line carries 2 mics (SEL low → even channel, SEL high → odd channel) |
| **CIC decimation** | 5-stage CIC, 64:1 per channel; 3.072 MHz → 48 kHz |
| **FIR compensation** | ~32-tap linear-phase FIR per channel; corrects CIC passband droop |
| **Sample alignment** | All 96 PCM outputs locked to the same 48 kHz word-select boundary |
| **GbE/UDP packetization** | Assemble N frames × 96 channels; prepend sequence number + timestamp; send over RGMII to PHY |
| **PPS input** (optional) | 1 Hz GPIO for absolute time-tagging; enables future multi-unit synchronization |

### Host responsibilities (flexible, Python)

| Block | Detail |
|---|---|
| **UDP ingestion** | Python socket; check sequence numbers; log packet drops |
| **Audio ring buffer** | Thread-safe deque of incoming PCM frames |
| **Cross-spectral matrix** | Windowed FFT → outer product → running average over N snapshots |
| **Beamforming** | D&S, MVDR, CLEAN-SC, MUSIC (reuse `src/acoustic_camera_p3.py` code as base) |
| **GPU acceleration** | Optional PyTorch / CuPy for CSM and beamforming at 96-channel scale |
| **Calibration** | Gain + phase correction vector applied to CSM: `R_cal = outer(1/e, conj(1/e)) * R` |
| **Camera capture** | OpenCV `VideoCapture` |
| **Heatmap rendering** | Power → dB → percentile normalize → colormap → resize to frame |
| **Video overlay + display** | `cv2.addWeighted`, crosshair, status label, frequency sliders |
| **Record / playback** | Write raw PCM packets + video frames; replay from file for offline analysis |

---

## Hardware Sub-Tasks

These can be parallelized once the HDL is stable on the Arty A7-100T dev board.

| Sub-task | Description | Dependency |
|---|---|---|
| **HDL development** | CIC + FIR + GbE pipeline in Verilog/VHDL; test on Arty A7-100T | None |
| **FPGA hub PCB** | Custom board: XC7A100T + 88E1111 PHY + 12.288 MHz TCXO + 48-pin PDM input header | HDL stable |
| **Mic array PCB** | 96× IM72D128 in Underbrink spiral; PDM clock fan-out; 48 data lines to FPGA header | Geometry finalized |
| **Host software** | UDP ingestion + 96-ch pipeline (extend `acoustic_camera_p3.py`) | HDL producing valid UDP |
| **Camera selection** | USB camera, FOV matched to array aperture at expected working distance | — |
| **Calibration** | Gain + phase estimation at scale; extend nb17 approach | Hardware assembled |

---

## PCB Bring-Up Checklist

*Populate as bring-up proceeds.*

- [ ] FPGA Artix-7 powers up; JTAG accessible
- [ ] 12.288 MHz TCXO oscillating (verify with scope)
- [ ] PLL locked; 3.072 MHz PDM clock present on header pin
- [ ] Single mic connected; PDM data line shows valid 1-bit stream on ILA
- [ ] CIC output produces 48 kHz PCM samples (verify with ILA + known tone)
- [ ] FIR compensation: flat frequency response confirmed on single channel
- [ ] All 48 data lines active; 96 channels valid
- [ ] GbE PHY link up (LED)
- [ ] UDP packets received on host; no sequence gaps
- [ ] Host pipeline: 96-channel CSM computed; beamform produces coherent energy map
- [ ] Calibration: cross-correlation gain/phase vectors captured and applied

---

## Key References

- Ben Wang, "192-channel phased array microphone" (2023) — similar FPGA + GbE architecture
- Underbrink multi-arm log-spiral array patent — geometry basis
- Xilinx TEMAC IP core product guide (PG051) — GbE MAC integration
- Infineon IM72D128 datasheet — mic specs and PDM timing
- `alexforencich/verilog-ethernet` — open-source GbE MAC (alternative to TEMAC)
