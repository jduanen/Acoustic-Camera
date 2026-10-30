# fpga/

This directory contains the Verilog for the acoustic camera's FPGA front-end.
There are currently two independent implementations of the same 96-mic array
(see `DESIGN.md`'s "FPGA Choice" section for the full tradeoff writeup and
`PHASE4.md` for the current phase's hardware decisions):

## Multiple FPGA (`multi_fpga/`) — primary design

5 small Xilinx 7-series FPGAs: 4 "cluster" FPGAs (Cmod A7-35T, one per 3-arm/24-mic
quadrant) + 1 "hub" FPGA (also Cmod A7-35T) that aggregates all 4 spokes and bridges to a
Raspberry Pi 5 over USB. Fully implemented and simulated bit-exact; synthesizes, places, and
routes clean on real hardware targets (cluster: 71% LUT; hub: 9.8% LUT / 18.4% register,
both with timing margin). Breadboard bring-up (hub + one cluster + one mic) is in progress —
reset/LED handshake and the audio path confirmed on real hardware, USB streaming to a host is
next. See `multi_fpga/README.md` for the full breakdown.

## Single FPGA (`single_fpga/`) — alternate design being explored

One larger FPGA (Xilinx Artix-7 XC7A200T, on an ALINX AC7200 module) handling all 96
channels directly and bridging to the host over Gigabit Ethernet/UDP instead of USB.
There is no inter-chip spoke bus, so no cross-board sample-alignment problem to solve.
This has been evaluated at roughly $240/unit cheaper in BOM cost than the multi-FPGA
design, but not the currently-chosen design; it's being kept and developed as a real,
working alternate, staged in three parts:

- **Stage 1** (96-channel CIC+FIR pipeline alone, `rtl/single_fpga_top_spike.v`): real placed
  result **43.52% Slice LUTs** on `xc7a200tfbg484-1`
- **Stage 2** (+ UDP packetizer + vendored open-source 1G RGMII MAC, `rtl/gbe_packetizer.v` + `rtl/third_party/verilog-ethernet/`): real placed result
  **48.14% Slice LUTs**, 12.97% DSP48E1, 20.70% Bonded IOB, control sets a clean 1.54% (down
  from 27.59%/"required reduction" before a payload-memory redesign; see
  `rtl/gbe_packetizer.v`'s header comment for what changed and why). Protocol documented in
  `GBE_FRAMING.md`; golden model + two bit-exact testbenches in `golden/`/`sim/`
  (`sim-gbe-packetizer`, `sim-gbe-pipeline`).
- **Stage 3** (carrier-board PCB for the AC7200 module) has not been started — see
  `pcb/single_fpga/`.
- **Board top** (`rtl/single_fpga_top.v`, constrained by `xdc/single_fpga_top.xdc`): TCXO_CLK
  in, PDM_CLK = TCXO/4 out, power-on reset, and the front-end status LED (`rtl/status_led.v`:
  YELLOW in reset, RED on a PHY configuration fault, GREEN running, GREEN/BLUE alternating
  while packets are going out;
  outputs active high; `sim-status-led`), and DATA sampling (`rtl/pdm_sampler.v`: the 48
  mic lines captured in IOB flip-flops two TCXO cycles after each PDM_CLK edge, split into
  L/R bits with a once-per-period `pdm_valid`; input delays and multicycle in the XDC;
  `sim-pdm-sampler`), feeding the 96-channel CIC+FIR+packetizer pipeline
  (`rtl/single_fpga_pipeline_top.v`, now taking separated L/R bits) on a 3.072 MHz
  `pipe_clk` gated from TCXO_CLK by a BUFGCE, and the vendored RGMII MAC sending its UDP
  frames to the DP83867 (125 MHz + 90-degree clocks from the module's 200 MHz SYS_CLK via an
  MMCM; on receive -- RX inputs centred by fixed IDELAYs -- `rtl/arp_responder.v` answers
  ARP for the board's IP and slips the replies in between UDP packets; PHY_RESET_N pulsed after configuration; `rtl/mdio_init.v` then sets the
  PHY to advertise 1000 Mb/s only over MDIO and reads it back, lighting the LED red if it
  didn't take; `sim-mdio-init`, against `sim/mdio_phy_model.v`). `sim-board-top`
  checks it from the pins: modelled mics on DATA, all 96 FIR outputs bit-exact, and the
  RGMII TX frames decoded back to bytes (preamble, body, FCS) plus the PHY's TXC-to-data
  timing, with ARP requests driven into RGMII RX and the replies checked on TX.
  `make build-top` runs `vivado/build_top.tcl` through to a bitstream
  (`vivado/build_top/single_fpga_top.bit`, plus routed timing/utilization/IO reports there).
  Routed: 43.5% Slice LUTs, 13.8% registers, 96 DSP48E1, 1 RAMB36 (the packetizer's
  ping-pong payload memory, three channels per 72-bit word), timing met on xc7a200t-2.

The MAC is `alexforencich/verilog-ethernet` (MIT license), vendored unmodified — see
`rtl/third_party/verilog-ethernet/README.md` for provenance and why it was chosen over
Xilinx's own Tri-Mode Ethernet MAC IP (blocked by a license entitlement this project doesn't
have) and other open-source alternatives.

## Running the simulations

Both trees' testbenches run from the repo root:

    make sim-all       # every RTL testbench, both trees
    make golden-test   # just the Python golden-model unit tests
    make clean-sim     # remove xsim build artifacts

See the top-level `Makefile` for the full per-module target list (`sim-cic`, `sim-hub-top`,
`sim-gbe-pipeline`, etc.) and each tree's own `README.md`/protocol docs (`multi_fpga/
SPOKE_FRAMING.md`, `multi_fpga/USB_FRAMING.md`, `single_fpga/GBE_FRAMING.md`) for
implementation detail.
