# fpga/

Verilog for the acoustic camera's FPGA front-end. Two independent hub architectures for the
same 96-mic array (see `DESIGN.md`'s "FPGA Choice" section for the full tradeoff writeup and
`PHASE4.md` for the current phase's hardware decisions):

## Multiple FPGA (`multi_fpga/`) — primary design

5 small Xilinx 7-series FPGAs: 4 "cluster" FPGAs (Cmod A7-35T, one per 3-arm/24-mic
quadrant) + 1 "hub" FPGA (also Cmod A7-35T) that aggregates all 4 spokes and bridges to a
Raspberry Pi 5 over USB. Fully implemented and simulated bit-exact; synthesizes, places, and
routes clean on real hardware targets (cluster: 71% LUT; hub: 9.8% LUT / 18.4% register,
both with timing margin). Breadboard bring-up (hub + one cluster + one mic) is in progress —
reset/LED handshake and the audio path confirmed on real hardware, USB streaming to a host is
next. See `multi_fpga/README.md` for the full breakdown.

## Single FPGA (`single_fpga/`) — documented alternate

One larger FPGA (Xilinx Artix-7 XC7A200T, on an ALINX AC7200 module) handling all 96
channels directly and bridging to the host over Gigabit Ethernet/UDP instead of USB — no
inter-chip spoke bus, so no cross-board sample-alignment problem to solve. Evaluated at
roughly $240/unit cheaper in BOM cost than the multi-FPGA design, but not the currently-chosen
design — kept and developed as a real, working alternate, staged in three parts:

- **Stage 1** (96-channel CIC+FIR pipeline alone, `rtl/single_fpga_top_spike.v`): real placed
  result **43.52% Slice LUTs** on `xc7a200tfbg484-1`.
- **Stage 2** (+ UDP packetizer + vendored open-source 1G RGMII MAC,
  `rtl/gbe_packetizer.v` + `rtl/third_party/verilog-ethernet/`): real placed result
  **48.14% Slice LUTs**, 12.97% DSP48E1, 20.70% Bonded IOB, control sets a clean 1.54% (down
  from 27.59%/"required reduction" before a payload-memory redesign — see
  `rtl/gbe_packetizer.v`'s header comment for what changed and why). Protocol documented in
  `GBE_FRAMING.md`; golden model + two bit-exact testbenches in `golden/`/`sim/`
  (`sim-gbe-packetizer`, `sim-gbe-pipeline`).
- **Stage 3** (carrier-board PCB for the AC7200 module) has not been started — see
  `pcb/single_fpga/`.

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
