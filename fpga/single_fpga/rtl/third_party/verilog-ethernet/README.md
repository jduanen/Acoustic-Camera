# Vendored: alexforencich/verilog-ethernet (1G RGMII MAC subset)

Pulled from <https://github.com/alexforencich/verilog-ethernet>, commit
`77320a9471d19c7dd383914bc049e02d9f4f1ffb` (2025-02-27), MIT license (see
`COPYING`, copied unmodified from the source repo).

Used to replace Xilinx's Tri Mode Ethernet MAC IP core (blocked by a real
IP-specific license entitlement this project didn't have -- see the plan
file's "Single-FPGA hub (ALINX AC7200 module)" section, Stage 2, for the
full story) for `fpga/single_fpga/`'s GbE host interface. Chosen over the
repo's own actively-maintained successor, `fpganinja/taxi`, specifically
because this repo is MIT-licensed (no reciprocal/share-source obligation,
unlike `taxi`'s CERN-OHL-S 2.0) and plain Verilog (matching every other
module in this project, unlike LiteEth's Migen/Python-generated approach).
"Deprecated" (per the source repo's own README) means no new features/fixes
upstream, not that the code is broken -- 1G RGMII MAC is a fixed, mature
protocol, so this is a low-risk trade for this project's purposes.

**Files are UNMODIFIED** from upstream (byte-for-byte, straight `curl` pulls
at the commit above) -- do not hand-edit them in place. If a real bug is
found, either patch it via a small wrapper module in
`fpga/single_fpga/rtl/` (not here) or re-vendor from a newer upstream
commit and update this file's commit hash.

## Files (minimal set for a transmit-mostly 1G RGMII MAC, `MAC_CTRL_ENABLE=0`)

- `eth_mac_1g_rgmii.v` -- top-level wrapper this project instantiates directly.
  Instantiates:
  - `rgmii_phy_if.v` -- RGMII-to-GMII conversion (DDR capture/output on the
    RGMII bus). Instantiates:
    - `ssio_ddr_in.v` -- DDR input capture (`TARGET="XILINX"`,
      `IODDR_STYLE="IODDR"` for 7-Series, see that file's own header comment).
      Itself instantiates `iddr.v` (Xilinx `IDDR` primitive wrapper --
      confirmed a real dependency by an actual xelab elaboration failure
      when first vendoring this, not caught by reading the top-level's own
      instantiation list alone).
    - `oddr.v` -- DDR output (same `TARGET`/`IODDR_STYLE`).
  - `eth_mac_1g.v` -- core MAC logic (frame gap, preamble/SFD, FCS). With
    `MAC_CTRL_ENABLE=0` (this project's setting -- no 802.3x pause/flow
    control needed for a fixed-rate transmit-mostly link), does NOT need
    `mac_ctrl_tx.v`/`mac_ctrl_rx.v`/`mac_pause_ctrl_tx.v`/
    `mac_pause_ctrl_rx.v` from upstream -- not vendored here, since they'd
    be dead code for this configuration. Instantiates:
    - `axis_gmii_tx.v` -- AXI-Stream-to-GMII framing (TX path this project
      actually drives).
    - `axis_gmii_rx.v` -- GMII-to-AXI-Stream deframing (RX path -- present
      because `eth_mac_1g.v` is a combined bidirectional MAC, not
      separately instantiable TX-only, but this project's own top-level
      never drives real RGMII RX data or reads `rx_axis_*`; a few LUTs of
      unused RX logic is an accepted, minor tradeoff for using this
      well-tested block unmodified rather than forking it).
      - `lfsr.v` -- parameterized CRC engine, used by both `axis_gmii_tx.v`
        and `axis_gmii_rx.v` for Ethernet FCS (instantiated under the local
        name `eth_crc_8`/similar in each -- same shared module either way).

## Not vendored (deliberately, per the file list above)

`mac_ctrl_tx.v`, `mac_ctrl_rx.v`, `mac_pause_ctrl_tx.v`, `mac_pause_ctrl_rx.v`
(802.3x flow control, `MAC_CTRL_ENABLE=0` disables the `generate` block that
would need them) -- and everything else in upstream's `rtl/` (10G/25G MACs,
the full UDP/IP/ARP stack, PTP timestamping, etc.) -- not needed for this
project's minimal, fixed-address, transmit-mostly framing (see
`fpga/single_fpga/GBE_FRAMING.md`), same "don't over-build" convention this
project already follows elsewhere (e.g. `usb_framer.v`).
