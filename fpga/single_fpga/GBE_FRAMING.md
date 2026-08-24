# GbE/UDP host framing protocol (v1)

Defines how the single-FPGA hub (ALINX AC7200 module, XC7A200T) packetizes its 96 channels'
decimated 48 kHz PCM data into UDP datagrams sent over Gigabit Ethernet (RGMII) to the Pi 5.
Mirrors `SPOKE_FRAMING.md`/`USB_FRAMING.md`'s style/precision -- written so the Pi 5-side
reader can be built against this doc alone, without reading `gbe_packetizer.v`.

Implementation: `fpga/single_fpga/rtl/gbe_packetizer.v`, driven by the CIC+FIR pipeline
(`single_fpga_top_spike.v`'s reused `pdm_line_demux`/`cic_decimator`/`fir_compensator` chain
-- see that file's header for the pipeline itself), feeding `tx_axis_*` on the vendored
`fpga/single_fpga/rtl/third_party/verilog-ethernet/eth_mac_1g_rgmii.v` MAC. No golden
model/testbench yet -- see the plan file's Stage 2 status for what's still pending.

## Why transmit-only, fixed-address UDP

Same "don't over-build" precedent `usb_framer.v` already established for the multi-FPGA
hub's host link, and confirmed with you earlier in this design's own scoping: this is a
**point-to-point hub-to-Pi5 GbE link, no switch in between** (see the plan file's Single-FPGA
hub section). That means:

- No ARP responder, no DHCP, no ICMP -- destination MAC/IP/port are fixed at build time
  (module parameters), not learned or negotiated.
- UDP checksum set to `0x0000` (disabled) -- valid and standard per RFC 768 over IPv4,
  avoiding a running accumulator with zero benefit on a link with no intermediate routers to
  corrupt payload silently.
- The IPv4 header checksum (which RFC 791 does *not* allow disabling) turns out to need no
  runtime computation at all here -- see "IP header checksum is a build-time constant" below.
- Everything channel-count/architecture-specific this doc assumes (96 channels, one chip, no
  inter-chip hop) only holds for this single-FPGA design -- it's a different framing problem
  from `SPOKE_FRAMING.md`/`USB_FRAMING.md`'s multi-FPGA link, not a replacement for either.

## Real architectural advantage over the multi-FPGA link: no cross-spoke skew

`SPOKE_FRAMING.md` explicitly defers cross-spoke sample alignment/cable-skew calibration --
each of the multi-FPGA hub's 4 spokes is deframed independently, with no shared notion of
"this sample, from all 4 boards, was taken at the same instant." The single-FPGA design has
no such problem: all 96 channels run through fully parallel, identically-parameterized
CIC+FIR instances on one chip (`single_fpga_top_spike.v`), so every channel's `valid` pulse
fires on the same clock cycle by construction -- there is no cross-board cable-length skew to
calibrate, because there's no second board. Worth calling out explicitly since it's a real
simplification this architecture gets for free, not something this protocol had to solve.

## Ethernet MAC handles preamble, FCS, and padding -- confirmed from the vendored MAC's source

Confirmed by reading `axis_gmii_tx.v`'s state machine (not assumed): `STATE_PREAMBLE` emits
the 7-byte preamble + SFD automatically, `STATE_FCS` computes and transmits the CRC-32 frame
check sequence automatically (`~crc_state`, using the `lfsr`-based CRC engine already
documented in `third_party/verilog-ethernet/README.md`), and `STATE_PAD`/`ENABLE_PADDING`
pad any frame under 64 bytes automatically (never triggers here -- this design's frames are
always 1494 bytes). **`gbe_packetizer.v` supplies only the raw frame body starting at the
destination MAC address's first byte, via `tx_axis_tdata`/`tvalid`/`tlast`/`tuser`, and never
computes or appends FCS itself.**

## IP header checksum is a build-time constant, not runtime logic

Every field in this design's 20-byte IPv4 header is fixed at build time: source/destination
IP (parameters), total length (every packet carries exactly 5 frames -- see "Packet batching"
below -- so length never varies), protocol (17/UDP), TTL, DSCP, and `Identification` (set to
`0x0000` -- valid per RFC 791 since `DF=1` and this link never fragments, so the field is
never actually used by a receiver). Since nothing in the header varies packet-to-packet, its
checksum doesn't either -- `gbe_packetizer.v` computes it once, at elaboration time, via a
Verilog function over the module's `SRC_IP`/`DST_IP` parameters (so it stays correct
automatically if those addresses ever change), not as per-packet runtime logic. No checksum
adder exists anywhere in the design.

## Packet batching

Each CIC+FIR "frame" (one 48 kHz sample period, all 96 channels) is 288 bytes of payload (96
x 24-bit, MSB-first per channel -- same convention as `SPOKE_FRAMING.md`/`USB_FRAMING.md`).
Sending one UDP packet per frame would mean a 42-byte header (14 Ethernet + 20 IP + 8 UDP) on
every 288-byte payload -- a real per-packet header write, but also a 48,000 pps rate.
Instead, **5 frames are batched into one UDP datagram**:

- Standard (non-jumbo) MTU budget: 1500-byte IP total length - 20 (IP) - 8 (UDP) = 1472-byte
  UDP payload ceiling.
- This design's UDP payload = 4-byte `seq_num` + 8-byte `timestamp` + 5 x 288-byte frames =
  **1452 bytes** -- under the 1472-byte ceiling, no jumbo frames needed.
- Ethernet frame size on the wire (before preamble/FCS, which the MAC adds): 14 + 20 + 8 +
  1452 = **1494 bytes** -- under the standard 1518-byte (1500 MTU + 14 header, no VLAN)
  maximum frame size with margin.
- Packet rate: 48,000 Hz / 5 frames-per-packet = **9,600 pps**.
- On-wire rate: 1494 bytes x 8 bits x 9,600 pps = **~114.7 Mbps**, ~11.5% of GbE line rate.

## Byte layout

### Ethernet + IP + UDP header (42 bytes, byte-identical on every packet)

| Bytes | Field | Value |
|---|---|---|
| 0-5 | Destination MAC | `DST_MAC` parameter (fixed, build-time) |
| 6-11 | Source MAC | `SRC_MAC` parameter (fixed, build-time) |
| 12-13 | EtherType | `0x0800` (IPv4) |
| 14 | IP Version/IHL | `0x45` |
| 15 | IP DSCP/ECN | `0x00` |
| 16-17 | IP Total Length | `0x05C8` (1480 = 20 + 8 + 1452, constant every packet) |
| 18-19 | IP Identification | `0x0000` (unused -- `DF=1`, never fragmented) |
| 20-21 | IP Flags/Fragment Offset | `0x4000` (Don't Fragment, offset 0) |
| 22 | IP TTL | `64` |
| 23 | IP Protocol | `17` (UDP) |
| 24-25 | IP Header Checksum | computed at elaboration time -- see above |
| 26-29 | Source IP | `SRC_IP` parameter (fixed, build-time) |
| 30-33 | Destination IP | `DST_IP` parameter (fixed, build-time) |
| 34-35 | UDP Source Port | `SRC_PORT` parameter (fixed, build-time) |
| 36-37 | UDP Destination Port | `DST_PORT` parameter (fixed, build-time) |
| 38-39 | UDP Length | `0x05B4` (1460 = 8 + 1452, constant every packet) |
| 40-41 | UDP Checksum | `0x0000` (disabled, valid per RFC 768) |

### UDP payload (1452 bytes, varies per packet)

| Bytes | Field | Notes |
|---|---|---|
| 0-3 | `seq_num` | 32-bit, increments once per **packet** (not per frame), rolls over |
| 4-11 | `timestamp` | 64-bit sample-period counter value at the first of this packet's 5 frames, increments once per **frame** (48 kHz), free-running from power-up |
| 12-1451 | 5 x 288-byte frames | Frame 0 first, frames in arrival order. Within each frame: 96 channels x 3 bytes, channel 0 first ascending, each 24-bit sample MSB-first -- identical convention to `SPOKE_FRAMING.md`/`USB_FRAMING.md`'s payload layout |

## Clock domain crossing

The CIC+FIR pipeline runs on the system clock (`clk`); the MAC's TX path runs on `tx_clk`
(from `eth_mac_1g_rgmii.v`'s own output, GMII-side, genuinely asynchronous to `clk` --
same "no fixed phase relationship" situation `USB_FRAMING.md` already documents for
`usb_clk`). `gbe_packetizer.v` uses the same technique already established for that
crossing: a double-buffered (ping-pong) packet payload memory, written one frame at a time in
the `clk` domain, with a toggle flip-flop flipped once per completed 5-frame batch;
`tx_clk` recovers a one-shot "packet ready" pulse via a standard 2-FF synchronizer + edge
detect (same pattern as `usb_framer.v`'s per-spoke toggle sync). The payload bytes underneath
need no per-bit synchronizer, only the toggle -- same "data held stable, only the pulse needs
synchronizing" reasoning `USB_FRAMING.md` already documents, just at batch (not per-frame)
granularity: a batch takes 5 x 1/48000s = ~104us to accumulate, while serializing the
previous batch's 1494-byte frame at 1 byte/`tx_clk` cycle (125 MHz GMII clock) takes
~12.0us -- about **8.7x margin**, comfortably faster than the next batch closes, so (as with
`USB_FRAMING.md`'s own margin note) there's no realistic risk of the write side starting a
new bank before the read side has finished draining the previous one at this design's data
rate.

## Host-side responsibilities (out of scope for this doc's implementation)

- UDP socket bound to `DST_PORT`, reading fixed-size 1452-byte UDP payload datagrams (the
  OS/NIC handles Ethernet/IP/UDP header stripping and FCS verification -- the host
  application only ever sees the 1452-byte UDP payload).
- `seq_num` gap detection (32-bit, packet-granularity).
- `timestamp` is already sample-aligned across all 96 channels by construction (see the
  "real architectural advantage" section above) -- no cross-channel reconciliation needed,
  unlike the multi-FPGA link's cross-spoke alignment problem.
