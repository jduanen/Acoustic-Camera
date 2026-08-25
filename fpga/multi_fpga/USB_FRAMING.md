# USB FIFO framing protocol (v1)

Defines how the hub FPGA (Cmod A7-35T) frames the 4 spokes' reassembled channel data onto
the synchronous FT245 FIFO link to the FT232H bridge (component A6 on `hub.kicad_sch`), for
the Pi 5 to read over USB. Mirrors `SPOKE_FRAMING.md`'s style/precision -- written so the
Pi 5-side reader can be built against this doc alone, without reading `usb_framer.v`.

Implementation: `fpga/multi_fpga/hub/rtl/usb_framer.v`, instantiated once in `hub_top.v`, fed by the 4
`spoke_deframer.v` instances' `chN_data_flat`/`chN_valid` outputs. Golden reference (source
of truth for testbench comparison): `fpga/multi_fpga/hub/golden/usb_framer_golden.py`.

## Why per-spoke records, not one unified 96-channel frame

The 4 spokes are deframed fully independently (`spoke_deframer.v` x4, no cross-spoke
alignment attempted -- see `SPOKE_FRAMING.md`'s "Hub-side deframer" section) and nothing in
this project has calibrated inter-spoke cable-length skew yet. A framing format that implied
"here are 96 channels from one instant" would be asserting something not actually true.
Instead, each spoke's completed frame is sent as its own tagged, independently-sequenced
record; reconciling spokes/timestamps into a single 96-channel snapshot is host-side work,
deferred along with the calibration itself.

## FT245 synchronous FIFO write interface

The FT232H, configured for synchronous 245 FIFO mode, presents `CLKOUT` (a 60MHz clock the
chip generates -- not the hub's own TCXO-derived clock; see `hub_top.v`'s `usb_clk` domain)
as the timing reference for all FIFO transfers. This design only ever writes to the FIFO (no
host-to-hub channel exists), so:

- `USB_D0-7` is driven by the FPGA at all times -- never read back.
- `USB_WR_N` pulses low for one `usb_clk` cycle per byte written.
- `USB_TXE_N` (driven by the FT232H) is sampled each `usb_clk` cycle: low means the FT232H's
  TX FIFO has room for another byte; high stalls the write (the FPGA holds its current byte,
  `USB_WR_N` deasserted, and simply waits -- see "Backpressure" below).
- `RD#`/`OE#` (the FT232H's read-direction control lines) aren't wired to the FPGA at all --
  since the FPGA never reads, they're tied permanently deasserted with pull-ups on the hub
  PCB instead of spending FPGA pins on constants that never toggle.

Both `USB_D0-7` and `USB_WR_N` are ordinary registered outputs, clocked by `usb_clk` --
already valid for a full `usb_clk` period before the FT232H's own next sampling edge, no
extra pipelining needed.

## Record format

76 bytes, one per spoke per completed `spoke_deframer` frame (i.e. one per spoke per 48kHz
sample period, absent backpressure drops -- see below):

| Bytes | Field | Notes |
|---|---|---|
| 0-1 | Sync `0xA5 0x5A` | Fixed marker. Records are fixed-length (not delimited), so this is purely a framing aid for a host that's lost byte alignment -- no in-band escaping needed |
| 2 | `spoke_id` | 0-3 |
| 3 | `seq_num` | 8-bit, rolls over, increments per spoke per record -- lets the host detect drops independently per spoke |
| 4-75 | 24 channels x 3 bytes | Channel 0 first, ascending order; each channel's 24-bit sample MSB-first (bits `[23:16]`, `[15:8]`, `[7:0]`) |

Rate check: 76 bytes x 48000/s x 4 spokes x 8 bits/byte = **116.7 Mbps** -- well under sync
mode's 320 Mbps (40 MB/s) capability, 36% utilized including header overhead.

## Arbitration

All 4 spokes' completed frames cross into the `usb_clk` domain independently (a toggle-bit +
2-FF-synchronizer + edge-detect handshake per spoke -- see "Clock domain crossing" below) and
compete for the single FT245 write interface. A rotating-priority arbiter (last-served spoke
lowest priority next time) selects among pending spokes and serializes one complete 76-byte
record before considering the next request -- non-preemptive, one spoke fully drained before
the next starts.

## Clock domain crossing

`usb_clk` (from the FT232H's own `CLKOUT`) has no fixed phase relationship to the hub's
TCXO-derived `clk`/`sclk` domains -- genuinely asynchronous, unlike `sclk` (which is
*generated from* `clk`). Each spoke's `valid` pulse (in `sclk`) flips a toggle flip-flop;
`usb_clk` recovers a one-cycle request pulse from that toggle via a standard 2-FF
synchronizer + edge detect. The 576-bit channel data itself needs no per-bit synchronizer --
`spoke_deframer.v` holds `ch_data_flat` stable until its own next `valid` pulse, so once the
synchronized request pulse says "safe to read," the data underneath it is already stable
(same technique `clk_reset.v`'s `pdm_phase` handling relies on for a different signal).

## Backpressure / drop behavior

Serializing one 76-byte record at 60MHz (1 byte/`usb_clk` cycle when `TXE#` stays low) takes
~1.3us -- about 16x margin under the 20.8us (48kHz) production period, even servicing all 4
spokes back-to-back worst case (~5.1us). Under normal operation there's no realistic risk of
a spoke's next frame arriving before its previous record has been drained.

If `TXE#` backpressure ever stalls long enough to violate that margin, the affected spoke's
**new** channel data simply overwrites the old, not-yet-served copy still waiting for the
arbiter -- the stale record is dropped, not queued. This is a documented limitation, not
solved here: it's a symptom of the USB link being oversubscribed (host not draining fast
enough), and this design deliberately doesn't build deep per-spoke buffering to paper over
that -- the host can detect the gap via `seq_num`'s skip.

## Host-side responsibilities (out of scope for this doc's implementation)

- Byte-stream sync recovery (locate `0xA5 0x5A`, then read fixed 76-byte records).
- Per-spoke `seq_num` gap detection.
- Cross-spoke alignment / cable-skew calibration -- not attempted anywhere in this project
  yet (see `SPOKE_FRAMING.md`).

`fpga/multi_fpga/host/usb_stream_test.py` is a reference implementation of the first two (sync recovery
+ gap detection), plus per-channel WAV dump -- written as a breadboard bring-up diagnostic,
not the production Pi 5 reader.
