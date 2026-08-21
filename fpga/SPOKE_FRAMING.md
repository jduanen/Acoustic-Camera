# Spoke bus framing protocol (v1)

Defines how a cluster FPGA (Cmod A7-35T) frames its 24 channels' decimated 48 kHz
PCM data onto the 6-wire spoke bus back to the hub FPGA (Cmod A7-35T). This
did not exist anywhere before this pass -- `pcb/multi_fpga/SCHEMATIC_NOTES.md`
and `PHASE4.md` both flagged the exact framing semantics as undesigned. It's
written precisely enough that a future hub-side deframer (out of scope for
`fpga/cluster/`) can be built against it independently, without needing to
read `spoke_framer.v`.

Implementation: `fpga/cluster/rtl/spoke_framer.v`. Golden reference (source of
truth for testbench comparison): `fpga/cluster/golden/spoke_framer_golden.py`.

## Why DDR is mandatory, not a design choice

*(Historical: this section's numbers are as originally derived, when
`SPOKE_CLK` was 3.072 MHz. It later doubled to 6.144 MHz when the cluster's
CIC decimator moved to a time-multiplexed L/R design (`cic_decimator_shared.v`)
-- see the note at the end of this section for what that changes and, just
as importantly, what it doesn't.)*

The only clock available is `SPOKE_CLK`, forwarded from the hub at 3.072 MHz
(no local PLL on the cluster -- see `PHASE4.md`'s "hub-and-spoke, single
shared clock domain" architecture). Payload is 24 channels x 48 kHz x 24-bit
= 27.648 Mbps, over the 6-wire `SPOKE_D0..D5` bus.

At single data rate (one 6-bit nibble per rising edge only): 6 bits x
3.072M edges/s = 18.432 Mbps max -- **does not fit**. Equivalently: 24ch x
24bit / 6 bits-per-transfer = 96 clock cycles needed per 48 kHz frame period,
but only 64 clock cycles exist in that period (3.072 MHz / 48 kHz = 64).
96 > 64, so SDR cannot carry this payload at all, regardless of encoding
efficiency.

At double data rate (a nibble on *both* the rising and falling edge of
`SPOKE_CLK`): 12 bits per clock period x 3.072M = 36.864 Mbps available,
27.648 Mbps payload fits with 25% margin. In cycle terms: 576 total bits /
frame / 12 bits-per-period = 48 clock periods busy, 16 idle, out of the 64
available.

This also reconciles with `SCHEMATIC_NOTES.md`'s "~4.6 MHz per wire, 5.4x
under the 25 MHz electrical cap" figure -- that's new-bits-per-wire-per-second
(27.648 Mbps / 6 wires = 4.608 Mbit/s/wire), not a literal toggle rate; with
DDR at 3.072 MHz the actual pin toggle rate stays <= 6.144 MHz, still
comfortably under the Cmod modules' 25 MHz series-resistor-imposed cap.

**Current state: `SPOKE_CLK` is 6.144 MHz, not 3.072 MHz.** The cluster's CIC
decimator shares one arithmetic path between each line's L and R channels
(`cic_decimator_shared.v`), so each channel only gets a new input sample
every *other* `SPOKE_CLK` cycle -- the fabric clock had to double to keep the
same real-world 3.072 MHz PDM sample rate and 48 kHz decimated output rate.
This *loosens* the bandwidth argument above (at 6.144 MHz even SDR would now
fit, with margin) but doesn't change the protocol: `spoke_framer.v` still
implements DDR exactly as designed, since it already worked and there was no
reason to rip it out. What *does* change is the frame period in cycle terms
-- see "Frame structure" below.

## Signals

| Signal | Direction (cluster -> hub) | Meaning |
|---|---|---|
| `SPOKE_D0`..`SPOKE_D5` | out | 6-bit DDR data bus |
| `SPOKE_STROBE` | out | frame marker, see below |
| `SPOKE_CLK` | in (not part of this doc's output side) | 6.144 MHz, forwarded from hub |

## Frame structure

One frame = one 48 kHz sample period = **128** `SPOKE_CLK` periods (6.144 MHz
/ 48 kHz), split into 48 busy cycles + **80** idle cycles. `BUSY_CYCLES` (48)
is unchanged from the original 3.072 MHz design -- it only depends on the DDR
bus width and payload (24ch x 4 nibbles/ch / 2 nibbles-per-DDR-period), not
on the clock rate. Only the idle-cycle count grew, since the frame period in
*time* (20.83us, set by the 48 kHz decimated sample rate) didn't change but
there are now twice as many `SPOKE_CLK` cycles in that same window.

- **Channel order**: fixed ascending physical index 0..23, matching the
  existing `data_line*2 + {L=0,R=1}` convention already used elsewhere in
  this project (`test/phase4/data_line_assignment.csv`) -- not a new
  numbering.
- **Per-channel encoding**: each channel's 24-bit PCM sample is sent as 4
  MSB-first 6-bit nibbles (bits `[23:18]`, `[17:12]`, `[11:6]`, `[5:0]`) -- 6
  divides 24 evenly, no padding needed. 4 nibbles / 2 nibbles-per-clock-period
  (DDR) = **2 clock periods per channel**. 24 channels x 2 = 48 busy cycles.
- **Cycle-to-content mapping** (0-indexed cycle `cyc`, 0..127, within the
  frame): for `cyc < 48`, channel index `c = cyc / 2`, and:
  - `cyc` even (`cyc % 2 == 0`): rising edge carries bits `[23:18]`, falling
    edge carries bits `[17:12]`.
  - `cyc` odd: rising edge carries bits `[11:6]`, falling edge carries bits
    `[5:0]`.
  - For `cyc >= 48` (idle): both DDR halves are driven `6'b000000`.
- **`SPOKE_STROBE`**: high for exactly cycle 0 of the frame (the cycle
  carrying channel 0's first nibble, both DDR halves), low every other cycle
  -- a whole-cycle level, not itself DDR. No preamble or sync word: the fixed,
  deterministic 48-busy/80-idle cadence means one STROBE edge is enough for a
  receiver to establish and then maintain frame lock indefinitely.
- **Idle cycles**: driven low (`6'b000000` on both DDR halves), not
  last-value-hold.

Implementation note (`spoke_framer.v`): its `frame_start` input is itself a
registered, one-cycle-wide pulse from another module (`fir_compensator`)
sharing the same clock, so `spoke_framer`'s own posedge-triggered logic can't
observe it until the cycle *after* it's asserted (ordinary flip-flop-to-
flip-flop timing, not a quirk of this protocol) -- `SPOKE_STROBE` and channel
0's first nibble appear one cycle after `frame_start`'s own pulse, not the
same cycle. This doesn't affect the protocol itself (still exactly one
STROBE cycle per 128, still the same cadence) -- it only matters if you're
driving `frame_start` from another registered source on the same clock and
expect zero latency. (`spoke_framer.v`'s `cyc_r` counter also saturates at
`BUSY_CYCLES` instead of free-running once armed, specifically so it doesn't
assume any particular idle-cycle count -- it re-derives lock from each real
`frame_start`/`SPOKE_STROBE` rather than a hard-coded period. This is what
let it survive the 64->128 cycle change transparently; a hub-side deframer
should use the same "re-sync from STROBE every time" approach rather than
hard-coding 128, in case this changes again.)

## Worked example

Channel 5 = `0xABCDEF` (only nonzero channel, all others 0): its nibbles land
at cycles 10 and 11 (`2*5`, `2*5+1`) and nowhere else -- cycle 10 carries
`0x2B` (rising) / `0x0F` (falling) [top 12 bits of `0xABCDEF`], cycle 11
carries `0x33` (rising) / `0x2F` (falling) [bottom 12 bits]. See
`test_channel_order_is_ascending` in
`fpga/cluster/golden/test_spoke_framer_golden.py` for the exact bit values.

## Hub-side deframer

Implementation: `fpga/hub/rtl/spoke_deframer.v`, one instance per spoke.
Golden reference: reuses `spoke_framer_golden.py`'s existing `deframe()`
directly (see `fpga/cluster/golden/spoke_framer_golden.py`) -- it only
depends on `BUSY_CYCLES`, which never changed.

1. Detect `SPOKE_STROBE`'s rising edge to establish cycle-0 alignment.
2. Sample both DDR halves of `SPOKE_D[5:0]` each `SPOKE_CLK` period for the
   next 48 cycles, reassembling 24 x 24-bit channel values per the mapping
   above.
3. Ignore the 80 idle cycles (or use them for whatever slack the hub design
   needs -- e.g. inter-spoke cable-skew calibration, per `PHASE4.md`'s
   "calibrate-once per-spoke cable-length skew" note -- not attempted by
   `spoke_deframer.v` yet; each spoke is deframed fully independently).
4. Because the cadence is fixed and deterministic, only the very first
   `SPOKE_STROBE` pulse is strictly required for lock; subsequent pulses (one
   every 128 cycles) can be used as a free-running sanity check but aren't
   needed to maintain alignment once locked. `spoke_deframer.v` re-syncs from
   every `SPOKE_STROBE` edge regardless (same reasoning as `spoke_framer.v`'s
   own `cyc_r` saturation fix -- don't hard-code a specific cycle count that
   might change again).
