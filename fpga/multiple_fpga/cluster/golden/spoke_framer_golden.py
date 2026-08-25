#!/usr/bin/env python3
"""spoke_framer_golden.py -- bit-exact reference for spoke_framer.v's v1
framing protocol. See fpga/cluster/SPOKE_FRAMING.md for the protocol spec
this implements; both that doc and fpga/cluster/rtl/spoke_framer.v must stay
in sync with this file (source of truth for testbench comparison)."""

N_CH = 24
DATA_WIDTH = 24
CHUNK_WIDTH = 6
BUSY_CYCLES = N_CH * (DATA_WIDTH // CHUNK_WIDTH) // 2  # 24 * 4 / 2 = 48
FRAME_CYCLES = 64  # CIC/FIR sample period


def spoke_frame(channels):
    """channels: list of N_CH ints (DATA_WIDTH-bit unsigned samples).
    Returns a list of (spoke_d_rise, spoke_d_fall, strobe) tuples, one per
    clock cycle, length FRAME_CYCLES -- spoke_d_rise is what's driven on the
    bus during the rising-edge half of that cycle, spoke_d_fall the
    falling-edge half (DDR), strobe is a whole-cycle level."""
    assert len(channels) == N_CH
    mask = (1 << DATA_WIDTH) - 1
    out = []
    for cyc in range(FRAME_CYCLES):
        if cyc < BUSY_CYCLES:
            c_idx = cyc // 2
            chunk_pair = cyc % 2
            v = channels[c_idx] & mask
            if chunk_pair == 0:
                rise = (v >> 18) & 0x3F
                fall = (v >> 12) & 0x3F
            else:
                rise = (v >> 6) & 0x3F
                fall = v & 0x3F
        else:
            rise, fall = 0, 0
        strobe = 1 if cyc == 0 else 0
        out.append((rise, fall, strobe))
    return out


def deframe(cycles):
    """Inverse of spoke_frame(): reconstruct N_CH channel values from a list
    of (rise, fall, strobe) tuples. Used by the top-level integration
    testbench (and, eventually, a real hub-side deframer) to check round-trip
    correctness rather than just individual nibble placement."""
    assert len(cycles) >= BUSY_CYCLES
    channels = [0] * N_CH
    for cyc in range(BUSY_CYCLES):
        rise, fall, _ = cycles[cyc]
        c_idx = cyc // 2
        chunk_pair = cyc % 2
        if chunk_pair == 0:
            channels[c_idx] |= (rise & 0x3F) << 18
            channels[c_idx] |= (fall & 0x3F) << 12
        else:
            channels[c_idx] |= (rise & 0x3F) << 6
            channels[c_idx] |= (fall & 0x3F)
    return channels


if __name__ == "__main__":
    import random

    rng = random.Random(1)
    channels = [rng.randint(0, (1 << DATA_WIDTH) - 1) for _ in range(N_CH)]
    cycles = spoke_frame(channels)
    assert len(cycles) == FRAME_CYCLES
    assert cycles[0][2] == 1, "STROBE must be high on cycle 0"
    assert all(c[2] == 0 for c in cycles[1:]), "STROBE must be low every other cycle"
    assert all(c == (0, 0, 0) for c in cycles[BUSY_CYCLES:]), "idle cycles must be driven low"
    roundtrip = deframe(cycles)
    assert roundtrip == channels, "deframe(spoke_frame(x)) != x"
    print(f"Self-check OK: {N_CH} channels, {BUSY_CYCLES} busy + "
          f"{FRAME_CYCLES - BUSY_CYCLES} idle cycles, round-trip matches.")
