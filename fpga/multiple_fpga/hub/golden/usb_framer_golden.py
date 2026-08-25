#!/usr/bin/env python3
"""usb_framer_golden.py -- bit-exact reference for usb_framer.v's per-spoke USB record
format. See fpga/USB_FRAMING.md for the protocol spec this implements; both that doc and
fpga/hub/rtl/usb_framer.v must stay in sync with this file (source of truth for testbench
comparison)."""

N_CH = 24
DATA_WIDTH = 24
SYNC0 = 0xA5
SYNC1 = 0x5A
HEADER_LEN = 4  # sync0, sync1, spoke_id, seq_num
PAYLOAD_LEN = N_CH * (DATA_WIDTH // 8)  # 24 * 3 = 72
RECORD_LEN = HEADER_LEN + PAYLOAD_LEN  # 76


def pack_record(spoke_id, seq_num, channels):
    """channels: list of N_CH ints (DATA_WIDTH-bit unsigned samples).
    Returns a bytes object of length RECORD_LEN: the exact byte stream
    usb_framer.v serializes onto USB_D0-7 for one spoke's completed frame."""
    assert len(channels) == N_CH
    assert 0 <= spoke_id <= 3
    assert 0 <= seq_num <= 0xFF
    mask = (1 << DATA_WIDTH) - 1
    out = bytearray([SYNC0, SYNC1, spoke_id, seq_num])
    for v in channels:
        v &= mask
        out += v.to_bytes(DATA_WIDTH // 8, byteorder="big")
    assert len(out) == RECORD_LEN
    return bytes(out)


if __name__ == "__main__":
    import random

    rng = random.Random(1)
    channels = [rng.randint(0, (1 << DATA_WIDTH) - 1) for _ in range(N_CH)]
    record = pack_record(2, 17, channels)
    assert len(record) == RECORD_LEN
    assert record[0] == SYNC0 and record[1] == SYNC1
    assert record[2] == 2
    assert record[3] == 17
    # channel 0's 3 bytes come right after the 4-byte header, MSB-first
    v0 = channels[0]
    assert record[4] == (v0 >> 16) & 0xFF
    assert record[5] == (v0 >> 8) & 0xFF
    assert record[6] == v0 & 0xFF
    print(f"Self-check OK: {N_CH} channels -> {RECORD_LEN}-byte record "
          f"({HEADER_LEN} header + {PAYLOAD_LEN} payload).")
