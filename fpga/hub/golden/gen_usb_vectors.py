#!/usr/bin/env python3
"""gen_usb_vectors.py -- generates checked-in .mem stimulus/expected-output fixtures
(fpga/hub/vectors/) from usb_framer_golden.py, used by tb_usb_framer.v.

Own fpga/hub/vectors/ directory, separate from fpga/cluster/vectors/, since this protocol
isn't derived from anything in fpga/cluster/ -- mirrors fpga/cluster/golden/gen_vectors.py's
approach (source of truth is the golden model; re-run and re-check-in whenever it changes).

8 records, spoke_id cycling 0,1,2,3,0,1,2,3 (seq_num 0,0,0,0,1,1,1,1 respectively) --
tb_usb_framer.v drives each spoke's `valid` pulse in this exact order and checks the
serialized byte stream against usb_expected.mem's matching 76-byte slice.
"""
import os
import random

from usb_framer_golden import DATA_WIDTH, N_CH, RECORD_LEN, pack_record

N_RECORDS = 8

VEC_DIR = os.path.join(os.path.dirname(__file__), "..", "vectors")


def gen_usb_vectors():
    os.makedirs(VEC_DIR, exist_ok=True)
    rng = random.Random(20260821)
    mask = (1 << DATA_WIDTH) - 1

    records = [
        [0] * N_CH,
        [mask] * N_CH,
        [(i * 0x010203) & mask for i in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
    ]
    assert len(records) == N_RECORDS

    ch_hex_digits = (DATA_WIDTH + 3) // 4
    with open(os.path.join(VEC_DIR, "usb_channels.mem"), "w") as f:
        for rec in records:
            for v in rec:
                f.write(f"{v:0{ch_hex_digits}x}\n")

    with open(os.path.join(VEC_DIR, "usb_expected.mem"), "w") as f:
        for i, rec in enumerate(records):
            spoke_id = i % 4
            seq_num = i // 4
            packed = pack_record(spoke_id, seq_num, rec)
            assert len(packed) == RECORD_LEN
            for b in packed:
                f.write(f"{b:02x}\n")

    print(f"usb: {N_RECORDS} records x ({N_CH} channels, {RECORD_LEN} bytes) "
          f"-> usb_channels.mem / usb_expected.mem")


if __name__ == "__main__":
    gen_usb_vectors()
