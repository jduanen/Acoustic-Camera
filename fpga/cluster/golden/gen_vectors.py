#!/usr/bin/env python3
"""gen_vectors.py -- generates checked-in .mem stimulus/expected-output
fixtures (fpga/cluster/vectors/) from the golden Python models, used by the
RTL testbenches in fpga/cluster/sim/.

Source of truth is the golden/*.py models -- re-run this script and re-check-in
the vectors/ output whenever a golden model changes. Testbenches hardcode the
vector sizes generated here (see e.g. tb_cic_decimator.v's N_SMALL/N_REAL
localparams); keep them in sync if these configs change.
"""
import os
import random

from cic_golden import cic_bitexact, hogenauer_width
from fir_design import (
    DATA_WIDTH as FIR_DATA_WIDTH,
    design_compensation_fir,
    fir_bitexact,
    gen_fir_vectors,
    quantize_coeffs,
)
from spoke_framer_golden import DATA_WIDTH as FRAMER_DATA_WIDTH, FRAME_CYCLES, N_CH, spoke_frame
from gen_pdm_stimulus import golden_channel_pipeline, gen_multitone_pdm

TOP_N_FRAMES = 20

VEC_DIR = os.path.join(os.path.dirname(__file__), "..", "vectors")

# (name, stages, r, n_frames)
CIC_CONFIGS = [
    ("cic_small", 2, 4, 300),
    ("cic_real", 5, 64, 500),
]

FIR_TEST_N_SAMPLES = 400


def _write_bits_mem(path, bits):
    with open(path, "w") as f:
        for b in bits:
            f.write(f"{b}\n")


def _write_hex_mem(path, values, width_bits):
    hex_digits = (width_bits + 3) // 4
    with open(path, "w") as f:
        for v in values:
            f.write(f"{v:0{hex_digits}x}\n")


def _mixed_bitstream(rng, r, n_frames):
    """Random bits with a few step/alternating segments mixed in -- decent
    coverage without needing a full sigma-delta model (see gen_pdm_stimulus.py
    for that, used by the end-to-end cluster_top integration test instead)."""
    bits = []
    n = r * n_frames
    i = 0
    while i < n:
        seg = min(r * 20, n - i)
        kind = rng.choice(["random", "zeros", "ones", "alt"])
        if kind == "random":
            bits += [rng.randint(0, 1) for _ in range(seg)]
        elif kind == "zeros":
            bits += [0] * seg
        elif kind == "ones":
            bits += [1] * seg
        else:
            bits += [(i + k) % 2 for k in range(seg)]
        i += seg
    return bits[:n]


def gen_cic_vectors():
    os.makedirs(VEC_DIR, exist_ok=True)
    rng = random.Random(20260730)

    for name, stages, r, n_frames in CIC_CONFIGS:
        bits = _mixed_bitstream(rng, r, n_frames)
        out, width = cic_bitexact(bits, stages, r)
        assert width == hogenauer_width(stages, r)
        _write_bits_mem(os.path.join(VEC_DIR, f"{name}_input.mem"), bits)
        _write_hex_mem(os.path.join(VEC_DIR, f"{name}_expected.mem"), out, width)
        print(f"{name}: stages={stages} R={r} width={width} "
              f"-> {len(bits)} input bits, {len(out)} expected output samples")


def gen_fir_test_vectors():
    """FIR data-path stimulus + expected output, separate from fir_coeffs.mem
    (written by fir_design.gen_fir_vectors()). Used by tb_fir_compensator.v."""
    os.makedirs(VEC_DIR, exist_ok=True)
    rng = random.Random(9973)
    coeffs = quantize_coeffs(design_compensation_fir())

    data = [rng.randint(0, (1 << FIR_DATA_WIDTH) - 1) for _ in range(FIR_TEST_N_SAMPLES)]
    out = fir_bitexact(data, coeffs)

    hex_digits = (FIR_DATA_WIDTH + 3) // 4
    with open(os.path.join(VEC_DIR, "fir_test_input.mem"), "w") as f:
        for v in data:
            f.write(f"{v:0{hex_digits}x}\n")
    with open(os.path.join(VEC_DIR, "fir_test_expected.mem"), "w") as f:
        for v in out:
            f.write(f"{v:0{hex_digits}x}\n")
    print(f"fir_test: {len(data)} data-in samples -> {len(out)} expected outputs "
          f"(DATA_WIDTH={FIR_DATA_WIDTH})")


def gen_framer_vectors():
    """Several test frames (edge cases + random) for tb_spoke_framer.v:
    fpga/cluster/vectors/framer_channels.mem -- N_FRAMES*N_CH lines, one
      DATA_WIDTH-bit hex channel value per line, frames concatenated.
    fpga/cluster/vectors/framer_expected.mem -- N_FRAMES*FRAME_CYCLES lines,
      one packed hex value per cycle: {strobe[12], fall[11:6], rise[5:0]}.
    """
    os.makedirs(VEC_DIR, exist_ok=True)
    rng = random.Random(31337)
    mask = (1 << FRAMER_DATA_WIDTH) - 1

    frames = [
        [0] * N_CH,
        [mask] * N_CH,
        [(i * 0x010203) & mask for i in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
        [rng.randint(0, mask) for _ in range(N_CH)],
    ]

    ch_hex_digits = (FRAMER_DATA_WIDTH + 3) // 4
    with open(os.path.join(VEC_DIR, "framer_channels.mem"), "w") as f:
        for frame in frames:
            for v in frame:
                f.write(f"{v:0{ch_hex_digits}x}\n")

    with open(os.path.join(VEC_DIR, "framer_expected.mem"), "w") as f:
        for frame in frames:
            for rise, fall, strobe in spoke_frame(frame):
                packed = (strobe << 12) | (fall << 6) | rise
                f.write(f"{packed:04x}\n")

    print(f"framer: {len(frames)} frames x ({N_CH} channels, {FRAME_CYCLES} cycles) "
          f"-> framer_channels.mem / framer_expected.mem")
    return len(frames)


def gen_cluster_top_vectors():
    """End-to-end stimulus + expected output for tb_cluster_top.v:
    fpga/cluster/vectors/top_pdm_bits.mem -- N_CH*n_samples lines, one bit
      (0/1) per line, channel-major (channel c's n_samples bits, then c+1's).
    fpga/cluster/vectors/top_expected.mem -- TOP_N_FRAMES*N_CH lines, one
      24-bit hex PCM value per line, frame-major (frame f's N_CH channel
      values, in ascending channel order, then frame f+1's).
    """
    os.makedirs(VEC_DIR, exist_ok=True)
    coeffs = quantize_coeffs(design_compensation_fir())
    pdm_bits, n_samples = gen_multitone_pdm(n_ch=N_CH, n_frames=TOP_N_FRAMES)

    with open(os.path.join(VEC_DIR, "top_pdm_bits.mem"), "w") as f:
        for ch_bits in pdm_bits:
            for b in ch_bits:
                f.write(f"{int(b)}\n")

    per_channel_out = [golden_channel_pipeline(b, coeffs, is_r_channel=bool(c % 2))
                       for c, b in enumerate(pdm_bits)]
    hex_digits = (FIR_DATA_WIDTH + 3) // 4
    with open(os.path.join(VEC_DIR, "top_expected.mem"), "w") as f:
        for frame_idx in range(TOP_N_FRAMES):
            for c in range(N_CH):
                f.write(f"{per_channel_out[c][frame_idx]:0{hex_digits}x}\n")

    print(f"top: {N_CH} channels x {n_samples} PDM bits -> {TOP_N_FRAMES} frames "
          f"-> top_pdm_bits.mem / top_expected.mem")


if __name__ == "__main__":
    gen_cic_vectors()
    gen_fir_vectors()
    gen_fir_test_vectors()
    gen_framer_vectors()
    gen_cluster_top_vectors()
