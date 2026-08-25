#!/usr/bin/env python3
"""cic_golden.py -- bit-exact integer CIC decimator reference model.

Two independent computation paths are provided so one can self-check the other
(see test_cic_golden.py):

  cic_bitexact()    mirrors the RTL structure exactly: STAGES cascaded integrators
                     running every input sample, decimate by R, STAGES cascaded
                     combs (M=1) running at the decimated rate. All arithmetic is
                     unsigned modulo 2**width, matching how plain adders/subtractors
                     wrap in hardware -- this is the same computation
                     fpga/multi_fpga/cluster/rtl/cic_decimator.v performs, and is what its
                     testbench (tb_cic_decimator.v) compares against.

  cic_reference_fir() an independent, structurally different computation: an
                     N-stage CIC decimator is LTI-equivalent (no overflow) to an
                     FIR filter whose impulse response is an R-length boxcar
                     self-convolved (stages-1) times, applied causally and then
                     downsampled by R. No integrator/comb wraparound arithmetic
                     is involved here at all.

hogenauer_width() computes the register width bound (Hogenauer 1981) both
implementations rely on to avoid overflow.
"""
import math

import numpy as np


def hogenauer_width(stages: int, r: int, in_width: int = 1) -> int:
    """Max internal register width (bits) needed for an N-stage, rate-R CIC
    decimator (differential delay M=1) with in_width-bit input, per Hogenauer's
    bound: growth = ceil(stages * log2(r * m))."""
    growth = math.ceil(stages * math.log2(r))
    return in_width + growth


def cic_bitexact(bits, stages: int = 5, r: int = 64, in_width: int = 1):
    """Hardware-mirroring integrator/comb CIC decimator.

    bits: iterable of 0/1 ints, the 1-bit input stream (one bit per input clock).
    Returns (output_samples, width) where output_samples is a list of ints,
    one per R input samples, and width is the register width used throughout
    (masking every add/subtract to width bits, matching fixed-width hardware
    registers).
    """
    width = hogenauer_width(stages, r, in_width)
    mask = (1 << width) - 1

    integ = [0] * stages
    comb_prev = [0] * stages
    out = []

    for i, b in enumerate(bits):
        x = b & mask
        for s in range(stages):
            integ[s] = (integ[s] + x) & mask
            x = integ[s]

        if (i + 1) % r == 0:
            y = x
            for s in range(stages):
                prev = comb_prev[s]
                comb_prev[s] = y
                y = (y - prev) & mask
            out.append(y)

    return out, width


def _boxcar_kernel(stages: int, r: int) -> np.ndarray:
    """Impulse response of `stages` cascaded length-r boxcar (moving-sum)
    filters, i.e. an R-length all-ones kernel self-convolved (stages-1) times."""
    h = np.ones(r, dtype=np.int64)
    for _ in range(stages - 1):
        h = np.convolve(h, np.ones(r, dtype=np.int64))
    return h


def cic_reference_fir(bits, stages: int = 5, r: int = 64):
    """Independent boxcar-convolution reference for the same decimator.

    Causal FIR filter (input zero-padded on the left) with the cascaded-boxcar
    impulse response, downsampled by r starting at index r-1 -- matches
    cic_bitexact()'s "decimate when (i+1) % r == 0" convention exactly.
    """
    x = np.asarray(list(bits), dtype=np.int64)
    h = _boxcar_kernel(stages, r)
    y_full = np.convolve(x, h, mode="full")[: len(x)]
    return y_full[r - 1 :: r].tolist()


if __name__ == "__main__":
    import random

    stages, r = 5, 64
    width = hogenauer_width(stages, r)
    print(f"CIC golden model: stages={stages} R={r} -> internal width={width} bits "
          f"(max value {r ** stages} < 2**{width}={2 ** width})")

    random.seed(0)
    bits = [random.randint(0, 1) for _ in range(64 * 200)]
    bitexact_out, w = cic_bitexact(bits, stages, r)
    ref_out = cic_reference_fir(bits, stages, r)
    match = bitexact_out == ref_out
    print(f"Self-check on {len(bits)} random bits -> {len(bitexact_out)} output samples: "
          f"{'MATCH' if match else 'MISMATCH'}")
    assert match, "cic_bitexact() and cic_reference_fir() disagree"
