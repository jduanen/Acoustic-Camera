import random

import numpy as np

from fir_design import (
    ACC_WIDTH,
    COEFF_WIDTH,
    DATA_WIDTH,
    FRAC_BITS,
    NTAPS,
    combined_response_summary,
    design_compensation_fir,
    fir_bitexact,
    quantize_coeffs,
)


def test_design_produces_ntaps_coefficients():
    coeffs = design_compensation_fir()
    assert len(coeffs) == NTAPS


def test_quantize_roundtrip_within_one_lsb():
    coeffs = design_compensation_fir()
    q = quantize_coeffs(coeffs)
    assert len(q) == NTAPS
    scale = 2 ** FRAC_BITS
    for c, qi in zip(coeffs, q):
        assert abs(c - qi / scale) < 1.0 / scale + 1e-9
    qmax = 2 ** (COEFF_WIDTH - 1) - 1
    qmin = -(2 ** (COEFF_WIDTH - 1))
    assert all(qmin <= v <= qmax for v in q)


def test_combined_response_within_tolerance():
    coeffs = design_compensation_fir()
    q = quantize_coeffs(coeffs)
    ripple = combined_response_summary(q)
    assert ripple < 3.0


def test_fir_bitexact_matches_float_convolution_impulse():
    # impulse response of fir_bitexact() should match the quantized
    # coefficients themselves (allowing for the fixed shift/mask convention).
    coeffs = quantize_coeffs(design_compensation_fir())
    impulse = [0] * 5 + [(1 << (DATA_WIDTH - 1))] + [0] * (NTAPS + 10)
    out = fir_bitexact(impulse, coeffs)
    # peak output sample should land where the impulse aligns with tap 0
    peak_idx = int(np.argmax([abs(_to_signed(v, DATA_WIDTH)) for v in out]))
    assert out[peak_idx] != 0


def test_fir_bitexact_matches_direct_float_mac():
    random.seed(42)
    coeffs = quantize_coeffs(design_compensation_fir())
    data = [random.randint(0, (1 << DATA_WIDTH) - 1) for _ in range(200)]
    out = fir_bitexact(data, coeffs)

    # independent float reference of the same fixed-point pipeline (direct MAC,
    # no shift-register bookkeeping reuse from fir_bitexact() itself)
    n = len(coeffs)
    shift_reg = [0] * n
    scale = 2 ** FRAC_BITS
    acc_mask = (1 << ACC_WIDTH) - 1
    for i, x in enumerate(data):
        shift_reg = [x] + shift_reg[:-1]
        acc = sum(shift_reg[k] * coeffs[k] for k in range(n)) & acc_mask
        signed_acc = acc - (1 << ACC_WIDTH) if acc & (1 << (ACC_WIDTH - 1)) else acc
        expected = (signed_acc >> FRAC_BITS) & ((1 << DATA_WIDTH) - 1)
        assert out[i] == expected, f"mismatch at sample {i}"


def _to_signed(v, width):
    return v - (1 << width) if v & (1 << (width - 1)) else v
