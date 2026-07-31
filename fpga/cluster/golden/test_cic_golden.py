import random

import pytest

from cic_golden import cic_bitexact, cic_reference_fir, hogenauer_width


@pytest.mark.parametrize("stages,r", [(2, 4), (3, 8), (5, 64)])
def test_bitexact_matches_boxcar_reference_random(stages, r):
    random.seed(1234 + stages * 100 + r)
    n_frames = 50
    bits = [random.randint(0, 1) for _ in range(r * n_frames)]
    out, width = cic_bitexact(bits, stages, r)
    ref = cic_reference_fir(bits, stages, r)
    assert width == hogenauer_width(stages, r)
    assert out == ref
    assert len(out) == n_frames


@pytest.mark.parametrize("stages,r", [(2, 4), (5, 64)])
def test_bitexact_matches_boxcar_reference_step(stages, r):
    # step input: 0 for a while, then all 1s -- exercises filter fill-up transient
    n_frames = 20
    bits = [0] * (r * 3) + [1] * (r * (n_frames - 3))
    out, _ = cic_bitexact(bits, stages, r)
    ref = cic_reference_fir(bits, stages, r)
    assert out == ref


def test_bitexact_matches_boxcar_reference_alternating():
    stages, r = 5, 64
    bits = [i % 2 for i in range(r * 30)]
    out, _ = cic_bitexact(bits, stages, r)
    ref = cic_reference_fir(bits, stages, r)
    assert out == ref


def test_hogenauer_width_known_value():
    # stages=5, R=64 (power of two, log2 exact) -> 1 + 5*6 = 31 bits
    assert hogenauer_width(5, 64) == 31


def test_output_never_exceeds_width():
    stages, r = 5, 64
    width = hogenauer_width(stages, r)
    random.seed(7)
    bits = [random.randint(0, 1) for _ in range(r * 200)]
    out, _ = cic_bitexact(bits, stages, r)
    assert max(out) < (1 << width)
    assert min(out) >= 0


def test_all_ones_reaches_max_gain():
    # steady-state DC gain of an N-stage CIC decimator is R**N
    stages, r = 5, 64
    bits = [1] * (r * 10)
    out, _ = cic_bitexact(bits, stages, r)
    assert out[-1] == r ** stages
