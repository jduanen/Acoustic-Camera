"""Synthetic tests for the UMA-16 beamforming core — no hardware required."""
import numpy as np
import pytest

from acoustic_camera_p3 import (C, FS, N_MICS, MIC_X, MIC_Y,
                                beamform_ds, beamform_mvdr, beamform_clean,
                                beamform_music, compute_csm)

AZ_GRID = np.linspace(-45.0, 45.0, 91)   # 1.0 deg/pt
EL_GRID = np.linspace(-30.0, 30.0, 61)
N_EL = len(EL_GRID)
FREQ = 3000.0


def peak_azel(P):
    k = int(np.argmax(P))
    return AZ_GRID[k // N_EL], EL_GRID[k % N_EL]


def synth_plane_wave(az_deg, el_deg, freq=FREQ, n_samp=16640, snr_db=30, seed=0):
    """16-ch time-domain sinusoid with per-channel phase matching a plane wave
    from (az, el), using the same direction convention as the steering matrix."""
    ux = np.sin(np.radians(az_deg)) * np.cos(np.radians(el_deg))
    uy = np.sin(np.radians(el_deg))
    tau = (MIC_X * ux + MIC_Y * uy) / C                      # (16,)
    t = np.arange(n_samp) / FS
    sig = np.sin(2 * np.pi * freq * (t[:, None] + tau[None, :]))
    rng = np.random.default_rng(seed)
    noise = rng.standard_normal(sig.shape) * 10 ** (-snr_db / 20)
    return (sig + noise).astype(np.float32)


# Expected positions in meters: _xy table * d/2 (d=0.042), x negated
# (Figure 1 is the sound-source side; camera side is x-mirrored — this
# negation was a hardware-debugged fix and must not silently regress).
GOLDEN_MIC_X = np.array([+1, +3, +1, +3, +1, +3, +1, +3,
                         -3, -1, -3, -1, -3, -1, -3, -1], dtype=float) * 0.021
GOLDEN_MIC_Y = np.array([-3, -3, -1, -1, +1, +1, +3, +3,
                         +3, +3, +1, +1, -1, -1, -3, -3], dtype=float) * 0.021


def test_geometry_golden():
    np.testing.assert_allclose(MIC_X, GOLDEN_MIC_X, atol=1e-12)
    np.testing.assert_allclose(MIC_Y, GOLDEN_MIC_Y, atol=1e-12)


@pytest.mark.parametrize('az,el', [(20.0, -10.0), (-20.0, 10.0)])
def test_ds_finds_plane_wave(az, el):
    # Two mirrored source positions so a global sign flip cannot pass by symmetry
    audio = synth_plane_wave(az, el)
    R = compute_csm(audio, FREQ)
    P = beamform_ds(R, FREQ, AZ_GRID, EL_GRID)
    az_pk, el_pk = peak_azel(P)
    assert abs(az_pk - az) <= 1.0
    assert abs(el_pk - el) <= 1.0


def test_algorithms_agree():
    audio = synth_plane_wave(20.0, -10.0)
    R = compute_csm(audio, FREQ)
    ds_pk = peak_azel(beamform_ds(R, FREQ, AZ_GRID, EL_GRID))
    for fn in (beamform_mvdr, beamform_clean, beamform_music):
        az_pk, el_pk = peak_azel(fn(R, FREQ, AZ_GRID, EL_GRID))
        assert abs(az_pk - ds_pk[0]) <= 1.0, fn.__name__
        assert abs(el_pk - ds_pk[1]) <= 1.0, fn.__name__


def test_csm_hermitian_psd():
    rng = np.random.default_rng(1)
    audio = rng.standard_normal((4096, N_MICS)).astype(np.float32)
    R = compute_csm(audio, FREQ)
    assert R.shape == (N_MICS, N_MICS)
    np.testing.assert_allclose(R, R.conj().T, rtol=0, atol=1e-12 * np.abs(R).max())
    ev = np.linalg.eigvalsh(R)
    assert ev.min() >= -1e-12 * ev.max()


def test_mvdr_singular_csm():
    # Noiseless rank-1 CSM: without diagonal loading inv() would blow up.
    ux = np.sin(np.radians(20.0)) * np.cos(np.radians(-10.0))
    uy = np.sin(np.radians(-10.0))
    a = np.exp(1j * 2 * np.pi * FREQ / C * (MIC_X * ux + MIC_Y * uy))
    R = np.outer(a, a.conj())
    P = beamform_mvdr(R, FREQ, AZ_GRID, EL_GRID)
    assert np.all(np.isfinite(P))
    az_pk, el_pk = peak_azel(P)
    assert abs(az_pk - 20.0) <= 1.0
    assert abs(el_pk + 10.0) <= 1.0
