"""Golden-value test against the real UMA-16 capture used by benchmark_algos.py.

The expected values below were produced by the pre-refactor code
(acoustic_camera_p3.py / benchmark_algos.py duplicated math) on
test/UMA16/capture_nb16.wav — this test proves the shared module
reproduces hardware-capture behavior bit-compatibly (rtol=1e-5).
Preprocessing replicates benchmark_algos.py exactly.
"""
from pathlib import Path

import numpy as np
import pytest
import scipy.io.wavfile as wavfile

from beamforming import (beamform_ds, beamform_mvdr, beamform_clean,
                         beamform_music, compute_csm)

REPO = Path(__file__).parent.parent
WAV = REPO / 'test' / 'UMA16' / 'capture_nb16.wav'
CAL = REPO / 'test' / 'UMA16' / 'cal.npy'

AZ_GRID = np.linspace(-45.0, 45.0, 91)
EL_GRID = np.linspace(-30.0, 30.0, 61)
FREQ = 3000.0

# (argmax, max, sum) per algorithm — captured from pre-refactor code
GOLDEN = {
    'ds':    (3687, 2.1442786629511023e-07, 0.0007526436475733717),
    'mvdr':  (4298, 5.858370581784219e-08,  0.00021535321819646285),
    'clean': (3687, 1.0721393314755514e-07, 6.727404692331099e-07),
    'music': (3565, 1.6352823826365699,     7036.777476595851),
}


@pytest.fixture(scope='module')
def csm():
    _, raw = wavfile.read(WAV)
    audio = raw.astype(np.float32)
    if np.abs(audio).max() > 1.0:
        audio /= 32768.0
    audio_slice = audio[:128 * 128 + 256]
    R = compute_csm(audio_slice, FREQ)
    cal_e = np.load(CAL)
    c = 1.0 / cal_e
    return np.outer(c, c.conj()) * R


@pytest.mark.parametrize('name,fn', [
    ('ds', beamform_ds), ('mvdr', beamform_mvdr),
    ('clean', beamform_clean), ('music', beamform_music),
])
def test_golden_capture_nb16(csm, name, fn):
    P = fn(csm, FREQ, AZ_GRID, EL_GRID)
    exp_argmax, exp_max, exp_sum = GOLDEN[name]
    assert int(np.argmax(P)) == exp_argmax
    np.testing.assert_allclose(P.max(), exp_max, rtol=1e-5)
    np.testing.assert_allclose(P.sum(), exp_sum, rtol=1e-5)
