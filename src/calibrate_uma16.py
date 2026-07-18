#!/usr/bin/env python3
"""Interactive UMA-16 v2 calibration: measure per-mic gain/phase offsets from a
boresight reference recording and save the correction vector used by
acoustic_camera_p3.py's --cal flag.

Terminal walkthrough of the same procedure as
notebooks/17_uma16_calibration.ipynb (frequency-domain cross-correlation delay
+ RMS gain, both relative to ch0), packaged as a standalone script so it can
be re-run directly on the Pi appliance — see the "Calibrate" desktop icon
(Desktop/calibrate.desktop) — without a dev machine / Jupyter.

Usage:
    python src/calibrate_uma16.py
"""
import shutil
import sys
from pathlib import Path

import numpy as np
import scipy.io.wavfile as wavfile
import sounddevice as sd

from beamforming import FS, N_MICS, compute_csm, beamform_ds, beamform_mvdr

CAL_FREQ = 1000.0   # calibration tone (Hz) — well below the ~4082 Hz spatial Nyquist
CAL_SECS = 5         # reference recording duration

_REPO = Path(__file__).resolve().parent.parent
CAL_PATH = _REPO / 'test' / 'UMA16' / 'cal.npy'
REF_WAV = _REPO / 'test' / 'UMA16' / 'cal_ref.wav'


def find_device():
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] >= N_MICS and 'uma' in d['name'].lower():
            return i
    return None


def estimate_delay(ref_ch, mic_ch, fs):
    """Return delay in seconds (positive = mic_ch lags ref_ch), via frequency-domain
    cross-correlation — same sub-sample estimator as notebooks/17_uma16_calibration."""
    n = len(ref_ch)
    F0 = np.fft.rfft(ref_ch, n=n)
    Fn = np.fft.rfft(mic_ch, n=n)
    xcorr = np.fft.irfft(np.conj(F0) * Fn, n=n)
    lag = np.argmax(np.abs(xcorr))
    if lag > n // 2:
        lag -= n
    return lag / fs


def apply_cal(R, e):
    c = 1.0 / e
    return np.outer(c, c.conj()) * R


def main():
    print('=== UMA-16 v2 Calibration ===\n')

    dev_idx = find_device()
    if dev_idx is None:
        sys.exit('ERROR: UMA-16 not found — check the USB connection')
    dev = sd.query_devices()[dev_idx]
    print(f'Device [{dev_idx}]: {dev["name"]}  ({dev["max_input_channels"]} ch)\n')

    print('Position a 1 kHz tone source at BORESIGHT (0 deg, straight ahead of')
    print('the array), about 0.5-1 m away -- a phone or laptop speaker works')
    print('fine. Keep ambient noise low.\n')
    input('Press Enter to start the 5-second recording (Ctrl+C to abort)...')

    print('Recording...', end=' ', flush=True)
    ref_rec = sd.rec(CAL_SECS * FS, samplerate=FS, channels=N_MICS,
                      dtype='float32', device=dev_idx)
    sd.wait()
    print('done.\n')

    REF_WAV.parent.mkdir(parents=True, exist_ok=True)
    wavfile.write(REF_WAV, FS, (ref_rec * 32767).astype(np.int16))
    print(f'Saved reference recording: {REF_WAV}')

    rms = np.sqrt(np.mean(ref_rec ** 2, axis=0))
    print('\nRMS per channel:')
    for i, r in enumerate(rms):
        bar = '#' * int(r / rms.max() * 30)
        print(f'  ch{i:2d}: {r:.2e}  {bar}')

    delays = np.zeros(N_MICS)
    for i in range(1, N_MICS):
        delays[i] = estimate_delay(ref_rec[:, 0], ref_rec[:, i], FS)
    gains = rms / rms[0]

    print('\nDelays relative to ch0 (should be ~0 at boresight):')
    for i in range(N_MICS):
        print(f'  ch{i:2d}: {delays[i]*1e6:+7.2f} us  ({delays[i]*FS:+6.3f} samples)')
    print('\nGain ratios relative to ch0:')
    for i in range(N_MICS):
        print(f'  ch{i:2d}: {gains[i]:.4f}  ({20*np.log10(gains[i]):+.2f} dB)')
    print(f'\nMax gain spread: {20*np.log10(gains.max()/gains.min()):.2f} dB')

    phi = 2 * np.pi * CAL_FREQ * delays
    e = gains * np.exp(1j * phi)

    if CAL_PATH.exists():
        backup = CAL_PATH.with_suffix('.npy.bak')
        shutil.copy2(CAL_PATH, backup)
        print(f'\nBacked up previous calibration to {backup}')

    CAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.save(CAL_PATH, e)
    print(f'Saved calibration vector: {CAL_PATH}')

    # --- Validation: boresight DoA before vs after, D&S and MVDR ---
    R_ref = compute_csm(ref_rec, CAL_FREQ, fs=FS)
    R_cal = apply_cal(R_ref, e)

    az_grid = np.linspace(-90, 90, 1801)
    el_grid = np.array([0.0])

    az_ds_u = az_grid[np.argmax(beamform_ds(R_ref, CAL_FREQ, az_grid, el_grid))]
    az_ds_c = az_grid[np.argmax(beamform_ds(R_cal, CAL_FREQ, az_grid, el_grid))]
    az_mv_u = az_grid[np.argmax(beamform_mvdr(R_ref, CAL_FREQ, az_grid, el_grid))]
    az_mv_c = az_grid[np.argmax(beamform_mvdr(R_cal, CAL_FREQ, az_grid, el_grid))]

    print('\n=== Validation: boresight DoA (true angle: 0 deg) ===')
    print(f'  D&S   uncal: {az_ds_u:+6.1f}   cal: {az_ds_c:+6.1f}')
    print(f'  MVDR  uncal: {az_mv_u:+6.1f}   cal: {az_mv_c:+6.1f}')
    if abs(az_ds_c) < abs(az_ds_u) and abs(az_mv_c) < abs(az_mv_u):
        print('\nCalibration improved boresight DoA -- looks good.')
    else:
        print('\nCalibration did NOT improve boresight DoA on both beamformers --')
        print('double-check the tone source was actually at boresight and try again.')

    input('\nPress Enter to close this window...')


if __name__ == '__main__':
    main()
