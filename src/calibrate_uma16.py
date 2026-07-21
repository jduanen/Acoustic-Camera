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
    python src/calibrate_uma16.py --device 4   # pin the sounddevice index explicitly
"""
import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
import scipy.io.wavfile as wavfile
import sounddevice as sd

from beamforming import FS, N_MICS, compute_csm, beamform_ds, beamform_mvdr

CAL_FREQ = 1000.0   # calibration tone (Hz) — well below the ~4082 Hz spatial Nyquist
CAL_SECS = 5         # reference recording duration
_MIN_SNR_DB = 20     # abort if CAL_FREQ isn't at least this far above the noise floor.
                     # NOT based on absolute amplitude: a real tone can be very quiet in
                     # absolute terms (distant source, low mic gain) yet still land a
                     # strong, clean peak at CAL_FREQ relative to the noise floor — SNR is
                     # what the delay/gain math actually needs, not loudness. (Observed:
                     # a genuine, usable recording measured ~46 dB SNR at well under 1% of
                     # full scale; a truly dead/silent recording has no such peak at all.)

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


def tone_snr_db(audio, freq, fs, block=8192):
    """How far `freq` sticks up above the surrounding noise floor (dB), averaged
    across channels — the real test of whether a usable tone was captured, since
    absolute level alone can't tell a quiet-but-clean tone from silence."""
    seg = audio[:block] * np.hanning(block)[:, np.newaxis]
    F = np.abs(np.fft.rfft(seg, axis=0)) ** 2
    freqs = np.fft.rfftfreq(block, 1 / fs)
    pk = np.argmin(np.abs(freqs - freq))
    band = F[pk - 3:pk + 4].mean(axis=0)
    noise_floor = np.median(F, axis=0)
    return float(10 * np.log10(np.mean(band / noise_floor)))


def main():
    ap = argparse.ArgumentParser(description='UMA-16 v2 calibration')
    ap.add_argument('--device', type=int, default=None,
                     help='sounddevice index (default: auto-detect via find_device(), '
                          'same logic acoustic_camera_p3.py uses — pass this explicitly '
                          'if that script needs --device too, e.g. because auto-detect '
                          'matches more than one entry for the same physical device)')
    args = ap.parse_args()

    print('=== UMA-16 v2 Calibration ===\n')

    dev_idx = args.device if args.device is not None else find_device()
    if dev_idx is None:
        sys.exit('ERROR: UMA-16 not found — check the USB connection, or pass --device IDX')
    dev = sd.query_devices()[dev_idx]
    print(f'Device [{dev_idx}]: {dev["name"]}  ({dev["max_input_channels"]} ch)\n')

    print('Position a 1 kHz tone source at BORESIGHT (0 deg, straight ahead of')
    print('the array), about 0.5-1 m away -- a phone or laptop speaker works')
    print('fine. Keep ambient noise low.\n')
    input('Press Enter to start the 5-second recording (Ctrl+C to abort)...')

    # Streamed via InputStream + callback (blocksize=256), matching acoustic_camera_p3.py's
    # capture method exactly, rather than the sd.rec()/sd.wait() convenience call (which
    # lets PortAudio pick its own default blocksize/latency) — removes one behavioral
    # difference between the two scripts, even though it turned out not to be the cause
    # of the near-silent-looking recordings seen in practice (see tone_snr_db below).
    print('Recording...', end=' ', flush=True)
    blocks = []

    def _cb(indata, frames, time_info, status):
        blocks.append(indata.copy())

    with sd.InputStream(samplerate=FS, channels=N_MICS, dtype='float32',
                        device=dev_idx, blocksize=256, callback=_cb):
        sd.sleep(int(CAL_SECS * 1000) + 200)  # small margin so trimming below never underruns
    print('done.\n')

    ref_rec = np.concatenate(blocks, axis=0)[:CAL_SECS * FS]

    REF_WAV.parent.mkdir(parents=True, exist_ok=True)
    wavfile.write(REF_WAV, FS, (ref_rec * 32767).astype(np.int16))
    print(f'Saved reference recording: {REF_WAV}')

    rms = np.sqrt(np.mean(ref_rec ** 2, axis=0))
    print('\nRMS per channel:')
    for i, r in enumerate(rms):
        bar = '#' * int(r / rms.max() * 30)
        print(f'  ch{i:2d}: {r:.2e}  {bar}')

    # Delay/gain estimates computed without a real tone present are meaningless noise,
    # not a real calibration. Catch it here rather than silently saving a bogus cal.npy
    # over a good one — checking SNR at CAL_FREQ, not absolute level (see _MIN_SNR_DB).
    snr_db = tone_snr_db(ref_rec, CAL_FREQ, FS)
    print(f'\n{CAL_FREQ:.0f} Hz tone SNR vs noise floor: {snr_db:.1f} dB')
    if snr_db < _MIN_SNR_DB:
        sys.exit(
            f'\nERROR: {CAL_FREQ:.0f} Hz is only {snr_db:.1f} dB above the noise floor '
            f'(expected at\nleast {_MIN_SNR_DB:.0f} dB) -- this does not look like a real tone. '
            f'This usually means\nthe tone source wasn\'t actually audible: check that it\'s '
            f'really playing,\nturn up its volume, move it closer to the array, and confirm '
            f'the UMA-16 is\nthe correct input device -- then try again. Nothing was saved; '
            f'the existing\n{CAL_PATH.name} is untouched.'
        )

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
