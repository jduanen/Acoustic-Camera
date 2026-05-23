#!/usr/bin/env python3
"""Phase 3 live acoustic camera: UMA-16 v2 (16-mic 4×4 URA) → 2D beamform → OpenCV overlay.

Usage:
    python src/acoustic_camera_p3.py
    python src/acoustic_camera_p3.py --algo mvdr --freq 3000
    python src/acoustic_camera_p3.py --algo music --freq 2000 --nsrc 2
    python src/acoustic_camera_p3.py --cal test/UMA16/cal.npy --video 4
"""
import argparse
import collections
import threading
import time

import cv2
import numpy as np
import sounddevice as sd
from scipy.linalg import inv

# --- UMA-16 v2 array geometry: 4×4 URA, 42 mm spacing, center-referenced ---
# Channel → (x, y) mapping from manual Figure 1 (sound-source side view).
# Mics are grouped as PDM L/R pairs sharing one data line.
# Verify by playing a tone from boresight: peak should land at az≈0°, el≈0°.
C = 343.0
FS = 48000
N_MICS = 16
_d = 0.042   # grid spacing [m]

# positions in units of _d/2 = 21 mm; multiply to get meters
_xy = np.array([
    (-1, -3),   # ch0  MIC1
    (-3, -3),   # ch1  MIC2
    (-1, -1),   # ch2  MIC3
    (-3, -1),   # ch3  MIC4
    (-1, +1),   # ch4  MIC5
    (-3, +1),   # ch5  MIC6
    (-1, +3),   # ch6  MIC7
    (-3, +3),   # ch7  MIC8
    (+3, +3),   # ch8  MIC9
    (+1, +3),   # ch9  MIC10
    (+3, +1),   # ch10 MIC11
    (+1, +1),   # ch11 MIC12
    (+3, -1),   # ch12 MIC13
    (+1, -1),   # ch13 MIC14
    (+3, -3),   # ch14 MIC15
    (+1, -3),   # ch15 MIC16
], dtype=float) * (_d / 2)

MIC_X = -_xy[:, 0]  # negated: Figure 1 is sound-source side; camera side is x-mirrored
MIC_Y = _xy[:, 1]   # (16,) meters


# --- Beamforming ---

def _sm(az_grid, el_grid, freq):
    """2D steering matrix, shape (N_mics, N_az * N_el).

    Convention: x = horizontal (right = positive az), y = vertical (up = positive el).
    """
    sin_az = np.sin(np.radians(az_grid))   # (N_az,)
    cos_el = np.cos(np.radians(el_grid))   # (N_el,)
    sin_el = np.sin(np.radians(el_grid))   # (N_el,)
    # ux[i,j] = sin(az[i])*cos(el[j]);  uy[j] = sin(el[j])
    ux = np.outer(sin_az, cos_el).ravel()  # (N_az*N_el,)
    uy = np.tile(sin_el, len(az_grid))     # (N_az*N_el,)
    phase = 2 * np.pi * freq / C * (np.outer(MIC_X, ux) + np.outer(MIC_Y, uy))
    return np.exp(1j * phase) / np.sqrt(N_MICS)   # (N_mics, N_az*N_el)


def beamform_ds(R, freq, az_grid, el_grid):
    H = _sm(az_grid, el_grid, freq)
    return np.real(np.sum(H.conj() * (R @ H), axis=0))


def beamform_mvdr(R, freq, az_grid, el_grid, diag_load=0.01):
    Rl = R + diag_load * np.trace(R) / N_MICS * np.eye(N_MICS)
    Ri = inv(Rl)
    H = _sm(az_grid, el_grid, freq)
    return 1.0 / np.maximum(np.real(np.sum(H.conj() * (Ri @ H), axis=0)), 1e-300)


def beamform_clean(R, freq, az_grid, el_grid, n_iter=20, loop_gain=0.5):
    H = _sm(az_grid, el_grid, freq)
    R_w = R.copy()
    clean = np.zeros(len(az_grid) * len(el_grid))
    for _ in range(n_iter):
        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))
        k = np.argmax(P)
        h = H[:, k]
        P_src = np.real(h.conj() @ (R_w @ h))
        R_w -= loop_gain * P_src * np.outer(h, h.conj())
        clean[k] += loop_gain * P_src
    return clean


def beamform_music(R, freq, az_grid, el_grid, n_src=1):
    _, V = np.linalg.eigh(R)          # ascending eigenvalues
    En = V[:, :N_MICS - n_src]        # noise subspace
    H = _sm(az_grid, el_grid, freq)
    proj = En.conj().T @ H
    return 1.0 / np.maximum(np.real(np.sum(proj.conj() * proj, axis=0)), 1e-300)


def compute_csm(audio, freq, block_size=256, hop=128):
    n_samp, n_ch = audio.shape
    freqs = np.fft.rfftfreq(block_size, 1.0 / FS)
    f_idx = np.argmin(np.abs(freqs - freq))
    R = np.zeros((n_ch, n_ch), dtype=complex)
    count = 0
    win = np.hanning(block_size)
    for start in range(0, n_samp - block_size, hop):
        block = audio[start:start + block_size] * win[:, np.newaxis]
        F = np.fft.rfft(block, axis=0)[f_idx]
        R += np.outer(F, F.conj())
        count += 1
    return R / max(count, 1)


# --- Overlay rendering ---

def acoustic_overlay(P_flat, frame, N_az, N_el, ref, alpha=0.5, db_range=30):
    """Blend 2D power map onto video frame as full-frame overlay."""
    h, w = frame.shape[:2]
    P_db = 10 * np.log10(np.maximum(P_flat.reshape(N_az, N_el) / max(ref, 1e-30), 1e-10))
    # Percentile stretch: maps 10th–100th percentile to full colormap range
    p_lo = np.percentile(P_db, 10)
    p_hi = P_db.max()
    norm = np.clip((P_db - p_lo) / max(p_hi - p_lo, 1e-6), 0, 1)
    # Remap axes: (N_az, N_el) → (N_el, N_az) with +el at top (screen y=0)
    img8 = (norm.T[::-1, :] * 255).astype(np.uint8)
    colored = cv2.applyColorMap(
        cv2.resize(img8, (w, h), interpolation=cv2.INTER_LINEAR),
        cv2.COLORMAP_JET,
    )
    return cv2.addWeighted(frame, 1 - alpha, colored, alpha, 0)


# --- Main ---

def find_device():
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] >= 16 and 'uma' in d['name'].lower():
            return i
    return None


def main():
    ap = argparse.ArgumentParser(description='Phase 3 acoustic camera — UMA-16 v2')
    ap.add_argument('--algo',   choices=['ds', 'mvdr', 'clean', 'music'], default='ds')
    ap.add_argument('--freq',   type=float, default=3000.0,  help='beamforming frequency (Hz)')
    ap.add_argument('--snap',   type=int,   default=128,     help='CSM blocks to average')
    ap.add_argument('--device', type=int,   default=None,    help='sounddevice index')
    ap.add_argument('--nsrc',   type=int,   default=1,       help='number of sources (MUSIC only)')
    ap.add_argument('--cal',    type=str,   default=None,    help='path to cal.npy')
    ap.add_argument('--az_fov', type=float, default=90.0,    help='azimuth display FOV (deg)')
    ap.add_argument('--el_fov', type=float, default=60.0,    help='elevation display FOV (deg)')
    ap.add_argument('--alpha',  type=float, default=0.5,     help='acoustic overlay opacity (0–1)')
    ap.add_argument('--smooth', type=float, default=0.7,     help='temporal smoothing factor (0=none, 0.9=heavy)')
    ap.add_argument('--video',  type=int,   default=4,       help='cv2.VideoCapture device index')
    args = ap.parse_args()

    cal_e = np.load(args.cal) if args.cal else None
    az_grid = np.linspace(-args.az_fov / 2, args.az_fov / 2, 181)
    el_grid = np.linspace(-args.el_fov / 2, args.el_fov / 2, 121)
    N_az, N_el = len(az_grid), len(el_grid)

    dev_idx = args.device if args.device is not None else find_device()
    if dev_idx is None:
        raise RuntimeError('UMA-16 not found — use --device IDX to specify')

    ALGO = {
        'ds':    lambda R, f: beamform_ds(R, f, az_grid, el_grid),
        'mvdr':  lambda R, f: beamform_mvdr(R, f, az_grid, el_grid),
        'clean': lambda R, f: beamform_clean(R, f, az_grid, el_grid),
        'music': lambda R, f: beamform_music(R, f, az_grid, el_grid, args.nsrc),
    }[args.algo]

    n_buf = args.snap * 128 + 256
    audio_buf: collections.deque = collections.deque(maxlen=n_buf)
    buf_lock = threading.Lock()

    def audio_cb(indata, frames, ts, status):
        with buf_lock:
            for row in indata:
                audio_buf.append(row.copy())

    stream = sd.InputStream(
        samplerate=FS, channels=N_MICS, dtype='float32',
        device=dev_idx, blocksize=256, callback=audio_cb,
    )

    cam = cv2.VideoCapture(args.video)
    if not cam.isOpened():
        print('No webcam — showing audio-only overlay on black frame')
        cam = None

    print(f'algo={args.algo}  freq={args.freq:.0f}Hz  '
          f'az_fov=±{args.az_fov/2:.0f}°  el_fov=±{args.el_fov/2:.0f}°  '
          f'snap={args.snap}  cal={"yes" if cal_e is not None else "no"}')
    print('Press q to quit.')

    P = None
    P_smooth = None
    ref_power = 1e-10
    az_peak = el_peak = 0.0
    label = 'Filling buffer...'
    t_last = time.monotonic()
    fps = 0.0

    win = 'Acoustic Camera — Phase 3'
    cv2.namedWindow(win, getattr(cv2, 'WINDOW_GUI_NORMAL', cv2.WINDOW_NORMAL))

    with stream:
        while True:
            time.sleep(0.05)

            if cam is not None:
                ret, frame = cam.read()
                if not ret:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)

            with buf_lock:
                n_avail = len(audio_buf)

            if n_avail >= n_buf:
                with buf_lock:
                    arr = np.array(list(audio_buf), dtype=np.float32)  # (n_buf, 16)

                R = compute_csm(arr, args.freq)
                if cal_e is not None:
                    c = 1.0 / cal_e
                    R = np.outer(c, c.conj()) * R

                P = ALGO(R, args.freq)
                # Temporal smoothing: reduces frame-to-frame jitter
                if P_smooth is None:
                    P_smooth = P.copy()
                else:
                    P_smooth = args.smooth * P_smooth + (1 - args.smooth) * P
                ref_power = max(ref_power * 0.98, P_smooth.max())

                k = np.argmax(P_smooth)
                az_peak = az_grid[k // N_el]
                el_peak = el_grid[k % N_el]
                label = (f'{args.algo.upper()}  {args.freq:.0f}Hz  '
                         f'az={az_peak:.1f}°  el={el_peak:.1f}°')

            if P_smooth is not None:
                frame = acoustic_overlay(P_smooth, frame, N_az, N_el, ref_power, alpha=args.alpha)

                # Cross-hair at peak direction
                h, w = frame.shape[:2]
                px = int((az_peak + args.az_fov / 2) / args.az_fov * w)
                py = int((args.el_fov / 2 - el_peak) / args.el_fov * h)
                px = max(1, min(w - 2, px))
                py = max(1, min(h - 2, py))
                cv2.line(frame, (px, 0), (px, h), (0, 255, 0), 1)
                cv2.line(frame, (0, py), (w, py), (0, 255, 0), 1)

            now = time.monotonic()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
            t_last = now

            cv2.putText(frame, f'{label}  {fps:.1f}fps', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow(win, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    if cam is not None:
        cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
