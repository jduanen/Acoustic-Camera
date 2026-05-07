#!/usr/bin/env python3
"""Phase 2 live acoustic camera: ReSpeaker 4-mic → beamform → OpenCV overlay.

Usage:
    python src/acoustic_camera_p2.py
    python src/acoustic_camera_p2.py --algo mvdr --freq 2000
    python src/acoustic_camera_p2.py --algo ds --freq 1000 --cal test/ReSpeaker/cal.npy
    python src/acoustic_camera_p2.py --video 2
"""
import argparse
import collections
import threading
import time

import cv2
import numpy as np
import sounddevice as sd
from scipy.linalg import inv

# --- Array geometry (4-mic circular, 90mm diameter) ---
C = 343.0
RADIUS = 0.045
_angles = np.array([0.0, 90.0, 180.0, 270.0])
MIC_X = RADIUS * np.cos(np.radians(_angles))
MIC_Y = RADIUS * np.sin(np.radians(_angles))
MIC_SLICE = slice(2, 6)   # USB ch2-5 = Mic 0-3 raw


# --- Beamforming ---

def _sm(az_grid, freq):
    u = np.sin(np.radians(az_grid))
    return np.exp(1j * 2 * np.pi * freq / C * np.outer(MIC_X, u)) / np.sqrt(4)


def beamform_ds(R, freq, az_grid):
    H = _sm(az_grid, freq)
    return np.real(np.sum(H.conj() * (R @ H), axis=0))


def beamform_mvdr(R, freq, az_grid, diag_load=0.01):
    N = R.shape[0]
    Rl = R + diag_load * np.trace(R) / N * np.eye(N)
    Ri = inv(Rl)
    H = _sm(az_grid, freq)
    d = np.real(np.sum(H.conj() * (Ri @ H), axis=0))
    return 1.0 / np.maximum(d, 1e-300)


def beamform_clean(R, freq, az_grid, n_iter=20, loop_gain=0.5):
    H = _sm(az_grid, freq)
    R_w = R.copy()
    clean = np.zeros(len(az_grid))
    for _ in range(n_iter):
        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))
        k = np.argmax(P)
        g = R_w @ H[:, k]
        R_w -= loop_gain * np.outer(g, g.conj())
        clean[k] += loop_gain * np.real(g.conj() @ g)
    return clean


def compute_csm(audio, freq, block_size=256, hop=128):
    n_samp, n_ch = audio.shape
    freqs = np.fft.rfftfreq(block_size, 1 / 16000)
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

def energy_strip(P, frame_w, strip_h, db_range=20):
    """1D power map → BGR color strip of shape (strip_h, frame_w, 3)."""
    p_max = P.max()
    if p_max > 0:
        P_db = 10 * np.log10(np.maximum(P / p_max, 1e-10))
        norm = np.clip((P_db + db_range) / db_range, 0, 1)
    else:
        norm = np.zeros(len(P))
    row = (norm * 255).astype(np.uint8).reshape(1, -1)
    row_w = cv2.resize(row, (frame_w, 1), interpolation=cv2.INTER_LINEAR)
    strip = cv2.applyColorMap(row_w, cv2.COLORMAP_INFERNO)
    return np.repeat(strip, strip_h, axis=0)


# --- Main ---

def find_device():
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] >= 6 and (
                'respeaker' in d['name'].lower() or 'xvf' in d['name'].lower()):
            return i
    return None


def main():
    ap = argparse.ArgumentParser(description='Phase 2 acoustic camera live demo')
    ap.add_argument('--algo',   choices=['ds', 'mvdr', 'clean'], default='ds')
    ap.add_argument('--freq',   type=float, default=1000.0,  help='beamforming frequency (Hz)')
    ap.add_argument('--snap',   type=int,   default=64,      help='CSM blocks to average')
    ap.add_argument('--device', type=int,   default=None,    help='sounddevice index')
    ap.add_argument('--cal',    type=str,   default=None,    help='path to cal.npy')
    ap.add_argument('--fov',    type=float, default=90.0,    help='display FOV (deg)')
    ap.add_argument('--video',  type=int,   default=0,       help='cv2.VideoCapture device index')
    args = ap.parse_args()

    cal_e = np.load(args.cal) if args.cal else None
    az_grid = np.linspace(-args.fov / 2, args.fov / 2, 901)

    dev_idx = args.device if args.device is not None else find_device()
    if dev_idx is None:
        raise RuntimeError('ReSpeaker not found — use --device IDX to specify')

    ALGO = {'ds': beamform_ds, 'mvdr': beamform_mvdr, 'clean': beamform_clean}[args.algo]

    # Audio circular buffer: holds enough samples for args.snap CSM blocks
    n_buf = args.snap * 128 + 256
    audio_buf: collections.deque = collections.deque(maxlen=n_buf)
    buf_lock = threading.Lock()

    def audio_cb(indata, frames, ts, status):  # ts avoids shadowing `time` module
        mic = indata[:, MIC_SLICE].copy()
        with buf_lock:
            for row in mic:
                audio_buf.append(row)

    stream = sd.InputStream(
        samplerate=16000, channels=6, dtype='float32',
        device=dev_idx, blocksize=256, callback=audio_cb,
    )

    cam = cv2.VideoCapture(args.video)
    if not cam.isOpened():
        print('No webcam — showing audio-only overlay on black frame')
        cam = None

    print(f'algo={args.algo}  freq={args.freq:.0f}Hz  fov=±{args.fov/2:.0f}°  '
          f'snap={args.snap}  cal={"yes" if cal_e is not None else "no"}')
    print('Press q to quit.')

    label = 'Filling buffer...'
    P = None
    t_last = time.monotonic()
    fps = 0.0

    win = 'Acoustic Camera — Phase 2'
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

            h, w = frame.shape[:2]
            strip_h = max(40, h // 8)

            with buf_lock:
                n_avail = len(audio_buf)

            if n_avail >= n_buf:
                with buf_lock:
                    arr = np.array(list(audio_buf), dtype=np.float32)  # (n_buf, 4)

                R = compute_csm(arr, args.freq)
                if cal_e is not None:
                    c = 1.0 / cal_e
                    R = np.outer(c, c.conj()) * R

                P = ALGO(R, args.freq, az_grid)
                az_peak = az_grid[np.argmax(P)]
                label = f'{args.algo.upper()}  {args.freq:.0f}Hz  peak={az_peak:.1f}'

            if P is not None:
                strip = energy_strip(P, w, strip_h)
                frame[-strip_h:] = cv2.addWeighted(frame[-strip_h:], 0.4, strip, 0.6, 0)

                # Peak marker
                px = int((az_peak + args.fov / 2) / args.fov * w)
                px = max(1, min(w - 2, px))
                cv2.line(frame, (px, 0), (px, h - strip_h), (0, 255, 0), 2)

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
