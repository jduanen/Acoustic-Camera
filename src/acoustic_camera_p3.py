#!/usr/bin/env python3
"""Phase 3 live acoustic camera: UMA-16 v2 (16-mic 4×4 URA) → 2D beamform → OpenCV overlay.

Usage:
    python src/acoustic_camera_p3.py
    python src/acoustic_camera_p3.py --algo mvdr
    python src/acoustic_camera_p3.py --algo music --nsrc 2
    python src/acoustic_camera_p3.py --cal test/UMA16/cal.npy --video 4

The beamforming frequency is set live by the on-screen F lo / F hi sliders
(freq = midpoint of the selected band), not by a CLI argument.
"""
import argparse
import collections
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import sounddevice as sd

from beamforming import (FS, N_MICS, NYQUIST,
                         beamform_ds, beamform_mvdr, beamform_clean,
                         beamform_music, compute_csm)


# --- Overlay rendering ---

_REF_POWER_FLOOR = 1e-10  # shared 0 dB reference: auto-mode's initial ref_power AND
                           # manual-mode's fixed reference


def acoustic_overlay(P_flat, frame, N_az, N_el, ref, alpha=0.5, db_range=30,
                      auto=True, thresh_db=30.0):
    """Blend 2D power map onto video frame as full-frame overlay.

    auto=True (default): percentile-stretch, ref-normalized against the caller's
    running-max `ref_power` — original behavior.
    auto=False: caller passes ref=_REF_POWER_FLOOR. Rescales against a fixed
    db_range-wide span starting at thresh_db instead of an auto-tracked percentile —
    cells below thresh_db fade toward the colormap's coolest color (not hidden);
    cells at/above thresh_db + db_range saturate at the hottest color.
    """
    h, w = frame.shape[:2]
    P_db = 10 * np.log10(np.maximum(P_flat.reshape(N_az, N_el) / max(ref, 1e-30), 1e-10))

    if auto:
        # Percentile stretch: maps 10th–100th percentile to full colormap range
        p_lo = np.percentile(P_db, 10)
        p_hi = P_db.max()
        norm = np.clip((P_db - p_lo) / max(p_hi - p_lo, 1e-6), 0, 1)
    else:
        norm = np.clip((P_db - thresh_db) / db_range, 0, 1)

    # Remap axes: (N_az, N_el) → (N_el, N_az) with +el at top (screen y=0)
    img8 = (norm.T[::-1, :] * 255).astype(np.uint8)
    colored = cv2.applyColorMap(
        cv2.resize(img8, (w, h), interpolation=cv2.INTER_LINEAR),
        cv2.COLORMAP_JET,
    )
    return cv2.addWeighted(frame, 1 - alpha, colored, alpha, 0)


def spectrum_panel(audio_arr, w, freq_mark, n_bars=64, height=90, fmin=0, fmax=6000):
    """Render real-time per-band power spectrum as a standalone strip
    (frequency axis left-to-right, magnitude as bar height)."""
    panel = np.full((height, w, 3), 28, dtype=np.uint8)
    block = 2048
    if audio_arr is None or audio_arr.shape[0] < block:
        return panel
    seg = audio_arr[-block:] * np.hanning(block)[:, np.newaxis]
    F = np.fft.rfft(seg, axis=0)
    psd = np.mean(np.abs(F) ** 2, axis=1)
    bin_freqs = np.fft.rfftfreq(block, 1.0 / FS)

    edges = np.linspace(fmin, fmax, n_bars + 1)
    bar_power = np.array([
        psd[(bin_freqs >= edges[k]) & (bin_freqs < edges[k + 1])].mean()
        if np.any((bin_freqs >= edges[k]) & (bin_freqs < edges[k + 1])) else 0.0
        for k in range(n_bars)
    ])
    bar_db = 10 * np.log10(bar_power + 1e-30)
    norm = np.clip((bar_db - bar_db.max() + 40) / 40, 0, 1)

    LABEL_H = 14   # pixels reserved at the bottom for axis labels
    bar_top = height - LABEL_H

    for k, v in enumerate(norm):
        x0 = int(k * w / n_bars)
        x1 = max(x0 + 1, int((k + 1) * w / n_bars))
        bar_h = int(v * (bar_top - 4))
        if bar_h > 0:
            cv2.rectangle(panel, (x0, bar_top - bar_h), (x1 - 1, bar_top - 1), (0, 200, 80), -1)

    span = max(fmax - fmin, 1)
    xf = max(1, min(w - 2, int((freq_mark - fmin) / span * w)))
    cv2.line(panel, (xf, 0), (xf, bar_top), (255, 255, 255), 2)
    if fmin < NYQUIST < fmax:
        xn = max(1, min(w - 2, int((NYQUIST - fmin) / span * w)))
        cv2.line(panel, (xn, 0), (xn, bar_top), (255, 0, 255), 1)

    # Frequency axis ticks and labels
    steps = [100, 200, 500, 1000, 2000, 5000]
    step = next((s for s in steps if span / s <= 6), 5000)
    first = int(np.ceil(fmin / step)) * step
    for f in range(first, fmax + 1, step):
        xl = int((f - fmin) / span * w)
        if xl < 2 or xl > w - 2:
            continue
        cv2.line(panel, (xl, bar_top), (xl, bar_top + 3), (140, 140, 140), 1)
        lbl = f'{f // 1000}k' if f % 1000 == 0 else f'{f / 1000:.1f}k' if f >= 1000 else str(f)
        cv2.putText(panel, lbl, (xl - 8, height - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (160, 160, 160), 1)

    return panel


# --- Virtual sliders (mouse/touch-draggable) ---
# Layout, top to bottom: video frame -> Fhi strip -> spectrum plot -> Flo strip.
# Fhi and Flo are on opposite sides of the spectrum plot (not stacked together) so
# they have enough vertical separation to be reliable touch targets on a touchscreen.

_SLIDER_H = 44          # height of each individual slider strip
_SPECTRUM_H = 90        # height of the spectrum panel between the two slider strips
_FREQ_MAX = 8000        # shared max for both sliders, so the same track position means the same Hz on either
_TRACK_X0 = 92          # left edge of slider track

# Settings tab + popup (auto/manual range toggle + energy threshold), drawn inside the
# video frame itself rather than as a new full-width strip, since the display is fit
# exactly to a 1280x720 touch panel (see Picam2Capture's size comment below).
_TAB_SIZE = 36          # settings tab is a square button in the video frame's corner
_TAB_MARGIN = 8         # inset from the video frame's top-right corner
_POPUP_W = 260          # popup panel width (height is derived — see _popup_layout)
_POPUP_PAD = 8          # inner padding for popup contents
_POPUP_BTN_H = 28       # AUTO/MANUAL toggle button height
_POPUP_ROW_GAP = 8      # vertical gap between stacked popup rows
_THRESH_MAX = 100       # thresh_db slider range: 0-100 dB
_ALGOS = ['ds', 'mvdr', 'clean', 'music']  # algorithm dropdown order
_ALGO_ROW_H = 26        # height of each option row in the expanded algorithm list

_sliders  = {'flo': 500, 'fhi': 4000, 'drag': None, 'frame_h': 480, 'frame_w': 640,
             'pan_x0': 0, 'pan_flo0': 500, 'pan_fhi0': 4000,
             'auto_range': True, 'thresh_db': 30, 'popup_open': False,
             'algo': 'ds', 'algo_open': False}
# Guards _sliders: written by _on_mouse (OpenCV's Qt backend may dispatch input
# callbacks off the main loop's thread) and read/corrected by the main loop each frame.
_sliders_lock = threading.Lock()


def _track_geom(w):
    """Shared by both sliders (and _on_mouse) so Flo/Hi always agree on where a
    given Hz value sits on screen: the intersection of the two labels' margins,
    i.e. a track inset by _TRACK_X0 on *both* sides regardless of label side."""
    return _TRACK_X0, max(w - 2 * _TRACK_X0, 1)


def _slider_strip(w, label, val, vmax, color, label_right=False):
    """label_right puts the text at the right edge instead of the left; the track
    itself (see _track_geom) is identical either way, so Flo and Fhi line up."""
    strip = np.full((_SLIDER_H, w, 3), 28, dtype=np.uint8)
    track_x0, track_w = _track_geom(w)
    yc = _SLIDER_H // 2
    cv2.rectangle(strip, (track_x0, yc - 3), (track_x0 + track_w, yc + 3), (80, 80, 80), -1)
    xh = track_x0 + int(val / vmax * track_w)
    cv2.rectangle(strip, (xh - 5, yc - 7), (xh + 5, yc + 7), color, -1)
    text = f'{label}: {val} Hz'
    if label_right:
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
        tx = w - text_w - 6
    else:
        tx = 4
    cv2.putText(strip, text, (tx, yc + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
    return strip


def _popup_layout(w, algo_open):
    """Settings tab + popup rects, in video-frame (x, y) coordinates, derived purely
    from frame width w (and algo_open, for the expandable algorithm list) — mirrors
    _track_geom's style so drawing and _on_mouse hit-testing can't disagree."""
    tab_x1 = w - _TAB_MARGIN
    tab_x0 = tab_x1 - _TAB_SIZE
    tab = (tab_x0, _TAB_MARGIN, tab_x1, _TAB_MARGIN + _TAB_SIZE)

    popup_x1 = tab_x1
    popup_x0 = max(popup_x1 - _POPUP_W, 0)
    popup_y0 = tab[3] + 4

    toggle = (popup_x0 + _POPUP_PAD, popup_y0 + _POPUP_PAD,
              popup_x1 - _POPUP_PAD, popup_y0 + _POPUP_PAD + _POPUP_BTN_H)

    label_y = toggle[3] + _POPUP_ROW_GAP + 10

    track_x0 = toggle[0] + 4
    track_w = max((toggle[2] - 4) - track_x0, 1)
    track_y = label_y + _POPUP_ROW_GAP + 10

    algo_btn = (toggle[0], track_y + 7 + _POPUP_ROW_GAP,
                toggle[2], track_y + 7 + _POPUP_ROW_GAP + _POPUP_BTN_H)

    algo_opts = []
    if algo_open:
        oy = algo_btn[3]
        for _ in _ALGOS:
            algo_opts.append((algo_btn[0], oy, algo_btn[2], oy + _ALGO_ROW_H))
            oy += _ALGO_ROW_H

    popup_bottom = (algo_opts[-1][3] if algo_opts else algo_btn[3]) + _POPUP_PAD
    popup = (popup_x0, popup_y0, popup_x1, popup_bottom)

    return {'tab': tab, 'popup': popup, 'toggle': toggle, 'label_y': label_y,
            'track_x0': track_x0, 'track_w': track_w, 'track_y': track_y,
            'algo_btn': algo_btn, 'algo_opts': algo_opts}


def _draw_tab(frame, layout):
    """Small always-visible tab in the video frame's corner; tapping it opens/closes
    the settings popup. Plain ASCII label — OpenCV's Hershey fonts don't reliably
    render gear/settings glyphs."""
    x0, y0, x1, y1 = layout['tab']
    fill = np.full((y1 - y0, x1 - x0, 3), 28, dtype=np.uint8)
    frame[y0:y1, x0:x1] = cv2.addWeighted(frame[y0:y1, x0:x1], 0.35, fill, 0.65, 0)
    cv2.putText(frame, 'E', (x0 + 11, y1 - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)


def _draw_popup(frame, layout, auto_range, thresh_db, algo, algo_open):
    """AUTO/MANUAL toggle + energy threshold slider + algorithm dropdown, drawn on
    top of the video frame."""
    x0, y0, x1, y1 = layout['popup']
    fill = np.full((y1 - y0, x1 - x0, 3), 28, dtype=np.uint8)
    frame[y0:y1, x0:x1] = cv2.addWeighted(frame[y0:y1, x0:x1], 0.25, fill, 0.75, 0)

    bx0, by0, bx1, by1 = layout['toggle']
    btn_color = (80, 160, 220) if auto_range else (80, 200, 80)  # echoes Fhi-blue / Flo-green
    cv2.rectangle(frame, (bx0, by0), (bx1, by1), btn_color, -1)
    label = 'AUTO' if auto_range else 'MANUAL'
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(frame, label, (bx0 + (bx1 - bx0 - tw) // 2, by0 + (by1 - by0 + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (20, 20, 20), 1)

    cv2.putText(frame, f'Thresh: {thresh_db:.0f} dB', (x0 + _POPUP_PAD, layout['label_y']),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

    track_x0, track_w, track_y = layout['track_x0'], layout['track_w'], layout['track_y']
    cv2.rectangle(frame, (track_x0, track_y - 3), (track_x0 + track_w, track_y + 3), (80, 80, 80), -1)
    xh = track_x0 + int(thresh_db / _THRESH_MAX * track_w)
    cv2.rectangle(frame, (xh - 5, track_y - 7), (xh + 5, track_y + 7), (220, 160, 80), -1)

    ax0, ay0, ax1, ay1 = layout['algo_btn']
    cv2.rectangle(frame, (ax0, ay0), (ax1, ay1), (70, 70, 70), -1)
    # Plain ASCII caret ('v'/'^') for the same reason the tab uses "E" instead of a
    # gear glyph — OpenCV's Hershey fonts don't reliably render unicode symbols.
    algo_label = f'Algo: {algo.upper()}  {"^" if algo_open else "v"}'
    (aw, ah), _ = cv2.getTextSize(algo_label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(frame, algo_label, (ax0 + (ax1 - ax0 - aw) // 2, ay0 + (ay1 - ay0 + ah) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

    for name, (ox0, oy0, ox1, oy1) in zip(_ALGOS, layout['algo_opts']):
        selected = name == algo
        cv2.rectangle(frame, (ox0, oy0), (ox1, oy1), (90, 90, 60) if selected else (45, 45, 45), -1)
        (ow, oh), _ = cv2.getTextSize(name.upper(), cv2.FONT_HERSHEY_SIMPLEX, 0.34, 1)
        cv2.putText(frame, name.upper(), (ox0 + 10, oy0 + (oy1 - oy0 + oh) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                    (230, 230, 230) if selected else (170, 170, 170), 1)


def _on_mouse(event, x, y, flags, _param):
    with _sliders_lock:
        if event == cv2.EVENT_LBUTTONUP:
            _sliders['drag'] = None
            return
        pressing = (event == cv2.EVENT_LBUTTONDOWN or
                    (event == cv2.EVENT_MOUSEMOVE and bool(flags & cv2.EVENT_FLAG_LBUTTON)))
        if not pressing:
            return
        rel_y = y - _sliders['frame_h']
        if rel_y < 0:
            # Inside the video frame: hit-test the settings tab/popup in raw (x, y)
            # video-frame coordinates (not rel_y strip space).
            layout = _popup_layout(_sliders['frame_w'], _sliders['algo_open'])
            if event == cv2.EVENT_LBUTTONDOWN:
                tx0, ty0, tx1, ty1 = layout['tab']
                if tx0 <= x <= tx1 and ty0 <= y <= ty1:
                    _sliders['popup_open'] = not _sliders['popup_open']
                    if not _sliders['popup_open']:
                        _sliders['algo_open'] = False
                    return
                if not _sliders['popup_open']:
                    return
                bx0, by0, bx1, by1 = layout['toggle']
                if bx0 <= x <= bx1 and by0 <= y <= by1:
                    _sliders['auto_range'] = not _sliders['auto_range']
                    return
                ax0, ay0, ax1, ay1 = layout['algo_btn']
                if ax0 <= x <= ax1 and ay0 <= y <= ay1:
                    _sliders['algo_open'] = not _sliders['algo_open']
                    return
                if _sliders['algo_open']:
                    for name, (ox0, oy0, ox1, oy1) in zip(_ALGOS, layout['algo_opts']):
                        if ox0 <= x <= ox1 and oy0 <= y <= oy1:
                            _sliders['algo'] = name
                            _sliders['algo_open'] = False
                            return
                px0, _, px1, _ = layout['popup']
                if px0 <= x <= px1 and abs(y - layout['track_y']) <= 12:
                    _sliders['drag'] = 'thresh'
                else:
                    return
            if _sliders['drag'] == 'thresh':
                frac = max(0.0, min(1.0, (x - layout['track_x0']) / layout['track_w']))
                _sliders['thresh_db'] = int(round(frac * _THRESH_MAX))
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if rel_y < _SLIDER_H:
                _sliders['drag'] = 'hi'
            elif rel_y < _SLIDER_H + _SPECTRUM_H:
                # Dragging the spectrum plot pans both sliders together, preserving
                # the Fhi-Flo gap, instead of resizing the range.
                _sliders['drag'] = 'pan'
                _sliders['pan_x0'] = x
                _sliders['pan_flo0'] = _sliders['flo']
                _sliders['pan_fhi0'] = _sliders['fhi']
            else:
                _sliders['drag'] = 'lo'
        if _sliders['drag'] in ('lo', 'hi'):
            track_x0, track_w = _track_geom(_sliders['frame_w'])
            frac = max(0.0, min(1.0, (x - track_x0) / track_w))
            if _sliders['drag'] == 'lo':
                _sliders['flo'] = max(100, min(_sliders['fhi'] - 100, int(frac * _FREQ_MAX)))
            else:
                _sliders['fhi'] = max(_sliders['flo'] + 100, int(frac * _FREQ_MAX))
        elif _sliders['drag'] == 'pan':
            _, track_w = _track_geom(_sliders['frame_w'])
            dx_hz = (x - _sliders['pan_x0']) / track_w * _FREQ_MAX
            gap = _sliders['pan_fhi0'] - _sliders['pan_flo0']
            new_flo = _sliders['pan_flo0'] + dx_hz
            new_fhi = _sliders['pan_fhi0'] + dx_hz
            if new_flo < 100:
                new_flo, new_fhi = 100, 100 + gap
            if new_fhi > _FREQ_MAX:
                new_flo, new_fhi = _FREQ_MAX - gap, _FREQ_MAX
            _sliders['flo'] = int(new_flo)
            _sliders['fhi'] = int(new_fhi)


# --- Main ---

class Picam2Capture:
    """Wraps picamera2 to match the cv2.VideoCapture .read()/.release() interface
    used by the main loop, so a MIPI-CSI camera is a drop-in swap for a USB webcam."""

    def __init__(self, size=(1280, 542)):
        # 542 = 720 - (2 * _SLIDER_H + _SPECTRUM_H) = 720 - (2*44 + 90): leaves exactly
        # enough room for the Fhi strip, spectrum plot, and Flo strip below, so the
        # combined display matches the 1280x720 panel with no letterboxing.
        try:
            from picamera2 import Picamera2
        except ImportError as e:
            raise RuntimeError(
                'picamera2 not found — install it with: sudo apt install python3-picamera2 '
                '(and create the venv with --system-site-packages, same as for cv2)'
            ) from e
        self._picam2 = Picamera2()
        # NB: picamera2's "RGB888" format is actually packed BGR in memory, which
        # happens to be exactly what cv2 wants — no cvtColor needed.
        config = self._picam2.create_video_configuration(main={'size': size, 'format': 'RGB888'})
        self._picam2.configure(config)
        self._picam2.start()

    def read(self):
        return True, self._picam2.capture_array()

    def release(self):
        self._picam2.stop()


def find_device():
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] >= 16 and 'uma' in d['name'].lower():
            return i
    return None


def main():
    ap = argparse.ArgumentParser(description='Phase 3 acoustic camera — UMA-16 v2')
    ap.add_argument('--algo',   choices=['ds', 'mvdr', 'clean', 'music'], default='ds')
    ap.add_argument('--snap',   type=int,   default=128,     help='CSM blocks to average')
    ap.add_argument('--device', type=int,   default=None,    help='sounddevice index')
    ap.add_argument('--nsrc',   type=int,   default=1,       help='number of sources (MUSIC only)')
    ap.add_argument('--cal',    type=str,   default=None,    help='path to cal.npy')
    # Matches the 100° CSI lens (see RASPBERRY_PI.md §4). el_fov is scaled from az_fov
    # by the captured frame's pixel aspect ratio (542/1280) rather than a claimed
    # vertical-FOV spec, for consistency with the linear degrees-per-pixel overlay
    # mapping below — treat as an approximation pending empirical verification.
    ap.add_argument('--az_fov',   type=float, default=100.0, help='azimuth display FOV (deg)')
    ap.add_argument('--el_fov',   type=float, default=42.0,  help='elevation display FOV (deg)')
    ap.add_argument('--grid_deg', type=float, default=0.5,   help='beamforming grid spacing (deg); 0.5=181×121, 1.0=91×61')
    ap.add_argument('--alpha',    type=float, default=0.5,   help='acoustic overlay opacity (0–1)')
    ap.add_argument('--smooth',   type=float, default=0.7,   help='temporal smoothing factor (0=none, 0.9=heavy)')
    ap.add_argument('--video',    type=int,   default=4,     help='cv2.VideoCapture device index (ignored if --csi)')
    ap.add_argument('--csi',       action='store_true',      help='use MIPI-CSI camera via picamera2 instead of a USB webcam')
    ap.add_argument('--fullscreen', action='store_true',     help='show the display fullscreen (borderless)')
    ap.add_argument('--profile', action='store_true',        help='print per-stage timing breakdown to stderr')
    args = ap.parse_args()

    cal_e = None
    if args.cal:
        if not Path(args.cal).exists():
            sys.exit(f'ERROR: calibration file not found: {args.cal}')
        cal_e = np.load(args.cal)
        if cal_e.shape != (N_MICS,):
            sys.exit(f'ERROR: calibration must have shape ({N_MICS},), '
                     f'got {cal_e.shape}: {args.cal}')
        if np.any(cal_e == 0):
            sys.exit(f'ERROR: calibration contains zero entries '
                     f'(would divide by zero): {args.cal}')
    az_grid = np.linspace(-args.az_fov / 2, args.az_fov / 2,
                          round(args.az_fov / args.grid_deg) + 1)
    el_grid = np.linspace(-args.el_fov / 2, args.el_fov / 2,
                          round(args.el_fov / args.grid_deg) + 1)
    N_az, N_el = len(az_grid), len(el_grid)

    dev_idx = args.device if args.device is not None else find_device()
    if dev_idx is None:
        raise RuntimeError('UMA-16 not found — use --device IDX to specify')

    ALGO_FNS = {
        'ds':    lambda R, f: beamform_ds(R, f, az_grid, el_grid),
        'mvdr':  lambda R, f: beamform_mvdr(R, f, az_grid, el_grid),
        'clean': lambda R, f: beamform_clean(R, f, az_grid, el_grid),
        'music': lambda R, f: beamform_music(R, f, az_grid, el_grid, args.nsrc),
    }

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

    if args.csi:
        cam = Picam2Capture()
    else:
        cam = cv2.VideoCapture(args.video)
        if not cam.isOpened():
            print('No webcam — showing audio-only overlay on black frame')
            cam = None

    print(f'algo={args.algo}  '
          f'az_fov=±{args.az_fov/2:.0f}°  el_fov=±{args.el_fov/2:.0f}°  '
          f'snap={args.snap}  cal={"yes" if cal_e is not None else "no"}  '
          f'(freq/algo set by on-screen controls)')
    print('Press q to quit.')

    P = None
    P_smooth = None
    latest_arr = None
    ref_power = _REF_POWER_FLOOR
    az_peak = el_peak = 0.0
    label = 'Filling buffer...'
    t_last = time.monotonic()
    fps = 0.0

    win = 'Acoustic Camera - Phase 3'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    _sliders['flo'], _sliders['fhi'] = 500, 4000
    _sliders['auto_range'], _sliders['thresh_db'], _sliders['popup_open'] = True, 30, False
    _sliders['algo'], _sliders['algo_open'] = args.algo, False
    mouse_registered = False

    prof = collections.defaultdict(float)
    prof_n = 0

    try:
        with stream:
            while True:
                loop_start = time.monotonic()

                t0 = time.monotonic()
                if cam is not None:
                    ret, frame = cam.read()
                    if not ret:
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                t1 = time.monotonic()

                with _sliders_lock:
                    flo = _sliders['flo']
                    fhi = _sliders['fhi']
                    if flo > fhi - 100:
                        # Self-healing ordering guarantee, kept under the same lock as
                        # _on_mouse's writes so this can't race with a concurrent update.
                        if _sliders['drag'] == 'lo':
                            fhi = _sliders['fhi'] = flo + 100
                        else:
                            flo = _sliders['flo'] = fhi - 100
                    algo = _sliders['algo']
                freq = (flo + fhi) / 2

                with buf_lock:
                    n_avail = len(audio_buf)

                if n_avail >= n_buf:
                    with buf_lock:
                        arr = np.array(list(audio_buf), dtype=np.float32)  # (n_buf, 16)
                    latest_arr = arr
                    t2 = time.monotonic()

                    R = compute_csm(arr, freq)
                    if cal_e is not None:
                        c = 1.0 / cal_e
                        R = np.outer(c, c.conj()) * R

                    P = ALGO_FNS[algo](R, freq)
                    # Temporal smoothing: reduces frame-to-frame jitter
                    if P_smooth is None:
                        P_smooth = P.copy()
                    else:
                        P_smooth = args.smooth * P_smooth + (1 - args.smooth) * P
                    ref_power = max(ref_power * 0.98, P_smooth.max())

                    k = np.argmax(P_smooth)
                    az_peak = az_grid[k // N_el]
                    el_peak = el_grid[k % N_el]
                    label = (f'{algo.upper()}  {freq:.0f}Hz  '
                             f'az={az_peak:.1f}°  el={el_peak:.1f}°')
                    t3 = time.monotonic()
                else:
                    t2 = t3 = time.monotonic()

                with _sliders_lock:
                    auto_range = _sliders['auto_range']
                    thresh_db = _sliders['thresh_db']
                    popup_open = _sliders['popup_open']
                    algo_open = _sliders['algo_open']

                if P_smooth is not None:
                    ref = ref_power if auto_range else _REF_POWER_FLOOR
                    frame = acoustic_overlay(P_smooth, frame, N_az, N_el, ref, alpha=args.alpha,
                                              auto=auto_range, thresh_db=thresh_db)

                    # Cross-hair at peak direction
                    h, w = frame.shape[:2]
                    px = int((az_peak + args.az_fov / 2) / args.az_fov * w)
                    py = int((args.el_fov / 2 - el_peak) / args.el_fov * h)
                    px = max(1, min(w - 2, px))
                    py = max(1, min(h - 2, py))
                    cv2.line(frame, (px, 0), (px, h), (0, 255, 0), 1)
                    cv2.line(frame, (0, py), (w, py), (0, 255, 0), 1)

                # Settings tab/popup live inside the video frame itself (not a new
                # strip), drawn every frame regardless of P_smooth so the tab is
                # visible even while the audio buffer is still filling.
                popup_layout = _popup_layout(frame.shape[1], algo_open)
                _draw_tab(frame, popup_layout)
                if popup_open:
                    _draw_popup(frame, popup_layout, auto_range, thresh_db, algo, algo_open)
                t4 = time.monotonic()

                now = time.monotonic()
                fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
                t_last = now

                cv2.putText(frame, f'{label}  {fps:.1f}fps', (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                h_f, w_f = frame.shape[:2]
                with _sliders_lock:
                    _sliders['frame_h'] = h_f
                    _sliders['frame_w'] = w_f
                    # Re-read live values for display: `flo`/`fhi` above were snapshotted
                    # before this frame's CSM/beamform work (~15-30 ms), and _on_mouse can
                    # update the real sliders during that window via touch events. Drawing
                    # the stale snapshot here would visibly lag the user's actual finger
                    # position, which looks like the handles crossing during a fast drag.
                    disp_flo = _sliders['flo']
                    disp_fhi = _sliders['fhi']
                spec_panel_img = spectrum_panel(latest_arr, w_f, freq, height=_SPECTRUM_H,
                                                 fmin=disp_flo, fmax=disp_fhi)
                display = np.vstack([
                    frame,
                    _slider_strip(w_f, 'F hi', disp_fhi, _FREQ_MAX, (80, 160, 220), label_right=True),
                    spec_panel_img,
                    _slider_strip(w_f, 'F lo', disp_flo, _FREQ_MAX, (80, 200, 80)),
                ])
                cv2.imshow(win, display)
                if not mouse_registered:
                    cv2.setMouseCallback(win, _on_mouse)
                    mouse_registered = True
                key_result = cv2.waitKey(1) & 0xFF
                t5 = time.monotonic()

                # Adaptive pacing: sleep only the remainder of the 50 ms (20 fps) budget,
                # rather than always sleeping the full 50 ms on top of the work above.
                time.sleep(max(0.0, 0.05 - (t5 - loop_start)))

                if args.profile:
                    prof['cam']      += t1 - t0
                    prof['buf_copy'] += t2 - t1
                    prof['csm_algo'] += t3 - t2
                    prof['overlay']  += t4 - t3
                    prof['imshow']   += t5 - t4
                    prof_n += 1
                    if prof_n >= 20:
                        parts = '  '.join(f'{k}={1000 * v / prof_n:5.1f}ms' for k, v in prof.items())
                        print(f'[profile] {parts}  total={1000 * sum(prof.values()) / prof_n:5.1f}ms',
                              file=sys.stderr)
                        prof.clear()
                        prof_n = 0

                if key_result == ord('q'):
                    break

    finally:
        if cam is not None:
            cam.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
