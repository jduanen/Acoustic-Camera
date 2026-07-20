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
import json
import subprocess
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

# UPS battery gauge (Waveshare UPS 3S, see RASPBERRY_PI.md §8) — its INA219 driver
# lives outside src/, so it needs its directory added to sys.path. Only present on
# the Pi deployment (needs apt's python3-smbus, not pip) — ImportError on a dev
# machine just means the battery indicator stays hidden (see _poll_battery).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'ups' / 'UPS_Module_3S_V2'))
try:
    from INA219 import INA219
except ImportError:
    INA219 = None


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

    LABEL_H = 18   # pixels reserved at the bottom for axis labels (fits the larger tick font)
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
        (lw, _), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        cv2.putText(panel, lbl, (xl - lw // 2, height - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

    return panel


# --- Virtual sliders (mouse/touch-draggable) ---
# Layout, top to bottom: video frame -> Fhi strip -> spectrum plot -> Flo strip.
# Fhi and Flo are on opposite sides of the spectrum plot (not stacked together) so
# they have enough vertical separation to be reliable touch targets on a touchscreen.

_SLIDER_H = 44          # height of each individual slider strip
_SPECTRUM_H = 90        # height of the spectrum panel between the two slider strips
_FREQ_MAX = 8000        # shared max for both sliders, so the same track position means the same Hz on either

_SLIDER_FONT_SCALE = 0.7   # larger than the track-side margins strictly need, for
_SLIDER_FONT_THICK = 2     # readability — label and value live outside the track's
                           # x-range (see _track_geom); the handle is clamped to that
                           # same range below so it can't overhang into the margins
                           # and collide with them at the extreme ends of the range.
_SUB_FONT_SCALE = 0.45     # "hi"/"lo" subscript on the "F" label — smaller and
_SUB_FONT_THICK = 1        # baseline-shifted down (see _slider_strip)
_SUB_GAP = 2               # horizontal gap between "F" and its subscript
_SUB_DROP = 5              # how far the subscript's baseline sits below "F"'s
_SLIDER_TEXT_PAD = 6       # gap from the strip's left/right edge to the label/value text

# Track insets are asymmetric on purpose: the label ("F" + subscript, narrow) and the
# value ("NNNN Hz", up to 4 digits, much wider) need different amounts of margin to
# leave the *same visual gap* to the track on both sides. Sized from actual glyph
# widths (worst case: "hi"/"lo", whichever is wider; value width at _FREQ_MAX, the
# widest the live value ever gets) rather than hardcoded, so this stays correct if the
# fonts or _FREQ_MAX ever change.
_TRACK_GAP = 30  # desired empty space between the track and the text on each side
_label_w = max(
    cv2.getTextSize('F', cv2.FONT_HERSHEY_SIMPLEX, _SLIDER_FONT_SCALE, _SLIDER_FONT_THICK)[0][0]
    + _SUB_GAP
    + cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, _SUB_FONT_SCALE, _SUB_FONT_THICK)[0][0]
    for sub in ('hi', 'lo')
)
_value_w_max = cv2.getTextSize(f'{_FREQ_MAX} Hz', cv2.FONT_HERSHEY_SIMPLEX,
                                _SLIDER_FONT_SCALE, _SLIDER_FONT_THICK)[0][0]
_TRACK_LEFT = _SLIDER_TEXT_PAD + _label_w + _TRACK_GAP
_TRACK_RIGHT = _SLIDER_TEXT_PAD + _value_w_max + _TRACK_GAP

# Settings tab + popup (auto/manual range toggle + energy threshold), drawn inside the
# video frame itself rather than as a new full-width strip, since the display is fit
# exactly to a 1280x720 touch panel (see Picam2Capture's size comment below).
_TAB_SIZE = 48          # settings tab is a square button in the video frame's corner
                        # (sized for a comfortable touch target, not just a mouse click)
_TAB_MARGIN = 8         # inset from the video frame's top-right corner
_BATT_TAB_GAP = 16      # horizontal gap between the battery indicator and the tab,
                        # wider than _TAB_MARGIN so the tab keeps clear touch space
_POPUP_W = 260          # popup panel width (height is derived — see _popup_layout)
_POPUP_PAD = 8          # inner padding for popup contents
_POPUP_BTN_H = 28       # AUTO/MANUAL toggle button height
_POPUP_ROW_GAP = 8      # vertical gap between stacked popup rows
_THRESH_MAX = 100       # thresh_db slider range: 0-100 dB
_ALGOS = ['ds', 'mvdr', 'clean', 'music']  # algorithm dropdown order
_ALGO_ROW_H = 26        # height of each option row in the expanded algorithm list
_NSRC_MAX = N_MICS - 1  # beamform_music's own constraint: N_MICS - n_src >= 1
_SNAP_FLASH_S = 0.15    # duration of the on-screen "camera flash" after a screenshot
_SNAP_TEXT_S = 1.5      # duration of the "Saved ..." confirmation text

_sliders  = {'flo': 500, 'fhi': 4000, 'drag': None, 'frame_h': 480, 'frame_w': 640,
             'pan_x0': 0, 'pan_flo0': 500, 'pan_fhi0': 4000,
             'auto_range': True, 'thresh_db': 30, 'popup_open': False,
             'algo': 'ds', 'algo_open': False, 'nsrc': 1, 'paused': False,
             'exit_requested': False, 'screenshot_requested': False}
# Guards _sliders: written by _on_mouse (OpenCV's Qt backend may dispatch input
# callbacks off the main loop's thread) and read/corrected by the main loop each frame.
_sliders_lock = threading.Lock()


def _track_geom(w):
    """Shared by both sliders (and _on_mouse) so Flo/Hi always agree on where a
    given Hz value sits on screen: inset by _TRACK_LEFT/_TRACK_RIGHT — asymmetric so
    the visual gap to the label and to the value text matches on both sides despite
    their very different widths (see the constants above)."""
    return _TRACK_LEFT, max(w - _TRACK_LEFT - _TRACK_RIGHT, 1)


def _slider_strip(w, sub, val, vmax, color):
    """"F" with a subscript ('hi' or 'lo') in the left margin, the live Hz value in
    the right margin — same layout for both sliders so Flo and Fhi line up."""
    strip = np.full((_SLIDER_H, w, 3), 28, dtype=np.uint8)
    track_x0, track_w = _track_geom(w)
    yc = _SLIDER_H // 2
    cv2.rectangle(strip, (track_x0, yc - 3), (track_x0 + track_w, yc + 3), (80, 80, 80), -1)
    xh = track_x0 + int(val / vmax * track_w)
    handle_x0 = max(track_x0, xh - 5)
    handle_x1 = min(track_x0 + track_w, xh + 5)
    cv2.rectangle(strip, (handle_x0, yc - 7), (handle_x1, yc + 7), color, -1)

    (f_w, f_h), _ = cv2.getTextSize('F', cv2.FONT_HERSHEY_SIMPLEX,
                                     _SLIDER_FONT_SCALE, _SLIDER_FONT_THICK)
    # Combined glyph block spans [main_y - f_h, main_y + _SUB_DROP] (F's top to the
    # subscript's baseline); solving (top + bottom) / 2 == yc for main_y:
    main_y = yc + (f_h - _SUB_DROP) // 2
    cv2.putText(strip, 'F', (_SLIDER_TEXT_PAD, main_y),
                cv2.FONT_HERSHEY_SIMPLEX, _SLIDER_FONT_SCALE, (180, 180, 180), _SLIDER_FONT_THICK)
    cv2.putText(strip, sub, (_SLIDER_TEXT_PAD + f_w + _SUB_GAP, main_y + _SUB_DROP),
                cv2.FONT_HERSHEY_SIMPLEX, _SUB_FONT_SCALE, (180, 180, 180), _SUB_FONT_THICK)

    value_text = f'{val} Hz'
    (value_w, value_h), _ = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX,
                                             _SLIDER_FONT_SCALE, _SLIDER_FONT_THICK)
    cv2.putText(strip, value_text, (w - value_w - _SLIDER_TEXT_PAD, yc + value_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, _SLIDER_FONT_SCALE, (180, 180, 180), _SLIDER_FONT_THICK)
    return strip


def _popup_layout(w, algo_open, algo):
    """Settings tab + popup rects, in video-frame (x, y) coordinates, derived purely
    from frame width w (and algo_open/algo, for the expandable algorithm list and the
    MUSIC-only Nsrc row) — mirrors _track_geom's style so drawing and _on_mouse
    hit-testing can't disagree."""
    tab_x1 = w - _TAB_MARGIN
    tab_x0 = tab_x1 - _TAB_SIZE
    tab = (tab_x0, _TAB_MARGIN, tab_x1, _TAB_MARGIN + _TAB_SIZE)

    popup_x1 = tab_x1
    popup_x0 = max(popup_x1 - _POPUP_W, 0)
    popup_y0 = tab[3] + 4

    pause_btn = (popup_x0 + _POPUP_PAD, popup_y0 + _POPUP_PAD,
                 popup_x1 - _POPUP_PAD, popup_y0 + _POPUP_PAD + _POPUP_BTN_H)

    toggle = (pause_btn[0], pause_btn[3] + _POPUP_ROW_GAP,
              pause_btn[2], pause_btn[3] + _POPUP_ROW_GAP + _POPUP_BTN_H)

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

    content_bottom = algo_opts[-1][3] if algo_opts else algo_btn[3]

    nsrc_label_y = nsrc_track_x0 = nsrc_track_w = nsrc_track_y = None
    if algo == 'music':
        nsrc_label_y = content_bottom + _POPUP_ROW_GAP + 10
        nsrc_track_x0 = toggle[0] + 4
        nsrc_track_w = max((toggle[2] - 4) - nsrc_track_x0, 1)
        nsrc_track_y = nsrc_label_y + _POPUP_ROW_GAP + 10
        content_bottom = nsrc_track_y + 7  # handle half-height

    snap_btn = (toggle[0], content_bottom + _POPUP_ROW_GAP,
                toggle[2], content_bottom + _POPUP_ROW_GAP + _POPUP_BTN_H)
    content_bottom = snap_btn[3]

    # Exit sits last, separated from the frequently-used controls above it, and is
    # always present regardless of algo/algo_open state.
    exit_btn = (toggle[0], content_bottom + _POPUP_ROW_GAP,
                toggle[2], content_bottom + _POPUP_ROW_GAP + _POPUP_BTN_H)
    content_bottom = exit_btn[3]

    popup_bottom = content_bottom + _POPUP_PAD
    popup = (popup_x0, popup_y0, popup_x1, popup_bottom)

    return {'tab': tab, 'popup': popup, 'pause_btn': pause_btn, 'toggle': toggle,
            'label_y': label_y, 'track_x0': track_x0, 'track_w': track_w,
            'track_y': track_y, 'algo_btn': algo_btn, 'algo_opts': algo_opts,
            'nsrc_label_y': nsrc_label_y, 'nsrc_track_x0': nsrc_track_x0,
            'nsrc_track_w': nsrc_track_w, 'nsrc_track_y': nsrc_track_y,
            'snap_btn': snap_btn,
            'exit_btn': exit_btn}


def _draw_tab(frame, layout):
    """Always-visible tab in the video frame's corner; tapping it opens/closes the
    settings popup. Drawn as a hamburger icon (three bars) rather than text — OpenCV's
    Hershey fonts don't reliably render gear/settings glyphs, and a plain geometric
    icon reads clearly at touch-target size."""
    x0, y0, x1, y1 = layout['tab']
    fill = np.full((y1 - y0, x1 - x0, 3), 28, dtype=np.uint8)
    frame[y0:y1, x0:x1] = cv2.addWeighted(frame[y0:y1, x0:x1], 0.35, fill, 0.65, 0)
    bar_x0, bar_x1 = x0 + 11, x1 - 11
    for frac in (0.32, 0.5, 0.68):
        by = y0 + int((y1 - y0) * frac)
        cv2.line(frame, (bar_x0, by), (bar_x1, by), (220, 220, 220), 2)


_BATT_W = 44          # battery icon body width
_BATT_H = 20           # battery icon body height (matches the settings tab's row)
_BATT_NUB_W = 4        # width of the small terminal nub on the icon's right edge


def _draw_battery(frame, x1, percent):
    """Battery icon + percentage text, right edge at x1, vertically centered on the
    (taller) settings tab. Fill color shifts green -> amber -> red as charge drops."""
    y0 = _TAB_MARGIN + (_TAB_SIZE - _BATT_H) // 2
    y1 = y0 + _BATT_H
    body_x1 = x1 - _BATT_NUB_W - 2
    body_x0 = body_x1 - _BATT_W
    nub_y0 = y0 + _BATT_H // 4
    nub_y1 = y1 - _BATT_H // 4

    if percent >= 50:
        color = (80, 200, 80)
    elif percent >= 20:
        color = (0, 200, 220)
    else:
        color = (0, 0, 220)

    cv2.rectangle(frame, (body_x0, y0), (body_x1, y1), (200, 200, 200), 1)
    cv2.rectangle(frame, (body_x1, nub_y0), (body_x1 + _BATT_NUB_W, nub_y1), (200, 200, 200), -1)
    fill_w = int((_BATT_W - 4) * percent / 100)
    if fill_w > 0:
        cv2.rectangle(frame, (body_x0 + 2, y0 + 2), (body_x0 + 2 + fill_w, y1 - 2), color, -1)

    label = f'{percent:.0f}%'
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
    cv2.putText(frame, label, (body_x0 - tw - 6, y0 + (_BATT_H + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)


def _draw_popup(frame, layout, auto_range, thresh_db, algo, algo_open, nsrc, paused):
    """Pause/Resume button + AUTO/MANUAL toggle + energy threshold slider + algorithm
    dropdown (+ MUSIC-only Nsrc slider) + Snap (screenshot) + Exit buttons, drawn on
    top of the video frame."""
    x0, y0, x1, y1 = layout['popup']
    fill = np.full((y1 - y0, x1 - x0, 3), 28, dtype=np.uint8)
    frame[y0:y1, x0:x1] = cv2.addWeighted(frame[y0:y1, x0:x1], 0.25, fill, 0.75, 0)

    px0, py0, px1, py1 = layout['pause_btn']
    pause_color = (60, 60, 200) if paused else (70, 70, 70)  # red while paused, neutral while running
    cv2.rectangle(frame, (px0, py0), (px1, py1), pause_color, -1)
    pause_label = 'RESUME' if paused else 'PAUSE'
    (pw, ph), _ = cv2.getTextSize(pause_label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(frame, pause_label, (px0 + (px1 - px0 - pw) // 2, py0 + (py1 - py0 + ph) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1)

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

    if layout['nsrc_track_y'] is not None:
        cv2.putText(frame, f'Nsrc: {nsrc}', (x0 + _POPUP_PAD, layout['nsrc_label_y']),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        nx0, nw, ny = layout['nsrc_track_x0'], layout['nsrc_track_w'], layout['nsrc_track_y']
        cv2.rectangle(frame, (nx0, ny - 3), (nx0 + nw, ny + 3), (80, 80, 80), -1)
        nh = nx0 + int(nsrc / _NSRC_MAX * nw)
        cv2.rectangle(frame, (nh - 5, ny - 7), (nh + 5, ny + 7), (200, 120, 220), -1)

    sx0, sy0, sx1, sy1 = layout['snap_btn']
    cv2.rectangle(frame, (sx0, sy0), (sx1, sy1), (70, 70, 70), -1)
    (sw, sh), _ = cv2.getTextSize('SNAP', cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(frame, 'SNAP', (sx0 + (sx1 - sx0 - sw) // 2, sy0 + (sy1 - sy0 + sh) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

    ex0, ey0, ex1, ey1 = layout['exit_btn']
    cv2.rectangle(frame, (ex0, ey0), (ex1, ey1), (50, 50, 220), -1)  # strong red, distinct from Pause's dimmer red
    (ew, eh), _ = cv2.getTextSize('EXIT', cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(frame, 'EXIT', (ex0 + (ex1 - ex0 - ew) // 2, ey0 + (ey1 - ey0 + eh) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)


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
            layout = _popup_layout(_sliders['frame_w'], _sliders['algo_open'], _sliders['algo'])
            if event == cv2.EVENT_LBUTTONDOWN:
                tx0, ty0, tx1, ty1 = layout['tab']
                if tx0 <= x <= tx1 and ty0 <= y <= ty1:
                    _sliders['popup_open'] = not _sliders['popup_open']
                    if not _sliders['popup_open']:
                        _sliders['algo_open'] = False
                    return
                if not _sliders['popup_open']:
                    return
                pbx0, pby0, pbx1, pby1 = layout['pause_btn']
                if pbx0 <= x <= pbx1 and pby0 <= y <= pby1:
                    _sliders['paused'] = not _sliders['paused']
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
                sx0, sy0, sx1, sy1 = layout['snap_btn']
                if sx0 <= x <= sx1 and sy0 <= y <= sy1:
                    _sliders['screenshot_requested'] = True
                    return
                ex0, ey0, ex1, ey1 = layout['exit_btn']
                if ex0 <= x <= ex1 and ey0 <= y <= ey1:
                    _sliders['exit_requested'] = True
                    return
                px0, _, px1, _ = layout['popup']
                if px0 <= x <= px1 and abs(y - layout['track_y']) <= 12:
                    _sliders['drag'] = 'thresh'
                elif (layout['nsrc_track_y'] is not None and px0 <= x <= px1
                      and abs(y - layout['nsrc_track_y']) <= 12):
                    _sliders['drag'] = 'nsrc'
                else:
                    return
            if _sliders['drag'] == 'thresh':
                frac = max(0.0, min(1.0, (x - layout['track_x0']) / layout['track_w']))
                _sliders['thresh_db'] = int(round(frac * _THRESH_MAX))
            elif _sliders['drag'] == 'nsrc' and layout['nsrc_track_y'] is not None:
                frac = max(0.0, min(1.0, (x - layout['nsrc_track_x0']) / layout['nsrc_track_w']))
                _sliders['nsrc'] = max(1, min(_NSRC_MAX, int(round(frac * _NSRC_MAX))))
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


# --- Pi power health (undervoltage/throttling) ---
# Bit layout of `vcgencmd get_throttled`'s hex value (Raspberry Pi firmware docs):
# bits 0-3 = currently active (undervoltage / arm freq capped / throttled / soft temp
# limit); bits 16-19 = the same four conditions, latched since last reboot.
_THROTTLE_NOW_MASK = 0xF
_THROTTLE_EVER_MASK = 0xF0000

_throttle_status = {'now': False, 'ever': False}
_throttle_lock = threading.Lock()


def _poll_throttled():
    """Background thread: polls `vcgencmd get_throttled` every few seconds. Exits
    quietly (leaving both flags False) if vcgencmd isn't available — e.g. running
    off a Raspberry Pi, or the firmware tools aren't installed."""
    while True:
        try:
            out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                                  text=True, timeout=2, check=True).stdout.strip()
            val = int(out.split('=')[1], 16)
        except (OSError, subprocess.SubprocessError, IndexError, ValueError):
            return
        with _throttle_lock:
            _throttle_status['now'] = bool(val & _THROTTLE_NOW_MASK)
            _throttle_status['ever'] = bool(val & _THROTTLE_EVER_MASK)
        time.sleep(3)


# --- UPS battery gauge ---
# Waveshare UPS 3S module (3x 18650 Li-ion in series) + INA219 fuel-gauge chip,
# see RASPBERRY_PI.md §8. 9.0V/12.6V empty/full and 0x41 I2C address match the
# vendor's own sample script (ups/UPS_Module_3S_V2/INA219.py's __main__).
_BATT_ADDR = 0x41
_BATT_EMPTY_V = 9.0
_BATT_SPAN_V = 3.6

_battery_status = {'percent': None}  # None until a reading succeeds, or forever if no UPS is present
_battery_lock = threading.Lock()


def _poll_battery():
    """Background thread: polls the INA219 fuel-gauge chip every few seconds.
    Exits quietly (leaving percent at None, so the overlay stays hidden) if the
    chip isn't present — e.g. running off a dev machine with no UPS attached."""
    if INA219 is None:
        return
    try:
        ina219 = INA219(addr=_BATT_ADDR)
    except Exception:
        return
    while True:
        try:
            percent = (ina219.getBusVoltage_V() - _BATT_EMPTY_V) / _BATT_SPAN_V * 100
        except Exception:
            return
        with _battery_lock:
            _battery_status['percent'] = max(0.0, min(100.0, percent))
        time.sleep(3)


# --- Settings persistence ---
# Only the touch-adjustable settings (frequency band, algorithm, source count,
# auto-range, threshold) round-trip through the config file — everything else
# (camera/display/hardware setup) stays CLI-only, same split as the settings popup.

_CONFIG_KEYS = ('flo', 'fhi', 'algo', 'nsrc', 'auto_range', 'thresh_db')


def _load_config(path):
    """Return the persisted settings dict, or {} if the file is missing, unreadable,
    or not a JSON object — never fatal (first run, corrupt/hand-edited file, etc.)."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _cfg_int(config, key, default, lo, hi):
    """Read an int setting from a loaded config dict, clamped to [lo, hi]; falls
    back to `default` if the key is missing or not a valid number (e.g. hand-edited
    to garbage) — config values are external input, never trusted blindly."""
    try:
        return max(lo, min(hi, int(config[key])))
    except (KeyError, TypeError, ValueError):
        return default


def _save_config(path):
    """Persist the current touch-adjustable settings so the next run resumes where
    this one left off. Failure (e.g. read-only filesystem) is a warning, not fatal —
    it happens during shutdown, where raising would just mask a clean exit."""
    settings = {k: _sliders[k] for k in _CONFIG_KEYS}
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            json.dump(settings, f, indent=2)
    except OSError as e:
        print(f'[warn] could not save settings to {path}: {e}', file=sys.stderr)


def main():
    default_config = str(Path.home() / 'Code' / 'Acoustic-Camera' / 'config.json')

    ap = argparse.ArgumentParser(description='Phase 3 acoustic camera — UMA-16 v2')
    ap.add_argument('--algo',   choices=['ds', 'mvdr', 'clean', 'music'], default='ds',
                    help='beamforming algorithm (default: ds; overridden by the saved '
                         'config value if the config file has one)')
    ap.add_argument('--snap',   type=int,   default=128,     help='CSM blocks to average')
    ap.add_argument('--device', type=int,   default=None,    help='sounddevice index')
    ap.add_argument('--nsrc',   type=int,   default=1,
                    help='number of sources, MUSIC only (default: 1; overridden by the '
                         'saved config value if the config file has one)')
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
    ap.add_argument('--config', type=str, default=default_config,
                    help=f'settings file to load on startup and save on exit — freq band, '
                         f'algo, nsrc, auto-range, threshold (default: {default_config})')
    args = ap.parse_args()

    config_path = Path(args.config)
    config = _load_config(config_path)
    # Config wins over the CLI flag when the config file has a value for it — the CLI
    # flag is only the fallback for a first run, before any config file exists.
    if config.get('algo') in _ALGOS:
        args.algo = config['algo']
    args.nsrc = _cfg_int(config, 'nsrc', args.nsrc, 1, _NSRC_MAX)

    screengrabs_dir = Path.home() / 'Code' / 'Acoustic-Camera' / 'screengrabs'
    screengrabs_dir.mkdir(parents=True, exist_ok=True)

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
        'ds':    lambda R, f, n: beamform_ds(R, f, az_grid, el_grid),
        'mvdr':  lambda R, f, n: beamform_mvdr(R, f, az_grid, el_grid),
        'clean': lambda R, f, n: beamform_clean(R, f, az_grid, el_grid),
        'music': lambda R, f, n: beamform_music(R, f, az_grid, el_grid, n),
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

    print(f'algo={args.algo}  nsrc={args.nsrc}  '
          f'az_fov=±{args.az_fov/2:.0f}°  el_fov=±{args.el_fov/2:.0f}°  '
          f'snap={args.snap}  cal={"yes" if cal_e is not None else "no"}  '
          f'(freq/algo/nsrc set by on-screen controls)')
    print('Press q to quit.')

    P = None
    P_smooth = None
    latest_arr = None
    last_frame = None
    snap_saved_at = -1e9   # far enough in the past that no flash/text shows at startup
    snap_filename = ''
    ref_power = _REF_POWER_FLOOR
    az_peak = el_peak = 0.0
    label = 'Filling buffer...'
    t_last = time.monotonic()
    fps = 0.0

    win = 'Acoustic Camera - Phase 3'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    _sliders['flo'] = _cfg_int(config, 'flo', 500, 100, _FREQ_MAX)
    _sliders['fhi'] = _cfg_int(config, 'fhi', 4000, 100, _FREQ_MAX)
    _sliders['auto_range'] = bool(config.get('auto_range', True))
    _sliders['thresh_db'] = _cfg_int(config, 'thresh_db', 30, 0, _THRESH_MAX)
    _sliders['popup_open'] = False
    _sliders['algo'], _sliders['algo_open'] = args.algo, False
    _sliders['nsrc'] = max(1, min(_NSRC_MAX, args.nsrc))
    _sliders['paused'] = False
    _sliders['exit_requested'] = False
    _sliders['screenshot_requested'] = False
    mouse_registered = False
    threading.Thread(target=_poll_throttled, daemon=True).start()
    threading.Thread(target=_poll_battery, daemon=True).start()

    prof = collections.defaultdict(float)
    prof_n = 0

    try:
        with stream:
            while True:
                loop_start = time.monotonic()

                t0 = time.monotonic()
                with _sliders_lock:
                    paused = _sliders['paused']
                if not paused:
                    if cam is not None:
                        ret, frame = cam.read()
                        if not ret:
                            frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    else:
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    # Copy (not alias): later drawing (overlay/crosshair/tab/popup)
                    # mutates `frame` in place when P_smooth is still None, and must
                    # not also mutate the cached copy this pause freezes on.
                    last_frame = frame.copy()
                else:
                    # Frozen: reuse the last captured frame instead of grabbing a new
                    # one, and (below) skip the CSM/beamform update too, so the whole
                    # picture — video and overlay — stays still while paused.
                    frame = (last_frame.copy() if last_frame is not None
                              else np.zeros((480, 640, 3), dtype=np.uint8))
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
                    nsrc = _sliders['nsrc']
                freq = (flo + fhi) / 2

                with buf_lock:
                    n_avail = len(audio_buf)

                if n_avail >= n_buf and not paused:
                    with buf_lock:
                        arr = np.array(list(audio_buf), dtype=np.float32)  # (n_buf, 16)
                    latest_arr = arr
                    t2 = time.monotonic()

                    R = compute_csm(arr, freq)
                    if cal_e is not None:
                        c = 1.0 / cal_e
                        R = np.outer(c, c.conj()) * R

                    P = ALGO_FNS[algo](R, freq, nsrc)
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
                    exit_requested = _sliders['exit_requested']
                    screenshot_requested = _sliders['screenshot_requested']
                    if screenshot_requested:
                        _sliders['screenshot_requested'] = False

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
                popup_layout = _popup_layout(frame.shape[1], algo_open, algo)
                _draw_tab(frame, popup_layout)
                with _battery_lock:
                    batt_percent = _battery_status['percent']
                if batt_percent is not None:
                    _draw_battery(frame, popup_layout['tab'][0] - _BATT_TAB_GAP, batt_percent)
                if popup_open:
                    _draw_popup(frame, popup_layout, auto_range, thresh_db, algo, algo_open,
                                nsrc, paused)
                t4 = time.monotonic()

                now = time.monotonic()
                fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
                t_last = now

                status = f'{"[PAUSED] " if paused else ""}{label}  {fps:.1f}fps'
                cv2.putText(frame, status, (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                with _throttle_lock:
                    throttled_now = _throttle_status['now']
                    throttled_ever = _throttle_status['ever']
                if throttled_now:
                    cv2.putText(frame, '! LOW VOLTAGE / THROTTLED !', (10, 54),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                elif throttled_ever:
                    cv2.putText(frame, '(low-voltage/throttle event occurred)', (10, 54),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 1)

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
                    _slider_strip(w_f, 'hi', disp_fhi, _FREQ_MAX, (80, 160, 220)),
                    spec_panel_img,
                    _slider_strip(w_f, 'lo', disp_flo, _FREQ_MAX, (80, 200, 80)),
                ])

                if screenshot_requested:
                    ts = time.strftime('%Y%m%d_%H%M%S')
                    snap_path = screengrabs_dir / f'screengrab_{ts}.png'
                    n = 1
                    while snap_path.exists():
                        snap_path = screengrabs_dir / f'screengrab_{ts}_{n}.png'
                        n += 1
                    cv2.imwrite(str(snap_path), display)  # save before the flash/text below — the file stays clean
                    print(f'Saved screenshot: {snap_path}')
                    snap_saved_at = time.monotonic()
                    snap_filename = snap_path.name

                # On-screen confirmation: a brief camera-style flash plus a longer-lived
                # "Saved ..." caption, both drawn only on the live preview (after the
                # imwrite above), never baked into the saved file itself.
                snap_elapsed = time.monotonic() - snap_saved_at
                if snap_elapsed < _SNAP_FLASH_S:
                    flash_alpha = 1.0 - snap_elapsed / _SNAP_FLASH_S
                    white = np.full_like(display, 255)
                    display = cv2.addWeighted(display, 1 - flash_alpha, white, flash_alpha, 0)
                if snap_elapsed < _SNAP_TEXT_S:
                    snap_text = f'Saved {snap_filename}'
                    (stw, _), _ = cv2.getTextSize(snap_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    # Below the fps status line (y=28), and below the throttle line
                    # too when that's also showing, so the two never overlap.
                    snap_y = 80 if (throttled_now or throttled_ever) else 54
                    cv2.putText(display, snap_text, ((display.shape[1] - stw) // 2, snap_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 255, 80), 2)

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

                if key_result == ord('q') or exit_requested:
                    break

    finally:
        if cam is not None:
            cam.release()
        cv2.destroyAllWindows()
        _save_config(config_path)


if __name__ == '__main__':
    main()
