#!/usr/bin/env python3
"""Renders assets/battery_icon.png — a battery glyph with the current charge %
baked in — so the Trixie desktop icon (Desktop/battery.desktop) shows live
battery state. Run periodically by battery-icon.timer, or on-demand by tapping
the desktop icon itself (see RASPBERRY_PI.md §8).

Pi-only: needs apt's python3-smbus and the UPS's INA219 chip on the I2C bus.
"""
import os
import sys
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO / 'ups' / 'UPS_Module_3S_V2'))
from INA219 import INA219  # noqa: E402

_ICON_PATH = _REPO / 'assets' / 'battery_icon.png'
_DESKTOP_FILE = _REPO / 'Desktop' / 'battery.desktop'

# Same INA219 address and voltage-to-percent mapping as the live overlay's
# battery indicator (src/acoustic_camera_p3.py) and the vendor demo script.
_BATT_ADDR = 0x41
_BATT_EMPTY_V = 9.0
_BATT_SPAN_V = 3.6

_SIZE = 64              # matches assets/power_off.png and reboot.gif
_BODY_W, _BODY_H = 40, 20
_NUB_W = 5


def read_percent():
    ina219 = INA219(addr=_BATT_ADDR)
    percent = (ina219.getBusVoltage_V() - _BATT_EMPTY_V) / _BATT_SPAN_V * 100
    return max(0.0, min(100.0, percent))


def render_icon(percent):
    """percent=None renders a neutral placeholder (outline + "--%"), used to seed
    assets/battery_icon.png before the first real reading is available."""
    img = np.zeros((_SIZE, _SIZE, 4), dtype=np.uint8)  # transparent background,
                                                         # matching power_off.png/reboot.gif
    if percent is None:
        color = (120, 120, 120, 255)
    elif percent >= 50:
        color = (80, 200, 80, 255)
    elif percent >= 20:
        color = (0, 200, 220, 255)
    else:
        color = (0, 0, 220, 255)

    x0 = (_SIZE - _BODY_W - _NUB_W) // 2
    y0 = 10
    x1 = x0 + _BODY_W
    y1 = y0 + _BODY_H
    nub_y0 = y0 + _BODY_H // 4
    nub_y1 = y1 - _BODY_H // 4

    cv2.rectangle(img, (x0, y0), (x1, y1), (235, 235, 235, 255), 2)
    cv2.rectangle(img, (x1 + 1, nub_y0), (x1 + _NUB_W, nub_y1), (235, 235, 235, 255), -1)
    fill_w = int((_BODY_W - 6) * percent / 100) if percent is not None else 0
    if fill_w > 0:
        cv2.rectangle(img, (x0 + 3, y0 + 3), (x0 + 3 + fill_w, y1 - 3), color, -1)

    label = f'{percent:.0f}%' if percent is not None else '--%'
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
    (tw, th), _ = cv2.getTextSize(label, font, scale, thick)
    cv2.putText(img, label, ((_SIZE - tw) // 2, y1 + th + 8), font, scale,
                (235, 235, 235, 255), thick, cv2.LINE_AA)

    _ICON_PATH.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(_ICON_PATH), img)


def main():
    try:
        percent = read_percent()
    except Exception as e:
        print(f'[warn] could not read battery: {e}', file=sys.stderr)
        return
    render_icon(percent)
    # pcmanfm's desktop watches ~/Desktop for changes to redraw icons; touching
    # the .desktop file's mtime (its content is unchanged) nudges a refresh
    # since it doesn't otherwise notice the Icon= target's bytes changing.
    if _DESKTOP_FILE.exists():
        os.utime(_DESKTOP_FILE, None)


if __name__ == '__main__':
    main()
