#!/usr/bin/env python3
"""Generate notebooks/17_uma16_calibration.ipynb"""
import json
from pathlib import Path


def code(src, cid):
    return {
        "cell_type": "code", "id": cid,
        "source": src, "metadata": {},
        "outputs": [], "execution_count": None,
    }


def md(src, cid):
    return {"cell_type": "markdown", "id": cid, "source": src, "metadata": {}}


cells = []

# ---------------------------------------------------------------------------
# Title
# ---------------------------------------------------------------------------
cells.append(md(
    "# 17 — UMA-16 v2 Calibration\n"
    "\n"
    "**Goal**: measure and correct per-mic gain and phase offsets using a\n"
    "cross-correlation approach on a boresight reference recording.\n"
    "\n"
    "Steps:\n"
    "1. Confirm device and geometry\n"
    "2. Record a reference clip — **1 kHz tone from boresight (0°)**\n"
    "3. Estimate per-mic delay via frequency-domain cross-correlation with ch0\n"
    "4. Estimate per-mic gain from RMS ratios\n"
    "5. Build calibration vector `e = g * exp(j*phi)` and save to `test/UMA16/cal.npy`\n"
    "6. Visualise effect on CSM phase\n"
    "7. Validate: beamform boresight recording before vs after — peak should move to 0°\n",
    "ab170001",
))

# ---------------------------------------------------------------------------
# Imports and constants
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import sounddevice as sd\n"
    "import scipy.io.wavfile as wavfile\n"
    "from scipy.linalg import inv\n"
    "from pathlib import Path\n"
    "\n"
    "plt.rcParams['figure.dpi'] = 120\n"
    "plt.rcParams['font.size'] = 10\n"
    "\n"
    "C        = 343.0\n"
    "FS       = 48000\n"
    "N_MICS   = 16\n"
    "CAL_FREQ = 1000.0   # calibration tone (Hz) — well below 4083 Hz Nyquist\n"
    "CAL_SECS = 5\n"
    "\n"
    "# UMA-16 v2 mic positions (camera/observer frame — x negated vs manual Figure 1)\n"
    "d = 0.042\n"
    "_xy = np.array([\n"
    "    (-1, -3), (-3, -3), (-1, -1), (-3, -1),\n"
    "    (-1, +1), (-3, +1), (-1, +3), (-3, +3),\n"
    "    (+3, +3), (+1, +3), (+3, +1), (+1, +1),\n"
    "    (+3, -1), (+1, -1), (+3, -3), (+1, -3),\n"
    "], dtype=float) * (d / 2)\n"
    "MIC_X = -_xy[:, 0]\n"
    "MIC_Y =  _xy[:, 1]\n"
    "\n"
    "CAL_PATH = Path('../test/UMA16/cal.npy')\n"
    "REF_WAV  = Path('../test/UMA16/cal_ref.wav')\n",
    "ab170002",
))

# ---------------------------------------------------------------------------
# Section 1: Device discovery
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1  Device Discovery",
    "ab170010",
))

cells.append(code(
    "devs = sd.query_devices()\n"
    "uma_idx = None\n"
    "for i, d in enumerate(devs):\n"
    "    if 'uma' in d['name'].lower():\n"
    "        uma_idx = i; uma_dev = d; break\n"
    "if uma_idx is None:\n"
    "    uma_idx = 12; uma_dev = devs[12]\n"
    "\n"
    "print(f'Device [{uma_idx}]: {uma_dev[\"name\"]}')\n"
    "print(f'  {uma_dev[\"max_input_channels\"]} ch  {FS} Hz  '\n"
    "      f'{uma_dev[\"default_low_input_latency\"]*1000:.1f} ms latency')\n",
    "ab170011",
))

# ---------------------------------------------------------------------------
# Section 2: Reference recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2  Reference Recording\n"
    "\n"
    "**Before running this cell**: place a tone source playing **1 kHz** directly in\n"
    "front of the array at **boresight (0°)**, approximately 0.5–1 m away.\n"
    "A phone or laptop speaker works fine.  Keep ambient noise low.\n"
    "\n"
    "Recording starts immediately when the cell runs.",
    "ab170020",
))

cells.append(code(
    "print(f'Recording {CAL_SECS}s reference (1 kHz boresight tone) ...')\n"
    "ref_rec = sd.rec(CAL_SECS * FS, samplerate=FS, channels=N_MICS,\n"
    "                 dtype='float32', device=uma_idx)\n"
    "sd.wait()\n"
    "print('Done.')\n"
    "\n"
    "REF_WAV.parent.mkdir(parents=True, exist_ok=True)\n"
    "wavfile.write(REF_WAV, FS, (ref_rec * 32767).astype(np.int16))\n"
    "print(f'Saved {REF_WAV}')\n"
    "\n"
    "rms = np.sqrt(np.mean(ref_rec**2, axis=0))\n"
    "print('\\nRMS per channel (should all be elevated above ambient):')\n"
    "for i, r in enumerate(rms):\n"
    "    bar = '#' * int(r / rms.max() * 30)\n"
    "    print(f'  ch{i:2d}: {r:.2e}  {bar}')\n",
    "ab170021",
))

# ---------------------------------------------------------------------------
# Section 3: Delay and gain estimation
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3  Delay & Gain Estimation\n"
    "\n"
    "Cross-correlate each channel with ch0 (frequency domain) to find the fractional-sample\n"
    "delay.  At boresight the true delays are all zero, so any measured delay is a\n"
    "hardware offset to be corrected.  Gain ratios come from RMS comparisons.",
    "ab170030",
))

cells.append(code(
    "def estimate_delay(ref_ch, mic_ch):\n"
    "    \"\"\"Return delay in seconds (positive = mic_ch lags ref_ch).\"\"\"\n"
    "    n = len(ref_ch)\n"
    "    F0 = np.fft.rfft(ref_ch, n=n)\n"
    "    Fn = np.fft.rfft(mic_ch, n=n)\n"
    "    xcorr = np.fft.irfft(np.conj(F0) * Fn, n=n)\n"
    "    lag = np.argmax(np.abs(xcorr))\n"
    "    if lag > n // 2:\n"
    "        lag -= n\n"
    "    return lag / FS\n"
    "\n"
    "\n"
    "delays = np.zeros(N_MICS)\n"
    "for i in range(1, N_MICS):\n"
    "    delays[i] = estimate_delay(ref_rec[:, 0], ref_rec[:, i])\n"
    "\n"
    "gains = rms / rms[0]\n"
    "\n"
    "print('Delays relative to ch0 (should be ~0 for boresight):')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  ch{i:2d}: {delays[i]*1e6:+7.2f} µs  ({delays[i]*FS:+6.3f} samples)')\n"
    "\n"
    "print('\\nGain ratios relative to ch0:')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  ch{i:2d}: {gains[i]:.4f}  ({20*np.log10(gains[i]):+.2f} dB)')\n"
    "\n"
    "print(f'\\nMax gain spread: {20*np.log10(gains.max()/gains.min()):.2f} dB')\n",
    "ab170031",
))

# ---------------------------------------------------------------------------
# Section 4: Build calibration vector
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4  Calibration Vector\n"
    "\n"
    "`e[i] = gain[i] * exp(j * 2π * f_cal * delay[i])`\n"
    "\n"
    "Applied to the CSM as `R_cal = outer(1/e, conj(1/e)) * R_uncal`.",
    "ab170040",
))

cells.append(code(
    "phi = 2 * np.pi * CAL_FREQ * delays\n"
    "e   = gains * np.exp(1j * phi)\n"
    "\n"
    "print('Calibration vector (gain × phase):')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  ch{i:2d}: |e|={np.abs(e[i]):.4f}  phase={np.degrees(np.angle(e[i])):+7.2f}°')\n"
    "\n"
    "CAL_PATH.parent.mkdir(parents=True, exist_ok=True)\n"
    "np.save(CAL_PATH, e)\n"
    "print(f'\\nSaved {CAL_PATH}')\n",
    "ab170041",
))

# ---------------------------------------------------------------------------
# Section 5: CSM before vs after
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5  CSM Phase Before vs After\n"
    "\n"
    "After calibration the off-diagonal phases should be near 0° — all mics appear\n"
    "in phase for a boresight source.",
    "ab170050",
))

cells.append(code(
    "def compute_csm(audio, freq, block_size=2048, hop=1024):\n"
    "    n_samp, n_ch = audio.shape\n"
    "    freqs  = np.fft.rfftfreq(block_size, 1 / FS)\n"
    "    f_idx  = np.argmin(np.abs(freqs - freq))\n"
    "    f_act  = freqs[f_idx]\n"
    "    win    = np.hanning(block_size)\n"
    "    R = np.zeros((n_ch, n_ch), dtype=complex)\n"
    "    count = 0\n"
    "    for start in range(0, n_samp - block_size, hop):\n"
    "        block = audio[start:start + block_size] * win[:, None]\n"
    "        F = np.fft.rfft(block, axis=0)[f_idx]\n"
    "        R += np.outer(F, F.conj())\n"
    "        count += 1\n"
    "    return R / count, f_act, count\n"
    "\n"
    "\n"
    "def apply_cal(R, e):\n"
    "    c = 1.0 / e\n"
    "    return np.outer(c, c.conj()) * R\n"
    "\n"
    "\n"
    "R_ref, f_ref, n_blocks = compute_csm(ref_rec, CAL_FREQ)\n"
    "R_cal = apply_cal(R_ref, e)\n"
    "\n"
    "print(f'CSM at {f_ref:.1f} Hz, {n_blocks} blocks')\n"
    "\n"
    "fig, axes = plt.subplots(2, 2, figsize=(11, 8))\n"
    "for row, (R, title) in enumerate([(R_ref, 'Uncalibrated'), (R_cal, 'Calibrated')]):\n"
    "    im0 = axes[row, 0].imshow(np.abs(R), cmap='inferno', aspect='auto')\n"
    "    axes[row, 0].set_title(f'{title} |CSM| @ {f_ref:.0f} Hz')\n"
    "    plt.colorbar(im0, ax=axes[row, 0])\n"
    "    im1 = axes[row, 1].imshow(np.degrees(np.angle(R)), cmap='hsv',\n"
    "                              aspect='auto', vmin=-180, vmax=180)\n"
    "    axes[row, 1].set_title(f'{title} phase (deg)')\n"
    "    plt.colorbar(im1, ax=axes[row, 1], label='degrees')\n"
    "    for ax in axes[row]:\n"
    "        ax.set_xlabel('channel'); ax.set_ylabel('channel')\n"
    "\n"
    "plt.suptitle(f'CSM before/after calibration at {f_ref:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_cal_csm.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print('Off-diagonal phase — BEFORE (sample of adjacent pairs):')\n"
    "for (i, j) in [(0,1),(0,2),(0,4),(4,8),(8,12)]:\n"
    "    print(f'  ch{i}-ch{j}: {np.degrees(np.angle(R_ref[i,j])):+.1f}°  '\n"
    "          f'→ after: {np.degrees(np.angle(R_cal[i,j])):+.1f}°')\n",
    "ab170051",
))

# ---------------------------------------------------------------------------
# Section 6: DoA before vs after on boresight recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6  DoA Validation — Boresight\n"
    "\n"
    "Beamform the reference recording before and after calibration.  The calibrated\n"
    "peak should land at or very near 0°.",
    "ab170060",
))

cells.append(code(
    "def steer(az_deg, el_deg, freq):\n"
    "    sin_az = np.sin(np.radians(az_deg))\n"
    "    cos_el = np.cos(np.radians(el_deg))\n"
    "    sin_el = np.sin(np.radians(el_deg))\n"
    "    ux = np.outer(sin_az, cos_el).ravel()\n"
    "    uy = np.tile(sin_el, len(az_deg))\n"
    "    ph = 2 * np.pi * freq / C * (np.outer(MIC_X, ux) + np.outer(MIC_Y, uy))\n"
    "    return np.exp(1j * ph) / np.sqrt(N_MICS)\n"
    "\n"
    "\n"
    "def bf_ds(R, H):\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def bf_mvdr(R, H, dl=0.01):\n"
    "    Rl = R + dl * np.trace(R) / N_MICS * np.eye(N_MICS)\n"
    "    Ri = inv(Rl)\n"
    "    return 1.0 / np.maximum(np.real(np.sum(H.conj() * (Ri @ H), axis=0)), 1e-300)\n"
    "\n"
    "\n"
    "def db_norm(p):\n"
    "    return 10 * np.log10(np.maximum(p / p.max(), 1e-10))\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "el_fix  = np.array([0.0])\n"
    "H = steer(az_grid, el_fix, f_ref)\n"
    "\n"
    "P_ds_u  = bf_ds(R_ref, H)\n"
    "P_ds_c  = bf_ds(R_cal, H)\n"
    "P_mv_u  = bf_mvdr(R_ref, H)\n"
    "P_mv_c  = bf_mvdr(R_cal, H)\n"
    "\n"
    "az_ds_u = az_grid[np.argmax(P_ds_u)]\n"
    "az_ds_c = az_grid[np.argmax(P_ds_c)]\n"
    "az_mv_u = az_grid[np.argmax(P_mv_u)]\n"
    "az_mv_c = az_grid[np.argmax(P_mv_c)]\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)\n"
    "for ax, label, P_u, P_c, az_u, az_c in [\n"
    "    (axes[0], 'D&S',  P_ds_u, P_ds_c, az_ds_u, az_ds_c),\n"
    "    (axes[1], 'MVDR', P_mv_u, P_mv_c, az_mv_u, az_mv_c),\n"
    "]:\n"
    "    ax.plot(az_grid, db_norm(P_u), 'steelblue',  lw=1.2, label=f'uncal {az_u:+.1f}°')\n"
    "    ax.plot(az_grid, db_norm(P_c), 'darkorange', lw=1.2, ls='--', label=f'cal {az_c:+.1f}°')\n"
    "    ax.axvline(0, color='k', ls=':', lw=0.8, label='true 0°')\n"
    "    ax.set_xlabel('Azimuth (deg)'); ax.set_ylabel('dB')\n"
    "    ax.set_title(label); ax.set_ylim(-30, 2)\n"
    "    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'Boresight DoA before/after calibration at {f_ref:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_cal_doa.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print(f'Boresight DoA (true: 0°):')\n"
    "print(f'  D&S  uncal: {az_ds_u:+.1f}°   cal: {az_ds_c:+.1f}°  '\n"
    "      f'  improvement: {abs(az_ds_u)-abs(az_ds_c):+.1f}°')\n"
    "print(f'  MVDR uncal: {az_mv_u:+.1f}°   cal: {az_mv_c:+.1f}°  '\n"
    "      f'  improvement: {abs(az_mv_u)-abs(az_mv_c):+.1f}°')\n",
    "ab170061",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n"
    "\n"
    "Key results to record in PHASE3.md:\n"
    "- Per-mic delay spread and gain spread\n"
    "- CSM phase change after calibration (off-diagonal phases → near 0°)\n"
    "- DoA error before and after on boresight recording\n"
    "- Calibration vector saved to `test/UMA16/cal.npy`\n",
    "ab170070",
))

cells.append(code(
    "print('=== nb17 Summary ===')\n"
    "print(f'Cal frequency: {CAL_FREQ:.0f} Hz   ({n_blocks} Welch blocks)')\n"
    "print(f'Delay spread: {delays.min()*1e6:.1f} – {delays.max()*1e6:.1f} µs')\n"
    "print(f'Gain spread:  {20*np.log10(gains.min()):.2f} – {20*np.log10(gains.max()):.2f} dB '\n"
    "      f'({20*np.log10(gains.max()/gains.min()):.2f} dB range)')\n"
    "print(f'Boresight DoA improvement:')\n"
    "print(f'  D&S:  {abs(az_ds_u):.1f}° → {abs(az_ds_c):.1f}°')\n"
    "print(f'  MVDR: {abs(az_mv_u):.1f}° → {abs(az_mv_c):.1f}°')\n"
    "print(f'Cal saved: {CAL_PATH}')\n",
    "ab170071",
))

# ---------------------------------------------------------------------------
# Write notebook
# ---------------------------------------------------------------------------
nb = {
    "nbformat": 4, "nbformat_minor": 5,
    "metadata": {
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "language_info": {"name": "python", "version": "3.12.0"},
    },
    "cells": cells,
}

out = Path(__file__).parent / "17_uma16_calibration.ipynb"
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
