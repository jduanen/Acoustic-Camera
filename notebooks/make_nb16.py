#!/usr/bin/env python3
"""Generate notebooks/16_uma16_capture.ipynb"""
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
    "# 16 — UMA-16 v2 Audio Capture & Offline Beamforming\n"
    "\n"
    "**Goal**: validate the full audio capture → CSM → 2D beamforming pipeline\n"
    "on real data from the miniDSP UMA-16 v2 16-mic array.\n"
    "\n"
    "Steps:\n"
    "1. Device discovery and channel verification (all 16 channels are raw mic audio)\n"
    "2. Record a 5-second clip with a known-direction source\n"
    "3. Compute the Cross-Spectral Matrix (CSM) from all 16 channels\n"
    "4. 1D azimuth beamforming — compare peaks to nb15 HPBW predictions\n"
    "5. 2D (azimuth × elevation) beamforming — full heatmap\n"
    "6. Frequency sweep — DoA consistency vs spatial Nyquist\n",
    "ab160001",
))

# ---------------------------------------------------------------------------
# Imports + constants
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import pandas as pd\n"
    "import sounddevice as sd\n"
    "import scipy.io.wavfile as wavfile\n"
    "import scipy.signal as signal\n"
    "from scipy.linalg import inv\n"
    "from pathlib import Path\n"
    "\n"
    "plt.rcParams['figure.dpi'] = 120\n"
    "plt.rcParams['font.size'] = 10\n"
    "\n"
    "C       = 343.0\n"
    "FS      = 48000\n"
    "N_MICS  = 16\n"
    "NYQUIST = C / (2 * 0.042)   # ~4083 Hz\n"
    "\n"
    "# UMA-16 v2 mic positions (camera/observer side — x negated vs manual Figure 1)\n"
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
    "WAV_OUT = Path('../test/UMA16/capture_nb16.wav')\n",
    "ab160002",
))

# ---------------------------------------------------------------------------
# Section 1: Device discovery
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1  Device Discovery\n"
    "\n"
    "Locate the UMA-16 v2 in the sounddevice device list and confirm it exposes\n"
    "16 input channels at 48 kHz.  All 16 channels are raw mic audio (no processed\n"
    "channels, unlike the ReSpeaker in Phase 2).",
    "ab160010",
))

cells.append(code(
    "devs = sd.query_devices()\n"
    "print('All input devices:')\n"
    "for i, d in enumerate(devs):\n"
    "    if d['max_input_channels'] > 0:\n"
    "        print(f'  [{i:2d}] {d[\"name\"]:50s}  '\n"
    "              f'{d[\"max_input_channels\"]}ch  '\n"
    "              f'{d[\"default_samplerate\"]:.0f}Hz')\n"
    "\n"
    "uma_idx = None\n"
    "for i, d in enumerate(devs):\n"
    "    if 'uma' in d['name'].lower() or 'uma16' in d['name'].lower():\n"
    "        uma_idx = i; uma_dev = d; break\n"
    "if uma_idx is None:\n"
    "    # fallback: device 12 is the known index from hardware testing\n"
    "    uma_idx = 12; uma_dev = devs[12]\n"
    "\n"
    "print(f'\\nUMA-16 v2 at device index {uma_idx}:')\n"
    "print(f'  Name:     {uma_dev[\"name\"]}')\n"
    "print(f'  Channels: {uma_dev[\"max_input_channels\"]}')\n"
    "print(f'  Rate:     {uma_dev[\"default_samplerate\"]:.0f} Hz')\n"
    "print(f'  Latency:  {uma_dev[\"default_low_input_latency\"]*1000:.1f} ms (low)')\n",
    "ab160011",
))

# ---------------------------------------------------------------------------
# Section 2: Channel RMS check
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2  Channel Verification\n"
    "\n"
    "Record 2 seconds of ambient audio and measure RMS level on each of the 16 channels.\n"
    "All channels should be similar (within ~2–3×) confirming all mics are live and\n"
    "the array is functioning correctly.",
    "ab160020",
))

cells.append(code(
    "print('Recording 2s ambient (keep quiet) ...')\n"
    "amb = sd.rec(2 * FS, samplerate=FS, channels=N_MICS, dtype='float32', device=uma_idx)\n"
    "sd.wait()\n"
    "print('Done.')\n"
    "\n"
    "rms_vals = np.sqrt(np.mean(amb**2, axis=0))\n"
    "print('\\nAmbient RMS per channel:')\n"
    "for i, r in enumerate(rms_vals):\n"
    "    bar = '#' * int(r / rms_vals.max() * 30)\n"
    "    print(f'  ch{i:2d}: {r:.2e}  {bar}')\n"
    "print(f'\\nMax/min ratio: {rms_vals.max()/rms_vals.min():.2f}x')\n",
    "ab160021",
))

# ---------------------------------------------------------------------------
# Section 3: Live recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3  Live Recording\n"
    "\n"
    "Record 5 seconds.  Place a sound source (tone, clap, speaker) at a known azimuth\n"
    "angle if possible — ideally 0° (boresight) or a clearly off-axis direction.\n"
    "The recording is saved to `test/UMA16/capture_nb16.wav` for offline replay.",
    "ab160030",
))

cells.append(code(
    "RECORD_SECS = 5\n"
    "print(f'Recording {RECORD_SECS}s — make a sound now ...')\n"
    "rec = sd.rec(RECORD_SECS * FS, samplerate=FS, channels=N_MICS,\n"
    "             dtype='float32', device=uma_idx)\n"
    "sd.wait()\n"
    "print('Done.')\n"
    "\n"
    "WAV_OUT.parent.mkdir(parents=True, exist_ok=True)\n"
    "wavfile.write(WAV_OUT, FS, (rec * 32767).astype(np.int16))\n"
    "print(f'Saved {WAV_OUT}  ({rec.shape[0]} samples, {N_MICS} channels)')\n"
    "\n"
    "t = np.arange(rec.shape[0]) / FS\n"
    "fig, axes = plt.subplots(4, 4, figsize=(14, 8), sharex=True, sharey=True)\n"
    "for i, ax in enumerate(axes.ravel()):\n"
    "    ax.plot(t, rec[:, i], linewidth=0.3, color='steelblue')\n"
    "    ax.set_title(f'ch{i}', fontsize=8)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "axes[-1][0].set_xlabel('Time (s)')\n"
    "plt.suptitle('UMA-16 v2 — 16-channel waveforms')\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_capture_waveforms.png', dpi=150)\n"
    "plt.show()\n",
    "ab160031",
))

# ---------------------------------------------------------------------------
# Section 4: CSM computation
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4  Cross-Spectral Matrix\n"
    "\n"
    "Compute the CSM from the 16-channel recording using Welch-style block averaging.\n"
    "Beamforming frequency: 3000 Hz — well below the ~4083 Hz spatial Nyquist and\n"
    "within the useful directionality window (>2.7 kHz for this 126 mm aperture).",
    "ab160040",
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
    "FREQ_BF = 3000.0\n"
    "R_csm, f_act, n_blocks = compute_csm(rec, FREQ_BF)\n"
    "print(f'CSM at {f_act:.1f} Hz,  {n_blocks} blocks')\n"
    "print(f'CSM diagonal (power per channel):')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  ch{i:2d}: {R_csm[i,i].real:.3e}')\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(10, 4))\n"
    "im0 = axes[0].imshow(np.abs(R_csm), cmap='inferno', aspect='auto')\n"
    "axes[0].set_title(f'|CSM| at {f_act:.0f} Hz')\n"
    "plt.colorbar(im0, ax=axes[0])\n"
    "im1 = axes[1].imshow(np.angle(R_csm), cmap='hsv', aspect='auto',\n"
    "                     vmin=-np.pi, vmax=np.pi)\n"
    "axes[1].set_title(f'angle(CSM) at {f_act:.0f} Hz')\n"
    "plt.colorbar(im1, ax=axes[1], label='rad')\n"
    "for ax in axes:\n"
    "    ax.set_xlabel('channel'); ax.set_ylabel('channel')\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_csm.png', dpi=150)\n"
    "plt.show()\n",
    "ab160041",
))

# ---------------------------------------------------------------------------
# Section 5: 1D azimuth beamforming
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5  1D Azimuth Beamforming\n"
    "\n"
    "Beamform along the azimuth axis (elevation fixed at 0°) with D&S, MVDR, and\n"
    "CLEAN-SC.  Expected HPBW from nb15 at 3000 Hz: ~43°.",
    "ab160050",
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
    "def bf_clean(R, H, n_iter=20, loop_gain=0.5):\n"
    "    R_w = R.copy()\n"
    "    out = np.zeros(H.shape[1])\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        h = H[:, k]\n"
    "        p = np.real(h.conj() @ (R_w @ h))\n"
    "        R_w -= loop_gain * p * np.outer(h, h.conj())\n"
    "        out[k] += loop_gain * p\n"
    "    return out\n"
    "\n"
    "\n"
    "def db_norm(p):\n"
    "    return 10 * np.log10(np.maximum(p / p.max(), 1e-10))\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "el_fix  = np.array([0.0])\n"
    "\n"
    "H1d = steer(az_grid, el_fix, f_act)\n"
    "P_ds    = bf_ds(R_csm, H1d)\n"
    "P_mvdr  = bf_mvdr(R_csm, H1d)\n"
    "P_clean = bf_clean(R_csm, H1d)\n"
    "\n"
    "az_ds    = az_grid[np.argmax(P_ds)]\n"
    "az_mvdr  = az_grid[np.argmax(P_mvdr)]\n"
    "az_clean = az_grid[np.argmax(P_clean)]\n"
    "\n"
    "fig, axes = plt.subplots(1, 3, figsize=(15, 4))\n"
    "colors = {'D&S': 'steelblue', 'MVDR': 'darkorange', 'CLEAN-SC': 'forestgreen'}\n"
    "for ax, (name, P, az_e) in zip(axes, [\n"
    "    ('D&S',      P_ds,    az_ds),\n"
    "    ('MVDR',     P_mvdr,  az_mvdr),\n"
    "    ('CLEAN-SC', P_clean, az_clean),\n"
    "]):\n"
    "    ax.plot(az_grid, db_norm(P), color=colors[name], lw=1.2)\n"
    "    ax.axvline(az_e, color=colors[name], ls='--', lw=1, label=f'peak={az_e:.1f}°')\n"
    "    ax.set_xlim(-90, 90); ax.set_ylim(-30, 2)\n"
    "    ax.set_title(f'{name}  (peak @ {az_e:.1f}°)')\n"
    "    ax.set_xlabel('Azimuth (deg)'); ax.set_ylabel('dB')\n"
    "    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'1D azimuth beamforming at {f_act:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_beamform_1d.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print(f'Peak DoA at {f_act:.0f} Hz:')\n"
    "print(f'  D&S:      {az_ds:.1f}°')\n"
    "print(f'  MVDR:     {az_mvdr:.1f}°')\n"
    "print(f'  CLEAN-SC: {az_clean:.1f}°')\n"
    "print(f'  nb15 predicted HPBW @ 3000 Hz: ~43°')\n",
    "ab160051",
))

# ---------------------------------------------------------------------------
# Section 6: 2D beamforming
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6  2D Beamforming (Azimuth × Elevation)\n"
    "\n"
    "Full 2D energy map.  The 4×4 URA has equal aperture in az and el so the beam\n"
    "pattern is symmetric.  Peak should appear at the same azimuth as Section 5.",
    "ab160060",
))

cells.append(code(
    "N_AZ, N_EL = 91, 61\n"
    "az_2d = np.linspace(-45, 45, N_AZ)\n"
    "el_2d = np.linspace(-30, 30, N_EL)\n"
    "\n"
    "H2d  = steer(az_2d, el_2d, f_act)      # (16, N_AZ * N_EL)\n"
    "P2d  = bf_ds(R_csm, H2d).reshape(N_AZ, N_EL)\n"
    "P2d_db = db_norm(P2d)\n"
    "\n"
    "peak_flat = np.unravel_index(np.argmax(P2d), P2d.shape)\n"
    "peak_az2d = az_2d[peak_flat[0]]\n"
    "peak_el2d = el_2d[peak_flat[1]]\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(8, 5))\n"
    "im = ax.imshow(P2d_db.T, origin='lower', aspect='auto',\n"
    "               extent=[az_2d[0], az_2d[-1], el_2d[0], el_2d[-1]],\n"
    "               cmap='jet', vmin=-20, vmax=0)\n"
    "ax.plot(peak_az2d, peak_el2d, 'w+', ms=14, mew=2, label=f'peak ({peak_az2d:.1f}°, {peak_el2d:.1f}°)')\n"
    "plt.colorbar(im, ax=ax, label='dB (normalized)')\n"
    "ax.set_xlabel('Azimuth (deg)')\n"
    "ax.set_ylabel('Elevation (deg)')\n"
    "ax.set_title(f'2D D&S energy map at {f_act:.0f} Hz')\n"
    "ax.legend(fontsize=9)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_beamform_2d.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print(f'2D peak: az={peak_az2d:.1f}°, el={peak_el2d:.1f}°')\n",
    "ab160061",
))

# ---------------------------------------------------------------------------
# Section 7: Frequency sweep
# ---------------------------------------------------------------------------
cells.append(md(
    "## 7  Peak DoA vs Frequency\n"
    "\n"
    "Sweep from 500 Hz to 5 kHz and record the D&S peak azimuth at each frequency.\n"
    "Below ~2700 Hz (aperture-limited) and above ~4083 Hz (spatial Nyquist) the\n"
    "estimate should become unstable.",
    "ab160070",
))

cells.append(code(
    "freqs_sweep = [500, 750, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000]\n"
    "peak_az_freq = []\n"
    "\n"
    "for freq in freqs_sweep:\n"
    "    R_f, f_a, _ = compute_csm(rec, freq)\n"
    "    H_f = steer(az_grid, el_fix, f_a)\n"
    "    P_f = bf_ds(R_f, H_f)\n"
    "    peak_az_freq.append(az_grid[np.argmax(P_f)])\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 4))\n"
    "ax.plot(freqs_sweep, peak_az_freq, 'o-', color='steelblue', lw=1.5)\n"
    "ax.axvline(NYQUIST, color='red', ls='--', lw=1, label=f'Spatial Nyquist {NYQUIST:.0f} Hz')\n"
    "ax.axvline(C / 0.126, color='gray', ls=':', lw=1, label=f'Aperture limit ~{C/0.126:.0f} Hz')\n"
    "ax.set_xlabel('Frequency (Hz)')\n"
    "ax.set_ylabel('Estimated DoA (deg)')\n"
    "ax.set_title('D&S peak azimuth vs frequency')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_freq_sweep.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print('Peak DoA by frequency:')\n"
    "for freq, az in zip(freqs_sweep, peak_az_freq):\n"
    "    flag = '  <- above Nyquist' if freq > NYQUIST else ''\n"
    "    print(f'  {freq:5d} Hz:  {az:+6.1f}°{flag}')\n",
    "ab160071",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n"
    "\n"
    "Key findings to record in PHASE3.md nb16 Results:\n"
    "- Device accessible, all 16 channels live, RMS levels balanced\n"
    "- CSM computed correctly; off-diagonal structure visible (inter-mic correlations)\n"
    "- 1D peak DoA from D&S, MVDR, CLEAN-SC (record values)\n"
    "- 2D peak az/el coordinates\n"
    "- Frequency sweep: stable DoA in 2–4 kHz window, instability above Nyquist\n",
    "ab160080",
))

cells.append(code(
    "print('=== nb16 Summary ===')\n"
    "print(f'Device: [{uma_idx}] {uma_dev[\"name\"]}')\n"
    "print(f'Channels: {uma_dev[\"max_input_channels\"]},  Rate: {FS} Hz')\n"
    "print(f'RMS spread: {rms_vals.max()/rms_vals.min():.2f}x  '\n"
    "      f'(max ch{rms_vals.argmax()}, min ch{rms_vals.argmin()})')\n"
    "print(f'CSM blocks: {n_blocks}  at {f_act:.0f} Hz')\n"
    "print(f'1D peaks at {f_act:.0f} Hz:')\n"
    "print(f'  D&S={az_ds:.1f}°  MVDR={az_mvdr:.1f}°  CLEAN-SC={az_clean:.1f}°')\n"
    "print(f'2D peak: az={peak_az2d:.1f}°  el={peak_el2d:.1f}°')\n"
    "if abs(az_ds - az_mvdr) < 5 and abs(az_ds - az_clean) < 5:\n"
    "    print('PASS: algorithms agree within 5°')\n"
    "else:\n"
    "    print('NOTE: algorithm estimates differ >5° — ambient or multi-source scene')\n",
    "ab160081",
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

out = Path(__file__).parent / "16_uma16_capture.ipynb"
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
