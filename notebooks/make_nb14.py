#!/usr/bin/env python3
"""Generate notebooks/14_respeaker_calibration.ipynb"""
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
    "# 14 — ReSpeaker Calibration\n"
    "\n"
    "**Goal**: measure and correct per-mic gain and phase offsets using a\n"
    "cross-correlation approach on a reference recording.\n"
    "\n"
    "Steps:\n"
    "1. Record a reference clip (boresight source — laptop speaker playing 1 kHz tone)\n"
    "2. Estimate per-mic delay via cross-correlation with mic 0\n"
    "3. Estimate per-mic gain from RMS ratios\n"
    "4. Build calibration vector `e = g * exp(j*phi)` and apply to CSM\n"
    "5. Quantify DoA error before vs after on a test recording\n"
    "6. Save correction vector to `test/ReSpeaker/cal.npy`\n",
    "cd140001",
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
    "C          = 343.0\n"
    "RADIUS     = 0.045\n"
    "FS         = 16000\n"
    "N_MICS     = 4\n"
    "MIC_SLICE  = slice(2, 6)   # ch2-5 = Mic 0-3 raw\n"
    "N_CHANNELS = 6\n"
    "CAL_FREQ   = 1000.0        # calibration tone frequency (Hz)\n"
    "CAL_SECS   = 5             # reference recording duration\n"
    "TEST_SECS  = 3             # off-axis test recording duration\n"
    "\n"
    "angles_deg = np.array([0.0, 90.0, 180.0, 270.0])\n"
    "x = RADIUS * np.cos(np.radians(angles_deg))\n"
    "y = RADIUS * np.sin(np.radians(angles_deg))\n"
    "\n"
    "CAL_PATH = Path('../test/ReSpeaker/cal.npy')\n"
    "REF_WAV  = Path('../test/ReSpeaker/cal_ref.wav')\n"
    "TEST_WAV = Path('../test/ReSpeaker/cal_test.wav')\n",
    "cd140002",
))

# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — Device Discovery\n",
    "cd140003",
))

cells.append(code(
    "devs = sd.query_devices()\n"
    "rs_idx = None\n"
    "for i, d in enumerate(devs):\n"
    "    if 'respeaker' in d['name'].lower() or 'xvf' in d['name'].lower():\n"
    "        rs_idx = i\n"
    "        rs_dev = d\n"
    "        break\n"
    "\n"
    "if rs_idx is None:\n"
    "    raise RuntimeError('ReSpeaker not found — check USB connection')\n"
    "\n"
    "print(f'Device [{rs_idx}]: {rs_dev[\"name\"]}')\n"
    "print(f'  {rs_dev[\"max_input_channels\"]} ch  {rs_dev[\"default_samplerate\"]:.0f} Hz  '\n"
    "      f'{rs_dev[\"default_low_input_latency\"]*1000:.1f} ms latency')\n",
    "cd140004",
))

# ---------------------------------------------------------------------------
# Section 2: Reference recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Reference Recording\n"
    "\n"
    "Play a 1 kHz tone from directly in front of the array (boresight, 0°) and\n"
    "record 5 seconds.  A laptop speaker or phone at roughly 0.5–1 m works fine.\n"
    "\n"
    "**Before running this cell**: position a tone source straight ahead of the\n"
    "ReSpeaker (boresight) and start the tone.  The recording starts immediately.\n",
    "cd140005",
))

cells.append(code(
    "print(f'Recording {CAL_SECS}s reference clip ... ', end='', flush=True)\n"
    "ref_rec = sd.rec(int(CAL_SECS * FS), samplerate=FS,\n"
    "                 channels=N_CHANNELS, dtype='float32', device=rs_idx)\n"
    "sd.wait()\n"
    "print('done')\n"
    "\n"
    "REF_WAV.parent.mkdir(parents=True, exist_ok=True)\n"
    "wavfile.write(REF_WAV, FS, (ref_rec * 32767).astype(np.int16))\n"
    "print(f'Saved {REF_WAV}')\n"
    "\n"
    "ref_mics = ref_rec[:, MIC_SLICE]   # shape (n_samp, 4)\n"
    "print('RMS per mic:')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  mic{i}: {np.sqrt(np.mean(ref_mics[:, i]**2)):.6f}')\n",
    "cd140006",
))

# ---------------------------------------------------------------------------
# Section 3: Delay estimation via cross-correlation
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Delay & Gain Estimation\n"
    "\n"
    "Cross-correlate each mic with mic 0 in the frequency domain to find the\n"
    "fractional-sample delay.  At boresight the true delays are all zero, so any\n"
    "measured delay is a hardware/cable offset to be corrected.\n",
    "cd140007",
))

cells.append(code(
    "# Frequency-domain cross-correlation for sub-sample delay\n"
    "def estimate_delay(ref_ch, mic_ch, fs):\n"
    "    \"\"\"Return delay in seconds: positive means mic_ch lags ref_ch.\"\"\"\n"
    "    n = len(ref_ch)\n"
    "    F0 = np.fft.rfft(ref_ch, n=n)\n"
    "    Fn = np.fft.rfft(mic_ch, n=n)\n"
    "    xcorr = np.fft.irfft(np.conj(F0) * Fn, n=n)\n"
    "    lag   = np.argmax(np.abs(xcorr))\n"
    "    if lag > n // 2:\n"
    "        lag -= n\n"
    "    return lag / fs\n"
    "\n"
    "\n"
    "delays = np.zeros(N_MICS)\n"
    "for i in range(1, N_MICS):\n"
    "    delays[i] = estimate_delay(ref_mics[:, 0], ref_mics[:, i], FS)\n"
    "\n"
    "print('Estimated delays relative to mic 0 (should be ~0 at boresight):')\n"
    "for i, d in enumerate(delays):\n"
    "    print(f'  mic{i}: {d*1e6:+.1f} µs  ({d*FS:+.3f} samples)')\n"
    "\n"
    "# Gain ratios\n"
    "rms = np.array([np.sqrt(np.mean(ref_mics[:, i]**2)) for i in range(N_MICS)])\n"
    "gains = rms / rms[0]\n"
    "print('\\nGain ratios (relative to mic 0):')\n"
    "for i, g in enumerate(gains):\n"
    "    print(f'  mic{i}: {g:.4f}  ({20*np.log10(g):+.2f} dB)')\n",
    "cd140008",
))

# ---------------------------------------------------------------------------
# Section 4: Build and apply calibration vector
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Calibration Vector\n"
    "\n"
    "The calibration vector `e` encodes the gain and phase offset of each mic\n"
    "relative to mic 0.  Applying `R_cal = outer(1/e, conj(1/e)) * R_uncal`\n"
    "removes the offsets from the cross-spectral matrix before beamforming.\n",
    "cd140009",
))

cells.append(code(
    "# Phase at calibration frequency\n"
    "phi = 2 * np.pi * CAL_FREQ * delays   # radians\n"
    "e   = gains * np.exp(1j * phi)         # complex calibration vector, shape (N_MICS,)\n"
    "\n"
    "print('Calibration vector e = g * exp(j*phi):')\n"
    "for i in range(N_MICS):\n"
    "    print(f'  mic{i}: |e|={np.abs(e[i]):.4f}  '\n"
    "          f'phi={np.degrees(np.angle(e[i])):+.2f}°')\n"
    "\n"
    "\n"
    "def apply_cal(R, e):\n"
    "    \"\"\"Apply calibration: R_cal = outer(1/e, conj(1/e)) * R\"\"\"\n"
    "    c = 1.0 / e\n"
    "    return np.outer(c, c.conj()) * R\n"
    "\n"
    "\n"
    "CAL_PATH.parent.mkdir(parents=True, exist_ok=True)\n"
    "np.save(CAL_PATH, e)\n"
    "print(f'\\nSaved calibration vector to {CAL_PATH}')\n",
    "cd140010",
))

# ---------------------------------------------------------------------------
# Section 5: Visualise calibration effect on reference CSM
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5 — Effect on Reference CSM\n"
    "\n"
    "Apply the calibration to the reference recording's CSM.  After correction\n"
    "the off-diagonal phases should be near zero (all mics in phase for a\n"
    "boresight source).\n",
    "cd140011",
))

cells.append(code(
    "def compute_csm(audio, fs, freq, block_size=256, hop=128):\n"
    "    n_samp, n_ch = audio.shape\n"
    "    freqs   = np.fft.rfftfreq(block_size, 1/fs)\n"
    "    f_idx   = np.argmin(np.abs(freqs - freq))\n"
    "    f_actual = freqs[f_idx]\n"
    "    R = np.zeros((n_ch, n_ch), dtype=complex)\n"
    "    count = 0\n"
    "    win = np.hanning(block_size)\n"
    "    for start in range(0, n_samp - block_size, hop):\n"
    "        block = audio[start:start + block_size] * win[:, np.newaxis]\n"
    "        F = np.fft.rfft(block, axis=0)[f_idx]\n"
    "        R    += np.outer(F, F.conj())\n"
    "        count += 1\n"
    "    return R / count, f_actual, count\n"
    "\n"
    "\n"
    "R_ref, f_ref, _ = compute_csm(ref_mics, FS, CAL_FREQ)\n"
    "R_cal_ref = apply_cal(R_ref, e)\n"
    "\n"
    "fig, axes = plt.subplots(2, 2, figsize=(10, 8))\n"
    "\n"
    "for row, (R, title) in enumerate([(R_ref, 'Uncalibrated'), (R_cal_ref, 'Calibrated')]):\n"
    "    ax0 = axes[row, 0]\n"
    "    ax1 = axes[row, 1]\n"
    "    im0 = ax0.imshow(np.abs(R), cmap='inferno', aspect='auto')\n"
    "    ax0.set_title(f'{title} |CSM| @ {f_ref:.0f} Hz')\n"
    "    ax0.set_xticks(range(4)); ax0.set_yticks(range(4))\n"
    "    plt.colorbar(im0, ax=ax0)\n"
    "    im1 = ax1.imshow(np.degrees(np.angle(R)), cmap='hsv', aspect='auto',\n"
    "                     vmin=-180, vmax=180)\n"
    "    ax1.set_title(f'{title} phase(CSM) (deg)')\n"
    "    ax1.set_xticks(range(4)); ax1.set_yticks(range(4))\n"
    "    plt.colorbar(im1, ax=ax1, label='degrees')\n"
    "\n"
    "plt.suptitle(f'CSM before and after calibration at {f_ref:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_cal_csm.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('Off-diagonal phase BEFORE calibration (degrees):')\n"
    "for i in range(N_MICS):\n"
    "    for j in range(i+1, N_MICS):\n"
    "        print(f'  mic{i}-mic{j}: {np.degrees(np.angle(R_ref[i,j])):+.1f}°')\n"
    "\n"
    "print('\\nOff-diagonal phase AFTER calibration (should be ~0°):')\n"
    "for i in range(N_MICS):\n"
    "    for j in range(i+1, N_MICS):\n"
    "        print(f'  mic{i}-mic{j}: {np.degrees(np.angle(R_cal_ref[i,j])):+.1f}°')\n",
    "cd140012",
))

# ---------------------------------------------------------------------------
# Section 6: DoA before vs after — on the reference recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6 — DoA Before vs After (Reference Recording)\n"
    "\n"
    "Beamform the reference (boresight) recording before and after calibration.\n"
    "Expected: calibrated peak should be closer to 0°.\n",
    "cd140013",
))

cells.append(code(
    "def sm(x, y, az_grid, freq):\n"
    "    u = np.sin(np.radians(az_grid))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * np.outer(x, u)) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def beamform_ds(x, y, R, freq, az_grid):\n"
    "    H = sm(x, y, az_grid, freq)\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_mvdr(x, y, R, freq, az_grid, diag_load=0.01):\n"
    "    N  = R.shape[0]\n"
    "    Rl = R + diag_load * np.trace(R) / N * np.eye(N)\n"
    "    Ri = inv(Rl)\n"
    "    H  = sm(x, y, az_grid, freq)\n"
    "    d  = np.real(np.sum(H.conj() * (Ri @ H), axis=0))\n"
    "    return 1.0 / np.maximum(d, 1e-300)\n"
    "\n"
    "\n"
    "def db_norm(p):\n"
    "    return 10 * np.log10(np.maximum(p / p.max(), 1e-10))\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "\n"
    "P_ds_uncal  = beamform_ds(x, y, R_ref, f_ref, az_grid)\n"
    "P_ds_cal    = beamform_ds(x, y, R_cal_ref, f_ref, az_grid)\n"
    "P_mv_uncal  = beamform_mvdr(x, y, R_ref, f_ref, az_grid)\n"
    "P_mv_cal    = beamform_mvdr(x, y, R_cal_ref, f_ref, az_grid)\n"
    "\n"
    "az_ds_uncal = az_grid[np.argmax(P_ds_uncal)]\n"
    "az_ds_cal   = az_grid[np.argmax(P_ds_cal)]\n"
    "az_mv_uncal = az_grid[np.argmax(P_mv_uncal)]\n"
    "az_mv_cal   = az_grid[np.argmax(P_mv_cal)]\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)\n"
    "for ax, (label, P_u, P_c, az_u, az_c) in zip(axes, [\n"
    "    ('D&S',  P_ds_uncal, P_ds_cal, az_ds_uncal, az_ds_cal),\n"
    "    ('MVDR', P_mv_uncal, P_mv_cal, az_mv_uncal, az_mv_cal),\n"
    "]):\n"
    "    ax.plot(az_grid, db_norm(P_u), 'steelblue', linewidth=1, label=f'uncal peak={az_u:.1f}°')\n"
    "    ax.plot(az_grid, db_norm(P_c), 'darkorange', linewidth=1, linestyle='--',\n"
    "            label=f'cal peak={az_c:.1f}°')\n"
    "    ax.axvline(0, color='k', linestyle=':', linewidth=0.8, label='true 0°')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('dB')\n"
    "    ax.set_title(label)\n"
    "    ax.set_ylim(-30, 2)\n"
    "    ax.legend(fontsize=8)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'Reference (boresight) beamformer — before/after calibration at {f_ref:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_cal_doa_ref.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print(f'Reference recording DoA estimates (true: 0°):')\n"
    "print(f'  D&S  uncal: {az_ds_uncal:+.1f}°   cal: {az_ds_cal:+.1f}°  '\n"
    "      f'  improvement: {abs(az_ds_uncal)-abs(az_ds_cal):+.1f}°')\n"
    "print(f'  MVDR uncal: {az_mv_uncal:+.1f}°   cal: {az_mv_cal:+.1f}°  '\n"
    "      f'  improvement: {abs(az_mv_uncal)-abs(az_mv_cal):+.1f}°')\n",
    "cd140014",
))

# ---------------------------------------------------------------------------
# Section 7: Off-axis test recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 7 — Off-Axis Test Recording\n"
    "\n"
    "Record a second clip with the source at a known off-axis angle to measure\n"
    "calibration benefit in a more demanding scenario.  Aim for ~30–45° off\n"
    "boresight for a clear test.\n"
    "\n"
    "**Before running**: move the tone source to a clearly off-axis position\n"
    "(estimate the angle visually to within ±5°).  Note the angle in the\n"
    "`TRUE_ANGLE` variable below before executing.\n",
    "cd140015",
))

cells.append(code(
    "TRUE_ANGLE = 30.0   # <-- update this to match your test source angle\n"
    "\n"
    "print(f'Recording {TEST_SECS}s off-axis clip (source at ~{TRUE_ANGLE}°) ... ',\n"
    "      end='', flush=True)\n"
    "test_rec = sd.rec(int(TEST_SECS * FS), samplerate=FS,\n"
    "                  channels=N_CHANNELS, dtype='float32', device=rs_idx)\n"
    "sd.wait()\n"
    "print('done')\n"
    "\n"
    "TEST_WAV.parent.mkdir(parents=True, exist_ok=True)\n"
    "wavfile.write(TEST_WAV, FS, (test_rec * 32767).astype(np.int16))\n"
    "\n"
    "test_mics = test_rec[:, MIC_SLICE]\n"
    "R_test, f_test, _ = compute_csm(test_mics, FS, CAL_FREQ)\n"
    "R_cal_test = apply_cal(R_test, e)\n"
    "\n"
    "P_ds_u  = beamform_ds(x, y, R_test, f_test, az_grid)\n"
    "P_ds_c  = beamform_ds(x, y, R_cal_test, f_test, az_grid)\n"
    "P_mv_u  = beamform_mvdr(x, y, R_test, f_test, az_grid)\n"
    "P_mv_c  = beamform_mvdr(x, y, R_cal_test, f_test, az_grid)\n"
    "\n"
    "az_ds_u  = az_grid[np.argmax(P_ds_u)]\n"
    "az_ds_c  = az_grid[np.argmax(P_ds_c)]\n"
    "az_mv_u  = az_grid[np.argmax(P_mv_u)]\n"
    "az_mv_c  = az_grid[np.argmax(P_mv_c)]\n"
    "\n"
    "print(f'\\nOff-axis test (true angle: {TRUE_ANGLE}°):')\n"
    "print(f'  D&S  uncal: {az_ds_u:+.1f}°  error={abs(az_ds_u-TRUE_ANGLE):.1f}°')\n"
    "print(f'  D&S  cal:   {az_ds_c:+.1f}°  error={abs(az_ds_c-TRUE_ANGLE):.1f}°')\n"
    "print(f'  MVDR uncal: {az_mv_u:+.1f}°  error={abs(az_mv_u-TRUE_ANGLE):.1f}°')\n"
    "print(f'  MVDR cal:   {az_mv_c:+.1f}°  error={abs(az_mv_c-TRUE_ANGLE):.1f}°')\n",
    "cd140016",
))

cells.append(code(
    "fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)\n"
    "for ax, (label, P_u, P_c, az_u, az_c) in zip(axes, [\n"
    "    ('D&S',  P_ds_u, P_ds_c, az_ds_u, az_ds_c),\n"
    "    ('MVDR', P_mv_u, P_mv_c, az_mv_u, az_mv_c),\n"
    "]):\n"
    "    ax.plot(az_grid, db_norm(P_u), 'steelblue', linewidth=1,\n"
    "            label=f'uncal peak={az_u:.1f}°')\n"
    "    ax.plot(az_grid, db_norm(P_c), 'darkorange', linewidth=1, linestyle='--',\n"
    "            label=f'cal peak={az_c:.1f}°')\n"
    "    ax.axvline(TRUE_ANGLE, color='k', linestyle=':', linewidth=0.8,\n"
    "               label=f'true {TRUE_ANGLE}°')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('dB')\n"
    "    ax.set_title(label)\n"
    "    ax.set_ylim(-30, 2)\n"
    "    ax.legend(fontsize=8)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'Off-axis test beamformer — before/after calibration at {f_test:.0f} Hz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_cal_doa_test.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "cd140017",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n"
    "\n"
    "Key findings to document in PHASE2.md:\n"
    "- Per-mic delays and gains\n"
    "- CSM phase change after calibration\n"
    "- DoA error before and after on boresight and off-axis sources\n"
    "- Whether mic1 (lowest power in nb13) shows a hardware defect or normal variation\n",
    "cd140018",
))

cells.append(code(
    "print('=== nb14 Summary ===')\n"
    "print(f'Calibration frequency: {CAL_FREQ:.0f} Hz')\n"
    "print(f'\\nPer-mic delays (relative to mic0):')\n"
    "for i, d in enumerate(delays):\n"
    "    print(f'  mic{i}: {d*1e6:+.1f} µs')\n"
    "print(f'\\nPer-mic gains (relative to mic0):')\n"
    "for i, g in enumerate(gains):\n"
    "    print(f'  mic{i}: {g:.4f} ({20*np.log10(g):+.2f} dB)')\n"
    "print(f'\\nCalibration saved to: {CAL_PATH}')\n"
    "print(f'\\nDoA improvement on reference (boresight, true=0°):')\n"
    "print(f'  D&S:  {abs(az_ds_uncal):.1f}° → {abs(az_ds_cal):.1f}°')\n"
    "print(f'  MVDR: {abs(az_mv_uncal):.1f}° → {abs(az_mv_cal):.1f}°')\n",
    "cd140019",
))

# ---------------------------------------------------------------------------
# Write notebook
# ---------------------------------------------------------------------------
nb = {
    "nbformat": 4,
    "nbformat_minor": 5,
    "metadata": {
        "kernelspec": {
            "display_name": "ACOUSTIC_CAMERA",
            "language": "python",
            "name": "acoustic_camera",
        },
        "language_info": {"name": "python", "version": "3.11"},
    },
    "cells": cells,
}

out = Path("14_respeaker_calibration.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
