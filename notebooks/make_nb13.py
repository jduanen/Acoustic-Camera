#!/usr/bin/env python3
"""Generate notebooks/13_respeaker_capture.ipynb"""
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
    "# 13 — Audio Capture & Offline Beamforming\n"
    "\n"
    "**Goal**: validate the full audio capture → CSM → beamforming pipeline\n"
    "on real data from the ReSpeaker XVF3800 4-mic array.\n"
    "\n"
    "Steps:\n"
    "1. Discover the device via sounddevice\n"
    "2. Analyse the channel structure (6 channels: 4 mics + 2 beamformed outputs)\n"
    "3. Record a fresh live clip\n"
    "4. Compute the Cross-Spectral Matrix (CSM) from the 4 mic channels\n"
    "5. Beamform with D&S, MVDR, and CLEAN-SC and plot energy maps\n"
    "6. Cross-check against the device's onboard DoA estimate (if pyusb available)\n",
    "bc130001",
))

# ---------------------------------------------------------------------------
# Imports
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
    "RADIUS  = 0.045        # 90mm aperture / 2\n"
    "FS      = 16000        # ReSpeaker sample rate\n"
    "N_MICS  = 4            # channels 0-3 are mic channels\n"
    "FREQ_BF = 1500.0       # beamforming centre frequency (safely below Nyquist)\n"
    "\n"
    "# Nominal 4-mic circular geometry (will be refined after AEC_MIC_ARRAY_GEO read)\n"
    "angles_deg = np.array([0.0, 90.0, 180.0, 270.0])\n"
    "x = RADIUS * np.cos(np.radians(angles_deg))\n"
    "y = RADIUS * np.sin(np.radians(angles_deg))\n"
    "\n"
    "WAV_EXISTING = Path('../test/ReSpeaker/output.wav')\n"
    "WAV_LIVE     = Path('../test/ReSpeaker/capture_nb13.wav')\n",
    "bc130002",
))

# ---------------------------------------------------------------------------
# Section 1: Device discovery
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — Device Discovery\n"
    "\n"
    "Find the ReSpeaker in the sounddevice device list and confirm it exposes\n"
    "6 input channels at 16 kHz.\n",
    "bc130003",
))

cells.append(code(
    "devs = sd.query_devices()\n"
    "print(f'All input devices:')\n"
    "for i, d in enumerate(devs):\n"
    "    if d['max_input_channels'] > 0:\n"
    "        print(f'  [{i:2d}] {d[\"name\"]:50s}  '\n"
    "              f'{d[\"max_input_channels\"]}ch  '\n"
    "              f'{d[\"default_samplerate\"]:.0f}Hz')\n"
    "\n"
    "# Find ReSpeaker\n"
    "rs_idx = None\n"
    "for i, d in enumerate(devs):\n"
    "    if 'respeaker' in d['name'].lower() or 'xvf' in d['name'].lower():\n"
    "        rs_idx = i\n"
    "        rs_dev = d\n"
    "        break\n"
    "\n"
    "if rs_idx is None:\n"
    "    print('WARNING: ReSpeaker not found — check USB connection')\n"
    "else:\n"
    "    print(f'\\nReSpeaker found at device index {rs_idx}:')\n"
    "    print(f'  Name:     {rs_dev[\"name\"]}')\n"
    "    print(f'  Channels: {rs_dev[\"max_input_channels\"]}')\n"
    "    print(f'  Rate:     {rs_dev[\"default_samplerate\"]:.0f} Hz')\n"
    "    print(f'  Latency:  {rs_dev[\"default_low_input_latency\"]*1000:.1f} ms (low)')\n",
    "bc130004",
))

# ---------------------------------------------------------------------------
# Section 2: Existing WAV analysis
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Channel Analysis (Existing Recording)\n"
    "\n"
    "The file `test/ReSpeaker/output.wav` was captured earlier (6 channels, 16 kHz, 16-bit).\n"
    "Analyse each channel to determine which are mic signals vs processed outputs.\n"
    "\n"
    "Expected channel layout (XVF3800 default):\n"
    "- Channels 0–3: raw or AEC-residual mic signals (one per mic)\n"
    "- Channels 4–5: beamformed / post-processed stereo output\n",
    "bc130005",
))

cells.append(code(
    "fs_wav, data_raw = wavfile.read(WAV_EXISTING)\n"
    "data = data_raw.astype(np.float32) / 32768.0   # normalise to ±1\n"
    "n_samp, n_ch = data.shape\n"
    "duration = n_samp / fs_wav\n"
    "print(f'File: {WAV_EXISTING}')\n"
    "print(f'Channels: {n_ch},  Rate: {fs_wav} Hz,  Duration: {duration:.2f} s')\n"
    "print(f'Max absolute values per channel:')\n"
    "for ch in range(n_ch):\n"
    "    rms = np.sqrt(np.mean(data[:, ch]**2))\n"
    "    pk  = np.abs(data[:, ch]).max()\n"
    "    print(f'  ch{ch}: RMS={rms:.5f}  peak={pk:.5f}')\n",
    "bc130006",
))

cells.append(code(
    "t = np.arange(n_samp) / fs_wav\n"
    "fig, axes = plt.subplots(n_ch, 1, figsize=(14, 8), sharex=True)\n"
    "ch_labels = ['Conference (processed)', 'ASR (processed)',\n"
    "             'Mic0 raw', 'Mic1 raw', 'Mic2 raw', 'Mic3 raw']\n"
    "for ch in range(n_ch):\n"
    "    axes[ch].plot(t, data[:, ch], linewidth=0.4, color='steelblue')\n"
    "    axes[ch].set_ylabel(ch_labels[ch], fontsize=8)\n"
    "    axes[ch].set_ylim(-0.5, 0.5)\n"
    "    axes[ch].grid(True, alpha=0.3)\n"
    "axes[-1].set_xlabel('Time (s)')\n"
    "plt.suptitle('All 6 channels — ReSpeaker output.wav')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_channels_waveform.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "bc130007",
))

cells.append(code(
    "# Spectrogram of all channels to see frequency content\n"
    "fig, axes = plt.subplots(2, 3, figsize=(15, 7))\n"
    "axes = axes.flatten()\n"
    "for ch in range(n_ch):\n"
    "    f_s, t_s, Sxx = signal.spectrogram(data[:, ch], fs=fs_wav,\n"
    "                                        nperseg=512, noverlap=384)\n"
    "    axes[ch].pcolormesh(t_s, f_s, 10*np.log10(Sxx + 1e-10),\n"
    "                        shading='gouraud', cmap='inferno', vmin=-80, vmax=-20)\n"
    "    axes[ch].set_ylim(0, fs_wav/2)\n"
    "    axes[ch].set_title(ch_labels[ch])\n"
    "    axes[ch].set_xlabel('Time (s)')\n"
    "    axes[ch].set_ylabel('Freq (Hz)')\n"
    "plt.suptitle('Spectrograms — 6 channels')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_spectrograms.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "bc130008",
))

cells.append(code(
    "# Cross-correlation between mic pairs to check timing consistency\n"
    "# Mic channels should show small inter-channel delays; processed channels may look different\n"
    "mic_data = data[:, :4]    # channels 0-3\n"
    "ref = mic_data[:, 0]\n"
    "\n"
    "fig, axes = plt.subplots(1, 3, figsize=(14, 3))\n"
    "for i, (ch, ax) in enumerate(zip([1, 2, 3], axes)):\n"
    "    xcorr = np.correlate(mic_data[:, ch], ref, mode='full')\n"
    "    lags  = np.arange(-len(ref) + 1, len(ref)) / fs_wav * 1000  # ms\n"
    "    # zoom to ±2ms\n"
    "    mask = np.abs(lags) <= 2\n"
    "    ax.plot(lags[mask], xcorr[mask], linewidth=0.8, color='steelblue')\n"
    "    peak_lag = lags[np.argmax(xcorr)]\n"
    "    ax.axvline(peak_lag, color='r', linestyle='--', linewidth=0.8)\n"
    "    ax.set_title(f'xcorr(mic{ch}, mic0)  peak lag={peak_lag:.3f} ms')\n"
    "    ax.set_xlabel('Lag (ms)')\n"
    "    ax.grid(True, alpha=0.3)\n"
    "plt.suptitle('Cross-correlation between mic pairs (mic0 as reference)')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_xcorr.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "bc130009",
))

# ---------------------------------------------------------------------------
# Section 3: Live recording
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Live Recording\n"
    "\n"
    "Record 3 seconds from the device.  Make a sound (clap, tone, speech)\n"
    "during recording from a known direction if possible.\n"
    "\n"
    "If recording fails (device busy or not found) the notebook falls back to\n"
    "the existing WAV for the beamforming sections.\n",
    "bc130010",
))

cells.append(code(
    "RECORD_SECS = 3\n"
    "N_CHANNELS  = 6\n"
    "MIC_SLICE   = slice(2, 6)   # ch2-5 = Mic 0-3 raw (ch0=Conference, ch1=ASR)\n"
    "\n"
    "if rs_idx is not None:\n"
    "    try:\n"
    "        print(f'Recording {RECORD_SECS}s from device {rs_idx}  '\n"
    "              f'(\"{rs_dev[\"name\"]}\")  ...', end=' ', flush=True)\n"
    "        rec = sd.rec(int(RECORD_SECS * FS), samplerate=FS,\n"
    "                     channels=N_CHANNELS, dtype='float32', device=rs_idx)\n"
    "        sd.wait()\n"
    "        print('done')\n"
    "        # Save\n"
    "        WAV_LIVE.parent.mkdir(parents=True, exist_ok=True)\n"
    "        wavfile.write(WAV_LIVE, FS, (rec * 32767).astype(np.int16))\n"
    "        print(f'Saved {WAV_LIVE}')\n"
    "        live_data = rec\n"
    "        live_ok   = True\n"
    "    except Exception as e:\n"
    "        print(f'Recording failed: {e}')\n"
    "        live_ok = False\n"
    "else:\n"
    "    print('No device — skipping live recording.')\n"
    "    live_ok = False\n"
    "\n"
    "# Choose which data to use for beamforming; extract raw mic channels 2-5\n"
    "if live_ok:\n"
    "    bf_data = live_data[:, MIC_SLICE]\n"
    "    source_label = 'Live recording'\n"
    "else:\n"
    "    bf_data = data[:, MIC_SLICE]\n"
    "    source_label = 'Existing output.wav'\n"
    "\n"
    "print(f'Using: {source_label}  shape={bf_data.shape}')\n",
    "bc130011",
))

cells.append(code(
    "if live_ok:\n"
    "    t_live = np.arange(live_data.shape[0]) / FS\n"
    "    fig, axes = plt.subplots(4, 1, figsize=(14, 7), sharex=True)\n"
    "    for i, ch in enumerate(range(2, 6)):\n"
    "        axes[i].plot(t_live, live_data[:, ch], linewidth=0.4, color='steelblue')\n"
    "        axes[i].set_ylabel(f'Mic{i} raw\\n(ch{ch})', fontsize=8)\n"
    "        axes[i].set_ylim(-0.5, 0.5)\n"
    "        axes[i].grid(True, alpha=0.3)\n"
    "    axes[-1].set_xlabel('Time (s)')\n"
    "    plt.suptitle('Live recording — 4 mic channels')\n"
    "    plt.tight_layout()\n"
    "    plt.savefig('respeaker_live_waveform.png', dpi=150, bbox_inches='tight')\n"
    "    plt.show()\n"
    "else:\n"
    "    print('No live recording to plot.')\n",
    "bc130012",
))

# ---------------------------------------------------------------------------
# Section 4: CSM and beamforming
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Cross-Spectral Matrix and Beamforming\n"
    "\n"
    "Compute the frequency-domain CSM from the 4 mic channels using Welch-style\n"
    "block averaging, then apply D&S, MVDR, and CLEAN-SC.\n"
    "\n"
    "Frequency: 1500 Hz (safely below the ~2695 Hz spatial Nyquist).\n",
    "bc130013",
))

cells.append(code(
    "def compute_csm(audio, fs, freq, block_size=256, hop=128):\n"
    "    \"\"\"Welch-style CSM: average outer products over overlapping FFT blocks.\"\"\"\n"
    "    n_samp, n_ch = audio.shape\n"
    "    freqs   = np.fft.rfftfreq(block_size, 1/fs)\n"
    "    f_idx   = np.argmin(np.abs(freqs - freq))\n"
    "    f_actual = freqs[f_idx]\n"
    "    R = np.zeros((n_ch, n_ch), dtype=complex)\n"
    "    count = 0\n"
    "    win = np.hanning(block_size)\n"
    "    for start in range(0, n_samp - block_size, hop):\n"
    "        block = audio[start:start + block_size] * win[:, np.newaxis]\n"
    "        F = np.fft.rfft(block, axis=0)[f_idx]   # (n_ch,) complex\n"
    "        R    += np.outer(F, F.conj())\n"
    "        count += 1\n"
    "    return R / count, f_actual, count\n"
    "\n"
    "\n"
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
    "def clean_sc(x, y, R, freq, az_grid, n_iter=40, loop_gain=0.5):\n"
    "    H     = sm(x, y, az_grid, freq)\n"
    "    R_w   = R.copy()\n"
    "    clean = np.zeros(len(az_grid))\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        g = R_w @ H[:, k]\n"
    "        R_w   -= loop_gain * np.outer(g, g.conj())\n"
    "        clean[k] += loop_gain * np.real(g.conj() @ g)\n"
    "    return clean\n"
    "\n"
    "\n"
    "def db_norm(p):\n"
    "    return 10 * np.log10(np.maximum(p / p.max(), 1e-10))\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "\n"
    "R_csm, f_act, n_blocks = compute_csm(bf_data, FS, FREQ_BF)\n"
    "print(f'CSM computed at {f_act:.1f} Hz using {n_blocks} blocks')\n"
    "print(f'CSM shape: {R_csm.shape},  trace = {np.trace(R_csm).real:.4f}')\n"
    "print(f'\\nCSM diagonal (power per mic):')\n"
    "for ch in range(N_MICS):\n"
    "    print(f'  mic{ch}: {R_csm[ch,ch].real:.6f}')\n",
    "bc130014",
))

cells.append(code(
    "P_ds    = beamform_ds(x, y, R_csm, f_act, az_grid)\n"
    "P_mvdr  = beamform_mvdr(x, y, R_csm, f_act, az_grid)\n"
    "P_clean = clean_sc(x, y, R_csm, f_act, az_grid)\n"
    "\n"
    "az_ds    = az_grid[np.argmax(P_ds)]\n"
    "az_mvdr  = az_grid[np.argmax(P_mvdr)]\n"
    "az_clean = az_grid[np.argmax(P_clean)]\n"
    "\n"
    "fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=False)\n"
    "for ax, (name, P, col, az_est) in zip(axes, [\n"
    "    ('D&S',      P_ds,    'steelblue',   az_ds),\n"
    "    ('MVDR',     P_mvdr,  'darkorange',  az_mvdr),\n"
    "    ('CLEAN-SC', P_clean, 'forestgreen', az_clean),\n"
    "]):\n"
    "    ax.plot(az_grid, db_norm(P), color=col, linewidth=1.2)\n"
    "    ax.axvline(az_est, color=col, linestyle='--', linewidth=1,\n"
    "               label=f'peak={az_est:.1f}°')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('dB')\n"
    "    ax.set_title(f'{name}  (peak @ {az_est:.1f}°)')\n"
    "    ax.set_ylim(-30, 2)\n"
    "    ax.legend(fontsize=8)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'Beamformer output at {f_act:.0f} Hz — {source_label}')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_beamform.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print(f'Peak DoA estimates at {f_act:.0f} Hz:')\n"
    "print(f'  D&S:      {az_ds:.1f}°')\n"
    "print(f'  MVDR:     {az_mvdr:.1f}°')\n"
    "print(f'  CLEAN-SC: {az_clean:.1f}°')\n",
    "bc130015",
))

# ---------------------------------------------------------------------------
# Section 5: Multi-frequency sweep
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5 — Beamformer Peak vs Frequency\n"
    "\n"
    "Check whether the dominant source direction is consistent across the\n"
    "beamforming range (500–2400 Hz).  A stable peak across frequencies\n"
    "indicates a coherent source; a wandering peak indicates ambient noise.\n",
    "bc130016",
))

cells.append(code(
    "FREQS_SWEEP = [500, 750, 1000, 1250, 1500, 1750, 2000, 2250]\n"
    "\n"
    "peak_ds   = []\n"
    "peak_mvdr = []\n"
    "\n"
    "for freq in FREQS_SWEEP:\n"
    "    R_f, f_a, _ = compute_csm(bf_data, FS, freq)\n"
    "    P_d = beamform_ds(x, y, R_f, f_a, az_grid)\n"
    "    P_m = beamform_mvdr(x, y, R_f, f_a, az_grid)\n"
    "    peak_ds.append(az_grid[np.argmax(P_d)])\n"
    "    peak_mvdr.append(az_grid[np.argmax(P_m)])\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 4))\n"
    "ax.plot(FREQS_SWEEP, peak_ds,   'o-', color='steelblue',  label='D&S',  linewidth=1.5)\n"
    "ax.plot(FREQS_SWEEP, peak_mvdr, 's-', color='darkorange', label='MVDR', linewidth=1.5)\n"
    "ax.set_xlabel('Frequency (Hz)')\n"
    "ax.set_ylabel('Estimated DoA (deg)')\n"
    "ax.set_title(f'Peak direction vs frequency — {source_label}')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_freq_sweep.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('Peak DoA by frequency (D&S | MVDR):')\n"
    "for freq, pd, pm in zip(FREQS_SWEEP, peak_ds, peak_mvdr):\n"
    "    print(f'  {freq:5d} Hz:  D&S={pd:+6.1f}°   MVDR={pm:+6.1f}°')\n",
    "bc130017",
))

# ---------------------------------------------------------------------------
# Section 6: CSM structure check
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6 — CSM Structure\n"
    "\n"
    "Visualise the CSM magnitude to confirm it looks physically reasonable:\n"
    "- Diagonal should be the largest (power per mic)\n"
    "- Off-diagonal should show the inter-mic correlations\n"
    "- All mics should have similar diagonal power (confirms calibration baseline)\n",
    "bc130018",
))

cells.append(code(
    "R_csm_show, f_show, _ = compute_csm(bf_data, FS, FREQ_BF)\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(10, 4))\n"
    "\n"
    "# Magnitude\n"
    "im0 = axes[0].imshow(np.abs(R_csm_show), cmap='inferno', aspect='auto')\n"
    "axes[0].set_title(f'|CSM| at {f_show:.0f} Hz')\n"
    "axes[0].set_xlabel('mic'); axes[0].set_ylabel('mic')\n"
    "axes[0].set_xticks(range(4)); axes[0].set_yticks(range(4))\n"
    "plt.colorbar(im0, ax=axes[0])\n"
    "\n"
    "# Phase\n"
    "im1 = axes[1].imshow(np.angle(R_csm_show), cmap='hsv', aspect='auto',\n"
    "                     vmin=-np.pi, vmax=np.pi)\n"
    "axes[1].set_title(f'angle(CSM) at {f_show:.0f} Hz')\n"
    "axes[1].set_xlabel('mic'); axes[1].set_ylabel('mic')\n"
    "axes[1].set_xticks(range(4)); axes[1].set_yticks(range(4))\n"
    "plt.colorbar(im1, ax=axes[1], label='radians')\n"
    "\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_csm.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Coherence matrix\n"
    "diag = np.sqrt(np.diag(np.abs(R_csm_show)))\n"
    "coh  = np.abs(R_csm_show) / np.outer(diag, diag)\n"
    "print('Inter-mic coherence matrix (off-diagonal = coherence between mic pairs):')\n"
    "print(np.round(coh, 3))\n",
    "bc130019",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n"
    "\n"
    "Key findings to document in PHASE2.md:\n"
    "- Device detected and accessible via sounddevice\n"
    "- Channel structure: channels 0-3 are mic signals, 4-5 are processed outputs\n"
    "- CSM computation from real audio works correctly\n"
    "- Beamformer peak directions agree across D&S, MVDR, and CLEAN-SC\n"
    "- Note any hardware surprises (clock drift, noise floor, latency) below\n",
    "bc130020",
))

cells.append(code(
    "print('=== nb13 Summary ===')\n"
    "if rs_idx is not None:\n"
    "    print(f'Device: [{rs_idx}] {rs_dev[\"name\"]}')\n"
    "    print(f'Channels: {rs_dev[\"max_input_channels\"]},  '\n"
    "          f'Rate: {rs_dev[\"default_samplerate\"]:.0f} Hz,  '\n"
    "          f'Latency: {rs_dev[\"default_low_input_latency\"]*1000:.1f} ms')\n"
    "print(f'Recording: {\"live\" if live_ok else \"fallback to output.wav\"}')\n"
    "print(f'CSM blocks: {n_blocks}')\n"
    "print(f'Beamformer peaks at {f_act:.0f} Hz:')\n"
    "print(f'  D&S={az_ds:.1f}°  MVDR={az_mvdr:.1f}°  CLEAN-SC={az_clean:.1f}°')\n"
    "if abs(az_ds - az_mvdr) < 5 and abs(az_ds - az_clean) < 5:\n"
    "    print('PASS: all three algorithms agree within 5°')\n"
    "else:\n"
    "    print('NOTE: algorithm estimates differ by >5° — ambient or multi-source scene')\n"
    "print('\\nDiagonal power per mic (should be similar for all 4):')\n"
    "for ch in range(N_MICS):\n"
    "    print(f'  mic{ch}: {R_csm[ch,ch].real:.6f}')\n",
    "bc130021",
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

out = Path("13_respeaker_capture.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
