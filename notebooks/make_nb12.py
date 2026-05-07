#!/usr/bin/env python3
"""Generate notebooks/12_respeaker_array.ipynb"""
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
    "# 12 — ReSpeaker Array Characterization\n"
    "\n"
    "**Goal**: establish quantitative expectations for the ReSpeaker XVF3800 4-mic circular\n"
    "array before any hardware experiments.  This is a pure simulation notebook — no hardware\n"
    "required.\n"
    "\n"
    "Hardware: 4 microphones, equal 90° spacing, ~45 mm radius (90 mm aperture).\n"
    "\n"
    "Key questions answered here:\n"
    "\n"
    "1. What beamwidth and spatial aliasing frequency does this geometry produce?\n"
    "2. How do D&S, MVDR, CLEAN-SC, and MUSIC perform with only 4 mics?\n"
    "3. How many snapshots are needed for CSM convergence?\n"
    "4. How does Phase 2 performance compare to the Phase 4 target (96-mic, 300 mm)?\n",
    "ab120001",
))

# ---------------------------------------------------------------------------
# Imports + constants
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import pandas as pd\n"
    "from scipy.linalg import inv\n"
    "\n"
    "plt.rcParams['figure.dpi'] = 120\n"
    "plt.rcParams['font.size'] = 10\n"
    "\n"
    "C       = 343.0        # speed of sound (m/s)\n"
    "RADIUS  = 0.045        # 90mm aperture / 2\n"
    "N_MICS  = 4\n"
    "SNR_DB  = 20.0\n"
    "N_SNAP  = 256\n"
    "colors  = {'D&S': 'steelblue', 'MVDR': 'darkorange',\n"
    "           'CLEAN-SC': 'forestgreen', 'MUSIC': 'crimson'}\n",
    "ab120002",
))

# ---------------------------------------------------------------------------
# Array geometry
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — Array Geometry\n"
    "\n"
    "Four microphones at 0°, 90°, 180°, 270° on a circle of radius 45 mm.\n"
    "Note: the actual mic positions will be read from the device via `AEC_MIC_ARRAY_GEO`\n"
    "in nb13; this notebook uses the nominal geometry for simulation.\n",
    "ab120003",
))

cells.append(code(
    "angles_deg = np.array([0.0, 90.0, 180.0, 270.0])\n"
    "x = RADIUS * np.cos(np.radians(angles_deg))   # (4,)\n"
    "y = RADIUS * np.sin(np.radians(angles_deg))   # (4,)\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(4, 4))\n"
    "ax.scatter(x * 1000, y * 1000, s=80, zorder=5)\n"
    "for i, (xi, yi) in enumerate(zip(x * 1000, y * 1000)):\n"
    "    ax.annotate(f'mic {i}', (xi, yi), textcoords='offset points',\n"
    "                xytext=(6, 4), fontsize=8)\n"
    "circle = plt.Circle((0, 0), RADIUS * 1000, fill=False,\n"
    "                     linestyle='--', color='gray', linewidth=0.8)\n"
    "ax.add_patch(circle)\n"
    "ax.set_aspect('equal')\n"
    "ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')\n"
    "ax.set_title('ReSpeaker 4-mic circular array')\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_geometry.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Key geometry metrics\n"
    "d_adjacent = 2 * RADIUS * np.sin(np.radians(45))   # chord between adjacent mics\n"
    "f_nyquist  = C / (2 * d_adjacent)\n"
    "print(f'Radius:          {RADIUS*1000:.1f} mm')\n"
    "print(f'Adjacent chord:  {d_adjacent*1000:.1f} mm')\n"
    "print(f'Spatial Nyquist: {f_nyquist:.0f} Hz  (effective ceiling ~{0.9*f_nyquist:.0f} Hz)')\n",
    "ab120004",
))

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
cells.append(code(
    "def sv(x, y, az_deg, freq):\n"
    "    \"\"\"Single far-field steering vector.\"\"\"\n"
    "    u = np.sin(np.radians(az_deg))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * u * x) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def sm(x, y, az_grid, freq):\n"
    "    \"\"\"Steering matrix (N_mics, N_az).\"\"\"\n"
    "    u = np.sin(np.radians(az_grid))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * np.outer(x, u)) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def make_R(x, y, src_azimuths, freq, snr_db):\n"
    "    \"\"\"Theoretical CSM for one or more equal-power far-field sources.\"\"\"\n"
    "    N = len(x)\n"
    "    noise_var = 1.0 / (10 ** (snr_db / 10))\n"
    "    R = noise_var * np.eye(N, dtype=complex)\n"
    "    for az in src_azimuths:\n"
    "        h = sv(x, y, az, freq)\n"
    "        R += np.outer(h, h.conj())\n"
    "    return R\n"
    "\n"
    "\n"
    "def sample_csm(R_true, n_snap, seed=None):\n"
    "    \"\"\"Cholesky sampling for finite-snapshot CSM.\"\"\"\n"
    "    rng = np.random.default_rng(seed)\n"
    "    N   = R_true.shape[0]\n"
    "    reg = 1e-10 * np.trace(R_true) / N\n"
    "    L   = np.linalg.cholesky(R_true + reg * np.eye(N))\n"
    "    R_s = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        z   = rng.standard_normal(N) + 1j * rng.standard_normal(N)\n"
    "        y_s = L @ z / np.sqrt(2)\n"
    "        R_s += np.outer(y_s, y_s.conj())\n"
    "    return R_s / n_snap\n"
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
    "    \"\"\"1-D CLEAN-SC (Sijtsma 2007).\"\"\"\n"
    "    H      = sm(x, y, az_grid, freq)   # (N, N_az)\n"
    "    R_w    = R.copy()\n"
    "    clean  = np.zeros(len(az_grid))\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        g = R_w @ H[:, k]              # coherence vector\n"
    "        R_w   -= loop_gain * np.outer(g, g.conj())\n"
    "        clean[k] += loop_gain * np.real(g.conj() @ g)\n"
    "    return clean\n"
    "\n"
    "\n"
    "def beamform_music(x, y, R, freq, az_grid, n_sources=1):\n"
    "    _, ev = np.linalg.eigh(R)\n"
    "    En    = ev[:, :R.shape[0] - n_sources]\n"
    "    Ep    = En @ En.conj().T\n"
    "    H     = sm(x, y, az_grid, freq)\n"
    "    d     = np.real(np.sum(H.conj() * (Ep @ H), axis=0))\n"
    "    return 1.0 / np.maximum(d, 1e-300)\n"
    "\n"
    "\n"
    "def db_norm(p):\n"
    "    return 10 * np.log10(np.maximum(p / p.max(), 1e-10))\n"
    "\n"
    "\n"
    "def measure_hpbw(az_grid, pattern):\n"
    "    \"\"\"Half-power (-3 dB) beamwidth in degrees.\"\"\"\n"
    "    half = pattern.max() / 2\n"
    "    above = pattern >= half\n"
    "    if not above.any():\n"
    "        return float('nan')\n"
    "    left  = az_grid[np.argmax(above)]\n"
    "    right = az_grid[len(above) - 1 - np.argmax(above[::-1])]\n"
    "    if right <= left:\n"
    "        return float('nan')\n"
    "    return right - left\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "print('Helpers defined.')\n",
    "ab120005",
))

# ---------------------------------------------------------------------------
# Section 2: PSF / HPBW sweep
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — PSF and HPBW vs Frequency\n"
    "\n"
    "Delay-and-Sum point spread function for a boresight source at each frequency.\n"
    "HPBW is measured as the −3 dB beamwidth.\n",
    "ab120006",
))

cells.append(code(
    "FREQS    = [500, 1000, 1500, 2000, 2500]\n"
    "hpbw_arr = []\n"
    "\n"
    "fig, axes = plt.subplots(1, len(FREQS), figsize=(16, 4), sharey=True)\n"
    "\n"
    "for ax, freq in zip(axes, FREQS):\n"
    "    h0 = sv(x, y, 0.0, freq)\n"
    "    R0 = np.outer(h0, h0.conj())          # noiseless, single source\n"
    "    P  = beamform_ds(x, y, R0, freq, az_grid)\n"
    "    P_db = db_norm(P)\n"
    "    hpbw = measure_hpbw(az_grid, P)\n"
    "    hpbw_arr.append(hpbw)\n"
    "    ax.plot(az_grid, P_db, color='steelblue', linewidth=1.2)\n"
    "    ax.axhline(-3, color='gray', linestyle='--', linewidth=0.8)\n"
    "    ax.set_ylim(-30, 2)\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_title(f'{freq} Hz\\nHPBW = {hpbw:.0f}°' if not np.isnan(hpbw)\n"
    "                 else f'{freq} Hz\\nHPBW = omni')\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "axes[0].set_ylabel('D&S response (dB)')\n"
    "plt.suptitle('D&S PSF — ReSpeaker 4-mic circular (boresight source)')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_psf.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('HPBW vs frequency:')\n"
    "for freq, hpbw in zip(FREQS, hpbw_arr):\n"
    "    tag = f'{hpbw:.0f}°' if not np.isnan(hpbw) else '>180° (omni)'\n"
    "    print(f'  {freq:5d} Hz  HPBW = {tag}')\n",
    "ab120007",
))

# ---------------------------------------------------------------------------
# Section 3: Spatial aliasing
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Spatial Aliasing\n"
    "\n"
    "The regular 90° spacing creates a single strong grating lobe when the\n"
    "frequency exceeds the spatial Nyquist.\n",
    "ab120008",
))

cells.append(code(
    "fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)\n"
    "for ax, freq in zip(axes, [2000, 2700, 3500]):\n"
    "    h0 = sv(x, y, 20.0, freq)\n"
    "    R0 = np.outer(h0, h0.conj())\n"
    "    P  = beamform_ds(x, y, R0, freq, az_grid)\n"
    "    ax.plot(az_grid, db_norm(P), color='steelblue', linewidth=1.2)\n"
    "    ax.axvline(20, color='k', linestyle='--', linewidth=0.8, label='true az')\n"
    "    ax.set_ylim(-15, 2)\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_title(f'{freq} Hz')\n"
    "    ax.grid(True, alpha=0.3)\n"
    "    ax.legend(fontsize=8)\n"
    "\n"
    "axes[0].set_ylabel('D&S response (dB)')\n"
    "plt.suptitle('Spatial aliasing above Nyquist — source at 20°')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_aliasing.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Note: regular 4-mic spacing creates ONE grating lobe (vs many weak lobes for Underbrink)\n"
    "print(f'Spatial Nyquist: {f_nyquist:.0f} Hz')\n"
    "print('Above Nyquist: single grating lobe appears (regular spacing = periodic grating)')\n"
    "print('Underbrink array distributes alias energy across many weak irregular lobes instead.')\n",
    "ab120009",
))

# ---------------------------------------------------------------------------
# Section 4: Algorithm comparison
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Algorithm Comparison\n"
    "\n"
    "Compare D&S, MVDR, CLEAN-SC, and MUSIC at 2 kHz (upper end of useful range).\n"
    "\n"
    "**Important caveat on MUSIC with N=4:** the noise subspace has only N−k = 3 (k=1) or\n"
    "N−k = 2 (k=2) dimensions.  With so few mics, the eigenvalue gap between signal and noise\n"
    "subspaces is small, especially at moderate SNR.  MUSIC works best when N >> k.\n"
    "\n"
    "Scenarios:\n"
    "1. Single source at 10°, SNR=20 dB, N_SNAP=256\n"
    "2. Two sources at ±45° (90° separation), SNR=20 dB, N_SNAP=256\n",
    "ab120010",
))

cells.append(code(
    "FREQ = 2000.0\n"
    "\n"
    "fig, axes = plt.subplots(2, 4, figsize=(16, 8))\n"
    "scenarios = [\n"
    "    ('Single source at 10°', [10.0],        {'n_sources': 1}, False),\n"
    "    ('Two sources at ±45°',  [-45.0, 45.0], {'n_sources': 2}, True),\n"
    "]\n"
    "alg_list = [\n"
    "    ('D&S',      lambda R: beamform_ds(x, y, R, FREQ, az_grid),              'steelblue'),\n"
    "    ('MVDR',     lambda R: beamform_mvdr(x, y, R, FREQ, az_grid),            'darkorange'),\n"
    "    ('CLEAN-SC', lambda R: clean_sc(x, y, R, FREQ, az_grid),                 'forestgreen'),\n"
    "    ('MUSIC',    lambda R, ns=1: beamform_music(x, y, R, FREQ, az_grid, ns), 'crimson'),\n"
    "]\n"
    "\n"
    "for row, (title, src_az, music_kw, two_src) in enumerate(scenarios):\n"
    "    R_t = make_R(x, y, src_az, FREQ, SNR_DB)\n"
    "    R   = sample_csm(R_t, N_SNAP, seed=42)\n"
    "    for col, (alg_name, fn, col_color) in enumerate(alg_list):\n"
    "        ax = axes[row, col]\n"
    "        if alg_name == 'MUSIC':\n"
    "            n_s = music_kw['n_sources']\n"
    "            P = beamform_music(x, y, R, FREQ, az_grid, n_sources=n_s)\n"
    "        else:\n"
    "            P = fn(R)\n"
    "        ax.plot(az_grid, db_norm(P), color=col_color, linewidth=1.2)\n"
    "        for az in src_az:\n"
    "            ax.axvline(az, color='k', linestyle='--', linewidth=0.8)\n"
    "        ax.set_ylim(-30, 2)\n"
    "        ax.grid(True, alpha=0.3)\n"
    "        if row == 0:\n"
    "            ax.set_title(alg_name)\n"
    "        if col == 0:\n"
    "            ax.set_ylabel(f'{title}\\ndB')\n"
    "        if row == 1:\n"
    "            ax.set_xlabel('Azimuth (deg)')\n"
    "\n"
    "plt.suptitle(f'Algorithm comparison — 4-mic, {FREQ/1000:.0f} kHz, '\n"
    "             f'SNR={SNR_DB:.0f} dB, N_SNAP={N_SNAP}')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_algo_comparison.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "ab120011",
))

# ---------------------------------------------------------------------------
# Section 5: SNR sensitivity
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5 — DoA Accuracy vs SNR\n"
    "\n"
    "Single source at 20°, 2 kHz, N_SNAP=256. Measure DoA error vs SNR.\n"
    "With only 4 mics, spatial averaging provides ~6 dB of array gain\n"
    "(vs ~20 dB for the 96-mic Phase 4 array), so the SNR floor is higher.\n",
    "ab120012",
))

cells.append(code(
    "SNR_VALS  = [0, 5, 10, 15, 20, 25, 30]\n"
    "AZ_TRUE   = 20.0\n"
    "N_TRIALS  = 30\n"
    "az_scan   = np.linspace(-90, 90, 1801)\n"
    "\n"
    "snr_results = {alg: [] for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for snr in SNR_VALS:\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    R_t  = make_R(x, y, [AZ_TRUE], FREQ, snr)\n"
    "    for trial in range(N_TRIALS):\n"
    "        R = sample_csm(R_t, N_SNAP, seed=trial * 100 + snr)\n"
    "        for alg, fn in [\n"
    "            ('D&S',   lambda R: beamform_ds(x, y, R, FREQ, az_scan)),\n"
    "            ('MVDR',  lambda R: beamform_mvdr(x, y, R, FREQ, az_scan)),\n"
    "            ('MUSIC', lambda R: beamform_music(x, y, R, FREQ, az_scan, n_sources=1)),\n"
    "        ]:\n"
    "            P = fn(R)\n"
    "            errs[alg].append(abs(az_scan[np.argmax(P)] - AZ_TRUE))\n"
    "    for alg in snr_results:\n"
    "        snr_results[alg].append(np.mean(errs[alg]))\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(8, 5))\n"
    "for alg, col in [('D&S','steelblue'),('MVDR','darkorange'),('MUSIC','crimson')]:\n"
    "    ax.plot(SNR_VALS, snr_results[alg], 'o-', color=col, label=alg, linewidth=1.5)\n"
    "ax.set_xlabel('SNR (dB)')\n"
    "ax.set_ylabel('Mean abs DoA error (deg)')\n"
    "ax.set_title(f'DoA accuracy vs SNR — 4-mic, {FREQ/1000:.0f} kHz, '\n"
    "             f'source @ {AZ_TRUE}°, N_SNAP={N_SNAP}')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_snr_sweep.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('Mean DoA error (deg):')\n"
    "print(f'{\"SNR\":>6}  {\"D&S\":>8}  {\"MVDR\":>8}  {\"MUSIC\":>8}')\n"
    "for i, snr in enumerate(SNR_VALS):\n"
    "    print(f'{snr:6d}  '\n"
    "          f'{snr_results[\"D&S\"][i]:8.3f}  '\n"
    "          f'{snr_results[\"MVDR\"][i]:8.3f}  '\n"
    "          f'{snr_results[\"MUSIC\"][i]:8.3f}')\n",
    "ab120013",
))

# ---------------------------------------------------------------------------
# Section 6: Snapshot count sweep
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6 — Snapshot Count Sweep\n"
    "\n"
    "With N=4 mics, the 4×4 CSM is theoretically full-rank at N_SNAP ≥ 4.\n"
    "CSM convergence should be fast compared to the 96-mic Phase 4 array.\n"
    "Test DoA error vs N_SNAP at SNR=20 dB.\n",
    "ab120014",
))

cells.append(code(
    "SNAP_VALS = [4, 8, 16, 32, 64, 128, 256, 512]\n"
    "N_TRIALS2 = 30\n"
    "\n"
    "snap_results = {alg: [] for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "R_t20 = make_R(x, y, [AZ_TRUE], FREQ, 20.0)\n"
    "\n"
    "for n_snap in SNAP_VALS:\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    for trial in range(N_TRIALS2):\n"
    "        R = sample_csm(R_t20, n_snap, seed=trial * 200 + n_snap)\n"
    "        for alg, fn in [\n"
    "            ('D&S',   lambda R: beamform_ds(x, y, R, FREQ, az_scan)),\n"
    "            ('MVDR',  lambda R: beamform_mvdr(x, y, R, FREQ, az_scan)),\n"
    "            ('MUSIC', lambda R: beamform_music(x, y, R, FREQ, az_scan, n_sources=1)),\n"
    "        ]:\n"
    "            P = fn(R)\n"
    "            errs[alg].append(abs(az_scan[np.argmax(P)] - AZ_TRUE))\n"
    "    for alg in snap_results:\n"
    "        snap_results[alg].append(np.mean(errs[alg]))\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(8, 5))\n"
    "for alg, col in [('D&S','steelblue'),('MVDR','darkorange'),('MUSIC','crimson')]:\n"
    "    ax.semilogx(SNAP_VALS, snap_results[alg], 'o-', color=col,\n"
    "                label=alg, linewidth=1.5)\n"
    "ax.axvline(N_MICS, color='gray', linestyle=':', linewidth=1,\n"
    "           label=f'N_SNAP = N_MICS = {N_MICS} (CSM full-rank)')\n"
    "ax.set_xlabel('N_SNAP')\n"
    "ax.set_ylabel('Mean abs DoA error (deg)')\n"
    "ax.set_title(f'DoA accuracy vs N_SNAP — 4-mic, {FREQ/1000:.0f} kHz, '\n"
    "             f'SNR=20 dB, source @ {AZ_TRUE}°')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3, which='both')\n"
    "plt.tight_layout()\n"
    "plt.savefig('respeaker_snap_sweep.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('Mean DoA error (deg) vs N_SNAP:')\n"
    "print(f'{\"N_SNAP\":>8}  {\"D&S\":>8}  {\"MVDR\":>8}  {\"MUSIC\":>8}')\n"
    "for i, n_snap in enumerate(SNAP_VALS):\n"
    "    print(f'{n_snap:8d}  '\n"
    "          f'{snap_results[\"D&S\"][i]:8.3f}  '\n"
    "          f'{snap_results[\"MVDR\"][i]:8.3f}  '\n"
    "          f'{snap_results[\"MUSIC\"][i]:8.3f}')\n",
    "ab120015",
))

# ---------------------------------------------------------------------------
# Section 7: Phase 2 vs Phase 4 comparison
# ---------------------------------------------------------------------------
cells.append(md(
    "## 7 — Phase 2 vs Phase 4 Expected Performance\n"
    "\n"
    "Summary table comparing what to expect from the 4-mic smoke-test array vs the\n"
    "96-mic Underbrink target.\n",
    "ab120016",
))

cells.append(code(
    "# Phase 4 reference values from Phase 1 notebooks\n"
    "p4_hpbw = {500: '>180°', 1000: '82°', 2000: '38°', 4000: '19°', 8000: '9°'}\n"
    "\n"
    "rows = []\n"
    "for freq, hpbw in zip(FREQS, hpbw_arr):\n"
    "    p4 = p4_hpbw.get(freq, '—')\n"
    "    p2 = f'{hpbw:.0f}°' if not np.isnan(hpbw) else '>180° (omni)'\n"
    "    rows.append({'Freq (Hz)': freq, 'Phase 2 HPBW (4-mic)': p2,\n"
    "                 'Phase 4 HPBW (96-mic)': p4})\n"
    "\n"
    "df = pd.DataFrame(rows)\n"
    "print(df.to_string(index=False))\n"
    "print()\n"
    "\n"
    "summary = [\n"
    "    ('Mic count',          '4',              '96'),\n"
    "    ('Aperture',           '90 mm',          '300 mm'),\n"
    "    ('Min spacing',        '63.6 mm',        '12.9 mm'),\n"
    "    ('Spatial Nyquist',    f'{f_nyquist:.0f} Hz', '~13 kHz'),\n"
    "    ('Useful freq range',  '≤ ~2.5 kHz',     '200 Hz – 8 kHz'),\n"
    "    ('Array gain',         '~6 dB',          '~20 dB'),\n"
    "    ('SNR floor (DoA)',    '~15 dB',          '~10 dB'),\n"
    "    ('Calibration need',   'High (4-ch avg)',  'Low (96-ch avg suppresses errors)'),\n"
    "    ('MUSIC benefit',      'Limited (N=4)',   'Strong (N=96)'),\n"
    "    ('Purpose',            'Pipeline smoke test', 'Production acoustic camera'),\n"
    "]\n"
    "df2 = pd.DataFrame(summary, columns=['Parameter', 'Phase 2 (ReSpeaker)', 'Phase 4 (target)'])\n"
    "print(df2.to_string(index=False))\n",
    "ab120017",
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

out = Path("12_respeaker_array.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
