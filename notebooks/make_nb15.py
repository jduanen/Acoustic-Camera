#!/usr/bin/env python3
"""Generate notebooks/15_uma16_array.ipynb"""
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
    "# 15 — UMA-16 v2 Array Characterization\n"
    "\n"
    "**Goal**: establish quantitative expectations for the miniDSP UMA-16 v2 before hardware\n"
    "experiments.  Pure simulation — no hardware required.\n"
    "\n"
    "Hardware: 16 mics in a 4×4 Uniform Rectangular Array (URA), 42 mm grid spacing,\n"
    "126 mm × 126 mm aperture.  Knowles SPH1668LM4H-1 MEMS mics, 65.5 dB SNR.\n"
    "USB audio at 48 kHz (UAC2, XMOS Xcore200).\n"
    "\n"
    "Key questions:\n"
    "\n"
    "1. What HPBW and spatial Nyquist does this 4×4 URA produce?\n"
    "2. What super-resolution benefit do MVDR and MUSIC give over D&S with N=16 mics?\n"
    "3. How many snapshots are needed for CSM convergence?\n"
    "4. How does Phase 3 (16-mic, 126 mm) compare to Phase 2 (4-mic, 90 mm) and\n"
    "   Phase 4 target (96-mic, 300 mm)?\n"
    "5. What do the 2D (az × el) PSF maps look like?\n",
    "ab150001",
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
    "C      = 343.0\n"
    "FS     = 48000\n"
    "N_MICS = 16\n"
    "SNR_DB = 20.0\n"
    "N_SNAP = 256\n"
    "colors = {'D&S': 'steelblue', 'MVDR': 'darkorange',\n"
    "          'CLEAN-SC': 'forestgreen', 'MUSIC': 'crimson'}\n",
    "ab150002",
))

# ---------------------------------------------------------------------------
# Section 1: Array geometry
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1  Array Geometry\n"
    "\n"
    "4×4 URA, 42 mm spacing.  Channel → mic mapping from UMA-16 v2 manual Figure 1\n"
    "(sound-source side view).  Channels are PDM L/R pairs; each stereo pair shares\n"
    "one data line, which is why the numbering is not row-major.",
    "ab150010",
))

cells.append(code(
    "d = 0.042   # grid spacing [m]\n"
    "\n"
    "# USB channel → (x, y) in units of d/2; multiply to get meters\n"
    "_xy = np.array([\n"
    "    (-1, -3),   # ch0  MIC1\n"
    "    (-3, -3),   # ch1  MIC2\n"
    "    (-1, -1),   # ch2  MIC3\n"
    "    (-3, -1),   # ch3  MIC4\n"
    "    (-1, +1),   # ch4  MIC5\n"
    "    (-3, +1),   # ch5  MIC6\n"
    "    (-1, +3),   # ch6  MIC7\n"
    "    (-3, +3),   # ch7  MIC8\n"
    "    (+3, +3),   # ch8  MIC9\n"
    "    (+1, +3),   # ch9  MIC10\n"
    "    (+3, +1),   # ch10 MIC11\n"
    "    (+1, +1),   # ch11 MIC12\n"
    "    (+3, -1),   # ch12 MIC13\n"
    "    (+1, -1),   # ch13 MIC14\n"
    "    (+3, -3),   # ch14 MIC15\n"
    "    (+1, -3),   # ch15 MIC16\n"
    "], dtype=float) * (d / 2)\n"
    "\n"
    "MIC_X = -_xy[:, 0]  # negated: Figure 1 is sound-source side; camera side is x-mirrored\n"
    "MIC_Y = _xy[:, 1]\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(5, 5))\n"
    "ax.scatter(MIC_X * 1000, MIC_Y * 1000, s=80, color='steelblue', zorder=3)\n"
    "for i, (x, y) in enumerate(zip(MIC_X * 1000, MIC_Y * 1000)):\n"
    "    ax.text(x + 3, y + 3, f'ch{i}', fontsize=7)\n"
    "ax.set_xlabel('x [mm]')\n"
    "ax.set_ylabel('y [mm]')\n"
    "ax.set_title('UMA-16 v2 — 4×4 URA (42 mm grid)')\n"
    "ax.set_aspect('equal')\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_geometry.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print(f'Aperture: {(MIC_X.max()-MIC_X.min())*1000:.0f} mm × {(MIC_Y.max()-MIC_Y.min())*1000:.0f} mm')\n"
    "print(f'Spatial Nyquist: {C / (2 * d):.0f} Hz  (spacing = {d*1000:.0f} mm)')\n"
    "print(f'Far-field distance @ 4 kHz: {2 * (MIC_X.max()-MIC_X.min())**2 / (C/4000):.3f} m')\n",
    "ab150011",
))

# ---------------------------------------------------------------------------
# Section 2: Steering matrix + beamforming functions
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2  Beamforming Functions\n"
    "\n"
    "2D steering matrix covers azimuth × elevation.  For 1D PSF sweeps we fix el = 0.",
    "ab150020",
))

cells.append(code(
    "def steering_matrix(az_deg, el_deg, freq):\n"
    "    \"\"\"(N_mics, N_az * N_el) steering matrix.\"\"\"\n"
    "    sin_az = np.sin(np.radians(az_deg))\n"
    "    cos_el = np.cos(np.radians(el_deg))\n"
    "    sin_el = np.sin(np.radians(el_deg))\n"
    "    ux = np.outer(sin_az, cos_el).ravel()\n"
    "    uy = np.tile(sin_el, len(az_deg))\n"
    "    phase = 2 * np.pi * freq / C * (np.outer(MIC_X, ux) + np.outer(MIC_Y, uy))\n"
    "    return np.exp(1j * phase) / np.sqrt(N_MICS)\n"
    "\n"
    "\n"
    "def beamform_ds(R, H):\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_mvdr(R, H, diag_load=0.01):\n"
    "    Rl = R + diag_load * np.trace(R) / N_MICS * np.eye(N_MICS)\n"
    "    Ri = inv(Rl)\n"
    "    return 1.0 / np.maximum(np.real(np.sum(H.conj() * (Ri @ H), axis=0)), 1e-300)\n"
    "\n"
    "\n"
    "def clean_sc(R, H, n_iter=20, loop_gain=0.5):\n"
    "    R_w = R.copy()\n"
    "    clean = np.zeros(H.shape[1])\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        h = H[:, k]\n"
    "        P_src = np.real(h.conj() @ (R_w @ h))\n"
    "        R_w -= loop_gain * P_src * np.outer(h, h.conj())\n"
    "        clean[k] += loop_gain * P_src\n"
    "    return clean\n"
    "\n"
    "\n"
    "def beamform_music(R, H, n_src=1):\n"
    "    _, V = np.linalg.eigh(R)\n"
    "    En = V[:, :N_MICS - n_src]\n"
    "    proj = En.conj().T @ H\n"
    "    return 1.0 / np.maximum(np.real(np.sum(proj.conj() * proj, axis=0)), 1e-300)\n"
    "\n"
    "\n"
    "def make_csm(az_src_deg, el_src_deg, freq, n_snap=N_SNAP, snr_db=SNR_DB):\n"
    "    \"\"\"Simulate CSM for a single source at (az_src_deg, el_src_deg).\"\"\"\n"
    "    snr = 10 ** (snr_db / 10)\n"
    "    h_src = steering_matrix([az_src_deg], [el_src_deg], freq)[:, 0]  # (N_mics,)\n"
    "    rng = np.random.default_rng(42)\n"
    "    s = rng.standard_normal(n_snap) + 1j * rng.standard_normal(n_snap)\n"
    "    noise = (rng.standard_normal((N_MICS, n_snap)) +\n"
    "             1j * rng.standard_normal((N_MICS, n_snap))) / np.sqrt(snr)\n"
    "    X = np.outer(h_src, s) + noise\n"
    "    return X @ X.conj().T / n_snap\n",
    "ab150021",
))

# ---------------------------------------------------------------------------
# Section 3: PSF and HPBW sweep
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3  PSF and HPBW vs Frequency\n"
    "\n"
    "1D azimuth PSF at el = 0° for frequencies from 500 Hz to 4 kHz.\n"
    "Spatial Nyquist is ~4.1 kHz; aliasing becomes visible above that.",
    "ab150030",
))

cells.append(code(
    "freqs_test = [500, 1000, 2000, 3000, 4000, 4500]\n"
    "az_grid = np.linspace(-90, 90, 1801)\n"
    "el_fix  = np.array([0.0])\n"
    "AZ_SRC  = 30.0\n"
    "\n"
    "fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharey=True)\n"
    "hpbw_ds = []\n"
    "\n"
    "for ax, freq in zip(axes.ravel(), freqs_test):\n"
    "    R = make_csm(AZ_SRC, 0.0, freq, n_snap=512)\n"
    "    H = steering_matrix(az_grid, el_fix, freq)\n"
    "    P = beamform_ds(R, H)\n"
    "    P_db = 10 * np.log10(P / P.max() + 1e-10)\n"
    "    ax.plot(az_grid, P_db, color='steelblue', lw=1.2)\n"
    "    ax.axvline(AZ_SRC, color='k', ls='--', lw=0.8, alpha=0.6)\n"
    "    ax.axhline(-3, color='gray', ls=':', lw=0.8)\n"
    "    ax.set_xlim(-90, 90)\n"
    "    ax.set_ylim(-40, 2)\n"
    "    ax.set_title(f'{freq} Hz')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Power (dB)')\n"
    "    above = P_db >= -3\n"
    "    if above.any():\n"
    "        span = az_grid[above].max() - az_grid[above].min()\n"
    "        hpbw_ds.append((freq, span))\n"
    "    else:\n"
    "        hpbw_ds.append((freq, float('nan')))\n"
    "\n"
    "plt.suptitle('UMA-16 v2 — D&S PSF (source at 30°)', fontsize=12)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_psf.png', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print('\\nD&S HPBW summary:')\n"
    "for f, bw in hpbw_ds:\n"
    "    print(f'  {f:5d} Hz  HPBW = {bw:.1f}°')\n",
    "ab150031",
))

# ---------------------------------------------------------------------------
# Section 4: Algorithm comparison
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4  Algorithm Comparison\n"
    "\n"
    "With N=16 mics (vs N=4 in Phase 2), MVDR and MUSIC have more meaningful super-resolution\n"
    "benefit.  The noise subspace has N−n_src = 15 dimensions, providing much stronger\n"
    "interference rejection than the N=4 case.",
    "ab150040",
))

cells.append(code(
    "FREQ = 3000.0\n"
    "az_grid_fine = np.linspace(-90, 90, 1801)\n"
    "\n"
    "R = make_csm(AZ_SRC, 0.0, FREQ, n_snap=N_SNAP)\n"
    "H = steering_matrix(az_grid_fine, el_fix, FREQ)\n"
    "\n"
    "results = {\n"
    "    'D&S':     beamform_ds(R, H),\n"
    "    'MVDR':    beamform_mvdr(R, H),\n"
    "    'CLEAN-SC': clean_sc(R, H),\n"
    "    'MUSIC':   beamform_music(R, H, n_src=1),\n"
    "}\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 4))\n"
    "for name, P in results.items():\n"
    "    P_db = 10 * np.log10(P / P.max() + 1e-10)\n"
    "    ax.plot(az_grid_fine, P_db, label=name, color=colors[name], lw=1.5)\n"
    "\n"
    "ax.axvline(AZ_SRC, color='k', ls='--', lw=0.8, alpha=0.6, label='True DoA')\n"
    "ax.axhline(-3, color='gray', ls=':', lw=0.8)\n"
    "ax.set_xlim(-60, 60)\n"
    "ax.set_ylim(-40, 2)\n"
    "ax.set_xlabel('Azimuth (deg)')\n"
    "ax.set_ylabel('Normalized power (dB)')\n"
    "ax.set_title(f'Algorithm comparison — {FREQ:.0f} Hz, source at {AZ_SRC}°')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_algo_comparison.png', dpi=150)\n"
    "plt.show()\n",
    "ab150041",
))

# ---------------------------------------------------------------------------
# Section 5: Two-source resolution
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5  Two-Source Resolution\n"
    "\n"
    "Test minimum resolvable angular separation.  Phase 3 has 1.4× wider aperture than\n"
    "Phase 2 (126 mm vs 90 mm) so should resolve sources ~1.4× closer.",
    "ab150050",
))

cells.append(code(
    "def make_csm_2src(az1, az2, freq, n_snap=N_SNAP, snr_db=SNR_DB):\n"
    "    snr = 10 ** (snr_db / 10)\n"
    "    h1 = steering_matrix([az1], [0.0], freq)[:, 0]\n"
    "    h2 = steering_matrix([az2], [0.0], freq)[:, 0]\n"
    "    rng = np.random.default_rng(42)\n"
    "    s1 = rng.standard_normal(n_snap) + 1j * rng.standard_normal(n_snap)\n"
    "    s2 = rng.standard_normal(n_snap) + 1j * rng.standard_normal(n_snap)\n"
    "    noise = (rng.standard_normal((N_MICS, n_snap)) +\n"
    "             1j * rng.standard_normal((N_MICS, n_snap))) / np.sqrt(snr)\n"
    "    X = np.outer(h1, s1) + np.outer(h2, s2) + noise\n"
    "    return X @ X.conj().T / n_snap\n"
    "\n"
    "\n"
    "seps = [5, 10, 15, 20, 30]\n"
    "FREQ2 = 3000.0\n"
    "\n"
    "fig, axes = plt.subplots(1, len(seps), figsize=(14, 3), sharey=True)\n"
    "for ax, sep in zip(axes, seps):\n"
    "    az1, az2 = -sep / 2, sep / 2\n"
    "    R2 = make_csm_2src(az1, az2, FREQ2)\n"
    "    H2 = steering_matrix(az_grid_fine, el_fix, FREQ2)\n"
    "    P_mvdr = beamform_mvdr(R2, H2)\n"
    "    P_ds   = beamform_ds(R2, H2)\n"
    "    ax.plot(az_grid_fine, 10 * np.log10(P_ds / P_ds.max() + 1e-10),\n"
    "            color=colors['D&S'], lw=1.2, label='D&S', alpha=0.8)\n"
    "    ax.plot(az_grid_fine, 10 * np.log10(P_mvdr / P_mvdr.max() + 1e-10),\n"
    "            color=colors['MVDR'], lw=1.2, label='MVDR')\n"
    "    ax.axvline(az1, color='k', ls='--', lw=0.7)\n"
    "    ax.axvline(az2, color='k', ls='--', lw=0.7)\n"
    "    ax.set_xlim(-35, 35)\n"
    "    ax.set_ylim(-30, 2)\n"
    "    ax.set_title(f'sep={sep}°')\n"
    "    ax.set_xlabel('az (deg)')\n"
    "    if ax is axes[0]:\n"
    "        ax.set_ylabel('dB')\n"
    "        ax.legend(fontsize=7)\n"
    "\n"
    "plt.suptitle(f'Two-source resolution @ {FREQ2:.0f} Hz', fontsize=11)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_two_source.png', dpi=150)\n"
    "plt.show()\n",
    "ab150051",
))

# ---------------------------------------------------------------------------
# Section 6: Snapshot convergence
# ---------------------------------------------------------------------------
cells.append(md(
    "## 6  Snapshot Convergence\n"
    "\n"
    "With N=16 mics, the CSM is full-rank at N_SNAP ≥ 16.  Convergence continues\n"
    "improving beyond that due to noise averaging.",
    "ab150060",
))

cells.append(code(
    "snap_vals = [4, 8, 16, 32, 64, 128, 256, 512]\n"
    "FREQ_SNAP = 3000.0\n"
    "N_TRIALS  = 50\n"
    "rng = np.random.default_rng(0)\n"
    "\n"
    "rows = []\n"
    "for n_snap in snap_vals:\n"
    "    errs_ds, errs_mv, errs_mu = [], [], []\n"
    "    for _ in range(N_TRIALS):\n"
    "        snr = 10 ** (SNR_DB / 10)\n"
    "        h_s = steering_matrix([AZ_SRC], [0.0], FREQ_SNAP)[:, 0]\n"
    "        s  = rng.standard_normal(n_snap) + 1j * rng.standard_normal(n_snap)\n"
    "        nz = (rng.standard_normal((N_MICS, n_snap)) +\n"
    "              1j * rng.standard_normal((N_MICS, n_snap))) / np.sqrt(snr)\n"
    "        X  = np.outer(h_s, s) + nz\n"
    "        R  = X @ X.conj().T / n_snap\n"
    "        H  = steering_matrix(az_grid, el_fix, FREQ_SNAP)\n"
    "        errs_ds.append(abs(az_grid[np.argmax(beamform_ds(R, H))] - AZ_SRC))\n"
    "        errs_mv.append(abs(az_grid[np.argmax(beamform_mvdr(R, H))] - AZ_SRC))\n"
    "        errs_mu.append(abs(az_grid[np.argmax(beamform_music(R, H))] - AZ_SRC))\n"
    "    rows.append({'N_SNAP': n_snap, 'D&S': np.mean(errs_ds),\n"
    "                 'MVDR': np.mean(errs_mv), 'MUSIC': np.mean(errs_mu)})\n"
    "\n"
    "df_snap = pd.DataFrame(rows)\n"
    "print(df_snap.to_string(index=False, float_format='{:.2f}'.format))\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(7, 4))\n"
    "for col in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    ax.plot(df_snap['N_SNAP'], df_snap[col], marker='o', label=col, color=colors[col])\n"
    "ax.axvline(N_MICS, color='gray', ls=':', label='CSM full-rank (N=16)')\n"
    "ax.set_xscale('log', base=2)\n"
    "ax.set_xlabel('N_SNAP')\n"
    "ax.set_ylabel('Mean DoA error (deg)')\n"
    "ax.set_title(f'Snapshot convergence — {FREQ_SNAP:.0f} Hz, SNR = {SNR_DB} dB')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_snap_convergence.png', dpi=150)\n"
    "plt.show()\n",
    "ab150061",
))

# ---------------------------------------------------------------------------
# Section 7: 2D PSF
# ---------------------------------------------------------------------------
cells.append(md(
    "## 7  2D PSF (Azimuth × Elevation)\n"
    "\n"
    "The 4×4 URA produces a symmetric 2D beam pattern.  Beamwidth is equal in az and el\n"
    "(symmetric aperture), unlike a rectangular array with unequal dimensions.",
    "ab150070",
))

cells.append(code(
    "FREQ_2D = 3000.0\n"
    "az_2d   = np.linspace(-60, 60, 241)\n"
    "el_2d   = np.linspace(-60, 60, 241)\n"
    "\n"
    "R_2d = make_csm(0.0, 0.0, FREQ_2D, n_snap=512)\n"
    "H_2d = steering_matrix(az_2d, el_2d, FREQ_2D)  # (16, 241*241)\n"
    "\n"
    "P_ds_2d   = beamform_ds(R_2d, H_2d).reshape(len(az_2d), len(el_2d))\n"
    "P_mvdr_2d = beamform_mvdr(R_2d, H_2d).reshape(len(az_2d), len(el_2d))\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(11, 5))\n"
    "for ax, P, title in zip(axes, [P_ds_2d, P_mvdr_2d], ['D&S', 'MVDR']):\n"
    "    P_db = 10 * np.log10(P / P.max() + 1e-10)\n"
    "    im = ax.imshow(P_db.T, origin='lower', aspect='equal',\n"
    "                   extent=[az_2d[0], az_2d[-1], el_2d[0], el_2d[-1]],\n"
    "                   cmap='inferno', vmin=-30, vmax=0)\n"
    "    plt.colorbar(im, ax=ax, label='dB')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Elevation (deg)')\n"
    "    ax.set_title(f'{title}  @  {FREQ_2D:.0f} Hz')\n"
    "\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_psf_2d.png', dpi=150)\n"
    "plt.show()\n",
    "ab150071",
))

# ---------------------------------------------------------------------------
# Section 8: Phase comparison table
# ---------------------------------------------------------------------------
cells.append(md(
    "## 8  Phase Comparison\n"
    "\n"
    "| Parameter | Phase 2 (ReSpeaker) | Phase 3 (UMA-16) | Phase 4 target |\n"
    "|---|---|---|---|\n"
    "| Mics | 4 | 16 | 96 |\n"
    "| Aperture | 90 mm | 126 mm × 126 mm | 300 mm |\n"
    "| Sample rate | 16 kHz | 48 kHz | 48 kHz |\n"
    "| Spatial Nyquist | 2.7 kHz | 4.1 kHz | 8+ kHz |\n"
    "| Array gain | ~6 dB | ~12 dB | ~20 dB |\n"
    "| HPBW @ 2 kHz | ~88° | ~78° | ~37° |\n"
    "| HPBW @ 3 kHz | — | ~52° | ~25° |\n",
    "ab150080",
))

cells.append(code(
    "freqs_comp = [1000, 1500, 2000, 2500, 3000, 3500]\n"
    "apertures  = {'Phase 2 (4-mic, 90mm)':  0.090,\n"
    "              'Phase 3 (16-mic, 126mm)': 0.126,\n"
    "              'Phase 4 (96-mic, 300mm)': 0.300}\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(8, 4))\n"
    "for label, D in apertures.items():\n"
    "    hpbw = [np.degrees(C / (f * D)) for f in freqs_comp]\n"
    "    ax.plot(freqs_comp, hpbw, marker='o', label=label)\n"
    "\n"
    "ax.axhline(10, color='gray', ls=':', lw=0.8, label='10° threshold')\n"
    "ax.set_xlabel('Frequency (Hz)')\n"
    "ax.set_ylabel('HPBW (deg, approx λ/D × 57.3°)')\n"
    "ax.set_title('Beamwidth vs frequency — Phase comparison')\n"
    "ax.legend(fontsize=9)\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('uma16_phase_comparison.png', dpi=150)\n"
    "plt.show()\n",
    "ab150081",
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

out = Path(__file__).parent / "15_uma16_array.ipynb"
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}")
