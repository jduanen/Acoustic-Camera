#!/usr/bin/env python3
"""Generate notebooks/11_reverberation.ipynb"""
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
    "# 11 — Reverberant / Multipath Environments\n"
    "\n"
    "**Goal**: quantify how room reverberation and specular reflections degrade\n"
    "beamformer performance, and identify which algorithm is most robust.\n"
    "\n"
    "Two physically distinct degradation mechanisms are studied:\n"
    "\n"
    "1. **Diffuse reverberation** (late reverb, >50 ms) — modelled as an\n"
    "   isotropic noise field: signals arriving from all directions with equal\n"
    "   average power.  The CSM becomes\n"
    "   `R = outer(h,h*) + (1/DRR)·R_diffuse + noise·I`\n"
    "   where DRR (Direct-to-Reverberant Ratio) is the key parameter.\n"
    "\n"
    "2. **Coherent specular reflection** (early reflection, <50 ms) — the\n"
    "   source signal arrives via a second path from a different direction.\n"
    "   At a single frequency the CSM has rank 1 with an *effective* steering\n"
    "   vector `h_eff = h_direct + c·h_reflect`, causing a DoA bias.\n"
    "\n"
    "Practical reference points for DRR:\n"
    "- **>20 dB**: near-anechoic / outdoors — negligible reverb\n"
    "- **10–15 dB**: typical office or lab at 1–2 m — moderate reverb\n"
    "- **3–6 dB**: large reverberant room — significant degradation expected\n"
    "- **<0 dB**: highly reverberant (T60 > 1 s) — reverb power exceeds direct\n",
    "ff600001",
))

# ---------------------------------------------------------------------------
# Imports
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
    "R_MIN  = 0.025\n"
    "R_MAX  = 0.150\n"
    "FREQ   = 4000.0\n"
    "SNR_DB = 20.0\n"
    "N_SNAP = 256\n",
    "ff600002",
))

# ---------------------------------------------------------------------------
# Array + helpers
# ---------------------------------------------------------------------------
cells.append(code(
    "def underbrink_array(n_arms, n_per_arm, r_min=R_MIN, r_max=R_MAX, spiral_angle_deg=22.0):\n"
    "    b         = 1.0 / np.tan(np.radians(spiral_angle_deg))\n"
    "    sq        = np.sqrt(1.0 + b**2)\n"
    "    theta_max = np.log(r_max / r_min) / b\n"
    "    S_total   = r_min * sq / b * (np.exp(b * theta_max) - 1.0)\n"
    "    s         = np.linspace(0.0, S_total, n_per_arm)\n"
    "    theta_arm = np.log(1.0 + b * s / (r_min * sq)) / b\n"
    "    r_arm     = r_min * np.exp(b * theta_arm)\n"
    "    xs, ys = [], []\n"
    "    for h in range(n_arms):\n"
    "        offset = h * 2.0 * np.pi / n_arms\n"
    "        t = theta_arm + offset\n"
    "        xs.append(r_arm * np.cos(t))\n"
    "        ys.append(r_arm * np.sin(t))\n"
    "    return np.concatenate(xs), np.concatenate(ys)\n"
    "\n"
    "\n"
    "x_arr, y_arr = underbrink_array(12, 8)\n"
    "N_MICS = len(x_arr)\n"
    "print(f'N_MICS = {N_MICS}')\n",
    "ff600003",
))

cells.append(code(
    "def sv(x, y, az_deg, freq):\n"
    "    u = np.sin(np.radians(az_deg))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * u * x) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def sm(x, y, az_grid, freq):\n"
    "    u = np.sin(np.radians(az_grid))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * np.outer(x, u)) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def make_R_diffuse(x, y, freq, n_pts=120, seed=0):\n"
    "    \"\"\"Isotropic noise CSM: average outer products over uniformly spaced azimuths.\"\"\"\n"
    "    az_pts = np.linspace(-90, 90, n_pts, endpoint=False)\n"
    "    H = sm(x, y, az_pts, freq)   # (N, n_pts)\n"
    "    return H @ H.conj().T / n_pts\n"
    "\n"
    "\n"
    "def make_R_true(x, y, src_az, freq, snr_db, drr_db, R_diff):\n"
    "    \"\"\"Theoretical CSM: direct source + diffuse reverb + sensor noise.\"\"\"\n"
    "    N = len(x)\n"
    "    h = sv(x, y, src_az, freq)\n"
    "    noise_var = 1.0 / (10 ** (snr_db / 10))\n"
    "    drr_lin   = 10 ** (drr_db / 10)\n"
    "    return (np.outer(h, h.conj())\n"
    "            + (1.0 / drr_lin) * R_diff\n"
    "            + noise_var * np.eye(N))\n"
    "\n"
    "\n"
    "def sample_csm(R_true, n_snap, seed=None):\n"
    "    \"\"\"Draw a finite-snapshot CSM from the true covariance via Cholesky sampling.\"\"\"\n"
    "    rng = np.random.default_rng(seed)\n"
    "    N   = R_true.shape[0]\n"
    "    reg = 1e-10 * np.trace(R_true) / N\n"
    "    L   = np.linalg.cholesky(R_true + reg * np.eye(N))\n"
    "    R_s = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        z     = rng.standard_normal(N) + 1j * rng.standard_normal(N)\n"
    "        y_s   = L @ z / np.sqrt(2)\n"
    "        R_s  += np.outer(y_s, y_s.conj())\n"
    "    return R_s / n_snap\n"
    "\n"
    "\n"
    "def beamform_ds(x, y, R, freq, az_grid):\n"
    "    H = sm(x, y, az_grid, freq)\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_mvdr(x, y, R, freq, az_grid, diag_load=0.01):\n"
    "    N = R.shape[0]\n"
    "    Rl = R + diag_load * np.trace(R) / N * np.eye(N)\n"
    "    Ri = inv(Rl)\n"
    "    H  = sm(x, y, az_grid, freq)\n"
    "    d  = np.real(np.sum(H.conj() * (Ri @ H), axis=0))\n"
    "    return 1.0 / np.maximum(d, 1e-300)\n"
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
    "def db_norm(x):\n"
    "    return 10 * np.log10(np.maximum(x / x.max(), 1e-10))\n"
    "\n"
    "\n"
    "az_grid  = np.linspace(-60, 60, 1201)\n"
    "R_diff   = make_R_diffuse(x_arr, y_arr, FREQ)\n"
    "DRR_VALS = [30, 20, 15, 10, 6, 3, 0, -3]\n"
    "colors   = {'D&S': 'steelblue', 'MVDR': 'darkorange', 'MUSIC': 'forestgreen'}\n",
    "ff600004",
))

# ---------------------------------------------------------------------------
# Section 1: DoA error vs DRR
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — DoA Error vs DRR (Diffuse Reverberation, Single Source)\n"
    "\n"
    "Source at az=25°, 4 kHz, 20 dB SNR, N_SNAP=256.\n"
    "For each DRR, generate N_TRIALS independent CSMs (different noise seeds)\n"
    "and measure peak DoA error.\n",
    "ff600005",
))

cells.append(code(
    "AZ_TRUE  = 25.0\n"
    "N_TRIALS = 20\n"
    "\n"
    "results = {alg: {'mean': [], 'std': []} for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for drr in DRR_VALS:\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    R_t  = make_R_true(x_arr, y_arr, AZ_TRUE, FREQ, SNR_DB, drr, R_diff)\n"
    "    for trial in range(N_TRIALS):\n"
    "        R = sample_csm(R_t, N_SNAP, seed=trial * 1000 + int(drr * 10 + 100))\n"
    "        for alg, fn, kw in [\n"
    "            ('D&S',   beamform_ds,    {}),\n"
    "            ('MVDR',  beamform_mvdr,  {}),\n"
    "            ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "        ]:\n"
    "            m   = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            errs[alg].append(abs(az_grid[np.argmax(m)] - AZ_TRUE))\n"
    "    for alg in results:\n"
    "        results[alg]['mean'].append(np.mean(errs[alg]))\n"
    "        results[alg]['std'].append(np.std(errs[alg]))\n"
    "\n"
    "print('DRR sweep complete')\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    for drr, m, s in zip(DRR_VALS, results[alg]['mean'], results[alg]['std']):\n"
    "        print(f'  {alg}  DRR={drr:4d} dB  err={m:.3f}° ± {s:.3f}°')\n",
    "ff600006",
))

cells.append(code(
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    mean = np.array(results[alg]['mean'])\n"
    "    std  = np.array(results[alg]['std'])\n"
    "    ax.plot(DRR_VALS, mean, 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "    ax.fill_between(DRR_VALS, mean - std, mean + std,\n"
    "                    color=colors[alg], alpha=0.15)\n"
    "ax.invert_xaxis()\n"
    "ax.axvline(10, color='gray', linestyle='--', linewidth=1, label='Typical office (10 dB)')\n"
    "ax.set_xlabel('DRR (dB) — decreasing left')\n"
    "ax.set_ylabel('Mean abs DoA error (deg)')\n"
    "ax.set_title(f'DoA Error vs DRR — source @ {AZ_TRUE}°, {FREQ/1000:.0f} kHz, '\n"
    "             f'{SNR_DB:.0f} dB SNR, N_SNAP={N_SNAP}')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('reverb_doa_vs_drr.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "ff600007",
))

# ---------------------------------------------------------------------------
# Section 2: spectral visualisation
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Spectrum at Different DRRs\n"
    "\n"
    "Show the beamformer spectrum for each algorithm at four DRR levels.\n"
    "A clean single-peak spectrum is the goal; sidelobe rise and peak shift\n"
    "indicate reverb degradation.\n",
    "ff600008",
))

cells.append(code(
    "drr_show = [20, 10, 3, -3]\n"
    "fig, axes = plt.subplots(3, 4, figsize=(16, 10), sharey='row')\n"
    "\n"
    "for col, drr in enumerate(drr_show):\n"
    "    R_t = make_R_true(x_arr, y_arr, AZ_TRUE, FREQ, SNR_DB, drr, R_diff)\n"
    "    R   = sample_csm(R_t, N_SNAP, seed=42)\n"
    "    for row, (alg, fn, kw) in enumerate([\n"
    "        ('D&S',   beamform_ds,    {}),\n"
    "        ('MVDR',  beamform_mvdr,  {}),\n"
    "        ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "    ]):\n"
    "        ax = axes[row, col]\n"
    "        m  = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "        ax.plot(az_grid, db_norm(m), color=colors[alg], linewidth=1.2)\n"
    "        ax.axvline(AZ_TRUE, color='k', linestyle='--', linewidth=0.8)\n"
    "        ax.set_ylim(-30, 2)\n"
    "        ax.grid(True, alpha=0.3)\n"
    "        if row == 0:\n"
    "            ax.set_title(f'DRR = {drr} dB')\n"
    "        if col == 0:\n"
    "            ax.set_ylabel(f'{alg}\\ndB')\n"
    "        if row == 2:\n"
    "            ax.set_xlabel('Azimuth (deg)')\n"
    "\n"
    "plt.suptitle(f'Beamformer spectra vs DRR — source @ {AZ_TRUE}°, {FREQ/1000:.0f} kHz')\n"
    "plt.tight_layout()\n"
    "plt.savefig('reverb_spectra.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "ff600009",
))

# ---------------------------------------------------------------------------
# Section 3: resolution under reverberation
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Resolution Reliability vs DRR\n"
    "\n"
    "Two sources at ±15°, 4 kHz, 20 dB SNR, N_SNAP=256.\n"
    "Resolution criterion: −6 dB valley between the two peaks.\n",
    "ff600010",
))

cells.append(code(
    "N_TRIALS_R = 20\n"
    "\n"
    "\n"
    "def is_resolved(az_grid, m, az1=-15.0, az2=15.0, thr=-6):\n"
    "    i1, i2 = np.argmin(np.abs(az_grid - az1)), np.argmin(np.abs(az_grid - az2))\n"
    "    lo, hi = min(i1, i2), max(i1, i2)\n"
    "    valley = m[lo:hi+1].min()\n"
    "    return 10 * np.log10(valley / m.max()) < thr\n"
    "\n"
    "\n"
    "src_two  = [(az, 1.0) for az in [-15.0, 15.0]]\n"
    "\n"
    "res_rate = {alg: [] for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "R_diff2  = make_R_diffuse(x_arr, y_arr, FREQ)\n"
    "\n"
    "for drr in DRR_VALS:\n"
    "    h1  = sv(x_arr, y_arr, -15.0, FREQ)\n"
    "    h2  = sv(x_arr, y_arr,  15.0, FREQ)\n"
    "    drr_lin   = 10 ** (drr / 10)\n"
    "    noise_var = 1.0 / (10 ** (SNR_DB / 10))\n"
    "    R_t = (np.outer(h1, h1.conj()) + np.outer(h2, h2.conj())\n"
    "           + (1.0 / drr_lin) * R_diff2\n"
    "           + noise_var * np.eye(N_MICS))\n"
    "    for alg in res_rate:\n"
    "        resolved = 0\n"
    "        for trial in range(N_TRIALS_R):\n"
    "            R = sample_csm(R_t, N_SNAP, seed=trial * 500 + int(drr * 10 + 200))\n"
    "            if alg == 'D&S':\n"
    "                m = beamform_ds(x_arr, y_arr, R, FREQ, az_grid)\n"
    "            elif alg == 'MVDR':\n"
    "                m = beamform_mvdr(x_arr, y_arr, R, FREQ, az_grid)\n"
    "            else:\n"
    "                m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=2)\n"
    "            if is_resolved(az_grid, m):\n"
    "                resolved += 1\n"
    "        res_rate[alg].append(resolved / N_TRIALS_R)\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    ax.plot(DRR_VALS, res_rate[alg], 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "ax.invert_xaxis()\n"
    "ax.axhline(0.9, color='gray', linestyle=':', linewidth=0.8, label='90% threshold')\n"
    "ax.axvline(10, color='gray', linestyle='--', linewidth=1, label='Typical office')\n"
    "ax.set_xlabel('DRR (dB) — decreasing left')\n"
    "ax.set_ylabel('Fraction of trials resolved')\n"
    "ax.set_title(f'Resolution reliability vs DRR — two sources ±15°, {FREQ/1000:.0f} kHz')\n"
    "ax.set_ylim(-0.05, 1.05)\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('reverb_resolution.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "ff600011",
))

# ---------------------------------------------------------------------------
# Section 4: coherent specular reflection
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Coherent Specular Reflection: Ghost Peaks\n"
    "\n"
    "A single strong early reflection from az=−35° (wall behind the source plane).\n"
    "Three reflection amplitudes tested: 0.3, 0.5, 0.7 (−10.5, −6, −3 dB).\n"
    "Unlike diffuse reverberation, a specular reflection at a fixed direction\n"
    "acts as a coherent second source at a wrong location, creating a ghost peak.\n",
    "ff600012",
))

cells.append(code(
    "REFLECT_AZ  = -35.0\n"
    "AMP_VALS    = [0.0, 0.3, 0.5, 0.7]   # 0 = no reflection\n"
    "AMP_LABELS  = ['No reflection', '0.3 (−10.5 dB)', '0.5 (−6 dB)', '0.7 (−3 dB)']\n"
    "\n"
    "fig, axes = plt.subplots(3, 4, figsize=(16, 9), sharey='row')\n"
    "\n"
    "for col, (amp, lbl) in enumerate(zip(AMP_VALS, AMP_LABELS)):\n"
    "    # Coherent reflection: effective steering vector h_eff = h_direct + amp*h_reflect\n"
    "    h_d   = sv(x_arr, y_arr, AZ_TRUE, FREQ)\n"
    "    h_r   = sv(x_arr, y_arr, REFLECT_AZ, FREQ)\n"
    "    # Random phase per snapshot makes it properly coherent at this freq\n"
    "    h_eff = h_d + amp * h_r\n"
    "    noise_var = 1.0 / (10 ** (SNR_DB / 10))\n"
    "    R_t = np.outer(h_eff, h_eff.conj()) + noise_var * np.eye(N_MICS)\n"
    "    R   = sample_csm(R_t, N_SNAP, seed=99)\n"
    "\n"
    "    for row, (alg, fn, kw) in enumerate([\n"
    "        ('D&S',   beamform_ds,    {}),\n"
    "        ('MVDR',  beamform_mvdr,  {}),\n"
    "        ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "    ]):\n"
    "        ax = axes[row, col]\n"
    "        m  = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "        ax.plot(az_grid, db_norm(m), color=colors[alg], linewidth=1.2)\n"
    "        ax.axvline(AZ_TRUE,      color='k',   linestyle='--', linewidth=0.8)\n"
    "        ax.axvline(REFLECT_AZ,   color='0.6', linestyle=':',  linewidth=0.8)\n"
    "        ax.set_ylim(-30, 2)\n"
    "        ax.grid(True, alpha=0.3)\n"
    "        if row == 0:\n"
    "            ax.set_title(lbl)\n"
    "        if col == 0:\n"
    "            ax.set_ylabel(f'{alg}\\ndB')\n"
    "        if row == 2:\n"
    "            ax.set_xlabel('Azimuth (deg)')\n"
    "        # mark estimated peak\n"
    "        az_est = az_grid[np.argmax(m)]\n"
    "        ax.axvline(az_est, color='r', linestyle='-', linewidth=0.7, alpha=0.6)\n"
    "\n"
    "plt.suptitle(f'Coherent reflection at {REFLECT_AZ}°, source at {AZ_TRUE}°  '\n"
    "             f'(dashed=source, dotted=reflection, red=estimated peak)')\n"
    "plt.tight_layout()\n"
    "plt.savefig('reverb_coherent_ghost.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Report peak shifts\n"
    "print(f'Peak az estimate vs reflection amplitude (true source @ {AZ_TRUE}°):')\n"
    "for amp, lbl in zip(AMP_VALS, AMP_LABELS):\n"
    "    h_eff = sv(x_arr, y_arr, AZ_TRUE, FREQ) + amp * sv(x_arr, y_arr, REFLECT_AZ, FREQ)\n"
    "    R_t   = np.outer(h_eff, h_eff.conj()) + 1e-4 * np.eye(N_MICS)\n"
    "    R     = sample_csm(R_t, N_SNAP, seed=99)\n"
    "    row_out = [lbl]\n"
    "    for alg, fn, kw in [\n"
    "        ('D&S',   beamform_ds,    {}),\n"
    "        ('MVDR',  beamform_mvdr,  {}),\n"
    "        ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "    ]:\n"
    "        m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "        row_out.append(f'{az_grid[np.argmax(m)]:+.1f}°')\n"
    "    print(f'  {lbl:25s}  D&S={row_out[1]}  MVDR={row_out[2]}  MUSIC={row_out[3]}')\n",
    "ff600013",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "ff600014",
))

cells.append(code(
    "print('=== DoA error at key DRR values (single source @ 25°) ===')\n"
    "hdr = f'  {\"DRR\":>8}  {\"D&S\":>10}  {\"MVDR\":>10}  {\"MUSIC\":>10}'\n"
    "print(hdr)\n"
    "for i, drr in enumerate(DRR_VALS):\n"
    "    row = f'  {drr:>5d} dB'\n"
    "    for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "        row += f'  {results[alg][\"mean\"][i]:8.3f}°'\n"
    "    print(row)\n"
    "print()\n"
    "print('=== Resolution reliability at key DRR values (±15° sources) ===')\n"
    "print(hdr.replace('°', ''))\n"
    "for i, drr in enumerate(DRR_VALS):\n"
    "    row = f'  {drr:>5d} dB'\n"
    "    for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "        row += f'  {res_rate[alg][i]:>10.2f}'\n"
    "    print(row)\n",
    "ff600015",
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

out = Path("11_reverberation.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
