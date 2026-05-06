#!/usr/bin/env python3
"""Generate notebooks/06_snapshot_sweep.ipynb"""
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
    "# 06 — Snapshot Count Sweep\n"
    "\n"
    "**Goal**: determine the minimum number of time-domain snapshots needed\n"
    "for reliable CSM estimation, and map that to real-time update rate.\n"
    "\n"
    "The CSM `R = (1/N) Σ y_n y_n^H` converges to its expectation at rate\n"
    "`1/√N_SNAP`.  MVDR inverts `R`; MUSIC decomposes it.  Both degrade badly\n"
    "when `N_SNAP ≪ N_MICS` because the sample CSM is rank-deficient.\n"
    "\n"
    "Key questions:\n"
    "1. What is the minimum N_SNAP for each algorithm at 20 dB SNR?\n"
    "2. How does N_SNAP map to update latency at fs = 48 kHz?\n"
    "3. Does diagonal loading let MVDR operate below the N_MICS threshold?\n",
    "aa100001",
))

# ---------------------------------------------------------------------------
# Imports & constants
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
    "C     = 343.0\n"
    "R_MIN = 0.025\n"
    "R_MAX = 0.150\n"
    "FS    = 48_000\n"
    "rng   = np.random.default_rng(0)\n",
    "aa100002",
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
    "aa100003",
))

cells.append(code(
    "def steering_vector(x, y, az_deg, freq):\n"
    "    u = np.sin(np.radians(az_deg))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * u * x) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def steering_matrix(x, y, az_grid, freq):\n"
    "    # Returns (N_mics, N_az)\n"
    "    u = np.sin(np.radians(az_grid))\n"
    "    phase = 2 * np.pi * freq / C * np.outer(x, u)\n"
    "    return np.exp(1j * phase) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def make_csm(x, y, sources, freq, snr_db, n_snap, seed=None):\n"
    "    local_rng = np.random.default_rng(seed)\n"
    "    N = len(x)\n"
    "    max_power = max(p for _, p in sources)\n"
    "    noise_var = max_power / (10 ** (snr_db / 10))\n"
    "    R = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        y_snap = np.zeros(N, dtype=complex)\n"
    "        for az, power in sources:\n"
    "            h = steering_vector(x, y, az, freq)\n"
    "            s = local_rng.standard_normal() + 1j * local_rng.standard_normal()\n"
    "            y_snap += np.sqrt(power / 2) * h * s\n"
    "        noise = local_rng.standard_normal(N) + 1j * local_rng.standard_normal(N)\n"
    "        y_snap += np.sqrt(noise_var / 2) * noise\n"
    "        R += np.outer(y_snap, y_snap.conj())\n"
    "    return R / n_snap\n"
    "\n"
    "\n"
    "def beamform_ds(x, y, R, freq, az_grid):\n"
    "    H = steering_matrix(x, y, az_grid, freq)\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_mvdr(x, y, R, freq, az_grid, diag_load=0.01):\n"
    "    N = R.shape[0]\n"
    "    R_loaded = R + diag_load * np.trace(R) / N * np.eye(N)\n"
    "    R_inv = inv(R_loaded)\n"
    "    H = steering_matrix(x, y, az_grid, freq)\n"
    "    denom = np.real(np.sum(H.conj() * (R_inv @ H), axis=0))\n"
    "    return 1.0 / np.maximum(denom, 1e-300)\n"
    "\n"
    "\n"
    "def beamform_music(x, y, R, freq, az_grid, n_sources=1):\n"
    "    _, eigvecs = np.linalg.eigh(R)\n"
    "    E_n = eigvecs[:, :R.shape[0] - n_sources]\n"
    "    En_proj = E_n @ E_n.conj().T\n"
    "    H = steering_matrix(x, y, az_grid, freq)\n"
    "    denom = np.real(np.sum(H.conj() * (En_proj @ H), axis=0))\n"
    "    return 1.0 / np.maximum(denom, 1e-300)\n",
    "aa100004",
))

# ---------------------------------------------------------------------------
# Section 1: latency table
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — Snapshot Count → Update Latency\n"
    "\n"
    "At fs = 48 kHz, one snapshot = one complex sample per mic channel.\n"
    "Update latency = N_SNAP / fs.  A real system can overlap windows\n"
    "(50% overlap halves latency), but this table uses non-overlapping windows.\n",
    "aa100005",
))

cells.append(code(
    "snap_vals = [16, 32, 64, 128, 256, 512, 1024, 2048, 4096]\n"
    "\n"
    "rows = []\n"
    "for n in snap_vals:\n"
    "    latency_ms = n / FS * 1000\n"
    "    fps        = FS / n\n"
    "    ratio      = n / N_MICS\n"
    "    rows.append({\n"
    "        'N_SNAP': n,\n"
    "        'Latency (ms)': round(latency_ms, 1),\n"
    "        'Update rate (fps)': round(fps, 1),\n"
    "        'N_SNAP / N_MICS': round(ratio, 2),\n"
    "    })\n"
    "\n"
    "df_lat = pd.DataFrame(rows).set_index('N_SNAP')\n"
    "print(df_lat.to_string())\n"
    "print(f'\\nN_MICS = {N_MICS}  (CSM becomes full-rank at N_SNAP >= N_MICS)')\n",
    "aa100006",
))

# ---------------------------------------------------------------------------
# Section 2: DoA error vs snapshot count (Monte Carlo)
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — DoA Error vs Snapshot Count (Single Source)\n"
    "\n"
    "Monte Carlo: for each N_SNAP, generate `N_TRIALS` independent CSMs\n"
    "(different random seeds), run each beamformer, record peak DoA.\n"
    "Report mean absolute error and standard deviation.\n"
    "\n"
    "Scenario: single source at 25°, 4 kHz, 20 dB SNR.\n",
    "aa100007",
))

cells.append(code(
    "FREQ     = 4000.0\n"
    "SNR_DB   = 20.0\n"
    "AZ_TRUE  = 25.0\n"
    "N_TRIALS = 15\n"
    "\n"
    "az_grid = np.linspace(-60, 60, 1201)\n"
    "src_single = [(AZ_TRUE, 1.0)]\n"
    "\n"
    "results = {alg: {'mean': [], 'std': []} for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for n_snap in snap_vals:\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    for trial in range(N_TRIALS):\n"
    "        R = make_csm(x_arr, y_arr, src_single, FREQ, SNR_DB, n_snap, seed=trial*1000+n_snap)\n"
    "        for alg, fn, kw in [\n"
    "            ('D&S',   beamform_ds,    {}),\n"
    "            ('MVDR',  beamform_mvdr,  {}),\n"
    "            ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "        ]:\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            est = az_grid[np.argmax(m)]\n"
    "            errs[alg].append(abs(est - AZ_TRUE))\n"
    "    for alg in results:\n"
    "        results[alg]['mean'].append(np.mean(errs[alg]))\n"
    "        results[alg]['std'].append(np.std(errs[alg]))\n"
    "\n"
    "print('Snapshot sweep complete')\n",
    "aa100008",
))

cells.append(code(
    "colors = {'D&S': 'steelblue', 'MVDR': 'darkorange', 'MUSIC': 'forestgreen'}\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    mean = np.array(results[alg]['mean'])\n"
    "    std  = np.array(results[alg]['std'])\n"
    "    ax.plot(snap_vals, mean, 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "    ax.fill_between(snap_vals, mean - std, mean + std,\n"
    "                    color=colors[alg], alpha=0.15)\n"
    "\n"
    "ax.axvline(N_MICS, color='gray', linestyle='--', linewidth=1,\n"
    "           label=f'N_SNAP = N_MICS ({N_MICS})')\n"
    "ax.set_xscale('log')\n"
    "ax.set_yscale('log')\n"
    "ax.set_xlabel('N_SNAP (log scale)')\n"
    "ax.set_ylabel('Mean abs DoA error (deg, log scale)')\n"
    "ax.set_title(f'DoA Error vs Snapshot Count — {FREQ/1000:.0f} kHz, {SNR_DB:.0f} dB SNR, '\n"
    "             f'source @ {AZ_TRUE}deg  (N_trials={N_TRIALS})')\n"
    "ax.legend()\n"
    "ax.grid(True, which='both', alpha=0.3)\n"
    "ax.set_xticks(snap_vals)\n"
    "ax.set_xticklabels([str(n) for n in snap_vals], fontsize=8)\n"
    "plt.tight_layout()\n"
    "plt.savefig('snap_doa_error.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa100009",
))

# ---------------------------------------------------------------------------
# Section 3: resolution vs snapshot count (two sources)
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Two-Source Resolution vs Snapshot Count\n"
    "\n"
    "Two sources at ±15° (30° separation), 4 kHz, 20 dB SNR.\n"
    "For each N_SNAP, count what fraction of N_TRIALS CSM realisations\n"
    "yield a resolved map (−6 dB valley criterion).\n",
    "aa100010",
))

cells.append(code(
    "src_two = [(-15.0, 1.0), (15.0, 1.0)]\n"
    "\n"
    "\n"
    "def is_resolved(az_grid, beam_map, az1=-15.0, az2=15.0, threshold_db=-6):\n"
    "    i1 = np.argmin(np.abs(az_grid - az1))\n"
    "    i2 = np.argmin(np.abs(az_grid - az2))\n"
    "    lo, hi = min(i1, i2), max(i1, i2)\n"
    "    valley = beam_map[lo:hi+1].min()\n"
    "    return 10 * np.log10(valley / beam_map.max()) < threshold_db\n"
    "\n"
    "\n"
    "res_frac = {alg: [] for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for n_snap in snap_vals:\n"
    "    counts = {'D&S': 0, 'MVDR': 0, 'MUSIC': 0}\n"
    "    for trial in range(N_TRIALS):\n"
    "        R = make_csm(x_arr, y_arr, src_two, FREQ, SNR_DB, n_snap, seed=trial*1000+n_snap+7)\n"
    "        for alg, fn, kw in [\n"
    "            ('D&S',   beamform_ds,    {}),\n"
    "            ('MVDR',  beamform_mvdr,  {}),\n"
    "            ('MUSIC', beamform_music, {'n_sources': 2}),\n"
    "        ]:\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            if is_resolved(az_grid, m):\n"
    "                counts[alg] += 1\n"
    "    for alg in res_frac:\n"
    "        res_frac[alg].append(counts[alg] / N_TRIALS)\n"
    "\n"
    "print('Resolution sweep complete')\n",
    "aa100011",
))

cells.append(code(
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    ax.plot(snap_vals, res_frac[alg], 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "\n"
    "ax.axvline(N_MICS, color='gray', linestyle='--', linewidth=1,\n"
    "           label=f'N_SNAP = N_MICS ({N_MICS})')\n"
    "ax.axhline(0.9, color='black', linestyle=':', linewidth=0.8, label='90% threshold')\n"
    "ax.set_xscale('log')\n"
    "ax.set_xlabel('N_SNAP (log scale)')\n"
    "ax.set_ylabel('Fraction of trials resolved')\n"
    "ax.set_title(f'Resolution Reliability vs Snapshot Count — two sources at ±15°, {FREQ/1000:.0f} kHz, '\n"
    "             f'{SNR_DB:.0f} dB SNR')\n"
    "ax.legend()\n"
    "ax.set_ylim(-0.05, 1.05)\n"
    "ax.grid(True, alpha=0.3)\n"
    "ax.set_xticks(snap_vals)\n"
    "ax.set_xticklabels([str(n) for n in snap_vals], fontsize=8)\n"
    "plt.tight_layout()\n"
    "plt.savefig('snap_resolution.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa100012",
))

# ---------------------------------------------------------------------------
# Section 4: low-SNR sensitivity
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Effect of SNR on Minimum Snapshot Count\n"
    "\n"
    "Repeat the DoA error sweep at 10 dB SNR (practical lower bound).\n"
    "Lower SNR degrades CSM quality and shifts the convergence point to\n"
    "higher N_SNAP.\n",
    "aa100013",
))

cells.append(code(
    "SNR_LOW = 10.0\n"
    "\n"
    "results_low = {alg: {'mean': [], 'std': []} for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for n_snap in snap_vals:\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    for trial in range(N_TRIALS):\n"
    "        R = make_csm(x_arr, y_arr, src_single, FREQ, SNR_LOW, n_snap, seed=trial*1000+n_snap+99)\n"
    "        for alg, fn, kw in [\n"
    "            ('D&S',   beamform_ds,    {}),\n"
    "            ('MVDR',  beamform_mvdr,  {}),\n"
    "            ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "        ]:\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            est = az_grid[np.argmax(m)]\n"
    "            errs[alg].append(abs(est - AZ_TRUE))\n"
    "    for alg in results_low:\n"
    "        results_low[alg]['mean'].append(np.mean(errs[alg]))\n"
    "        results_low[alg]['std'].append(np.std(errs[alg]))\n"
    "\n"
    "# Side-by-side: 20 dB vs 10 dB\n"
    "fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)\n"
    "\n"
    "for ax, res, snr_label in [(axes[0], results, '20 dB SNR'), (axes[1], results_low, '10 dB SNR')]:\n"
    "    for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "        mean = np.array(res[alg]['mean'])\n"
    "        std  = np.array(res[alg]['std'])\n"
    "        ax.plot(snap_vals, mean, 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "        ax.fill_between(snap_vals, mean - std, mean + std,\n"
    "                        color=colors[alg], alpha=0.15)\n"
    "    ax.axvline(N_MICS, color='gray', linestyle='--', linewidth=1,\n"
    "               label=f'N_SNAP = N_MICS')\n"
    "    ax.set_xscale('log')\n"
    "    ax.set_yscale('log')\n"
    "    ax.set_xlabel('N_SNAP')\n"
    "    ax.set_title(snr_label)\n"
    "    ax.legend(fontsize=9)\n"
    "    ax.grid(True, which='both', alpha=0.3)\n"
    "    ax.set_xticks(snap_vals)\n"
    "    ax.set_xticklabels([str(n) for n in snap_vals], fontsize=7)\n"
    "\n"
    "axes[0].set_ylabel('Mean abs DoA error (deg, log scale)')\n"
    "fig.suptitle('DoA Error vs Snapshot Count — 20 dB vs 10 dB SNR', fontsize=11)\n"
    "plt.tight_layout()\n"
    "plt.savefig('snap_snr_comparison.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa100014",
))

# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "aa100015",
))

cells.append(code(
    "# Find the N_SNAP where mean error drops below 0.5 deg (20 dB SNR)\n"
    "THRESHOLD_DEG = 0.5\n"
    "\n"
    "rows_sum = []\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    mean = np.array(results[alg]['mean'])\n"
    "    converged = [n for n, e in zip(snap_vals, mean) if e < THRESHOLD_DEG]\n"
    "    n_conv = converged[0] if converged else '>4096'\n"
    "    # 90% resolution reliability\n"
    "    rel90 = [n for n, f in zip(snap_vals, res_frac[alg]) if f >= 0.9]\n"
    "    n_res90 = rel90[0] if rel90 else '>4096'\n"
    "    latency_conv = (n_conv / FS * 1000) if isinstance(n_conv, int) else 'n/a'\n"
    "    rows_sum.append({\n"
    "        'Algorithm': alg,\n"
    "        f'N_SNAP for <{THRESHOLD_DEG}deg error': n_conv,\n"
    "        'N_SNAP for 90% resolution': n_res90,\n"
    "        'Latency at convergence (ms)': round(latency_conv, 1) if isinstance(latency_conv, float) else latency_conv,\n"
    "    })\n"
    "\n"
    "df_sum = pd.DataFrame(rows_sum).set_index('Algorithm')\n"
    "print(df_sum.to_string())\n"
    "print()\n"
    "print('Latency table:')\n"
    "print(df_lat.to_string())\n",
    "aa100016",
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

out = Path("06_snapshot_sweep.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
