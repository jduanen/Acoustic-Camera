#!/usr/bin/env python3
"""Generate notebooks/10_2d_azimuth_elevation.ipynb"""
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
    "# 10 — 2D Azimuth × Elevation Maps\n"
    "\n"
    "**Goal**: extend beamforming from 1D azimuth to 2D (azimuth × elevation),\n"
    "producing the full hemisphere energy map that an acoustic camera displays.\n"
    "\n"
    "The Underbrink spiral is a 2D planar array in the x-y plane.  It has aperture\n"
    "in both horizontal (x) and vertical (y) directions, so it resolves both\n"
    "azimuth and elevation simultaneously.  For a circularly symmetric spiral,\n"
    "the 2D PSF should be approximately circular — equal resolution in both\n"
    "axes at boresight.\n"
    "\n"
    "**Steering vector** for a far-field source at (az, el):\n"
    "```\n"
    "h_n = exp(j·2π·f/c·(sin(az)cos(el)·x_n + sin(el)·y_n)) / √N\n"
    "```\n"
    "\n"
    "Key questions:\n"
    "1. Is the 2D PSF circularly symmetric?  How does HPBW compare in az vs el?\n"
    "2. Does the 2D map correctly locate sources at off-axis (az, el) positions?\n"
    "3. How do D&S, MVDR, and CLEAN-SC compare on the 2D map?\n"
    "4. What is the usable field of view at each frequency?\n",
    "ee500001",
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
    "SNR_DB = 20.0\n"
    "N_SNAP = 256\n",
    "ee500002",
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
    "ee500003",
))

cells.append(code(
    "def steering_matrix_2d(x, y, az_grid, el_grid, freq):\n"
    "    \"\"\"Returns (N_mics, N_az, N_el).  Far-field 2D steering.\"\"\"\n"
    "    az_rad = np.radians(az_grid)  # (N_az,)\n"
    "    el_rad = np.radians(el_grid)  # (N_el,)\n"
    "    # Direction cosines on (az, el) grid\n"
    "    ux = np.sin(az_rad)[:, np.newaxis] * np.cos(el_rad)[np.newaxis, :]  # (N_az, N_el)\n"
    "    uy = np.ones(len(az_rad))[:, np.newaxis] * np.sin(el_rad)[np.newaxis, :]  # (N_az, N_el)\n"
    "    # Phase per mic: (N, N_az, N_el)\n"
    "    phase = 2 * np.pi * freq / C * (\n"
    "        x[:, np.newaxis, np.newaxis] * ux[np.newaxis] +\n"
    "        y[:, np.newaxis, np.newaxis] * uy[np.newaxis]\n"
    "    )\n"
    "    return np.exp(1j * phase) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def make_csm_2d(x, y, sources_ae, freq, snr_db, n_snap, seed=None):\n"
    "    \"\"\"sources_ae: list of (az_deg, el_deg, power) tuples.\"\"\"\n"
    "    local_rng = np.random.default_rng(seed)\n"
    "    N = len(x)\n"
    "    max_p     = max(p for _, _, p in sources_ae)\n"
    "    noise_var = max_p / (10 ** (snr_db / 10))\n"
    "    R = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        y_snap = np.zeros(N, dtype=complex)\n"
    "        for az, el, power in sources_ae:\n"
    "            ux = np.sin(np.radians(az)) * np.cos(np.radians(el))\n"
    "            uy = np.sin(np.radians(el))\n"
    "            h  = np.exp(1j * 2 * np.pi * freq / C * (ux * x + uy * y)) / np.sqrt(N)\n"
    "            s  = local_rng.standard_normal() + 1j * local_rng.standard_normal()\n"
    "            y_snap += np.sqrt(power / 2) * h * s\n"
    "        noise  = local_rng.standard_normal(N) + 1j * local_rng.standard_normal(N)\n"
    "        y_snap += np.sqrt(noise_var / 2) * noise\n"
    "        R += np.outer(y_snap, y_snap.conj())\n"
    "    return R / n_snap\n"
    "\n"
    "\n"
    "def psf_2d(x, y, freq, az_grid, el_grid, az0=0.0, el0=0.0):\n"
    "    \"\"\"Analytic 2D beam pattern for a source at (az0, el0).\"\"\"\n"
    "    H = steering_matrix_2d(x, y, az_grid, el_grid, freq)   # (N, N_az, N_el)\n"
    "    N, N_az, N_el = H.shape\n"
    "    ux0 = np.sin(np.radians(az0)) * np.cos(np.radians(el0))\n"
    "    uy0 = np.sin(np.radians(el0))\n"
    "    h0  = np.exp(1j * 2 * np.pi * freq / C * (ux0 * x + uy0 * y)) / np.sqrt(N)\n"
    "    psf = np.abs(H.reshape(N, -1).conj().T @ h0) ** 2  # (N_az*N_el,)\n"
    "    return psf.reshape(N_az, N_el)\n"
    "\n"
    "\n"
    "def beamform_ds_2d(x, y, R, freq, az_grid, el_grid):\n"
    "    H = steering_matrix_2d(x, y, az_grid, el_grid, freq)\n"
    "    N, N_az, N_el = H.shape\n"
    "    H_flat = H.reshape(N, -1)\n"
    "    P = np.real(np.sum(H_flat.conj() * (R @ H_flat), axis=0))\n"
    "    return P.reshape(N_az, N_el)\n"
    "\n"
    "\n"
    "def beamform_mvdr_2d(x, y, R, freq, az_grid, el_grid, diag_load=0.01):\n"
    "    N = R.shape[0]\n"
    "    R_loaded = R + diag_load * np.trace(R) / N * np.eye(N)\n"
    "    R_inv = inv(R_loaded)\n"
    "    H = steering_matrix_2d(x, y, az_grid, el_grid, freq)\n"
    "    H_flat = H.reshape(N, -1)\n"
    "    denom = np.real(np.sum(H_flat.conj() * (R_inv @ H_flat), axis=0))\n"
    "    return (1.0 / np.maximum(denom, 1e-300)).reshape(len(az_grid), len(el_grid))\n"
    "\n"
    "\n"
    "def clean_sc_2d(x, y, R, freq, az_grid, el_grid, n_iter=40, loop_gain=0.5):\n"
    "    H = steering_matrix_2d(x, y, az_grid, el_grid, freq)\n"
    "    N, N_az, N_el = H.shape\n"
    "    H_flat = H.reshape(N, -1)\n"
    "    R_w    = R.copy()\n"
    "    clean  = np.zeros(N_az * N_el)\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H_flat.conj() * (R_w @ H_flat), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        g = R_w @ H_flat[:, k]\n"
    "        R_w -= loop_gain * np.outer(g, g.conj())\n"
    "        clean[k] += loop_gain * np.real(g.conj() @ g)\n"
    "    return clean.reshape(N_az, N_el)\n"
    "\n"
    "\n"
    "def db_norm(P):\n"
    "    return 10 * np.log10(np.maximum(P / P.max(), 1e-10))\n"
    "\n"
    "\n"
    "def measure_hpbw(grid, profile):\n"
    "    \"\"\"Half-power beamwidth from a 1-D normalised power profile.\"\"\"\n"
    "    half = profile.max() / 2\n"
    "    above = np.where(profile >= half)[0]\n"
    "    if len(above) < 2:\n"
    "        return None\n"
    "    return grid[above[-1]] - grid[above[0]]\n"
    "\n"
    "\n"
    "# Scan grid: 91 pts in each dimension, 1.5° step\n"
    "az_grid = np.linspace(-60, 60, 91)\n"
    "el_grid = np.linspace(-60, 60, 91)\n"
    "print(f'2D grid: {len(az_grid)} × {len(el_grid)} = {len(az_grid)*len(el_grid)} pts')\n",
    "ee500004",
))

# ---------------------------------------------------------------------------
# Section 1: 2D PSF at multiple frequencies
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — 2D Beam Pattern (PSF) at 1, 4, 8 kHz\n"
    "\n"
    "Analytic PSF for a point source at boresight (az=0°, el=0°).  The Underbrink\n"
    "spiral is symmetric about the origin, so the PSF should be approximately\n"
    "circular — equal HPBW in azimuth and elevation.\n",
    "ee500005",
))

cells.append(code(
    "freqs    = [1000, 4000, 8000]\n"
    "fig, axes = plt.subplots(1, 3, figsize=(15, 5))\n"
    "extent   = [az_grid[0], az_grid[-1], el_grid[-1], el_grid[0]]\n"
    "\n"
    "hpbw_table = []\n"
    "for ax, freq in zip(axes, freqs):\n"
    "    P = psf_2d(x_arr, y_arr, freq, az_grid, el_grid)\n"
    "    P_db = db_norm(P)\n"
    "    im = ax.imshow(P_db.T, extent=extent, aspect='equal',\n"
    "                   vmin=-20, vmax=0, cmap='hot_r', origin='upper')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Elevation (deg)')\n"
    "    ax.set_title(f'{freq/1000:.0f} kHz')\n"
    "    plt.colorbar(im, ax=ax, label='dB', fraction=0.046)\n"
    "\n"
    "    # HPBW slices\n"
    "    el0_idx = np.argmin(np.abs(el_grid))\n"
    "    az0_idx = np.argmin(np.abs(az_grid))\n"
    "    hpbw_az = measure_hpbw(az_grid, P[:, el0_idx])\n"
    "    hpbw_el = measure_hpbw(el_grid, P[az0_idx, :])\n"
    "    hpbw_table.append({'Freq (Hz)': freq,\n"
    "                        'Az HPBW (°)': round(hpbw_az, 1) if hpbw_az else '>120',\n"
    "                        'El HPBW (°)': round(hpbw_el, 1) if hpbw_el else '>120'})\n"
    "    label = f'Az {hpbw_az:.0f}° / El {hpbw_el:.0f}°' if hpbw_az else 'Omnidirectional'\n"
    "    ax.set_title(f'{freq/1000:.0f} kHz  —  {label}')\n"
    "\n"
    "plt.suptitle('2D PSF: Underbrink H=12×8, α=22°, 300mm aperture (source at boresight)')\n"
    "plt.tight_layout()\n"
    "plt.savefig('2d_psf_freq.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "df_hpbw = pd.DataFrame(hpbw_table).set_index('Freq (Hz)')\n"
    "print(df_hpbw.to_string())\n",
    "ee500006",
))

# ---------------------------------------------------------------------------
# Section 2: algorithm comparison on two off-axis sources
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Algorithm Comparison: D&S vs MVDR vs CLEAN-SC\n"
    "\n"
    "Two equal-power sources: (az=−20°, el=+10°) and (az=+15°, el=−15°).\n"
    "SNR=20 dB, N_SNAP=256, f=4 kHz.\n"
    "True source positions marked with cyan crosses.\n",
    "ee500007",
))

cells.append(code(
    "FREQ     = 4000.0\n"
    "sources  = [(-20.0, 10.0, 1.0), (15.0, -15.0, 1.0)]\n"
    "R_src    = make_csm_2d(x_arr, y_arr, sources, FREQ, SNR_DB, N_SNAP, seed=10)\n"
    "\n"
    "P_ds    = beamform_ds_2d(x_arr, y_arr, R_src, FREQ, az_grid, el_grid)\n"
    "P_mv    = beamform_mvdr_2d(x_arr, y_arr, R_src, FREQ, az_grid, el_grid)\n"
    "P_cl    = clean_sc_2d(x_arr, y_arr, R_src, FREQ, az_grid, el_grid)\n"
    "\n"
    "fig, axes = plt.subplots(1, 3, figsize=(16, 5))\n"
    "extent = [az_grid[0], az_grid[-1], el_grid[-1], el_grid[0]]\n"
    "\n"
    "for ax, P, title in [\n"
    "    (axes[0], P_ds, 'D&S'),\n"
    "    (axes[1], P_mv, 'MVDR'),\n"
    "    (axes[2], P_cl, 'CLEAN-SC'),\n"
    "]:\n"
    "    im = ax.imshow(db_norm(P).T, extent=extent, aspect='equal',\n"
    "                   vmin=-20, vmax=0, cmap='hot_r', origin='upper')\n"
    "    for az_t, el_t, _ in sources:\n"
    "        ax.plot(az_t, el_t, 'c+', markersize=14, markeredgewidth=2)\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Elevation (deg)')\n"
    "    ax.set_title(title)\n"
    "    plt.colorbar(im, ax=ax, label='dB', fraction=0.046)\n"
    "\n"
    "plt.suptitle(f'2D energy map: 2 sources at (−20°,+10°) and (+15°,−15°), '\n"
    "             f'{FREQ/1000:.0f} kHz, {SNR_DB:.0f} dB SNR')\n"
    "plt.tight_layout()\n"
    "plt.savefig('2d_algo_comparison.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Report peak locations\n"
    "for P, lbl in [(P_ds, 'D&S'), (P_mv, 'MVDR'), (P_cl, 'CLEAN-SC')]:\n"
    "    idx = np.unravel_index(np.argmax(P), P.shape)\n"
    "    print(f'{lbl}: strongest peak at az={az_grid[idx[0]]:.1f}°, el={el_grid[idx[1]]:.1f}°')\n",
    "ee500008",
))

# ---------------------------------------------------------------------------
# Section 3: az vs el symmetry across frequencies
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Azimuth vs Elevation HPBW Symmetry\n"
    "\n"
    "For a circularly symmetric array, HPBW should be equal in azimuth and elevation\n"
    "at boresight.  Measure both at each octave frequency to verify.\n",
    "ee500009",
))

cells.append(code(
    "freq_list = [500, 1000, 2000, 4000, 8000]\n"
    "rows = []\n"
    "for freq in freq_list:\n"
    "    P = psf_2d(x_arr, y_arr, freq, az_grid, el_grid)\n"
    "    el0 = np.argmin(np.abs(el_grid))\n"
    "    az0 = np.argmin(np.abs(az_grid))\n"
    "    haz = measure_hpbw(az_grid, P[:, el0])\n"
    "    hel = measure_hpbw(el_grid, P[az0, :])\n"
    "    rows.append({\n"
    "        'Freq (Hz)': freq,\n"
    "        'Az HPBW (°)': round(haz, 1) if haz else '>120',\n"
    "        'El HPBW (°)': round(hel, 1) if hel else '>120',\n"
    "        'Az/El ratio': round(haz / hel, 3) if (haz and hel) else 'n/a',\n"
    "    })\n"
    "\n"
    "df_sym = pd.DataFrame(rows).set_index('Freq (Hz)')\n"
    "print(df_sym.to_string())\n"
    "print()\n"
    "print('Az/El ratio = 1.0 indicates perfect circular symmetry.')\n",
    "ee500010",
))

# ---------------------------------------------------------------------------
# Section 4: source localisation accuracy across the FOV
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Source Localisation Accuracy Across the FOV\n"
    "\n"
    "Single source at five positions spanning the FOV.  Report D&S and CLEAN-SC\n"
    "peak locations vs true positions at 4 kHz.\n",
    "ee500011",
))

cells.append(code(
    "test_sources = [\n"
    "    ( 0.0,   0.0, 'Boresight'),\n"
    "    (25.0,   0.0, 'Az=25°, El=0°'),\n"
    "    ( 0.0,  25.0, 'Az=0°, El=25°'),\n"
    "    (30.0,  20.0, 'Az=30°, El=20°'),\n"
    "    (-40.0, -15.0, 'Az=−40°, El=−15°'),\n"
    "]\n"
    "\n"
    "rows_loc = []\n"
    "for i, (az_t, el_t, label) in enumerate(test_sources):\n"
    "    R = make_csm_2d(x_arr, y_arr, [(az_t, el_t, 1.0)], FREQ, SNR_DB, N_SNAP,\n"
    "                    seed=i * 100 + 1)\n"
    "    for alg, fn in [('D&S', beamform_ds_2d), ('CLEAN-SC', clean_sc_2d)]:\n"
    "        P = fn(x_arr, y_arr, R, FREQ, az_grid, el_grid)\n"
    "        idx = np.unravel_index(np.argmax(P), P.shape)\n"
    "        az_est, el_est = az_grid[idx[0]], el_grid[idx[1]]\n"
    "        rows_loc.append({\n"
    "            'Position': label,\n"
    "            'Algorithm': alg,\n"
    "            'Az error (°)': round(abs(az_est - az_t), 1),\n"
    "            'El error (°)': round(abs(el_est - el_t), 1),\n"
    "            'Total error (°)': round(np.sqrt((az_est-az_t)**2 + (el_est-el_t)**2), 2),\n"
    "        })\n"
    "\n"
    "df_loc = pd.DataFrame(rows_loc)\n"
    "print(df_loc.to_string(index=False))\n",
    "ee500012",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "ee500013",
))

cells.append(code(
    "print('=== 2D PSF HPBW (azimuth and elevation) ===')\n"
    "print(df_sym.to_string())\n"
    "print()\n"
    "print('=== Source localisation across the FOV (4 kHz, 20 dB SNR) ===')\n"
    "print(df_loc.to_string(index=False))\n",
    "ee500014",
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

out = Path("10_2d_azimuth_elevation.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
