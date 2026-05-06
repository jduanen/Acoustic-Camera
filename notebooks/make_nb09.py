#!/usr/bin/env python3
"""Generate notebooks/09_nearfield_cleansc.ipynb"""
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
    "# 09 — Near-Field CLEAN-SC\n"
    "\n"
    "**Goal**: extend CLEAN-SC to spherical-wave (near-field) steering so that\n"
    "the iterative subtraction is coherent with the actual wavefront geometry.\n"
    "\n"
    "**Far-field CLEAN-SC** scans a 1D azimuth grid with plane-wave steering\n"
    "vectors.  It cannot distinguish two sources at the same azimuth but\n"
    "different ranges, and it accrues a DoA bias when the source is closer\n"
    "than the Fraunhofer distance.\n"
    "\n"
    "**Near-field CLEAN-SC** scans a 2D (range × azimuth) grid using spherical-\n"
    "wave steering: `h_n ∝ exp(−j·2πf/c·d_n) / d_n` where `d_n` is the true\n"
    "distance from mic n to the scan point.  The iterative subtraction then\n"
    "removes exactly the contribution of a point source at the estimated location.\n"
    "\n"
    "Key questions:\n"
    "1. Does NF CLEAN-SC correctly localize in 2D (range and azimuth)?\n"
    "2. Can it separate two sources that share an azimuth but differ in range?\n"
    "3. How does DoA error compare to FF CLEAN-SC vs source distance?\n"
    "4. Does the 2D iterative subtraction give cleaner maps than 2D D&S?\n",
    "dd400001",
))

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import pandas as pd\n"
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
    "dd400002",
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
    "print(f'N_MICS = {N_MICS}')\n"
    "\n"
    "# Fraunhofer distance at 4 kHz for 300 mm aperture\n"
    "D_AP  = 2 * R_MAX\n"
    "lam   = C / FREQ\n"
    "r_ff  = 2 * D_AP**2 / lam\n"
    "print(f'Fraunhofer distance at {FREQ/1000:.0f} kHz: {r_ff:.2f} m')\n",
    "dd400003",
))

cells.append(code(
    "# --- Far-field (plane wave) helpers ---\n"
    "def steering_matrix_ff(x, y, az_grid, freq):\n"
    "    u = np.sin(np.radians(az_grid))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * np.outer(x, u)) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "# --- Near-field (spherical wave) helpers ---\n"
    "def steering_matrix_nf(x, y, r_grid, az_grid, freq):\n"
    "    \"\"\"Returns (N_mics, N_r, N_az) — normalised spherical-wave steering.\"\"\"\n"
    "    az_rad = np.radians(az_grid)                   # (N_az,)\n"
    "    src_x  = r_grid[:, np.newaxis] * np.sin(az_rad)  # (N_r, N_az)\n"
    "    src_z  = r_grid[:, np.newaxis] * np.cos(az_rad)  # (N_r, N_az)\n"
    "    dx = src_x[np.newaxis] - x[:, np.newaxis, np.newaxis]   # (N, N_r, N_az)\n"
    "    dy = -y[:, np.newaxis, np.newaxis]                        # source at y=0\n"
    "    dz = src_z[np.newaxis]                                    # (1, N_r, N_az)\n"
    "    dist = np.sqrt(dx**2 + dy**2 + dz**2)\n"
    "    H = np.exp(-1j * 2 * np.pi * freq / C * dist) / dist\n"
    "    return H / np.linalg.norm(H, axis=0, keepdims=True)\n"
    "\n"
    "\n"
    "# --- CSM generator ---\n"
    "def make_csm_nf(x, y, sources_rd, freq, snr_db, n_snap, seed=None):\n"
    "    \"\"\"sources_rd: list of (az_deg, range_m, power) tuples.\"\"\"\n"
    "    local_rng = np.random.default_rng(seed)\n"
    "    N = len(x)\n"
    "    max_p     = max(p for _, _, p in sources_rd)\n"
    "    noise_var = max_p / (10 ** (snr_db / 10))\n"
    "    R = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        y_snap = np.zeros(N, dtype=complex)\n"
    "        for az, r, power in sources_rd:\n"
    "            az_rad = np.radians(az)\n"
    "            src_x  = r * np.sin(az_rad)\n"
    "            src_z  = r * np.cos(az_rad)\n"
    "            dist   = np.sqrt((src_x - x)**2 + y**2 + src_z**2)\n"
    "            h      = np.exp(-1j * 2 * np.pi * freq / C * dist) / dist\n"
    "            h     /= np.linalg.norm(h)\n"
    "            s      = local_rng.standard_normal() + 1j * local_rng.standard_normal()\n"
    "            y_snap += np.sqrt(power / 2) * h * s\n"
    "        noise  = local_rng.standard_normal(N) + 1j * local_rng.standard_normal(N)\n"
    "        y_snap += np.sqrt(noise_var / 2) * noise\n"
    "        R += np.outer(y_snap, y_snap.conj())\n"
    "    return R / n_snap\n"
    "\n"
    "\n"
    "# --- Beamformers ---\n"
    "def beamform_ds_ff(x, y, R, freq, az_grid):\n"
    "    H = steering_matrix_ff(x, y, az_grid, freq)\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_ds_nf(x, y, R, freq, r_grid, az_grid):\n"
    "    H = steering_matrix_nf(x, y, r_grid, az_grid, freq)\n"
    "    N, N_r, N_az = H.shape\n"
    "    H_flat = H.reshape(N, -1)\n"
    "    P = np.real(np.sum(H_flat.conj() * (R @ H_flat), axis=0))\n"
    "    return P.reshape(N_r, N_az)\n"
    "\n"
    "\n"
    "def clean_sc_ff(x, y, R, freq, az_grid, n_iter=40, loop_gain=0.5):\n"
    "    H = steering_matrix_ff(x, y, az_grid, freq)\n"
    "    R_w = R.copy()\n"
    "    clean = np.zeros(len(az_grid))\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        g = R_w @ H[:, k]\n"
    "        R_w -= loop_gain * np.outer(g, g.conj())\n"
    "        clean[k] += loop_gain * np.real(g.conj() @ g)\n"
    "    return clean\n"
    "\n"
    "\n"
    "def clean_sc_nf(x, y, R, freq, r_grid, az_grid, n_iter=40, loop_gain=0.5):\n"
    "    H = steering_matrix_nf(x, y, r_grid, az_grid, freq)\n"
    "    N, N_r, N_az = H.shape\n"
    "    H_flat = H.reshape(N, -1)\n"
    "    R_w = R.copy()\n"
    "    clean = np.zeros(N_r * N_az)\n"
    "    for _ in range(n_iter):\n"
    "        P = np.real(np.sum(H_flat.conj() * (R_w @ H_flat), axis=0))\n"
    "        k = np.argmax(P)\n"
    "        g = R_w @ H_flat[:, k]\n"
    "        R_w -= loop_gain * np.outer(g, g.conj())\n"
    "        clean[k] += loop_gain * np.real(g.conj() @ g)\n"
    "    return clean.reshape(N_r, N_az)\n"
    "\n"
    "\n"
    "def db_norm(x):\n"
    "    return 10 * np.log10(np.maximum(x / x.max(), 1e-10))\n"
    "\n"
    "\n"
    "# Grids used throughout\n"
    "r_grid  = np.linspace(0.4, 4.5, 24)     # 24 range pts, ~0.17 m resolution\n"
    "az_grid = np.linspace(-60, 60, 61)       # 61 az pts,    2° resolution\n"
    "az_fine = np.linspace(-60, 60, 1201)     # fine grid for 1-D slices\n"
    "print(f'2D grid: {len(r_grid)} × {len(az_grid)} = {len(r_grid)*len(az_grid)} pts')\n",
    "dd400004",
))

# ---------------------------------------------------------------------------
# Section 1: 2D map validation — single source
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — 2D Map Validation (Single Source)\n"
    "\n"
    "Single source at r=1.0 m, az=25°, f=4 kHz (near-field: r_FF=2.1 m).\n"
    "Compare 2D NF D&S and 2D NF CLEAN-SC maps.  Expect:\n"
    "- D&S: correct peak location but wide, streaky PSF along the range axis\n"
    "- CLEAN-SC: compact peak, suppressed sidelobes\n",
    "dd400005",
))

cells.append(code(
    "R1 = make_csm_nf(x_arr, y_arr, [(25.0, 1.0, 1.0)], FREQ, SNR_DB, N_SNAP, seed=1)\n"
    "\n"
    "P_ds    = beamform_ds_nf(x_arr, y_arr, R1, FREQ, r_grid, az_grid)\n"
    "P_clean = clean_sc_nf(x_arr, y_arr, R1, FREQ, r_grid, az_grid)\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(13, 5))\n"
    "extent = [az_grid[0], az_grid[-1], r_grid[-1], r_grid[0]]\n"
    "\n"
    "for ax, P, title in [\n"
    "    (axes[0], P_ds,    'NF D&S (2D)'),\n"
    "    (axes[1], P_clean, 'NF CLEAN-SC (2D)'),\n"
    "]:\n"
    "    im = ax.imshow(db_norm(P), extent=extent, aspect='auto',\n"
    "                   vmin=-20, vmax=0, cmap='hot_r', origin='upper')\n"
    "    ax.plot(25.0, 1.0, 'c+', markersize=12, markeredgewidth=2, label='True source')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Range (m)')\n"
    "    ax.set_title(title)\n"
    "    ax.legend(loc='upper right', fontsize=9)\n"
    "    plt.colorbar(im, ax=ax, label='dB')\n"
    "\n"
    "fig.suptitle(f'Single source: r=1.0m, az=25°, {FREQ/1000:.0f} kHz, {SNR_DB:.0f} dB SNR')\n"
    "plt.tight_layout()\n"
    "plt.savefig('nf_cleansc_single.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Report estimated peak location\n"
    "for P, lbl in [(P_ds, 'NF D&S'), (P_clean, 'NF CLEAN-SC')]:\n"
    "    idx = np.unravel_index(np.argmax(P), P.shape)\n"
    "    print(f'{lbl}: peak at r={r_grid[idx[0]]:.2f} m, az={az_grid[idx[1]]:.1f}° '\n"
    "          f'(true: r=1.00m, az=25.0°)')\n",
    "dd400006",
))

# ---------------------------------------------------------------------------
# Section 2: range separation — two co-azimutal sources
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Range Separation: Two Sources at the Same Azimuth\n"
    "\n"
    "Two equal-power sources at az=0°, r=0.8 m and r=2.5 m.\n"
    "Far-field CLEAN-SC sees them as a single merged peak at 0°.\n"
    "Near-field CLEAN-SC 2D resolves them in the range dimension.\n",
    "dd400007",
))

cells.append(code(
    "src_range = [(0.0, 0.8, 1.0), (0.0, 2.5, 1.0)]\n"
    "R2 = make_csm_nf(x_arr, y_arr, src_range, FREQ, SNR_DB, N_SNAP, seed=2)\n"
    "\n"
    "c_ff  = clean_sc_ff(x_arr, y_arr, R2, FREQ, az_fine)\n"
    "P_nf2 = beamform_ds_nf(x_arr, y_arr, R2, FREQ, r_grid, az_grid)\n"
    "c_nf2 = clean_sc_nf(x_arr, y_arr, R2, FREQ, r_grid, az_grid)\n"
    "\n"
    "fig, axes = plt.subplots(1, 3, figsize=(16, 5))\n"
    "\n"
    "# FF CLEAN-SC (1D)\n"
    "axes[0].plot(az_fine, db_norm(c_ff))\n"
    "axes[0].axvline(0.0, color='r', linestyle='--', linewidth=1, label='True az=0°')\n"
    "axes[0].set_xlabel('Azimuth (deg)')\n"
    "axes[0].set_ylabel('dB')\n"
    "axes[0].set_title('FF CLEAN-SC (1D, az only)')\n"
    "axes[0].set_ylim(-25, 2)\n"
    "axes[0].legend(fontsize=9)\n"
    "axes[0].grid(True, alpha=0.3)\n"
    "\n"
    "extent = [az_grid[0], az_grid[-1], r_grid[-1], r_grid[0]]\n"
    "for ax, P, title in [(axes[1], P_nf2, 'NF D&S (2D)'),\n"
    "                      (axes[2], c_nf2, 'NF CLEAN-SC (2D)')]:\n"
    "    im = ax.imshow(db_norm(P), extent=extent, aspect='auto',\n"
    "                   vmin=-20, vmax=0, cmap='hot_r', origin='upper')\n"
    "    ax.plot(0.0, 0.8, 'c+', markersize=12, markeredgewidth=2)\n"
    "    ax.plot(0.0, 2.5, 'c+', markersize=12, markeredgewidth=2, label='True sources')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('Range (m)')\n"
    "    ax.set_title(title)\n"
    "    ax.legend(loc='upper right', fontsize=9)\n"
    "    plt.colorbar(im, ax=ax, label='dB')\n"
    "\n"
    "fig.suptitle('Range separation: 2 sources at az=0°, r=0.8m and r=2.5m')\n"
    "plt.tight_layout()\n"
    "plt.savefig('nf_cleansc_range_sep.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Locate peaks in NF CLEAN-SC map\n"
    "C_sorted = np.argsort(c_nf2.ravel())[::-1]\n"
    "found, seen = [], []\n"
    "for k in C_sorted:\n"
    "    if c_nf2.ravel()[k] <= 0:\n"
    "        break\n"
    "    ir, iaz = np.unravel_index(k, c_nf2.shape)\n"
    "    r_k, az_k = r_grid[ir], az_grid[iaz]\n"
    "    if all(abs(r_k - r_p) > 0.3 or abs(az_k - a_p) > 5 for r_p, a_p in seen):\n"
    "        found.append((r_k, az_k, c_nf2[ir, iaz]))\n"
    "        seen.append((r_k, az_k))\n"
    "    if len(found) >= 3:\n"
    "        break\n"
    "print('NF CLEAN-SC 2D peak locations (range, az):')\n"
    "for r_k, az_k, pw in found:\n"
    "    print(f'  r={r_k:.2f} m, az={az_k:.1f}°  (power={pw:.4f})')\n"
    "print('True: (0.8m, 0°) and (2.5m, 0°)')\n",
    "dd400008",
))

# ---------------------------------------------------------------------------
# Section 3: DoA error vs source distance
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — DoA Error vs Source Distance\n"
    "\n"
    "Single source at az=25°, SNR=20 dB, N_SNAP=256.  Sweep source range from\n"
    "0.5 m (deeply near-field) to 5 m (approaching far-field).\n"
    "\n"
    "Compare:\n"
    "- **FF CLEAN-SC**: 1D azimuth scan, plane-wave steering\n"
    "- **NF CLEAN-SC 2D**: scans (r, az) grid, reports az of strongest clean peak\n"
    "\n"
    "Fraunhofer distance at 4 kHz = 2.1 m.  Expect NF to be better below this,\n"
    "and both to converge above it.\n",
    "dd400009",
))

cells.append(code(
    "range_vals = [0.5, 0.7, 1.0, 1.5, 2.1, 3.0, 5.0]\n"
    "AZ_TRUE    = 25.0\n"
    "\n"
    "err_ff = []\n"
    "err_nf = []\n"
    "err_r  = []  # range error from NF CLEAN-SC\n"
    "\n"
    "for r_src in range_vals:\n"
    "    R = make_csm_nf(x_arr, y_arr, [(AZ_TRUE, r_src, 1.0)],\n"
    "                    FREQ, SNR_DB, N_SNAP, seed=int(r_src*100))\n"
    "\n"
    "    # FF CLEAN-SC\n"
    "    c_ff = clean_sc_ff(x_arr, y_arr, R, FREQ, az_fine)\n"
    "    err_ff.append(abs(az_fine[np.argmax(c_ff)] - AZ_TRUE))\n"
    "\n"
    "    # NF CLEAN-SC 2D\n"
    "    c_nf = clean_sc_nf(x_arr, y_arr, R, FREQ, r_grid, az_grid)\n"
    "    ir, iaz = np.unravel_index(np.argmax(c_nf), c_nf.shape)\n"
    "    err_nf.append(abs(az_grid[iaz] - AZ_TRUE))\n"
    "    err_r.append(abs(r_grid[ir] - r_src))\n"
    "\n"
    "fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))\n"
    "\n"
    "ax1.plot(range_vals, err_ff, 'o-', color='steelblue', label='FF CLEAN-SC', linewidth=1.5)\n"
    "ax1.plot(range_vals, err_nf, 's-', color='darkorange', label='NF CLEAN-SC 2D', linewidth=1.5)\n"
    "ax1.axvline(r_ff, color='gray', linestyle='--', linewidth=1,\n"
    "            label=f'Fraunhofer ({r_ff:.1f} m)')\n"
    "ax1.set_xlabel('Source range (m)')\n"
    "ax1.set_ylabel('DoA error (deg)')\n"
    "ax1.set_title('Azimuth error vs source distance')\n"
    "ax1.legend(fontsize=9)\n"
    "ax1.grid(True, alpha=0.3)\n"
    "\n"
    "ax2.plot(range_vals, err_r, '^-', color='forestgreen', linewidth=1.5)\n"
    "ax2.axvline(r_ff, color='gray', linestyle='--', linewidth=1,\n"
    "            label=f'Fraunhofer ({r_ff:.1f} m)')\n"
    "ax2.set_xlabel('Source range (m)')\n"
    "ax2.set_ylabel('Range error (m)')\n"
    "ax2.set_title('NF CLEAN-SC 2D: range error vs source distance')\n"
    "ax2.legend(fontsize=9)\n"
    "ax2.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle(f'FF vs NF CLEAN-SC — source @ az={AZ_TRUE}°, {SNR_DB:.0f} dB SNR, N_SNAP={N_SNAP}')\n"
    "plt.tight_layout()\n"
    "plt.savefig('nf_cleansc_doa_vs_range.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print(f'{'Range':>7}  {'FF err':>8}  {'NF az err':>10}  {'NF r err':>10}')\n"
    "for r_src, ef, en, er in zip(range_vals, err_ff, err_nf, err_r):\n"
    "    flag = '  <-- NF' if ef > en + 0.05 else ''\n"
    "    print(f'{r_src:7.2f}  {ef:8.3f}°  {en:10.3f}°  {er:10.3f} m{flag}')\n",
    "dd400010",
))

# ---------------------------------------------------------------------------
# Section 4: two sources different range AND azimuth
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — General Two-Source Case (Different Range and Azimuth)\n"
    "\n"
    "Source 1: r=0.8 m, az=−20°.  Source 2: r=2.0 m, az=+12°.\n"
    "NF CLEAN-SC 2D should place two clean peaks at the correct locations.\n"
    "FF CLEAN-SC 1D will show the correct azimuths but merge the range information.\n",
    "dd400011",
))

cells.append(code(
    "src_2d = [(-20.0, 0.8, 1.0), (12.0, 2.0, 1.0)]\n"
    "R4 = make_csm_nf(x_arr, y_arr, src_2d, FREQ, SNR_DB, N_SNAP, seed=4)\n"
    "\n"
    "c4_ff  = clean_sc_ff(x_arr, y_arr, R4, FREQ, az_fine)\n"
    "c4_nf  = clean_sc_nf(x_arr, y_arr, R4, FREQ, r_grid, az_grid)\n"
    "\n"
    "fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))\n"
    "\n"
    "ax1.plot(az_fine, db_norm(c4_ff))\n"
    "for az_t, r_t, _ in src_2d:\n"
    "    ax1.axvline(az_t, color='r', linestyle='--', linewidth=1)\n"
    "ax1.set_xlabel('Azimuth (deg)')\n"
    "ax1.set_ylabel('dB')\n"
    "ax1.set_title('FF CLEAN-SC (1D) — loses range info')\n"
    "ax1.set_ylim(-25, 2)\n"
    "ax1.grid(True, alpha=0.3)\n"
    "\n"
    "extent = [az_grid[0], az_grid[-1], r_grid[-1], r_grid[0]]\n"
    "im = ax2.imshow(db_norm(c4_nf), extent=extent, aspect='auto',\n"
    "                vmin=-20, vmax=0, cmap='hot_r', origin='upper')\n"
    "for az_t, r_t, _ in src_2d:\n"
    "    ax2.plot(az_t, r_t, 'c+', markersize=14, markeredgewidth=2)\n"
    "ax2.set_xlabel('Azimuth (deg)')\n"
    "ax2.set_ylabel('Range (m)')\n"
    "ax2.set_title('NF CLEAN-SC (2D) — recovers range + azimuth')\n"
    "plt.colorbar(im, ax=ax2, label='dB')\n"
    "\n"
    "plt.suptitle('Two sources: (−20°, 0.8m) and (+12°, 2.0m)')\n"
    "plt.tight_layout()\n"
    "plt.savefig('nf_cleansc_twosrc_2d.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "# Report NF CLEAN-SC peak locations\n"
    "C_sorted = np.argsort(c4_nf.ravel())[::-1]\n"
    "found, seen = [], []\n"
    "for k in C_sorted:\n"
    "    if c4_nf.ravel()[k] <= 0:\n"
    "        break\n"
    "    ir, iaz = np.unravel_index(k, c4_nf.shape)\n"
    "    r_k, az_k = r_grid[ir], az_grid[iaz]\n"
    "    if all(abs(r_k - r_p) > 0.3 or abs(az_k - a_p) > 5 for r_p, a_p in seen):\n"
    "        found.append((r_k, az_k, c4_nf[ir, iaz]))\n"
    "        seen.append((r_k, az_k))\n"
    "    if len(found) >= 3:\n"
    "        break\n"
    "print('NF CLEAN-SC 2D top peaks:')\n"
    "for r_k, az_k, pw in found:\n"
    "    print(f'  r={r_k:.2f} m, az={az_k:.1f}°')\n"
    "print('True: (r=0.80m, az=−20°) and (r=2.00m, az=+12°)')\n",
    "dd400012",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "dd400013",
))

cells.append(code(
    "print('=== DoA error table: FF vs NF CLEAN-SC ===')\n"
    "print(f'  Fraunhofer distance at {FREQ/1000:.0f} kHz = {r_ff:.2f} m')\n"
    "print()\n"
    "print(f'  {\"Source range\":>14}  {\"FF az err\":>10}  {\"NF az err\":>10}  {\"NF r err\":>10}')\n"
    "for r_src, ef, en, er in zip(range_vals, err_ff, err_nf, err_r):\n"
    "    tag = '  FF biased' if ef > 0.15 else ''\n"
    "    print(f'  {r_src:14.2f}  {ef:10.3f}°  {en:10.3f}°  {er:10.3f} m{tag}')\n"
    "print()\n"
    "print('=== NF CLEAN-SC 2D grid resolution ===')\n"
    "print(f'  Range: {len(r_grid)} pts from {r_grid[0]:.1f} to {r_grid[-1]:.1f} m  '\n"
    "        f'(step {r_grid[1]-r_grid[0]:.2f} m)')\n"
    "print(f'  Az:    {len(az_grid)} pts from {az_grid[0]:.0f}° to {az_grid[-1]:.0f}°  '\n"
    "        f'(step {az_grid[1]-az_grid[0]:.1f}°)')\n",
    "dd400014",
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

out = Path("09_nearfield_cleansc.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
