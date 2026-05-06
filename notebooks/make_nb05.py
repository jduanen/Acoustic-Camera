#!/usr/bin/env python3
"""Generate notebooks/05_broadband.ipynb"""
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
    "# 05 — Broadband / Frequency-Swept Beamforming\n"
    "\n"
    "**Goal**: characterise algorithm performance across the full operating\n"
    "range (200 Hz – 8 kHz) using incoherent octave-band averaging — the\n"
    "standard output mode for commercial acoustic cameras.\n"
    "\n"
    "Key questions:\n"
    "1. How does HPBW scale with frequency for the chosen Underbrink array?\n"
    "2. Does incoherent octave-band averaging produce stable broadband localisation?\n"
    "3. Are algorithm rankings (D&S vs MVDR vs MUSIC) consistent across bands?\n"
    "4. Where does spatial aliasing appear, and how does it manifest?\n",
    "aa000001",
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
    "C      = 343.0\n"
    "R_MIN  = 0.025\n"
    "R_MAX  = 0.150\n"
    "N_SNAP = 256\n"
    "rng    = np.random.default_rng(42)\n",
    "aa000002",
))

# ---------------------------------------------------------------------------
# Array geometry
# ---------------------------------------------------------------------------
cells.append(code(
    "def underbrink_array(n_arms, n_per_arm, r_min=R_MIN, r_max=R_MAX, spiral_angle_deg=22.0):\n"
    "    # Equal arc-length Underbrink multi-arm log-spiral\n"
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
    "x_arr, y_arr = underbrink_array(12, 8)   # H=12 x 8, primary candidate\n"
    "d_min = np.min([np.sort(np.sqrt((x_arr[i]-x_arr)**2 + (y_arr[i]-y_arr)**2))[1]\n"
    "                for i in range(len(x_arr))])\n"
    "f_nyquist = C / (2.0 * d_min)\n"
    "print(f'N_MICS = {len(x_arr)}')\n"
    "print(f'Min spacing = {d_min*1e3:.1f} mm')\n"
    "print(f'Spatial Nyquist = {f_nyquist:.0f} Hz')\n",
    "aa000003",
))

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
cells.append(code(
    "def steering_vector(x, y, az_deg, freq):\n"
    "    u = np.sin(np.radians(az_deg))\n"
    "    phase = 2 * np.pi * freq / C * u * x\n"
    "    h = np.exp(1j * phase) / np.sqrt(len(x))\n"
    "    return h\n"
    "\n"
    "\n"
    "def make_csm(x, y, sources, freq, snr_db=20, n_snap=N_SNAP):\n"
    "    # sources: list of (az_deg, power_linear) pairs\n"
    "    N = len(x)\n"
    "    max_power = max(p for _, p in sources)\n"
    "    noise_var = max_power / (10 ** (snr_db / 10))\n"
    "    R = np.zeros((N, N), dtype=complex)\n"
    "    for _ in range(n_snap):\n"
    "        y_snap = np.zeros(N, dtype=complex)\n"
    "        for az, power in sources:\n"
    "            h = steering_vector(x, y, az, freq)\n"
    "            s = rng.standard_normal() + 1j * rng.standard_normal()\n"
    "            y_snap += np.sqrt(power / 2) * h * s\n"
    "        noise = rng.standard_normal(N) + 1j * rng.standard_normal(N)\n"
    "        y_snap += np.sqrt(noise_var / 2) * noise\n"
    "        R += np.outer(y_snap, y_snap.conj())\n"
    "    return R / n_snap\n"
    "\n"
    "\n"
    "def beamform_ds(x, y, R, freq, az_grid):\n"
    "    H = np.array([steering_vector(x, y, az, freq) for az in az_grid]).T  # (N, n_az)\n"
    "    return np.real(np.sum(H.conj() * (R @ H), axis=0))\n"
    "\n"
    "\n"
    "def beamform_mvdr(x, y, R, freq, az_grid, diag_load=0.01):\n"
    "    N = R.shape[0]\n"
    "    R_loaded = R + diag_load * np.trace(R) / N * np.eye(N)\n"
    "    R_inv = inv(R_loaded)\n"
    "    out = np.zeros(len(az_grid))\n"
    "    for i, az in enumerate(az_grid):\n"
    "        h = steering_vector(x, y, az, freq)\n"
    "        denom = np.real(h.conj() @ R_inv @ h)\n"
    "        out[i] = 1.0 / denom if denom > 0 else 0.0\n"
    "    return out\n"
    "\n"
    "\n"
    "def beamform_music(x, y, R, freq, az_grid, n_sources=1):\n"
    "    _, eigvecs = np.linalg.eigh(R)\n"
    "    E_n = eigvecs[:, :R.shape[0] - n_sources]\n"
    "    En_proj = E_n @ E_n.conj().T\n"
    "    out = np.zeros(len(az_grid))\n"
    "    for i, az in enumerate(az_grid):\n"
    "        h = steering_vector(x, y, az, freq)\n"
    "        denom = np.real(h.conj() @ En_proj @ h)\n"
    "        out[i] = 1.0 / max(denom, 1e-300)\n"
    "    return out\n"
    "\n"
    "\n"
    "def measure_hpbw(az_grid, beam_map):\n"
    "    # Half-power beamwidth: width at -3dB from peak\n"
    "    peak = beam_map.max()\n"
    "    half = peak / 2.0\n"
    "    above = az_grid[beam_map >= half]\n"
    "    if len(above) < 2:\n"
    "        return np.nan\n"
    "    return above[-1] - above[0]\n"
    "\n"
    "\n"
    "def db_norm(x):\n"
    "    # normalise map to dB re peak\n"
    "    x = np.clip(x, 1e-30, None)\n"
    "    return 10 * np.log10(x / x.max())\n",
    "aa000004",
))

# ---------------------------------------------------------------------------
# Section 1: HPBW vs frequency
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — HPBW vs Frequency\n"
    "\n"
    "Resolution (HPBW) is proportional to λ/D.  For our 300 mm-diameter array:\n"
    "\n"
    "    HPBW ≈ 0.886 · c / (f · D)\n"
    "\n"
    "This section measures HPBW empirically across the 200 Hz – 16 kHz range\n"
    "(including beyond our 8 kHz target to show where aliasing begins).\n",
    "aa000005",
))

cells.append(code(
    "az_grid = np.linspace(-90, 90, 3601)\n"
    "freqs_sweep = np.array([200, 500, 1000, 2000, 4000, 8000, 11000, 14000])\n"
    "\n"
    "# Single source at 0 deg, high SNR\n"
    "src_boresight = [(0.0, 1.0)]\n"
    "\n"
    "hpbw_measured = []\n"
    "for f in freqs_sweep:\n"
    "    R = make_csm(x_arr, y_arr, src_boresight, f, snr_db=30)\n"
    "    m = beamform_ds(x_arr, y_arr, R, f, az_grid)\n"
    "    hpbw_measured.append(measure_hpbw(az_grid, m))\n"
    "\n"
    "D_aperture = 2 * R_MAX  # 0.3 m diameter\n"
    "hpbw_theory = np.degrees(0.886 * C / (freqs_sweep * D_aperture))\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(8, 4))\n"
    "ax.plot(freqs_sweep / 1e3, hpbw_measured, 'o-', label='Measured (D&S)', color='steelblue')\n"
    "ax.plot(freqs_sweep / 1e3, hpbw_theory, '--', label='Theory: 0.886 λ/D', color='gray')\n"
    "ax.axvline(8, color='tomato', linestyle=':', linewidth=1.5, label='8 kHz target')\n"
    "ax.axvline(f_nyquist / 1e3, color='orange', linestyle=':', linewidth=1.5,\n"
    "           label=f'Nyquist ({f_nyquist/1e3:.1f} kHz)')\n"
    "ax.set_xlabel('Frequency (kHz)')\n"
    "ax.set_ylabel('HPBW (degrees)')\n"
    "ax.set_title('D&S Half-Power Beamwidth vs Frequency — Underbrink H=12x8, D=300mm')\n"
    "ax.legend(fontsize=9)\n"
    "ax.grid(True, alpha=0.3)\n"
    "ax.set_ylim(0, None)\n"
    "plt.tight_layout()\n"
    "plt.savefig('bb_hpbw_vs_freq.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('Frequency (Hz)  HPBW (meas)  HPBW (theory)')\n"
    "for f, hm, ht in zip(freqs_sweep, hpbw_measured, hpbw_theory):\n"
    "    print(f'  {f:>6}         {hm:>6.1f} deg     {ht:>5.1f} deg')\n",
    "aa000006",
))

# ---------------------------------------------------------------------------
# Section 2: Incoherent broadband averaging
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Incoherent Octave-Band Averaging\n"
    "\n"
    "A real acoustic camera produces one map per octave (or 1/3-octave) band,\n"
    "not a single-frequency map.  The map is built by:\n"
    "\n"
    "1. Splitting the audio into frequency bins (FFT)\n"
    "2. Estimating the CSM at each bin: `R(f)`\n"
    "3. Running the beamformer at each bin: `P(θ, f)`\n"
    "4. Averaging power incoherently across the band: `P_band(θ) = mean_f P(θ, f)`\n"
    "\n"
    "Incoherent averaging suppresses noise (√K improvement with K bins) and\n"
    "produces a PSF that is the average of the frequency-dependent PSFs — broader\n"
    "at low frequencies, narrower at high.  The net effect narrows with the upper\n"
    "edge of the band.\n",
    "aa000007",
))

cells.append(code(
    "# Standard octave bands (ISO 266), center frequencies in Hz\n"
    "OCTAVE_CENTERS = np.array([250, 500, 1000, 2000, 4000, 8000])\n"
    "\n"
    "# 5 log-spaced frequencies per band (within ±0.5 octave of centre)\n"
    "def octave_freqs(f_center, n_per_band=5):\n"
    "    return np.geomspace(f_center / np.sqrt(2), f_center * np.sqrt(2), n_per_band)\n"
    "\n"
    "\n"
    "def broadband_map_ds(x, y, sources, f_center, snr_db=20, n_snap=N_SNAP, n_per_band=5):\n"
    "    # Incoherently average D&S maps across n_per_band frequencies in one octave band\n"
    "    freqs = octave_freqs(f_center, n_per_band)\n"
    "    az = np.linspace(-60, 60, 1201)\n"
    "    maps = [beamform_ds(x, y, make_csm(x, y, sources, f, snr_db, n_snap), f, az)\n"
    "            for f in freqs]\n"
    "    return az, np.mean(maps, axis=0)\n"
    "\n"
    "\n"
    "# Single broadband source at 25 degrees\n"
    "src_bb = [(25.0, 1.0)]\n"
    "\n"
    "fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)\n"
    "az_plot = np.linspace(-60, 60, 1201)\n"
    "\n"
    "for ax, fc in zip(axes.flat, OCTAVE_CENTERS):\n"
    "    az_out, map_out = broadband_map_ds(x_arr, y_arr, src_bb, fc, snr_db=20)\n"
    "    ax.plot(az_out, db_norm(map_out), color='steelblue', linewidth=1.2)\n"
    "    ax.axvline(25, color='tomato', linestyle='--', linewidth=1, alpha=0.8, label='True az')\n"
    "    ax.axhline(-3, color='gray', linestyle=':', linewidth=0.8)\n"
    "    ax.set_title(f'{fc} Hz band')\n"
    "    ax.set_ylim(-30, 1)\n"
    "    ax.set_ylabel('dB re peak')\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "for ax in axes[1]:\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "\n"
    "fig.suptitle('D&S Octave-Band Maps — Source at 25°', fontsize=12)\n"
    "plt.tight_layout()\n"
    "plt.savefig('bb_octave_maps.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa000008",
))

# ---------------------------------------------------------------------------
# Section 2b: Full-band overlay
# ---------------------------------------------------------------------------
cells.append(code(
    "# Overlay all octave maps on one axes to show HPBW narrowing\n"
    "fig, ax = plt.subplots(figsize=(9, 4))\n"
    "colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(OCTAVE_CENTERS)))\n"
    "\n"
    "for fc, color in zip(OCTAVE_CENTERS, colors):\n"
    "    az_out, map_out = broadband_map_ds(x_arr, y_arr, src_bb, fc, snr_db=20)\n"
    "    hpbw = measure_hpbw(az_out, map_out)\n"
    "    ax.plot(az_out, db_norm(map_out), color=color, linewidth=1.2,\n"
    "            label=f'{fc} Hz  (HPBW {hpbw:.1f}deg)')\n"
    "\n"
    "ax.axvline(25, color='tomato', linestyle='--', linewidth=1.5, label='True az')\n"
    "ax.axhline(-3, color='gray', linestyle=':', linewidth=0.8)\n"
    "ax.set_xlabel('Azimuth (deg)')\n"
    "ax.set_ylabel('dB re peak')\n"
    "ax.set_title('Octave-Band Maps — All Bands Overlaid (D&S, source @ 25°)')\n"
    "ax.legend(fontsize=8, loc='lower right')\n"
    "ax.set_ylim(-30, 1)\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('bb_octave_overlay.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa000009",
))

# ---------------------------------------------------------------------------
# Section 3: Algorithm comparison across octave bands
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Algorithm Comparison Across Octave Bands\n"
    "\n"
    "Two sources at −15° and +15° (30° separation).  At low frequencies the\n"
    "HPBW exceeds the source separation — no algorithm can resolve them.  The\n"
    "cross-over frequency where resolution is possible is a fundamental limit of\n"
    "the aperture, not of the algorithm.\n"
    "\n"
    "MVDR and MUSIC are expected to resolve the sources at a lower frequency\n"
    "than D&S (super-resolution behaviour).\n",
    "aa000010",
))

cells.append(code(
    "src_two = [(-15.0, 1.0), (15.0, 1.0)]\n"
    "az_fine = np.linspace(-60, 60, 3601)\n"
    "\n"
    "\n"
    "def is_resolved(az_grid, beam_map, az1=-15.0, az2=15.0, threshold_db=-6):\n"
    "    # Two peaks are 'resolved' if the valley between them is > threshold_db below peak\n"
    "    i1 = np.argmin(np.abs(az_grid - az1))\n"
    "    i2 = np.argmin(np.abs(az_grid - az2))\n"
    "    lo, hi = min(i1, i2), max(i1, i2)\n"
    "    valley = beam_map[lo:hi+1].min()\n"
    "    return 10 * np.log10(valley / beam_map.max()) < threshold_db\n"
    "\n"
    "\n"
    "rows = []\n"
    "for fc in OCTAVE_CENTERS:\n"
    "    R = make_csm(x_arr, y_arr, src_two, fc, snr_db=20)\n"
    "    m_ds    = beamform_ds(x_arr, y_arr, R, fc, az_fine)\n"
    "    m_mvdr  = beamform_mvdr(x_arr, y_arr, R, fc, az_fine)\n"
    "    m_music = beamform_music(x_arr, y_arr, R, fc, az_fine, n_sources=2)\n"
    "    rows.append({\n"
    "        'Band (Hz)': fc,\n"
    "        'HPBW (D&S)': f'{measure_hpbw(az_fine, m_ds):.1f}',\n"
    "        'D&S': 'Yes' if is_resolved(az_fine, m_ds) else 'No',\n"
    "        'MVDR': 'Yes' if is_resolved(az_fine, m_mvdr) else 'No',\n"
    "        'MUSIC': 'Yes' if is_resolved(az_fine, m_music) else 'No',\n"
    "    })\n"
    "\n"
    "df = pd.DataFrame(rows).set_index('Band (Hz)')\n"
    "print(df.to_string())\n",
    "aa000011",
))

cells.append(code(
    "# Visual: show maps at 1 kHz (transitional) and 4 kHz (easy) for all three algorithms\n"
    "fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharey=True)\n"
    "algo_names = ['D&S', 'MVDR', 'MUSIC']\n"
    "\n"
    "for row_idx, fc in enumerate([1000, 4000]):\n"
    "    R = make_csm(x_arr, y_arr, src_two, fc, snr_db=20)\n"
    "    maps = [\n"
    "        beamform_ds(x_arr, y_arr, R, fc, az_fine),\n"
    "        beamform_mvdr(x_arr, y_arr, R, fc, az_fine),\n"
    "        beamform_music(x_arr, y_arr, R, fc, az_fine, n_sources=2),\n"
    "    ]\n"
    "    for col_idx, (m, name) in enumerate(zip(maps, algo_names)):\n"
    "        ax = axes[row_idx, col_idx]\n"
    "        ax.plot(az_fine, db_norm(m), linewidth=1.2)\n"
    "        for az_true in [-15, 15]:\n"
    "            ax.axvline(az_true, color='tomato', linestyle='--', linewidth=1, alpha=0.7)\n"
    "        ax.axhline(-3, color='gray', linestyle=':', linewidth=0.8)\n"
    "        ax.set_title(f'{name} @ {fc} Hz')\n"
    "        ax.set_ylim(-40, 1)\n"
    "        ax.grid(True, alpha=0.3)\n"
    "        if col_idx == 0:\n"
    "            ax.set_ylabel('dB re peak')\n"
    "        if row_idx == 1:\n"
    "            ax.set_xlabel('Azimuth (deg)')\n"
    "\n"
    "fig.suptitle('Two Sources at ±15° — Algorithm Comparison at 1 kHz and 4 kHz', fontsize=11)\n"
    "plt.tight_layout()\n"
    "plt.savefig('bb_algo_vs_freq.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "aa000012",
))

# ---------------------------------------------------------------------------
# Section 4: Spatial aliasing
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Spatial Aliasing\n"
    "\n"
    "Spatial aliasing occurs when the minimum inter-element spacing `d_min`\n"
    "exceeds half a wavelength: `d_min > λ/2`.  For our array:\n"
    "\n"
    "    d_min ≈ 12.9 mm  →  alias-free up to c / (2 · d_min) ≈ 13.3 kHz\n"
    "\n"
    "This is well above our 8 kHz target.  This section:\n"
    "- Confirms clean operation at 8 kHz\n"
    "- Shows what aliasing looks like at 16 kHz and 20 kHz (above Nyquist)\n"
    "- Illustrates that the Underbrink irregular geometry spreads alias energy\n"
    "  across many weak lobes instead of a single strong grating lobe\n",
    "aa000013",
))

cells.append(code(
    "# PSF (single source at 0 deg) at several frequencies including above Nyquist\n"
    "alias_freqs = [4000, 8000, int(f_nyquist), int(f_nyquist * 1.5), int(f_nyquist * 2)]\n"
    "az_full = np.linspace(-90, 90, 3601)\n"
    "\n"
    "fig, axes = plt.subplots(1, len(alias_freqs), figsize=(18, 4), sharey=True)\n"
    "\n"
    "for ax, f in zip(axes, alias_freqs):\n"
    "    R = make_csm(x_arr, y_arr, [(0.0, 1.0)], f, snr_db=40)\n"
    "    m = beamform_ds(x_arr, y_arr, R, f, az_full)\n"
    "    ax.plot(az_full, db_norm(m), linewidth=0.9)\n"
    "    ax.axvline(0, color='tomato', linestyle='--', linewidth=1, alpha=0.7)\n"
    "    ax.axhline(-3, color='gray', linestyle=':', linewidth=0.7)\n"
    "    label = f'{f/1000:.1f} kHz'\n"
    "    if f > f_nyquist:\n"
    "        label += ' (ALIAS)'\n"
    "    ax.set_title(label)\n"
    "    ax.set_ylim(-40, 1)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "\n"
    "axes[0].set_ylabel('dB re peak')\n"
    "fig.suptitle('PSF vs Frequency — Aliasing Above Nyquist (Underbrink H=12x8, d_min=12.9mm)',\n"
    "             fontsize=10)\n"
    "plt.tight_layout()\n"
    "plt.savefig('bb_aliasing.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print(f'Spatial Nyquist: {f_nyquist:.0f} Hz  (d_min = {d_min*1e3:.1f} mm)')\n"
    "print(f'Target max freq: 8000 Hz  (margin: {f_nyquist/8000:.2f}x)')\n",
    "aa000014",
))

# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "aa000015",
))

cells.append(code(
    "rows_hpbw = []\n"
    "for f in OCTAVE_CENTERS:\n"
    "    R = make_csm(x_arr, y_arr, src_boresight, f, snr_db=30)\n"
    "    m = beamform_ds(x_arr, y_arr, R, f, az_grid)\n"
    "    hpbw = measure_hpbw(az_grid, m)\n"
    "    rows_hpbw.append({'Freq (Hz)': f, 'HPBW D&S (deg)': round(hpbw, 1)})\n"
    "\n"
    "df_hpbw = pd.DataFrame(rows_hpbw).set_index('Freq (Hz)')\n"
    "print('=== D&S HPBW vs Octave Band ===')\n"
    "print(df_hpbw.to_string())\n"
    "print()\n"
    "print('=== Two-Source Resolution (±15 deg) ===')\n"
    "print(df.to_string())\n"
    "print()\n"
    "print(f'Spatial Nyquist: {f_nyquist:.0f} Hz  |  8 kHz target margin: {f_nyquist/8000:.1f}x')\n",
    "aa000016",
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

out = Path("05_broadband.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
