#!/usr/bin/env python3
"""Generate notebooks/07_music_robustness.ipynb"""
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
    "# 07 — MUSIC Robustness to Wrong Source Count\n"
    "\n"
    "**Goal**: determine how MUSIC degrades when `n_sources` is mis-specified.\n"
    "\n"
    "MUSIC partitions the N-dimensional signal space into a signal subspace\n"
    "(spanned by the k largest eigenvectors) and a noise subspace (the remaining\n"
    "N−k eigenvectors).  The spectrum is the inverse projection of each steering\n"
    "vector onto the noise subspace.  When k is wrong:\n"
    "\n"
    "- **Undercount (n_sources < true k)**: the signal subspace is too small;\n"
    "  some signal energy leaks into the noise subspace.  True source peaks may\n"
    "  weaken or merge.\n"
    "- **Overcount (n_sources > true k)**: the signal subspace is too large;\n"
    "  some noise eigenvectors are absorbed into the signal subspace, reducing\n"
    "  the noise subspace.  Spurious peaks can appear at directions corresponding\n"
    "  to excluded noise eigenvectors.\n"
    "\n"
    "Key questions:\n"
    "1. Does undercount still resolve two sources at typical SNR?\n"
    "2. Does overcount create false detections?\n"
    "3. At what SNR does mismatch become tolerable/intolerable?\n"
    "4. Does higher N_SNAP compensate for mismatch?\n",
    "bb200001",
))

# ---------------------------------------------------------------------------
# Imports & constants
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import pandas as pd\n"
    "from scipy.linalg import inv\n"
    "from scipy.signal import find_peaks\n"
    "\n"
    "plt.rcParams['figure.dpi'] = 120\n"
    "plt.rcParams['font.size'] = 10\n"
    "\n"
    "C      = 343.0\n"
    "R_MIN  = 0.025\n"
    "R_MAX  = 0.150\n"
    "FS     = 48_000\n"
    "FREQ   = 4000.0\n"
    "N_SNAP = 256\n"
    "SNR_DB = 20.0\n",
    "bb200002",
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
    "bb200003",
))

cells.append(code(
    "def steering_vector(x, y, az_deg, freq):\n"
    "    u = np.sin(np.radians(az_deg))\n"
    "    return np.exp(1j * 2 * np.pi * freq / C * u * x) / np.sqrt(len(x))\n"
    "\n"
    "\n"
    "def steering_matrix(x, y, az_grid, freq):\n"
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
    "def beamform_music(x, y, R, freq, az_grid, n_sources=1):\n"
    "    _, eigvecs = np.linalg.eigh(R)\n"
    "    E_n = eigvecs[:, :R.shape[0] - n_sources]\n"
    "    En_proj = E_n @ E_n.conj().T\n"
    "    H = steering_matrix(x, y, az_grid, freq)\n"
    "    denom = np.real(np.sum(H.conj() * (En_proj @ H), axis=0))\n"
    "    return 1.0 / np.maximum(denom, 1e-300)\n"
    "\n"
    "\n"
    "def db_norm(x):\n"
    "    return 10 * np.log10(x / x.max())\n"
    "\n"
    "\n"
    "def detect_peaks(az_grid, music_map, min_rel=0.05, min_sep_deg=5.0):\n"
    "    \"\"\"Return azimuth angles of local maxima above min_rel * global_max.\"\"\"\n"
    "    m_norm = music_map / music_map.max()\n"
    "    min_dist = max(1, int(min_sep_deg / (az_grid[1] - az_grid[0])))\n"
    "    idx, _ = find_peaks(m_norm, height=min_rel, distance=min_dist)\n"
    "    return az_grid[idx]\n"
    "\n"
    "\n"
    "az_grid = np.linspace(-60, 60, 1201)\n",
    "bb200004",
))

# ---------------------------------------------------------------------------
# Section 1: spectrum comparison — undercount and overcount
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — Spectrum Comparison: Under- and Over-count\n"
    "\n"
    "Fix SNR=20dB, N_SNAP=256, single CSM realisation (seed=42).\n"
    "Left block: 2 true sources at ±15°, n_sources varied 1–4.\n"
    "Right block: 1 true source at 20°, n_sources varied 1–4.\n",
    "bb200005",
))

cells.append(code(
    "src_two  = [(-15.0, 1.0), (15.0, 1.0)]\n"
    "src_one  = [(20.0, 1.0)]\n"
    "n_range  = [1, 2, 3, 4]\n"
    "colors_n = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']\n"
    "\n"
    "R2 = make_csm(x_arr, y_arr, src_two, FREQ, SNR_DB, N_SNAP, seed=42)\n"
    "R1 = make_csm(x_arr, y_arr, src_one, FREQ, SNR_DB, N_SNAP, seed=42)\n"
    "\n"
    "fig, axes = plt.subplots(1, 2, figsize=(14, 5))\n"
    "\n"
    "for ax, R, sources, title in [\n"
    "    (axes[0], R2, src_two, 'True k=2 (sources at ±15°)'),\n"
    "    (axes[1], R1, src_one, 'True k=1 (source at 20°)'),\n"
    "]:\n"
    "    for n_spec, col in zip(n_range, colors_n):\n"
    "        m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=n_spec)\n"
    "        ax.plot(az_grid, db_norm(m), color=col, label=f'n_sources={n_spec}',\n"
    "                linewidth=1.4, alpha=0.85)\n"
    "    for az, _ in sources:\n"
    "        ax.axvline(az, color='k', linestyle='--', linewidth=0.8)\n"
    "    ax.axhline(-6, color='gray', linestyle=':', linewidth=0.7, label='−6dB')\n"
    "    ax.set_xlabel('Azimuth (deg)')\n"
    "    ax.set_ylabel('MUSIC spectrum (dB, normalised)')\n"
    "    ax.set_title(title)\n"
    "    ax.set_ylim(-50, 2)\n"
    "    ax.legend(fontsize=9)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "fig.suptitle(f'MUSIC spectrum vs n_sources — {FREQ/1000:.0f} kHz, {SNR_DB:.0f} dB SNR, N_SNAP={N_SNAP}')\n"
    "plt.tight_layout()\n"
    "plt.savefig('music_spectrum_mismatch.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "bb200006",
))

# ---------------------------------------------------------------------------
# Section 2: peak counts — undercount and overcount quantified
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Peak Counts vs n_sources Specification\n"
    "\n"
    "For each CSM, count detected peaks (local maxima above 5% of global max,\n"
    "separated by ≥5°).  Ideal: count equals true source count.\n"
    "\n"
    "Average over N_TRIALS=20 realisations to get stable estimates.\n",
    "bb200007",
))

cells.append(code(
    "N_TRIALS = 20\n"
    "\n"
    "rows = []\n"
    "for true_k, sources, src_label in [\n"
    "    (1, src_one,  '1 source @ 20°'),\n"
    "    (2, src_two,  '2 sources @ ±15°'),\n"
    "]:\n"
    "    for n_spec in [1, 2, 3, 4]:\n"
    "        peak_counts = []\n"
    "        for trial in range(N_TRIALS):\n"
    "            R = make_csm(x_arr, y_arr, sources, FREQ, SNR_DB, N_SNAP,\n"
    "                         seed=trial * 1000 + n_spec)\n"
    "            m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=n_spec)\n"
    "            peaks = detect_peaks(az_grid, m)\n"
    "            peak_counts.append(len(peaks))\n"
    "        rows.append({\n"
    "            'True k': true_k,\n"
    "            'Sources': src_label,\n"
    "            'n_sources specified': n_spec,\n"
    "            'Mismatch': n_spec - true_k,\n"
    "            'Mean peak count': round(np.mean(peak_counts), 2),\n"
    "            'Std': round(np.std(peak_counts), 2),\n"
    "        })\n"
    "\n"
    "df_peaks = pd.DataFrame(rows)\n"
    "print(df_peaks.to_string(index=False))\n",
    "bb200008",
))

# ---------------------------------------------------------------------------
# Section 3: detection rate and false alarm rate vs SNR
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Detection Rate vs SNR for Undercounted MUSIC\n"
    "\n"
    "Scenario: 2 true sources at ±15°, **n_sources=1** (undercount by 1).\n"
    "For each SNR, run N_TRIALS CSM realisations.  Count the fraction where\n"
    "**both** true sources are detected (peak within ±5° of each true direction).\n"
    "Compare with correctly-specified n_sources=2 as reference.\n",
    "bb200009",
))

cells.append(code(
    "SNR_VALS  = [0, 5, 10, 15, 20, 25, 30]\n"
    "TOL_DEG   = 5.0\n"
    "N_TRIALS2 = 30\n"
    "\n"
    "\n"
    "def both_detected(peaks, sources, tol=TOL_DEG):\n"
    "    return all(any(abs(p - az) < tol for p in peaks) for az, _ in sources)\n"
    "\n"
    "\n"
    "rate_under = []   # n_sources=1 (wrong)\n"
    "rate_exact = []   # n_sources=2 (correct)\n"
    "\n"
    "for snr in SNR_VALS:\n"
    "    det_under = det_exact = 0\n"
    "    for trial in range(N_TRIALS2):\n"
    "        R = make_csm(x_arr, y_arr, src_two, FREQ, snr, N_SNAP,\n"
    "                     seed=trial * 1000 + int(snr))\n"
    "        for n_spec, store in [(1, 'under'), (2, 'exact')]:\n"
    "            m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=n_spec)\n"
    "            pks = detect_peaks(az_grid, m)\n"
    "            if both_detected(pks, src_two):\n"
    "                if store == 'under':\n"
    "                    det_under += 1\n"
    "                else:\n"
    "                    det_exact += 1\n"
    "    rate_under.append(det_under / N_TRIALS2)\n"
    "    rate_exact.append(det_exact / N_TRIALS2)\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "ax.plot(SNR_VALS, rate_exact, 'o-', color='forestgreen', label='n_sources=2 (correct)',\n"
    "        linewidth=1.5)\n"
    "ax.plot(SNR_VALS, rate_under, 's--', color='darkorange', label='n_sources=1 (undercount)',\n"
    "        linewidth=1.5)\n"
    "ax.axhline(0.9, color='gray', linestyle=':', linewidth=0.8, label='90% threshold')\n"
    "ax.set_xlabel('SNR (dB)')\n"
    "ax.set_ylabel('Fraction of trials: both sources detected')\n"
    "ax.set_title(f'Detection rate vs SNR — 2 sources at ±15°, N_SNAP={N_SNAP}, N_trials={N_TRIALS2}')\n"
    "ax.set_ylim(-0.05, 1.05)\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('music_detect_vs_snr.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('SNR vs detection rate (undercount n_sources=1):')\n"
    "for snr, r_u, r_e in zip(SNR_VALS, rate_under, rate_exact):\n"
    "    print(f'  SNR={snr:3d} dB  undercount={r_u:.2f}  correct={r_e:.2f}')\n",
    "bb200010",
))

# ---------------------------------------------------------------------------
# Section 4: false alarm rate vs SNR (overcount)
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — False Alarm Rate vs SNR for Overcounted MUSIC\n"
    "\n"
    "Scenario: 1 true source at 20°, **n_sources=2 or 3** (overcount).\n"
    "A false alarm is any detected peak more than 5° from the true source.\n"
    "Count the fraction of trials that produce ≥1 false alarm.\n",
    "bb200011",
))

cells.append(code(
    "fa_rate_2 = []   # n_sources=2 (overcount by 1)\n"
    "fa_rate_3 = []   # n_sources=3 (overcount by 2)\n"
    "det_rate_1 = []  # n_sources=1 (correct)\n"
    "\n"
    "\n"
    "def has_false_alarm(peaks, true_source_az, tol=TOL_DEG):\n"
    "    return any(abs(p - true_source_az) >= tol for p in peaks)\n"
    "\n"
    "\n"
    "for snr in SNR_VALS:\n"
    "    fa2 = fa3 = det1 = 0\n"
    "    for trial in range(N_TRIALS2):\n"
    "        R = make_csm(x_arr, y_arr, src_one, FREQ, snr, N_SNAP,\n"
    "                     seed=trial * 2000 + int(snr))\n"
    "        for n_spec, store in [(1, 'det1'), (2, 'fa2'), (3, 'fa3')]:\n"
    "            m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=n_spec)\n"
    "            pks = detect_peaks(az_grid, m)\n"
    "            if store == 'det1':\n"
    "                if any(abs(p - 20.0) < TOL_DEG for p in pks):\n"
    "                    det1 += 1\n"
    "            elif store == 'fa2':\n"
    "                if has_false_alarm(pks, 20.0):\n"
    "                    fa2 += 1\n"
    "            else:\n"
    "                if has_false_alarm(pks, 20.0):\n"
    "                    fa3 += 1\n"
    "    fa_rate_2.append(fa2 / N_TRIALS2)\n"
    "    fa_rate_3.append(fa3 / N_TRIALS2)\n"
    "    det_rate_1.append(det1 / N_TRIALS2)\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "ax.plot(SNR_VALS, fa_rate_2, 's--', color='darkorange', label='n_sources=2 (overcount +1)',\n"
    "        linewidth=1.5)\n"
    "ax.plot(SNR_VALS, fa_rate_3, '^--', color='firebrick',  label='n_sources=3 (overcount +2)',\n"
    "        linewidth=1.5)\n"
    "ax.plot(SNR_VALS, det_rate_1, 'o-', color='forestgreen', label='n_sources=1 (correct, det rate)',\n"
    "        linewidth=1.5)\n"
    "ax.axhline(0.1, color='gray', linestyle=':', linewidth=0.8, label='10% FA threshold')\n"
    "ax.set_xlabel('SNR (dB)')\n"
    "ax.set_ylabel('Fraction of trials')\n"
    "ax.set_title(f'False alarm rate vs SNR — 1 true source at 20°, N_SNAP={N_SNAP}, N_trials={N_TRIALS2}')\n"
    "ax.set_ylim(-0.05, 1.05)\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('music_fa_vs_snr.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('SNR vs false alarm rate (overcount):')\n"
    "for snr, f2, f3, d1 in zip(SNR_VALS, fa_rate_2, fa_rate_3, det_rate_1):\n"
    "    print(f'  SNR={snr:3d} dB  FA(n=2)={f2:.2f}  FA(n=3)={f3:.2f}  det(n=1)={d1:.2f}')\n",
    "bb200012",
))

# ---------------------------------------------------------------------------
# Section 5: N_SNAP effect on undercount robustness
# ---------------------------------------------------------------------------
cells.append(md(
    "## 5 — N_SNAP Effect on Undercount Robustness\n"
    "\n"
    "Fix SNR=20 dB, 2 true sources at ±15°, n_sources=1 (undercount).\n"
    "Sweep N_SNAP and measure both-source detection rate.\n"
    "More snapshots improve CSM quality — does that rescue a wrong n_sources?\n",
    "bb200013",
))

cells.append(code(
    "snap_vals  = [16, 32, 64, 128, 256, 512, 1024, 2048]\n"
    "N_TRIALS3  = 20\n"
    "\n"
    "rate_under_snap = []\n"
    "rate_exact_snap = []\n"
    "\n"
    "for n_snap in snap_vals:\n"
    "    det_under = det_exact = 0\n"
    "    for trial in range(N_TRIALS3):\n"
    "        R = make_csm(x_arr, y_arr, src_two, FREQ, SNR_DB, n_snap,\n"
    "                     seed=trial * 3000 + n_snap)\n"
    "        for n_spec, store in [(1, 'under'), (2, 'exact')]:\n"
    "            m = beamform_music(x_arr, y_arr, R, FREQ, az_grid, n_sources=n_spec)\n"
    "            pks = detect_peaks(az_grid, m)\n"
    "            if both_detected(pks, src_two):\n"
    "                if store == 'under':\n"
    "                    det_under += 1\n"
    "                else:\n"
    "                    det_exact += 1\n"
    "    rate_under_snap.append(det_under / N_TRIALS3)\n"
    "    rate_exact_snap.append(det_exact / N_TRIALS3)\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "ax.plot(snap_vals, rate_exact_snap, 'o-', color='forestgreen',\n"
    "        label='n_sources=2 (correct)', linewidth=1.5)\n"
    "ax.plot(snap_vals, rate_under_snap, 's--', color='darkorange',\n"
    "        label='n_sources=1 (undercount)', linewidth=1.5)\n"
    "ax.axhline(0.9, color='gray', linestyle=':', linewidth=0.8, label='90% threshold')\n"
    "ax.axvline(N_MICS, color='gray', linestyle='--', linewidth=0.8,\n"
    "           label=f'N_SNAP = N_MICS ({N_MICS})')\n"
    "ax.set_xscale('log')\n"
    "ax.set_xlabel('N_SNAP (log scale)')\n"
    "ax.set_ylabel('Fraction of trials: both sources detected')\n"
    "ax.set_title(f'Detection rate vs N_SNAP — 2 sources ±15°, SNR={SNR_DB:.0f} dB, N_trials={N_TRIALS3}')\n"
    "ax.set_ylim(-0.05, 1.05)\n"
    "ax.legend()\n"
    "ax.set_xticks(snap_vals)\n"
    "ax.set_xticklabels([str(n) for n in snap_vals], fontsize=8)\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('music_detect_vs_snap.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n"
    "\n"
    "print('N_SNAP vs detection rate (undercount n_sources=1, SNR=20dB):')\n"
    "for n, r_u, r_e in zip(snap_vals, rate_under_snap, rate_exact_snap):\n"
    "    print(f'  N_SNAP={n:5d}  undercount={r_u:.2f}  correct={r_e:.2f}')\n",
    "bb200014",
))

# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "bb200015",
))

cells.append(code(
    "print('=== Undercount robustness (n_sources=1, true k=2, ±15° sources) ===')\n"
    "print('Detection rate at 90% threshold:')\n"
    "snr_90_under = next((s for s, r in zip(SNR_VALS, rate_under) if r >= 0.9), None)\n"
    "snr_90_exact = next((s for s, r in zip(SNR_VALS, rate_exact) if r >= 0.9), None)\n"
    "print(f'  Correct (n_sources=2): SNR >= {snr_90_exact} dB')\n"
    "print(f'  Undercount (n_sources=1): SNR >= {snr_90_under} dB  (None = never reached)')\n"
    "print()\n"
    "print('=== Overcount false alarms (n_sources=2/3, true k=1, source @ 20°) ===')\n"
    "print('SNR where FA rate drops below 10%:')\n"
    "snr_fa2_ok = next((s for s, r in zip(SNR_VALS, fa_rate_2) if r <= 0.1), None)\n"
    "snr_fa3_ok = next((s for s, r in zip(SNR_VALS, fa_rate_3) if r <= 0.1), None)\n"
    "print(f'  Overcount +1 (n_sources=2): SNR >= {snr_fa2_ok} dB')\n"
    "print(f'  Overcount +2 (n_sources=3): SNR >= {snr_fa3_ok} dB')\n"
    "print()\n"
    "print('=== N_SNAP effect on undercount (SNR=20dB, n_sources=1, true k=2) ===')\n"
    "snap_90_under = next((n for n, r in zip(snap_vals, rate_under_snap) if r >= 0.9), None)\n"
    "snap_90_exact = next((n for n, r in zip(snap_vals, rate_exact_snap) if r >= 0.9), None)\n"
    "print(f'  Correct (n_sources=2): N_SNAP >= {snap_90_exact}')\n"
    "print(f'  Undercount (n_sources=1): N_SNAP >= {snap_90_under}  (None = never reached)')\n",
    "bb200016",
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

out = Path("07_music_robustness.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
