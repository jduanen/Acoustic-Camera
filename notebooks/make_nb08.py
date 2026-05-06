#!/usr/bin/env python3
"""Generate notebooks/08_calibration_sensitivity.ipynb"""
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
    "# 08 — Calibration Sensitivity\n"
    "\n"
    "**Goal**: quantify how mic-to-mic gain and phase mismatch degrades beamformer\n"
    "performance, and how well calibration correction restores it.\n"
    "\n"
    "The Infineon IM69D120 MEMS microphone specifies:\n"
    "- Sensitivity tolerance: **±1 dB** (gain mismatch)\n"
    "- Phase tolerance: **±2°** at 1 kHz\n"
    "\n"
    "Mismatch model: each mic n receives a complex gain error\n"
    "`e_n = g_n · exp(j·φ_n)` where `g_n ~ Uniform(−G, +G)` dB and\n"
    "`φ_n ~ Uniform(−Φ, +Φ)` degrees.  Applied to the snapshot vector\n"
    "before CSM accumulation.\n"
    "\n"
    "Key questions:\n"
    "1. Does IM69D120-spec mismatch materially degrade DoA accuracy?\n"
    "2. Which algorithm is most sensitive?\n"
    "3. How much mismatch can each algorithm tolerate?\n"
    "4. Does perfect calibration fully restore performance?\n"
    "5. How much residual calibration error can we tolerate?\n",
    "cc300001",
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
    "FS     = 48_000\n"
    "FREQ   = 4000.0\n"
    "SNR_DB = 20.0\n"
    "N_SNAP = 512   # well above CSM convergence — isolates mismatch effect\n",
    "cc300002",
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
    "cc300003",
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
    "def make_mismatch(N, gain_db_max, phase_deg_max, rng):\n"
    "    \"\"\"Complex gain error per mic: magnitude from gain_db_max, phase from phase_deg_max.\"\"\"\n"
    "    gain_db = rng.uniform(-gain_db_max, gain_db_max, N)\n"
    "    phi     = np.radians(rng.uniform(-phase_deg_max, phase_deg_max, N))\n"
    "    return 10 ** (gain_db / 20) * np.exp(1j * phi)\n"
    "\n"
    "\n"
    "def make_csm(x, y, sources, freq, snr_db, n_snap, mismatch=None, seed=None):\n"
    "    local_rng = np.random.default_rng(seed)\n"
    "    N = len(x)\n"
    "    e = mismatch if mismatch is not None else np.ones(N, dtype=complex)\n"
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
    "        y_snap = e * y_snap\n"
    "        R += np.outer(y_snap, y_snap.conj())\n"
    "    return R / n_snap\n"
    "\n"
    "\n"
    "def calibrate_csm(R, mismatch):\n"
    "    \"\"\"Correct CSM for known complex gain errors: R_corr = diag(1/e) R diag(1/e*).\"\"\"\n"
    "    e_inv = 1.0 / mismatch\n"
    "    return np.outer(e_inv, e_inv.conj()) * R\n"
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
    "    return 1.0 / np.maximum(denom, 1e-300)\n"
    "\n"
    "\n"
    "az_grid  = np.linspace(-60, 60, 1201)\n"
    "src_one  = [(25.0, 1.0)]\n"
    "src_two  = [(-15.0, 1.0), (15.0, 1.0)]\n"
    "colors   = {'D&S': 'steelblue', 'MVDR': 'darkorange', 'MUSIC': 'forestgreen'}\n",
    "cc300004",
))

# ---------------------------------------------------------------------------
# Section 1: DoA error vs mismatch severity
# ---------------------------------------------------------------------------
cells.append(md(
    "## 1 — DoA Error vs Mismatch Severity\n"
    "\n"
    "Single source at 25°, SNR=20 dB, N_SNAP=512.\n"
    "For each mismatch level, draw N_TRIALS independent mismatch vectors\n"
    "(different random gain/phase per mic), compute CSM, measure peak DoA.\n"
    "Mismatch levels expressed as `(gain_db_max, phase_deg_max)`.\n"
    "The IM69D120 spec point (1 dB, 2°) is marked.\n",
    "cc300005",
))

cells.append(code(
    "# Sweep: gain_db_max from 0 to 4; phase_deg_max = 2 * gain_db_max\n"
    "GAIN_STEPS  = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0]\n"
    "PHASE_STEPS = [g * 2 for g in GAIN_STEPS]   # keep ratio from IM69D120 spec\n"
    "N_TRIALS    = 30\n"
    "\n"
    "results = {alg: {'mean': [], 'std': []} for alg in ['D&S', 'MVDR', 'MUSIC']}\n"
    "\n"
    "for gain_db, phase_deg in zip(GAIN_STEPS, PHASE_STEPS):\n"
    "    errs = {'D&S': [], 'MVDR': [], 'MUSIC': []}\n"
    "    for trial in range(N_TRIALS):\n"
    "        mismatch_rng = np.random.default_rng(trial * 100)\n"
    "        e = make_mismatch(N_MICS, gain_db, phase_deg, mismatch_rng)\n"
    "        R = make_csm(x_arr, y_arr, src_one, FREQ, SNR_DB, N_SNAP,\n"
    "                     mismatch=e, seed=trial)\n"
    "        for alg, fn, kw in [\n"
    "            ('D&S',   beamform_ds,    {}),\n"
    "            ('MVDR',  beamform_mvdr,  {}),\n"
    "            ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "        ]:\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            errs[alg].append(abs(az_grid[np.argmax(m)] - 25.0))\n"
    "    for alg in results:\n"
    "        results[alg]['mean'].append(np.mean(errs[alg]))\n"
    "        results[alg]['std'].append(np.std(errs[alg]))\n"
    "\n"
    "print('Mismatch sweep complete')\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    print(f'  {alg}: mean DoA error at IM69D120 spec (1dB, 2°) = '\n"
    "          f'{results[alg][\"mean\"][2]:.3f}° ± {results[alg][\"std\"][2]:.3f}°')\n",
    "cc300006",
))

cells.append(code(
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    mean = np.array(results[alg]['mean'])\n"
    "    std  = np.array(results[alg]['std'])\n"
    "    ax.plot(GAIN_STEPS, mean, 'o-', color=colors[alg], label=alg, linewidth=1.5)\n"
    "    ax.fill_between(GAIN_STEPS, mean - std, mean + std,\n"
    "                    color=colors[alg], alpha=0.15)\n"
    "\n"
    "ax.axvline(1.0, color='gray', linestyle='--', linewidth=1,\n"
    "           label='IM69D120 spec (1 dB, 2°)')\n"
    "ax.set_xlabel('Gain mismatch ±G (dB)  [phase = ±2G°]')\n"
    "ax.set_ylabel('Mean abs DoA error (deg)')\n"
    "ax.set_title(f'DoA Error vs Mismatch Severity — source @ 25°, '\n"
    "             f'{FREQ/1000:.0f} kHz, {SNR_DB:.0f} dB SNR, N_SNAP={N_SNAP}')\n"
    "ax.legend()\n"
    "ax.grid(True, alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('cal_doa_vs_mismatch.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "cc300007",
))

# ---------------------------------------------------------------------------
# Section 2: gain-only vs phase-only contribution
# ---------------------------------------------------------------------------
cells.append(md(
    "## 2 — Gain vs Phase: Which Matters More?\n"
    "\n"
    "Separate the gain and phase contributions at the IM69D120 spec level.\n"
    "Run the same sweep with only gain mismatch, only phase mismatch, and combined.\n",
    "cc300008",
))

cells.append(code(
    "IM_GAIN  = 1.0   # dB\n"
    "IM_PHASE = 2.0   # deg\n"
    "\n"
    "conditions = [\n"
    "    ('No mismatch',        0.0,     0.0),\n"
    "    ('Gain only (±1dB)',   IM_GAIN, 0.0),\n"
    "    ('Phase only (±2°)',   0.0,     IM_PHASE),\n"
    "    ('Combined (spec)',    IM_GAIN, IM_PHASE),\n"
    "]\n"
    "\n"
    "rows = []\n"
    "for label, gdb, pdeg in conditions:\n"
    "    for alg, fn, kw in [\n"
    "        ('D&S',   beamform_ds,    {}),\n"
    "        ('MVDR',  beamform_mvdr,  {}),\n"
    "        ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "    ]:\n"
    "        errs = []\n"
    "        for trial in range(N_TRIALS):\n"
    "            mismatch_rng = np.random.default_rng(trial * 100)\n"
    "            e = make_mismatch(N_MICS, gdb, pdeg, mismatch_rng)\n"
    "            R = make_csm(x_arr, y_arr, src_one, FREQ, SNR_DB, N_SNAP,\n"
    "                         mismatch=e, seed=trial)\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            errs.append(abs(az_grid[np.argmax(m)] - 25.0))\n"
    "        rows.append({\n"
    "            'Condition': label,\n"
    "            'Algorithm': alg,\n"
    "            'Mean error (°)': round(np.mean(errs), 3),\n"
    "            'Std (°)': round(np.std(errs), 3),\n"
    "        })\n"
    "\n"
    "df = pd.DataFrame(rows)\n"
    "pivot = df.pivot(index='Condition', columns='Algorithm', values='Mean error (°)')\n"
    "# maintain meaningful row order\n"
    "pivot = pivot.reindex([c[0] for c in conditions])\n"
    "print(pivot.to_string())\n",
    "cc300009",
))

# ---------------------------------------------------------------------------
# Section 3: resolution reliability under mismatch
# ---------------------------------------------------------------------------
cells.append(md(
    "## 3 — Resolution Reliability Under IM69D120 Mismatch\n"
    "\n"
    "Two sources at ±15°, 4 kHz, 20 dB SNR, N_SNAP=256.\n"
    "Compare no-mismatch vs IM69D120-spec mismatch (1 dB, 2°).\n"
    "Resolution criterion: −6 dB valley between the two peaks.\n",
    "cc300010",
))

cells.append(code(
    "N_SNAP_RES = 256\n"
    "N_TRIALS_R = 30\n"
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
    "rows_res = []\n"
    "for label, gdb, pdeg in [('No mismatch', 0.0, 0.0),\n"
    "                          ('IM69D120 spec (1dB, 2°)', IM_GAIN, IM_PHASE)]:\n"
    "    for alg, fn, kw in [\n"
    "        ('D&S',   beamform_ds,    {}),\n"
    "        ('MVDR',  beamform_mvdr,  {}),\n"
    "        ('MUSIC', beamform_music, {'n_sources': 2}),\n"
    "    ]:\n"
    "        resolved = 0\n"
    "        for trial in range(N_TRIALS_R):\n"
    "            mismatch_rng = np.random.default_rng(trial * 200)\n"
    "            e = make_mismatch(N_MICS, gdb, pdeg, mismatch_rng)\n"
    "            R = make_csm(x_arr, y_arr, src_two, FREQ, SNR_DB, N_SNAP_RES,\n"
    "                         mismatch=e, seed=trial + 500)\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            if is_resolved(az_grid, m):\n"
    "                resolved += 1\n"
    "        rows_res.append({\n"
    "            'Condition': label, 'Algorithm': alg,\n"
    "            'Resolution rate': round(resolved / N_TRIALS_R, 2),\n"
    "        })\n"
    "\n"
    "df_res = pd.DataFrame(rows_res)\n"
    "pivot_res = df_res.pivot(index='Condition', columns='Algorithm', values='Resolution rate')\n"
    "pivot_res = pivot_res.reindex(['No mismatch', 'IM69D120 spec (1dB, 2°)'])\n"
    "print(pivot_res.to_string())\n",
    "cc300011",
))

# ---------------------------------------------------------------------------
# Section 4: calibration correction
# ---------------------------------------------------------------------------
cells.append(md(
    "## 4 — Calibration Correction\n"
    "\n"
    "Apply `R_corr = diag(1/e) · R_mismatch · diag(1/e*)` to undo the mismatch.\n"
    "Four conditions compared:\n"
    "1. No mismatch (ideal baseline)\n"
    "2. IM69D120 mismatch, uncorrected\n"
    "3. Perfect calibration (correction = true e)\n"
    "4. Imperfect calibration (residual ±0.2 dB, ±0.5°)\n",
    "cc300012",
))

cells.append(code(
    "N_TRIALS_CAL = 30\n"
    "CAL_RESIDUAL_GAIN  = 0.2   # dB — post-calibration residual\n"
    "CAL_RESIDUAL_PHASE = 0.5   # deg\n"
    "\n"
    "rows_cal = []\n"
    "for alg, fn, kw in [\n"
    "    ('D&S',   beamform_ds,    {}),\n"
    "    ('MVDR',  beamform_mvdr,  {}),\n"
    "    ('MUSIC', beamform_music, {'n_sources': 1}),\n"
    "]:\n"
    "    errs = {'No mismatch': [], 'Uncorrected': [], 'Perfect cal': [], 'Imperfect cal': []}\n"
    "    for trial in range(N_TRIALS_CAL):\n"
    "        rng_m = np.random.default_rng(trial * 300)\n"
    "        e_true = make_mismatch(N_MICS, IM_GAIN, IM_PHASE, rng_m)\n"
    "\n"
    "        # residual error in the calibration estimate\n"
    "        rng_r = np.random.default_rng(trial * 300 + 1)\n"
    "        e_residual = make_mismatch(N_MICS, CAL_RESIDUAL_GAIN, CAL_RESIDUAL_PHASE, rng_r)\n"
    "        e_estimated = e_true * e_residual\n"
    "\n"
    "        R_ideal = make_csm(x_arr, y_arr, src_one, FREQ, SNR_DB, N_SNAP,\n"
    "                           mismatch=None, seed=trial)\n"
    "        R_mis   = make_csm(x_arr, y_arr, src_one, FREQ, SNR_DB, N_SNAP,\n"
    "                           mismatch=e_true, seed=trial)\n"
    "        R_pcal  = calibrate_csm(R_mis, e_true)\n"
    "        R_ical  = calibrate_csm(R_mis, e_estimated)\n"
    "\n"
    "        for label, R in [('No mismatch', R_ideal), ('Uncorrected', R_mis),\n"
    "                          ('Perfect cal', R_pcal), ('Imperfect cal', R_ical)]:\n"
    "            m = fn(x_arr, y_arr, R, FREQ, az_grid, **kw)\n"
    "            errs[label].append(abs(az_grid[np.argmax(m)] - 25.0))\n"
    "\n"
    "    for label in errs:\n"
    "        rows_cal.append({\n"
    "            'Algorithm': alg,\n"
    "            'Condition': label,\n"
    "            'Mean error (°)': round(np.mean(errs[label]), 3),\n"
    "            'Std (°)': round(np.std(errs[label]), 3),\n"
    "        })\n"
    "\n"
    "df_cal = pd.DataFrame(rows_cal)\n"
    "pivot_cal = df_cal.pivot(index='Condition', columns='Algorithm', values='Mean error (°)')\n"
    "pivot_cal = pivot_cal.reindex(['No mismatch', 'Uncorrected', 'Perfect cal', 'Imperfect cal'])\n"
    "print('Mean DoA error (°) by condition and algorithm:')\n"
    "print(pivot_cal.to_string())\n",
    "cc300013",
))

cells.append(code(
    "cond_order = ['No mismatch', 'Uncorrected', 'Imperfect cal', 'Perfect cal']\n"
    "algs_plot  = ['D&S', 'MVDR', 'MUSIC']\n"
    "x_pos      = np.arange(len(cond_order))\n"
    "width      = 0.25\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(10, 5))\n"
    "for i, alg in enumerate(algs_plot):\n"
    "    vals = [pivot_cal.loc[c, alg] for c in cond_order]\n"
    "    ax.bar(x_pos + i * width, vals, width, label=alg, color=list(colors.values())[i],\n"
    "           alpha=0.85)\n"
    "\n"
    "ax.set_xticks(x_pos + width)\n"
    "ax.set_xticklabels(cond_order)\n"
    "ax.set_ylabel('Mean abs DoA error (deg)')\n"
    "ax.set_title(f'Calibration Effect — source @ 25°, {FREQ/1000:.0f} kHz, '\n"
    "             f'{SNR_DB:.0f} dB SNR, N_SNAP={N_SNAP}')\n"
    "ax.legend()\n"
    "ax.grid(True, axis='y', alpha=0.3)\n"
    "plt.tight_layout()\n"
    "plt.savefig('cal_correction_effect.png', dpi=150, bbox_inches='tight')\n"
    "plt.show()\n",
    "cc300014",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n",
    "cc300015",
))

cells.append(code(
    "print('=== DoA error at IM69D120 spec (1dB gain, 2° phase) ===')\n"
    "for alg in ['D&S', 'MVDR', 'MUSIC']:\n"
    "    m = results[alg]['mean'][2]  # index 2 = gain=1.0dB step\n"
    "    s = results[alg]['std'][2]\n"
    "    print(f'  {alg}: {m:.3f}° ± {s:.3f}°')\n"
    "print()\n"
    "print('=== Resolution reliability (±15° sources, N_SNAP=256) ===')\n"
    "print(pivot_res.to_string())\n"
    "print()\n"
    "print('=== Calibration correction (DoA error, source @ 25°) ===')\n"
    "print(pivot_cal.to_string())\n",
    "cc300016",
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

out = Path("08_calibration_sensitivity.ipynb")
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}  ({len(cells)} cells)")
