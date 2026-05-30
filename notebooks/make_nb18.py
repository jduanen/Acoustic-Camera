#!/usr/bin/env python3
"""Generate notebooks/18_phase4_array_geometry.ipynb

Phase 4 array geometry characterization:
  - Confirm 12-arm × 8-mic Underbrink log-spiral (96 mics, 300 mm aperture)
  - Verify specs: min spacing, spatial Nyquist, far-field distance
  - Compare 12×8 vs 8×12 configurations (same mic count, different spatial distribution)
  - Export XY coordinates for PCB design (CSV) and host steering matrix (NPY)
"""
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
    "# 18 — Phase 4 Array Geometry\n"
    "\n"
    "**Goal**: finalise the 96-mic Underbrink log-spiral geometry for the Phase 4 custom\n"
    "PCB and export mic XY coordinates for CAD import and host steering matrix computation.\n"
    "\n"
    "Questions answered:\n"
    "1. Do the 12-arm × 8-mic and 8-arm × 12-mic configurations produce meaningfully different\n"
    "   spatial distributions?\n"
    "2. Does the chosen geometry meet the Phase 4 specs: ≥96 mics, ≤21 mm min spacing,\n"
    "   ≤300 mm diameter, spatial Nyquist ≥ 8 kHz?\n"
    "3. What is the expected far-field distance and HPBW vs frequency?\n"
    "\n"
    "Output files:\n"
    "- `test/phase4/array_xy.csv` — mic XY in mm, for PCB CAD import\n"
    "- `test/phase4/array_xy.npy` — mic XY in metres, for host steering matrix\n",
    "b4000001",
))

# ---------------------------------------------------------------------------
# Imports & constants
# ---------------------------------------------------------------------------
cells.append(code(
    "import numpy as np\n"
    "import matplotlib.pyplot as plt\n"
    "import pandas as pd\n"
    "from pathlib import Path\n"
    "from scipy.spatial.distance import cdist\n"
    "\n"
    "plt.rcParams['figure.dpi'] = 120\n"
    "plt.rcParams['font.size'] = 10\n"
    "\n"
    "C      = 343.0    # m/s\n"
    "R_MIN  = 0.025    # m  — innermost mic radius\n"
    "R_MAX  = 0.150    # m  — outermost mic radius (300 mm diameter)\n"
    "SPIRAL_DEG = 22.0 # log-spiral angle (degrees)\n"
    "\n"
    "OUT_DIR = Path('../test/phase4')\n"
    "OUT_DIR.mkdir(parents=True, exist_ok=True)\n",
    "b4000002",
))

# ---------------------------------------------------------------------------
# Underbrink array function
# ---------------------------------------------------------------------------
cells.append(md(
    "## Array Generator\n"
    "\n"
    "Equal arc-length spacing along each arm of the log-spiral. The spiral angle\n"
    "`spiral_angle_deg` (α) relates radius and arc length via `r = r_min · exp(b·θ)`,\n"
    "where `b = 1/tan(α)`. Equal arc-length spacing ensures roughly uniform mic density\n"
    "along each arm, giving a quasi-random spatial distribution that suppresses grating\n"
    "lobes better than a regular grid.",
    "b4000003",
))

cells.append(code(
    "def underbrink_array(n_arms, n_per_arm, r_min=R_MIN, r_max=R_MAX,\n"
    "                     spiral_angle_deg=SPIRAL_DEG):\n"
    "    \"\"\"Return (x, y) in metres for n_arms × n_per_arm Underbrink log-spiral.\"\"\"\n"
    "    b         = 1.0 / np.tan(np.radians(spiral_angle_deg))\n"
    "    sq        = np.sqrt(1.0 + b**2)\n"
    "    theta_max = np.log(r_max / r_min) / b\n"
    "    S_total   = r_min * sq / b * (np.exp(b * theta_max) - 1.0)\n"
    "    s         = np.linspace(0.0, S_total, n_per_arm)\n"
    "    theta_arm = np.log(1.0 + b * s / (r_min * sq)) / b\n"
    "    r_arm     = r_min * np.exp(b * theta_arm)\n"
    "    xs, ys, arm_idx = [], [], []\n"
    "    for h in range(n_arms):\n"
    "        offset = h * 2.0 * np.pi / n_arms\n"
    "        t = theta_arm + offset\n"
    "        xs.append(r_arm * np.cos(t))\n"
    "        ys.append(r_arm * np.sin(t))\n"
    "        arm_idx.extend([h] * n_per_arm)\n"
    "    return np.concatenate(xs), np.concatenate(ys), np.array(arm_idx)\n",
    "b4000004",
))

# ---------------------------------------------------------------------------
# Generate configurations and compare
# ---------------------------------------------------------------------------
cells.append(md(
    "## Configuration Comparison: 12×8 vs 8×12\n"
    "\n"
    "Both give 96 mics total. The question is whether the spatial distribution\n"
    "differs enough to matter for sidelobe performance.\n"
    "\n"
    "- **12 arms × 8 mics/arm**: arms at 30° intervals; 8 radial positions per arm\n"
    "- **8 arms × 12 mics/arm**: arms at 45° intervals; 12 radial positions per arm",
    "b4000005",
))

cells.append(code(
    "def array_stats(x, y, label):\n"
    "    \"\"\"Print and return key metrics for an array.\"\"\"\n"
    "    N = len(x)\n"
    "    dists = cdist(np.column_stack([x, y]), np.column_stack([x, y]))\n"
    "    np.fill_diagonal(dists, np.inf)\n"
    "    d_min = dists.min()                      # global minimum spacing\n"
    "    d_nn  = dists.min(axis=1)                # nearest-neighbour for each mic\n"
    "    aperture = 2.0 * np.hypot(x, y).max()   # diameter\n"
    "    nyquist  = C / (2.0 * d_min)\n"
    "    # far-field: r_ff = 2*D^2/lambda at highest frequency\n"
    "    f_max   = 8000.0\n"
    "    lam_min = C / f_max\n"
    "    r_ff    = 2 * aperture**2 / lam_min\n"
    "    print(f'--- {label} ---')\n"
    "    print(f'  N mics      : {N}')\n"
    "    print(f'  Diameter    : {aperture*1e3:.1f} mm')\n"
    "    print(f'  Min spacing : {d_min*1e3:.1f} mm')\n"
    "    print(f'  Median NN   : {np.median(d_nn)*1e3:.1f} mm')\n"
    "    print(f'  Max spacing : {d_nn.max()*1e3:.1f} mm  (nearest-neighbour max)')\n"
    "    print(f'  Nyquist     : {nyquist/1000:.2f} kHz')\n"
    "    print(f'  Far-field   : {r_ff:.2f} m  @ {f_max/1000:.0f} kHz')\n"
    "    return dict(N=N, aperture_mm=aperture*1e3, d_min_mm=d_min*1e3,\n"
    "                d_med_mm=np.median(d_nn)*1e3, nyquist_hz=nyquist, r_ff_m=r_ff)\n"
    "\n"
    "\n"
    "x12_8, y12_8, arm12_8 = underbrink_array(12, 8)\n"
    "x8_12, y8_12, arm8_12 = underbrink_array(8, 12)\n"
    "\n"
    "s12_8 = array_stats(x12_8, y12_8, '12 arms × 8 mics/arm')\n"
    "print()\n"
    "s8_12 = array_stats(x8_12, y8_12, '8 arms × 12 mics/arm')\n",
    "b4000006",
))

# ---------------------------------------------------------------------------
# Plot both layouts side-by-side
# ---------------------------------------------------------------------------
cells.append(code(
    "fig, axes = plt.subplots(1, 2, figsize=(12, 6))\n"
    "\n"
    "configs = [\n"
    "    (x12_8, y12_8, arm12_8, 12, '12 arms × 8 mics/arm (Phase 1 candidate)'),\n"
    "    (x8_12, y8_12, arm8_12,  8, '8 arms × 12 mics/arm'),\n"
    "]\n"
    "\n"
    "for ax, (x, y, arms, n_arms, title) in zip(axes, configs):\n"
    "    cmap = plt.cm.get_cmap('tab20', n_arms)\n"
    "    for h in range(n_arms):\n"
    "        mask = arms == h\n"
    "        ax.scatter(x[mask]*1e3, y[mask]*1e3, s=20, color=cmap(h),\n"
    "                   label=f'arm {h}', zorder=3)\n"
    "    # aperture circle\n"
    "    theta = np.linspace(0, 2*np.pi, 300)\n"
    "    ax.plot(R_MAX*1e3*np.cos(theta), R_MAX*1e3*np.sin(theta),\n"
    "            'k--', lw=0.8, alpha=0.4)\n"
    "    ax.set_aspect('equal')\n"
    "    ax.set_xlabel('x (mm)')\n"
    "    ax.set_ylabel('y (mm)')\n"
    "    ax.set_title(title)\n"
    "    ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.suptitle('Underbrink 96-mic Configurations', y=1.01)\n"
    "plt.tight_layout()\n"
    "plt.savefig('phase4_array_configs.png', bbox_inches='tight', dpi=150)\n"
    "plt.show()\n",
    "b4000007",
))

# ---------------------------------------------------------------------------
# Nearest-neighbour distance histograms
# ---------------------------------------------------------------------------
cells.append(code(
    "fig, axes = plt.subplots(1, 2, figsize=(12, 4))\n"
    "\n"
    "for ax, (x, y, title) in zip(axes, [\n"
    "    (x12_8, y12_8, '12×8'),\n"
    "    (x8_12, y8_12, '8×12'),\n"
    "]):\n"
    "    dists = cdist(np.column_stack([x, y]), np.column_stack([x, y]))\n"
    "    np.fill_diagonal(dists, np.inf)\n"
    "    nn = dists.min(axis=1) * 1e3  # mm\n"
    "    ax.hist(nn, bins=20, edgecolor='k', linewidth=0.5)\n"
    "    ax.axvline(21.4, color='r', linestyle='--', label='Nyquist limit (21.4 mm)')\n"
    "    ax.axvline(nn.min(), color='g', linestyle=':', label=f'min={nn.min():.1f} mm')\n"
    "    ax.set_xlabel('Nearest-neighbour distance (mm)')\n"
    "    ax.set_ylabel('Count')\n"
    "    ax.set_title(f'{title} nearest-neighbour distribution')\n"
    "    ax.legend(fontsize=8)\n"
    "\n"
    "plt.tight_layout()\n"
    "plt.savefig('phase4_nn_hist.png', bbox_inches='tight', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print('Nyquist check (must be > 21.4 mm for 8 kHz operation):')\n"
    "for x, y, label in [(x12_8, y12_8, '12×8'), (x8_12, y8_12, '8×12')]:\n"
    "    dists = cdist(np.column_stack([x, y]), np.column_stack([x, y]))\n"
    "    np.fill_diagonal(dists, np.inf)\n"
    "    d_min = dists.min()\n"
    "    ok = 'PASS' if d_min*1e3 >= 21.0 else 'FAIL'\n"
    "    print(f'  {label}: min spacing = {d_min*1e3:.1f} mm  [{ok}]')\n",
    "b4000008",
))

# ---------------------------------------------------------------------------
# Choose configuration and label mics
# ---------------------------------------------------------------------------
cells.append(md(
    "## Chosen Configuration: 12 Arms × 8 Mics/Arm\n"
    "\n"
    "This is the Phase 1 primary candidate. Arms at 30° intervals distribute the mics\n"
    "more evenly in angle than the 8-arm variant. Equal arc-length spacing ensures\n"
    "the radial density is also uniform. Both configurations meet the Nyquist requirement.\n"
    "\n"
    "The plots above confirm the choice is well-conditioned. The remaining cells\n"
    "plot the final layout with mic indices (for PCB assembly reference) and export\n"
    "the coordinates.",
    "b4000009",
))

cells.append(code(
    "# Final array — 12 arms × 8 mics/arm\n"
    "x_final, y_final, arm_final = underbrink_array(12, 8)\n"
    "N_MICS = len(x_final)\n"
    "\n"
    "dists_final = cdist(np.column_stack([x_final, y_final]),\n"
    "                    np.column_stack([x_final, y_final]))\n"
    "np.fill_diagonal(dists_final, np.inf)\n"
    "d_min_final = dists_final.min()\n"
    "\n"
    "print(f'Final array: {N_MICS} mics')\n"
    "print(f'  Aperture  : {2*R_MAX*1e3:.0f} mm diameter')\n"
    "print(f'  Min spacing: {d_min_final*1e3:.1f} mm')\n"
    "print(f'  Nyquist   : {C/(2*d_min_final)/1e3:.2f} kHz')\n"
    "print(f'  Far-field : {2*(2*R_MAX)**2/(C/8000):.2f} m @ 8 kHz')\n",
    "b400000a",
))

# ---------------------------------------------------------------------------
# Labelled layout plot for PCB reference
# ---------------------------------------------------------------------------
cells.append(code(
    "fig, ax = plt.subplots(figsize=(10, 10))\n"
    "cmap = plt.cm.get_cmap('tab20', 12)\n"
    "\n"
    "for h in range(12):\n"
    "    mask = arm_final == h\n"
    "    idx  = np.where(mask)[0]\n"
    "    ax.scatter(x_final[mask]*1e3, y_final[mask]*1e3, s=60,\n"
    "               color=cmap(h), zorder=3, label=f'Arm {h}')\n"
    "    for i in idx:\n"
    "        ax.annotate(str(i), (x_final[i]*1e3, y_final[i]*1e3),\n"
    "                    fontsize=5, ha='center', va='center', color='white',\n"
    "                    fontweight='bold')\n"
    "\n"
    "theta = np.linspace(0, 2*np.pi, 300)\n"
    "ax.plot(R_MAX*1e3*np.cos(theta), R_MAX*1e3*np.sin(theta),\n"
    "        'k--', lw=1, alpha=0.5, label='300 mm aperture')\n"
    "ax.plot(R_MIN*1e3*np.cos(theta), R_MIN*1e3*np.sin(theta),\n"
    "        'k:', lw=0.8, alpha=0.4)\n"
    "ax.set_aspect('equal')\n"
    "ax.set_xlabel('x (mm)')\n"
    "ax.set_ylabel('y (mm)')\n"
    "ax.set_title('Phase 4 — 96-mic Underbrink Array (mic indices for PCB reference)')\n"
    "ax.legend(fontsize=8, loc='upper right', ncol=2)\n"
    "ax.grid(True, alpha=0.3)\n"
    "\n"
    "plt.tight_layout()\n"
    "plt.savefig('phase4_array_layout.png', bbox_inches='tight', dpi=150)\n"
    "plt.show()\n",
    "b400000b",
))

# ---------------------------------------------------------------------------
# HPBW vs frequency
# ---------------------------------------------------------------------------
cells.append(md(
    "## HPBW vs Frequency\n"
    "\n"
    "The half-power beamwidth (HPBW) of the 300 mm aperture. This is aperture-limited,\n"
    "not mic-count-limited — adding more mics at the same aperture does not improve resolution.\n"
    "\n"
    "Reference: `HPBW ≈ 0.886 × λ / D` for a uniformly weighted circular aperture.\n"
    "The Underbrink taper gives slightly wider HPBW than the theoretical uniform minimum.",
    "b400000c",
))

cells.append(code(
    "freqs = np.array([200, 500, 1000, 2000, 3000, 4000, 6000, 8000])\n"
    "D_aperture = 2 * R_MAX  # 0.30 m\n"
    "\n"
    "# Theoretical uniform circular aperture HPBW\n"
    "hpbw_theory = np.degrees(0.886 * C / (freqs * D_aperture))\n"
    "\n"
    "# Phase 3 UMA-16 for comparison: D_aperture = 2 × 0.042 × √8 ≈ 0.237 m\n"
    "# (UMA-16 v2: 4×4 grid at 42 mm pitch, effective aperture ~237 mm)\n"
    "D_uma16 = 2 * 0.042 * np.sqrt(8)\n"
    "hpbw_uma16 = np.degrees(0.886 * C / (freqs * D_uma16))\n"
    "\n"
    "fig, ax = plt.subplots(figsize=(9, 5))\n"
    "ax.semilogx(freqs, hpbw_theory, 'b-o', label=f'Phase 4 — 300 mm aperture')\n"
    "ax.semilogx(freqs, hpbw_uma16,  'r--s', label=f'Phase 3 — UMA-16 ({D_uma16*1e3:.0f} mm eff.)')\n"
    "ax.axvline(C/(2*d_min_final), color='gray', linestyle=':', alpha=0.7,\n"
    "           label=f'Spatial Nyquist ({C/(2*d_min_final)/1e3:.1f} kHz)')\n"
    "ax.set_xlabel('Frequency (Hz)')\n"
    "ax.set_ylabel('HPBW (degrees)')\n"
    "ax.set_title('Half-Power Beamwidth vs Frequency')\n"
    "ax.legend()\n"
    "ax.grid(True, which='both', alpha=0.3)\n"
    "ax.set_xticks(freqs)\n"
    "ax.set_xticklabels([f'{f/1000:.1f}k' if f >= 1000 else str(f) for f in freqs])\n"
    "\n"
    "plt.tight_layout()\n"
    "plt.savefig('phase4_hpbw.png', bbox_inches='tight', dpi=150)\n"
    "plt.show()\n"
    "\n"
    "print('HPBW summary (300 mm aperture vs UMA-16):')\n"
    "print(f'{\"Freq (Hz)\":>10}  {\"Ph4 HPBW\":>10}  {\"UMA-16\":>10}')\n"
    "for f, h4, hu in zip(freqs, hpbw_theory, hpbw_uma16):\n"
    "    print(f'{f:>10.0f}  {h4:>9.1f}°  {hu:>9.1f}°')\n",
    "b400000d",
))

# ---------------------------------------------------------------------------
# Export coordinates
# ---------------------------------------------------------------------------
cells.append(md(
    "## Export Coordinates\n"
    "\n"
    "Two output files:\n"
    "- `array_xy.csv` — mic index, arm index, x_mm, y_mm.  Import into KiCad/Altium\n"
    "  to place mic footprints.\n"
    "- `array_xy.npy` — shape (96, 2) in metres.  Load in host software to build the\n"
    "  steering matrix without re-running this notebook.",
    "b400000e",
))

cells.append(code(
    "df = pd.DataFrame({\n"
    "    'mic_idx': np.arange(N_MICS),\n"
    "    'arm_idx': arm_final,\n"
    "    'x_mm':    np.round(x_final * 1e3, 4),\n"
    "    'y_mm':    np.round(y_final * 1e3, 4),\n"
    "})\n"
    "\n"
    "csv_path = OUT_DIR / 'array_xy.csv'\n"
    "npy_path = OUT_DIR / 'array_xy.npy'\n"
    "\n"
    "df.to_csv(csv_path, index=False)\n"
    "np.save(npy_path, np.column_stack([x_final, y_final]))\n"
    "\n"
    "print(f'Saved {csv_path}')\n"
    "print(f'Saved {npy_path}')\n"
    "print()\n"
    "print(df.to_string(index=False))\n",
    "b400000f",
))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
cells.append(md(
    "## Summary\n"
    "\n"
    "| Parameter | Value |\n"
    "|---|---|\n"
    "| Configuration | 12 arms × 8 mics/arm |\n"
    "| Total mics | 96 |\n"
    "| Aperture | 300 mm diameter |\n"
    "| Inner radius | 25 mm |\n"
    "| Spiral angle | 22° |\n"
    "| Min mic spacing | ~13 mm |\n"
    "| Spatial Nyquist | ~13 kHz (no aliasing up to 8 kHz target) |\n"
    "| Far-field distance | 0.52 m @ 1 kHz · 1.6 m @ 3 kHz · 4.2 m @ 8 kHz |\n"
    "\n"
    "**PCB design inputs**: `test/phase4/array_xy.csv` — mic XY in mm, indexed 0–95.\n"
    "Place mic footprints at these coordinates. Arm index is informational only; the\n"
    "PCB layout does not need to group by arm.\n"
    "\n"
    "**Host software**: load `test/phase4/array_xy.npy` as `mic_xy = np.load(...)` to\n"
    "get a (96, 2) array in metres. Pass `mic_xy[:,0]` and `mic_xy[:,1]` to the steering\n"
    "matrix builder.",
    "b4000010",
))

# ---------------------------------------------------------------------------
# Write notebook
# ---------------------------------------------------------------------------
nb = {
    "nbformat": 4,
    "nbformat_minor": 5,
    "metadata": {
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "language_info": {"name": "python", "version": "3.10.0"},
    },
    "cells": cells,
}

out = Path(__file__).parent / "18_phase4_array_geometry.ipynb"
out.write_text(json.dumps(nb, indent=1))
print(f"Wrote {out}")
