#!/usr/bin/env python3
"""Generate 2D microphone array configuration figure for README.md.

Shows all six patterns from the '2D patterns' table:
  Regular grid | Archimedean spiral | Dougherty log-spiral
  Arcondoulis  | Underbrink         | Brüel & Kjær

Each panel: mic positions (scatter) + 2D PSF heatmap at 4 kHz.
Saves to assets/concept_figures/array_patterns_2d.png
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial.distance import pdist
from pathlib import Path

OUT  = Path("assets/concept_figures")
OUT.mkdir(parents=True, exist_ok=True)

C     = 343.0
R_MAX = 0.150   # 150 mm radius → 300 mm aperture for all arrays
FREQ  = 4000.0  # PSF evaluation frequency

plt.rcParams.update({"figure.dpi": 150, "font.size": 9,
                     "axes.spines.top": False, "axes.spines.right": False})

# ── array generators ─────────────────────────────────────────────────────────

def regular_grid(n=8):
    """Uniform n×n grid; outer edge-to-edge = 2×R_MAX."""
    spacing = 2 * R_MAX / (n - 1)
    xi = np.arange(n) * spacing - R_MAX
    X, Y = np.meshgrid(xi, xi)
    return X.ravel(), Y.ravel()


def archimedean_spiral(N=72, r_min=0.015, n_turns=5.5):
    """Single-arm Archimedean: r grows linearly with θ."""
    theta = np.linspace(0, n_turns * 2 * np.pi, N)
    r = r_min + (R_MAX - r_min) * theta / theta[-1]
    return r * np.cos(theta), r * np.sin(theta)


def dougherty_spiral(N=72, r_min=0.015, spiral_deg=85.0):
    """Single-arm log-spiral with equal arc-length spacing (Dougherty).
    spiral_deg≈85° gives ~4 full turns from r_min to R_MAX, ensuring
    good angular coverage across the aperture."""
    b  = 1.0 / np.tan(np.radians(spiral_deg))
    sq = np.sqrt(1.0 + b ** 2)
    theta_max = np.log(R_MAX / r_min) / b
    S_total   = r_min * sq / b * (np.exp(b * theta_max) - 1.0)
    s         = np.linspace(0.0, S_total, N)
    theta     = np.log(1.0 + b * s / (r_min * sq)) / b
    r         = r_min * np.exp(b * theta)
    return r * np.cos(theta), r * np.sin(theta)


def arcondoulis_array(N=72, r_min=0.015, spiral_deg=85.0, squash_y=0.60):
    """Dougherty log-spiral with y-axis squash → elliptical aperture.
    Matches non-square camera FOV; squash_y controls aspect ratio."""
    x, y = dougherty_spiral(N, r_min, spiral_deg)
    return x, y * squash_y


def underbrink_array(n_arms=12, n_per_arm=8, r_min=0.025, spiral_deg=22.0):
    """Multi-arm log-spiral (Underbrink H=12×8, α=22°)."""
    b  = 1.0 / np.tan(np.radians(spiral_deg))
    sq = np.sqrt(1.0 + b ** 2)
    theta_max = np.log(R_MAX / r_min) / b
    S_total   = r_min * sq / b * (np.exp(b * theta_max) - 1.0)
    s         = np.linspace(0.0, S_total, n_per_arm)
    theta_arm = np.log(1.0 + b * s / (r_min * sq)) / b
    r_arm     = r_min * np.exp(b * theta_arm)
    xs, ys = [], []
    for h in range(n_arms):
        offset = h * 2.0 * np.pi / n_arms
        t = theta_arm + offset
        xs.append(r_arm * np.cos(t))
        ys.append(r_arm * np.sin(t))
    return np.concatenate(xs), np.concatenate(ys)


def bk_hoops(n_inner=20, n_outer=28, r_inner=0.085, seed=7):
    """Brüel & Kjær-style: two concentric hoops with non-uniform angular
    spacing (achieved by applying random jitter to uniform base angles)."""
    rng = np.random.default_rng(seed)

    def jittered_ring(N, r, jitter=0.30):
        base = np.linspace(0, 2 * np.pi, N, endpoint=False)
        delta = rng.uniform(-jitter, jitter, N) * (2 * np.pi / N)
        a = base + delta
        return r * np.cos(a), r * np.sin(a)

    xi, yi = jittered_ring(n_inner, r_inner, jitter=0.35)
    xo, yo = jittered_ring(n_outer, R_MAX,   jitter=0.25)
    return np.concatenate([xi, xo]), np.concatenate([yi, yo])


# ── 2-D PSF helper ───────────────────────────────────────────────────────────

def psf_ds_2d(x, y, freq, az_grid, el_grid, src_az=0.0, src_el=0.0):
    """Vectorised D&S PSF: beam = |H^H h0|² for all (az,el) directions."""
    AZ, EL = np.meshgrid(az_grid, el_grid)
    ux = (np.sin(np.radians(AZ)) * np.cos(np.radians(EL))).ravel()
    uy = np.sin(np.radians(EL)).ravel()
    ph = 2 * np.pi * freq / C * (np.outer(x, ux) + np.outer(y, uy))
    H  = np.exp(1j * ph) / np.sqrt(len(x))
    ux0 = np.sin(np.radians(src_az)) * np.cos(np.radians(src_el))
    uy0 = np.sin(np.radians(src_el))
    h0  = np.exp(1j * 2 * np.pi * freq / C * (x * ux0 + y * uy0)) / np.sqrt(len(x))
    vals = H.conj().T @ h0
    return np.abs(vals).reshape(len(el_grid), len(az_grid)) ** 2


def sll_db(psf2d, az, el, main_lobe_half_deg=25):
    """Peak sidelobe level outside the main-lobe exclusion cone."""
    AZ, EL = np.meshgrid(az, el)
    mask = np.sqrt(AZ ** 2 + EL ** 2) > main_lobe_half_deg
    db = 10 * np.log10(np.maximum(psf2d / psf2d.max(), 1e-10))
    return db[mask].max()


# ── build all arrays ──────────────────────────────────────────────────────────

arrays = [
    ("Regular Grid",        "8×8 uniform spacing",               *regular_grid()),
    ("Archimedean Spiral",  "single arm, linear r",              *archimedean_spiral()),
    ("Dougherty Log-Spiral","single arm, equal arc-length",      *dougherty_spiral()),
    ("Arcondoulis",         "squashed spiral (y×0.55),\nelliptical aperture", *arcondoulis_array()),
    ("Underbrink ★",        "12-arm log-spiral\n(H=12×8, α=22°)", *underbrink_array()),
    ("Brüel & Kjær",        "two concentric hoops,\nnon-uniform spacing", *bk_hoops()),
]

# PSF grid
az_psf = np.linspace(-60, 60, 91)
el_psf = np.linspace(-45, 45, 61)
AZ_G, EL_G = np.meshgrid(az_psf, el_psf)

# ── figure layout ─────────────────────────────────────────────────────────────
# 4 rows: [geo1-3] [psf1-3] [geo4-6] [psf4-6], 3 columns
# height_ratios: geometry rows taller than PSF rows

fig = plt.figure(figsize=(13, 14))
gs  = gridspec.GridSpec(
    4, 3,
    height_ratios=[1.5, 1.0, 1.5, 1.0],
    hspace=0.55, wspace=0.35,
    top=0.95, bottom=0.03, left=0.06, right=0.97,
)

colours = ["steelblue", "darkorange", "seagreen", "mediumpurple", "crimson", "saddlebrown"]

for idx, (name, subtitle, x, y) in enumerate(arrays):
    row_geo = (idx // 3) * 2      # 0 for idx 0-2, 2 for idx 3-5
    row_psf = row_geo + 1
    col     = idx % 3

    N     = len(x)
    d_min = pdist(np.column_stack([x, y])).min()
    f_nyq = C / (2 * d_min)
    col_c = colours[idx]

    # ── geometry panel ───────────────────────────────────────────────────────
    ax_geo = fig.add_subplot(gs[row_geo, col])
    ax_geo.scatter(x * 1000, y * 1000, s=12, color=col_c, alpha=0.85, linewidths=0)
    # aperture reference circle
    theta_c = np.linspace(0, 2 * np.pi, 300)
    ax_geo.plot(R_MAX * 1000 * np.cos(theta_c), R_MAX * 1000 * np.sin(theta_c),
                "--", color="gray", linewidth=0.8, alpha=0.5)
    ax_geo.set_aspect("equal")
    ax_geo.set_xlim(-175, 175); ax_geo.set_ylim(-175, 175)
    ax_geo.set_xlabel("x (mm)", fontsize=8)
    ax_geo.set_ylabel("y (mm)", fontsize=8)
    ax_geo.tick_params(labelsize=7)
    ax_geo.set_title(
        f"$\\bf{{{name.replace('★','*')}}}$\n{subtitle}",
        fontsize=8.5, pad=4,
    )
    ax_geo.text(
        0.97, 0.03,
        f"N={N}  d_min={d_min*1000:.1f} mm\nf_Nyq={f_nyq/1000:.1f} kHz",
        transform=ax_geo.transAxes, ha="right", va="bottom",
        fontsize=7, color="gray",
        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.7),
    )
    ax_geo.grid(True, alpha=0.2)

    # ── PSF panel ────────────────────────────────────────────────────────────
    ax_psf = fig.add_subplot(gs[row_psf, col])
    psf = psf_ds_2d(x, y, FREQ, az_psf, el_psf)
    psf_db = 10 * np.log10(np.maximum(psf / psf.max(), 1e-10))
    sll = sll_db(psf, az_psf, el_psf)
    im = ax_psf.pcolormesh(az_psf, el_psf, psf_db,
                            cmap="inferno", vmin=-40, vmax=0, shading="auto")
    ax_psf.set_xlabel("Azimuth (°)", fontsize=8)
    ax_psf.set_ylabel("El (°)", fontsize=8)
    ax_psf.tick_params(labelsize=7)
    ax_psf.set_title(
        f"D&S PSF @ {FREQ/1000:.0f} kHz  —  SLL = {sll:.0f} dB",
        fontsize=8, pad=3,
    )
    cbar = plt.colorbar(im, ax=ax_psf, fraction=0.046, pad=0.03)
    cbar.set_label("dB", fontsize=7)
    cbar.ax.tick_params(labelsize=6)

    print(f"  [{idx+1}] {name:28s}  N={N:3d}  d_min={d_min*1000:5.1f} mm  "
          f"f_Nyq={f_nyq/1000:5.1f} kHz  SLL={sll:+5.1f} dB")

fig.suptitle(
    "2D Microphone Array Configurations — geometry (top) and D&S PSF @ 4 kHz (bottom)\n"
    "All arrays normalised to 300 mm aperture (dashed circle).  "
    "★ = recommended pattern for this project.",
    fontsize=9.5, y=0.975,
)

out_path = OUT / "array_patterns_2d.png"
fig.savefig(out_path, bbox_inches="tight")
plt.close(fig)
print(f"\nSaved: {out_path.resolve()}")
