#!/usr/bin/env python3
"""Generate 8 concept figures for the 'Key Concepts in Beamforming' README section.

Saves PNG files to docs/concept_figures/.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.colors import Normalize
from matplotlib import cm
from scipy.linalg import inv
from pathlib import Path

# ── output directory ─────────────────────────────────────────────────────────
OUT = Path("docs/concept_figures")
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

C = 343.0

# ── shared array helpers ─────────────────────────────────────────────────────

def underbrink_array(n_arms=12, n_per_arm=8, r_min=0.025, r_max=0.150, spiral_deg=22.0):
    b = 1.0 / np.tan(np.radians(spiral_deg))
    sq = np.sqrt(1.0 + b**2)
    theta_max = np.log(r_max / r_min) / b
    S_total = r_min * sq / b * (np.exp(b * theta_max) - 1.0)
    s = np.linspace(0.0, S_total, n_per_arm)
    theta_arm = np.log(1.0 + b * s / (r_min * sq)) / b
    r_arm = r_min * np.exp(b * theta_arm)
    xs, ys = [], []
    for h in range(n_arms):
        offset = h * 2.0 * np.pi / n_arms
        t = theta_arm + offset
        xs.append(r_arm * np.cos(t))
        ys.append(r_arm * np.sin(t))
    return np.concatenate(xs), np.concatenate(ys)


def grid_array(n=4, spacing=0.021):
    x, y = np.meshgrid(np.arange(n) * spacing, np.arange(n) * spacing)
    x -= x.mean(); y -= y.mean()
    return x.ravel(), y.ravel()


def steer_1d(x, az_grid, freq):
    u = np.sin(np.radians(az_grid))
    phase = 2 * np.pi * freq / C * np.outer(x, u)
    return np.exp(1j * phase) / np.sqrt(len(x))


def steer_2d(x, y, az_grid, el_grid, freq):
    az_r = np.radians(az_grid)
    el_r = np.radians(el_grid)
    ux = np.outer(np.sin(az_r), np.cos(el_r)).ravel()
    uy = np.sin(el_r)
    uy = np.tile(uy, len(az_grid))
    ph = 2 * np.pi * freq / C * (
        np.outer(x, ux) + np.outer(y, uy)
    )
    return np.exp(1j * ph) / np.sqrt(len(x))


def psf_ds_1d(x, freq, az_grid, src_az=0.0):
    H = steer_1d(x, az_grid, freq)
    h0 = steer_1d(x, np.array([src_az]), freq)[:, 0]
    R = np.outer(h0, h0.conj())
    return np.real(np.sum(H.conj() * (R @ H), axis=0))


def make_csm_1d(x, sources, freq, snr_db=25, n_snap=512, seed=0):
    rng = np.random.default_rng(seed)
    N = len(x)
    max_p = max(p for _, p in sources)
    noise_var = max_p / 10**(snr_db / 10)
    R = np.zeros((N, N), dtype=complex)
    for _ in range(n_snap):
        snap = np.zeros(N, dtype=complex)
        for az, pwr in sources:
            h = steer_1d(x, np.array([az]), freq)[:, 0]
            s = rng.standard_normal() + 1j * rng.standard_normal()
            snap += np.sqrt(pwr / 2) * h * s
        noise = rng.standard_normal(N) + 1j * rng.standard_normal(N)
        snap += np.sqrt(noise_var / 2) * noise
        R += np.outer(snap, snap.conj())
    return R / n_snap


def beamform_ds(R, H):
    return np.real(np.sum(H.conj() * (R @ H), axis=0))


def beamform_mvdr(R, H, dl=0.01):
    N = R.shape[0]
    Rl = R + dl * np.trace(R) / N * np.eye(N)
    Ri = inv(Rl)
    denom = np.real(np.sum(H.conj() * (Ri @ H), axis=0))
    return 1.0 / np.maximum(denom, 1e-300)


def db_norm(p, floor=-40):
    pdb = 10 * np.log10(np.maximum(p / p.max(), 1e-10))
    return np.maximum(pdb, floor)


def measure_hpbw(az, p):
    p_norm = p / p.max()
    half = 0.5
    above = p_norm >= half
    if above.sum() < 2:
        return 0.0
    idx = np.where(above)[0]
    return az[idx[-1]] - az[idx[0]]


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 1 — HPBW
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 1: HPBW …")

x, y = underbrink_array()
az = np.linspace(-60, 60, 1201)
FREQ = 4000.0

p = psf_ds_1d(x, FREQ, az)
p_db = db_norm(p)
hpbw = measure_hpbw(az, p)

fig, ax = plt.subplots(figsize=(7, 4))
ax.plot(az, p_db, color="steelblue", linewidth=1.8)
ax.axhline(-3, color="tomato", linestyle="--", linewidth=1.2, label="−3 dB threshold")

# find the -3 dB crossing points
p_norm = p / p.max()
crossings = np.where(np.diff((p_norm >= 0.5).astype(int)))[0]
if len(crossings) >= 2:
    az_l = az[crossings[0]]
    az_r = az[crossings[-1]]
    ax.annotate(
        "", xy=(az_r, -3), xytext=(az_l, -3),
        arrowprops=dict(arrowstyle="<->", color="tomato", lw=1.8),
    )
    ax.text(
        (az_l + az_r) / 2, -2.0,
        f"HPBW = {hpbw:.1f}°",
        ha="center", va="bottom", color="tomato", fontsize=9, fontweight="bold",
    )

ax.set_xlim(-60, 60)
ax.set_ylim(-40, 2)
ax.set_xlabel("Azimuth (degrees)")
ax.set_ylabel("Beam power (dB, normalized)")
ax.set_title(f"Half-Power Beam Width (HPBW)  —  Underbrink 96-mic array @ {FREQ/1e3:.0f} kHz")
ax.legend(loc="upper right")
ax.text(
    -59, -38,
    "HPBW = angular width of the main lobe at the −3 dB (half-power) points",
    fontsize=8, color="gray",
)
fig.tight_layout()
fig.savefig(OUT / "concept_hpbw.png")
plt.close(fig)
print(f"  HPBW = {hpbw:.1f}°")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 2 — Sidelobe Level  (2D PSF comparison)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 2: Sidelobe Level …")


def psf_ds_2d(x, y, freq, az_grid, el_grid, src_az=0.0, src_el=0.0):
    """Full 2D D&S PSF via vectorised H^H h0."""
    AZ, EL = np.meshgrid(az_grid, el_grid)
    ux = (np.sin(np.radians(AZ)) * np.cos(np.radians(EL))).ravel()
    uy = np.sin(np.radians(EL)).ravel()
    ph = 2 * np.pi * freq / C * (np.outer(x, ux) + np.outer(y, uy))
    H = np.exp(1j * ph) / np.sqrt(len(x))
    ux0 = np.sin(np.radians(src_az)) * np.cos(np.radians(src_el))
    uy0 = np.sin(np.radians(src_el))
    h0 = np.exp(1j * 2 * np.pi * freq / C * (x * ux0 + y * uy0)) / np.sqrt(len(x))
    vals = H.conj().T @ h0
    return np.abs(vals).reshape(len(el_grid), len(az_grid)) ** 2


az2 = np.linspace(-60, 60, 91)
el2 = np.linspace(-45, 45, 61)
FREQ2 = 4000.0

x_ub, y_ub = underbrink_array()
x_grid, y_grid = grid_array(n=10, spacing=0.030)  # 10×10, 30mm spacing

psf_ub   = psf_ds_2d(x_ub,   y_ub,   FREQ2, az2, el2)
psf_grid = psf_ds_2d(x_grid, y_grid, FREQ2, az2, el2)

psf_ub_db   = 10 * np.log10(np.maximum(psf_ub   / psf_ub.max(),   1e-10))
psf_grid_db = 10 * np.log10(np.maximum(psf_grid / psf_grid.max(), 1e-10))

# SLL: outside the main-lobe cone (radius > HPBW/2 ≈ 12–20°; use 25° to be safe)
AZ2, EL2 = np.meshgrid(az2, el2)
mask_sl = np.sqrt(AZ2 ** 2 + EL2 ** 2) > 25
sll_ub   = psf_ub_db[mask_sl].max()
sll_grid = psf_grid_db[mask_sl].max()

fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))

im1 = axes[0].pcolormesh(az2, el2, psf_ub_db, cmap="inferno", vmin=-40, vmax=0, shading="auto")
axes[0].set_title(f"Underbrink 96-mic\nSLL = {sll_ub:.1f} dB")
axes[0].set_xlabel("Azimuth (°)")
axes[0].set_ylabel("Elevation (°)")
plt.colorbar(im1, ax=axes[0], label="dB")

im2 = axes[1].pcolormesh(az2, el2, psf_grid_db, cmap="inferno", vmin=-40, vmax=0, shading="auto")
axes[1].set_title(f"10×10 Regular Grid (100 mics)\nSLL = {sll_grid:.1f} dB")
axes[1].set_xlabel("Azimuth (°)")
axes[1].set_ylabel("Elevation (°)")
plt.colorbar(im2, ax=axes[1], label="dB")

# Azimuth slice at el = 0
el0 = np.argmin(np.abs(el2))
sl_ub   = psf_ub_db[el0, :]
sl_grid = psf_grid_db[el0, :]
axes[2].plot(az2, sl_ub,   color="steelblue",  linewidth=1.8, label=f"Underbrink  (SLL = {sll_ub:.0f} dB)")
axes[2].plot(az2, sl_grid, color="darkorange", linewidth=1.8, label=f"Regular grid (SLL = {sll_grid:.0f} dB)")
axes[2].axhline(sll_ub,   color="steelblue",  linestyle="--", linewidth=1.0, alpha=0.7)
axes[2].axhline(sll_grid, color="darkorange", linestyle="--", linewidth=1.0, alpha=0.7)
axes[2].text(58, sll_ub   + 0.8, f"{sll_ub:.0f} dB",   color="steelblue",  ha="right", fontsize=8)
axes[2].text(58, sll_grid + 0.8, f"{sll_grid:.0f} dB", color="darkorange", ha="right", fontsize=8)
axes[2].set_xlabel("Azimuth (°)")
axes[2].set_ylabel("dB (normalized)")
axes[2].set_title("Azimuth slice (el = 0°)\nSidelobe suppression comparison")
axes[2].legend(fontsize=8)
axes[2].set_ylim(-40, 2)

fig.suptitle(
    f"Sidelobe Level (SLL) — D&S PSF @ {FREQ2/1e3:.0f} kHz  (both arrays below spatial Nyquist)\n"
    "Underbrink multi-arm log-spiral spreads energy uniformly;  "
    "regular grid concentrates it into periodic sidelobe ridges",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_sidelobe.png")
plt.close(fig)
print(f"  SLL: Underbrink={sll_ub:.1f} dB  Grid={sll_grid:.1f} dB")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 3 — Far-Field vs Near-Field
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 3: Far-Field vs Near-Field …")

D = 0.300   # 300 mm aperture
freqs_ff = np.array([500, 1000, 2000, 4000, 8000])
r_fraunhofer = 2 * D**2 * freqs_ff / C

# Left panel: delay profile across array for a source at various distances vs. far-field
N_MICS_1D = 50
x_line = np.linspace(-D/2, D/2, N_MICS_1D)
distances = [0.3, 0.5, 1.0, 5.0]   # metres
az_src = 30.0                        # degrees, far-field reference direction

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

# ── Left: wavefront curvature → delay across a 300 mm linear array ───────────
colors_nf = plt.cm.plasma(np.linspace(0.1, 0.9, len(distances)))
az_rad = np.radians(az_src)
for r, c in zip(distances, colors_nf):
    # True source at (r*sin(az), r*cos(az)) in 2D — time-of-flight to each mic
    src_x = r * np.sin(az_rad)
    src_y = r * np.cos(az_rad)
    tof = np.sqrt((x_line - src_x)**2 + src_y**2) / C
    delay_us = (tof - tof.min()) * 1e6
    ax1.plot(x_line * 1000, delay_us, color=c, linewidth=1.6, label=f"r = {r} m")

# Far-field plane-wave approximation
delay_ff = (x_line * np.sin(az_rad) / C) * 1e6
delay_ff -= delay_ff.min()
ax1.plot(x_line * 1000, delay_ff, "k--", linewidth=2, label="Far-field (plane wave)")
ax1.set_xlabel("Mic position along array (mm)")
ax1.set_ylabel("Time of arrival relative to ref (µs)")
ax1.set_title(f"Wavefront curvature: source at {az_src:.0f}°,\nvarious distances (300 mm aperture)")
ax1.legend(fontsize=8)

# ── Right: Fraunhofer distance vs frequency ───────────────────────────────────
f_range = np.linspace(200, 8000, 500)
r_ff = 2 * D**2 * f_range / C
ax2.fill_between(f_range / 1000, 0, r_ff, alpha=0.15, color="tomato", label="Near-field zone")
ax2.fill_between(f_range / 1000, r_ff, 30, alpha=0.10, color="steelblue", label="Far-field zone")
ax2.plot(f_range / 1000, r_ff, color="tomato", linewidth=2, label=f"Fraunhofer distance\nr = 2D²/λ  (D={D*1000:.0f} mm)")
for r_marker, lbl in [(0.5, "0.5 m"), (1.0, "1 m"), (2.0, "2 m"), (5.0, "5 m")]:
    ax2.axhline(r_marker, color="gray", linestyle=":", linewidth=0.8)
    ax2.text(8.1, r_marker, lbl, va="center", color="gray", fontsize=8)
ax2.set_xlim(0.2, 8.5)
ax2.set_ylim(0, 10)
ax2.set_xlabel("Frequency (kHz)")
ax2.set_ylabel("Distance from array (m)")
ax2.set_title("Fraunhofer distance: r = 2D²/λ\n(above this line → far-field, plane-wave steering)")
ax2.legend(fontsize=8)
ax2.text(
    0.5, 0.04,
    "Red zone: near-field — spherical-wave steering required\n"
    "Blue zone: far-field — plane-wave (angle-only) steering valid",
    transform=ax2.transAxes, fontsize=7.5, color="gray", va="bottom",
)

fig.tight_layout()
fig.savefig(OUT / "concept_farfield.png")
plt.close(fig)
print("  done")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 4 — Spatial Nyquist
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 4: Spatial Nyquist …")

# Use a 1D uniform line array with 60mm spacing so the Nyquist (~2.86 kHz) and
# grating lobes fall squarely within the audible range — much clearer pedagogy.
D_1D = 0.060    # 60 mm inter-element spacing
N_1D = 12       # 12 elements → 660 mm aperture
f_nyq = C / (2 * D_1D)
print(f"  1D demo array: {N_1D} elements, {D_1D*1000:.0f} mm spacing")
print(f"  Nyquist = {f_nyq:.0f} Hz  (grating lobe visible in FOV above {C/D_1D:.0f} Hz)")

x_1d = np.arange(N_1D, dtype=float) * D_1D
x_1d -= x_1d.mean()
az = np.linspace(-90, 90, 1801)

test_freqs = [1000, 2000, int(f_nyq), int(f_nyq * 2), int(f_nyq * 3.5)]
freq_labels = [
    f"1 kHz\n(0.35× Nyq)",
    f"2 kHz\n(0.70× Nyq)",
    f"{int(f_nyq)} Hz\n(Nyquist)",
    f"{int(f_nyq*2)} Hz\n(2× Nyq)",
    f"{int(f_nyq*3.5)} Hz\n(3.5× Nyq)",
]
colors_ny = ["steelblue", "seagreen", "darkorange", "tomato", "purple"]

fig, axes = plt.subplots(1, 5, figsize=(14, 4.2), sharey=True)
for ax, freq, title, col in zip(axes, test_freqs, freq_labels, colors_ny):
    p = psf_ds_1d(x_1d, freq, az)
    p_db = db_norm(p, floor=-40)
    ax.plot(az, p_db, color=col, linewidth=1.4)
    ax.set_title(title, fontsize=8.5)
    ax.set_xlabel("Az (°)", fontsize=8)
    ax.set_xlim(-90, 90)
    ax.set_ylim(-40, 2)
    # mark grating lobe positions when visible within FOV
    sin_gl = C / (freq * D_1D)   # sin(az_grating) = λ/d
    if sin_gl <= 1.0:
        az_gl = np.degrees(np.arcsin(sin_gl))
        for sign in [+1, -1]:
            ax.axvline(sign * az_gl, color="red", linestyle=":", linewidth=1.2, alpha=0.7)
        ax.text(az_gl, -30, f"±{az_gl:.0f}°", ha="center", color="red", fontsize=7.5,
                fontweight="bold")
        ax.axhspan(-40, 2, alpha=0.04, color="red")

axes[0].set_ylabel("Beam power (dB, normalized)")
for ax in axes[1:]:
    ax.tick_params(labelleft=False)

fig.suptitle(
    f"Spatial Nyquist — uniform 1D array, d = {D_1D*1000:.0f} mm,  f_Nyq = c/(2d) = {f_nyq/1000:.2f} kHz\n"
    "At Nyquist: grating lobe appears at ±90°.  "
    "Above Nyquist: lobe folds into the field of view (marked in red).",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_spatial_nyquist.png")
plt.close(fig)


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 5 — Low-Frequency Limit
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 5: Low-Frequency Limit …")

x_ub, y_ub = underbrink_array()
D_ap = 0.300
f_aperture = C / D_ap   # ≈ 1143 Hz

# HPBW vs frequency (empirical + theory)
freqs_sweep = np.array([200, 400, 600, 800, 1000, 1500, 2000, 3000, 4000, 6000, 8000])
az_fine = np.linspace(-90, 90, 3601)
hpbw_meas = []
for f in freqs_sweep:
    p = psf_ds_1d(x_ub, f, az_fine)
    hpbw_meas.append(measure_hpbw(az_fine, p))
hpbw_theory = np.degrees(0.886 * C / (freqs_sweep * D_ap))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

# ── Left: HPBW vs frequency ───────────────────────────────────────────────────
ax1.plot(freqs_sweep / 1000, hpbw_meas, "o-", color="steelblue",
         label="Measured (D&S)", linewidth=1.8, markersize=5)
ax1.plot(freqs_sweep / 1000, hpbw_theory, "--", color="gray",
         label="Theory: 0.886 λ/D", linewidth=1.5)
ax1.axvline(f_aperture / 1000, color="tomato", linestyle=":", linewidth=1.8,
            label=f"f = c/D = {f_aperture:.0f} Hz")
ax1.axhline(57, color="tomato", linestyle=":", linewidth=1.2, alpha=0.6)
ax1.text(f_aperture / 1000 + 0.1, 175, f"c/D = {f_aperture:.0f} Hz\n(HPBW > 57° below this)",
         color="tomato", fontsize=8)
ax1.set_xlabel("Frequency (kHz)")
ax1.set_ylabel("HPBW (degrees)")
ax1.set_title("HPBW vs Frequency  (300 mm aperture)")
ax1.set_ylim(0, 185)
ax1.legend(fontsize=8)
ax1.fill_betweenx([0, 185], 0, f_aperture / 1000, alpha=0.07, color="tomato",
                  label="Marginal directionality")

# ── Right: example beam patterns at 3 frequencies ────────────────────────────
az_beam = np.linspace(-90, 90, 1801)
show_freqs = [500, 1500, 4000]
show_colors = ["tomato", "darkorange", "steelblue"]
show_labels = ["500 Hz  (~omni)", "1500 Hz  (marginal)", "4000 Hz  (good)"]

for f, col, lbl in zip(show_freqs, show_colors, show_labels):
    p = psf_ds_1d(x_ub, f, az_beam)
    p_db = db_norm(p, floor=-40)
    ax2.plot(az_beam, p_db, color=col, linewidth=1.6, label=f"{lbl}\nHPBW≈{measure_hpbw(az_beam, p):.0f}°")

ax2.set_xlabel("Azimuth (degrees)")
ax2.set_ylabel("Beam power (dB, normalized)")
ax2.set_title("Beam patterns: 500 Hz / 1.5 kHz / 4 kHz")
ax2.legend(fontsize=8, loc="upper right")
ax2.set_xlim(-90, 90)
ax2.set_ylim(-40, 2)

fig.suptitle(
    "Low-Frequency Limit — useful directionality requires f > c/D (≈ 1.1 kHz for 300 mm aperture)",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_lowfreq_limit.png")
plt.close(fig)
print(f"  c/D = {f_aperture:.0f} Hz")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 6 — Cross-Spectral Matrix (CSM)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 6: CSM …")

# Use a compact subset (16 mics from Underbrink) for a readable matrix image
x_ub, y_ub = underbrink_array()
x16, y16 = x_ub[:16], y_ub[:16]

FREQ6 = 3000.0
snap_list = [1, 4, 16, 256]

fig, axes = plt.subplots(1, 5, figsize=(14, 3.5))

# First panel: matrix structure diagram
ax = axes[0]
# show the anatomy of a 16×16 CSM
R_full = make_csm_1d(x16, [(15.0, 1.0)], FREQ6, snr_db=20, n_snap=512)
im = ax.imshow(
    np.abs(R_full),
    cmap="viridis", aspect="auto",
    origin="upper",
)
ax.set_title("CSM structure\n|R| (N=16 mics)", fontsize=8)
ax.set_xlabel("Mic index", fontsize=8)
ax.set_ylabel("Mic index", fontsize=8)
# annotate diagonal
ax.plot([0, 15], [0, 15], color="white", linewidth=1.2, linestyle=":", alpha=0.7)
ax.text(8, -1.5, "← diagonal: auto-power →", ha="center", va="bottom",
        color="white", fontsize=7, clip_on=False)
plt.colorbar(im, ax=ax, fraction=0.05, pad=0.03)

# Panels 2-5: CSM magnitude for increasing N_SNAP
for ax, n_snap in zip(axes[1:], snap_list):
    R = make_csm_1d(x16, [(15.0, 1.0)], FREQ6, snr_db=15, n_snap=n_snap)
    im = ax.imshow(np.abs(R), cmap="viridis", aspect="auto", origin="upper",
                   vmin=0, vmax=np.abs(R_full).max())
    ax.set_title(f"N_snap = {n_snap}\n(rank-{'deficient' if n_snap < 16 else 'full'})", fontsize=8)
    ax.set_xlabel("Mic index", fontsize=8)
    ax.tick_params(left=False, labelleft=False)
    plt.colorbar(im, ax=ax, fraction=0.05, pad=0.03)

fig.suptitle(
    "Cross-Spectral Matrix (CSM)  —  R = (1/N) Σ y·yᴴ  at 3 kHz\n"
    "Left panel: CSM anatomy (16-mic subset);  right panels: CSM convergence with N_snap",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_csm.png")
plt.close(fig)
print("  done")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 7 — Point Spread Function (PSF)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 7: PSF …")

x_ub, y_ub = underbrink_array()
az = np.linspace(-90, 90, 3601)
FREQ7 = 4000.0
src_az = 20.0

# PSF: what D&S returns for a single point source at src_az
p_psf = psf_ds_1d(x_ub, FREQ7, az, src_az=src_az)
p_psf_db = db_norm(p_psf, floor=-40)

# True source: ideal delta-like indicator
p_true = np.zeros_like(az)
idx_true = np.argmin(np.abs(az - src_az))
p_true[idx_true] = 1.0

# Two-source scene: CLEAN-SC deconvolution sketch
def clean_sc_1d(x, R, freq, az_grid, n_iter=15, loop_gain=0.5):
    H = steer_1d(x, az_grid, freq)
    dirty = beamform_ds(R, H)
    clean_map = np.zeros_like(dirty)
    R_res = R.copy()
    for _ in range(n_iter):
        peak_idx = np.argmax(dirty)
        h_peak = H[:, peak_idx]
        peak_power = dirty[peak_idx]
        clean_map[peak_idx] += loop_gain * peak_power
        R_res -= loop_gain * peak_power * np.outer(h_peak, h_peak.conj())
        dirty = beamform_ds(R_res, H)
        dirty = np.maximum(dirty, 0)
    clean_map += dirty
    return clean_map


src_two = [(10.0, 1.0), (35.0, 0.6)]
R2 = make_csm_1d(x_ub, src_two, FREQ7, snr_db=20, n_snap=512)
H2 = steer_1d(x_ub, az, FREQ7)
p_ds2   = beamform_ds(R2, H2)
p_clean = clean_sc_1d(x_ub, R2, FREQ7, az)

fig, axes = plt.subplots(1, 3, figsize=(12, 4.2), sharey=False)

# Panel 1: PSF of a single point source
ax = axes[0]
ax.fill_between(az, -40, p_psf_db, alpha=0.2, color="steelblue")
ax.plot(az, p_psf_db, color="steelblue", linewidth=1.5)
ax.axvline(src_az, color="green", linestyle="--", linewidth=1.2, label=f"True source ({src_az}°)")
hpbw_psf = measure_hpbw(az, p_psf)
ax.set_title(f"PSF — single source @ {src_az}°\nHPBW = {hpbw_psf:.1f}°")
ax.set_xlabel("Azimuth (°)")
ax.set_ylabel("dB (normalized)")
ax.set_ylim(-40, 2)
ax.legend(fontsize=8)

# Panel 2: Two-source scene, D&S (PSF smearing masks the weaker source)
ax = axes[1]
p_ds2_db = db_norm(p_ds2, floor=-40)
ax.fill_between(az, -40, p_ds2_db, alpha=0.2, color="darkorange")
ax.plot(az, p_ds2_db, color="darkorange", linewidth=1.5, label="D&S (PSF smeared)")
for az_s, pwr in src_two:
    ax.axvline(az_s, color="green", linestyle="--", linewidth=1.2)
ax.set_title(f"Two sources @ {src_two[0][0]}° and {src_two[1][0]}°\nD&S (PSF sidelobes mask weaker peak)")
ax.set_xlabel("Azimuth (°)")
ax.set_ylim(-40, 2)
ax.legend(fontsize=8)
ax.text(10, -38, "True src", ha="center", color="green", fontsize=7)
ax.text(35, -38, "True src", ha="center", color="green", fontsize=7)

# Panel 3: Same scene, CLEAN-SC (deconvolved)
ax = axes[2]
p_clean_db = db_norm(p_clean, floor=-40)
ax.fill_between(az, -40, p_clean_db, alpha=0.2, color="seagreen")
ax.plot(az, p_clean_db, color="seagreen", linewidth=1.5, label="CLEAN-SC (deconvolved)")
for az_s, pwr in src_two:
    ax.axvline(az_s, color="green", linestyle="--", linewidth=1.2)
ax.set_title("Same scene — CLEAN-SC\n(PSF removed; sources revealed)")
ax.set_xlabel("Azimuth (°)")
ax.set_ylim(-40, 2)
ax.legend(fontsize=8)

fig.suptitle(
    f"Point Spread Function (PSF)  —  Underbrink 96-mic @ {FREQ7/1e3:.0f} kHz\n"
    "The PSF is the array's imprint on every source; deconvolution subtracts it to reveal weak secondary sources",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_psf.png")
plt.close(fig)
print("  done")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 8 — Incoherent Octave-Band Averaging
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 8: Incoherent Octave-Band Averaging …")

x_ub, y_ub = underbrink_array()
az = np.linspace(-60, 60, 1201)
src_az8 = 20.0
FC = 4000.0   # octave band centre frequency

def octave_freqs(fc, n=7):
    return np.exp(np.linspace(np.log(fc / np.sqrt(2)), np.log(fc * np.sqrt(2)), n))

freqs_band = octave_freqs(FC, n=7)
maps = []
for f in freqs_band:
    p = psf_ds_1d(x_ub, f, az, src_az=src_az8)
    maps.append(p)

avg_map = np.mean(maps, axis=0)

fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))

# Panel 1: individual per-frequency maps (overlaid)
ax = axes[0]
cmap_lines = plt.cm.cool(np.linspace(0, 1, len(freqs_band)))
for i, (f, p, col) in enumerate(zip(freqs_band, maps, cmap_lines)):
    p_db = db_norm(p, floor=-40)
    hpbw_i = measure_hpbw(az, p)
    ax.plot(az, p_db, color=col, linewidth=1.2, alpha=0.8,
            label=f"{f/1000:.1f} kHz ({hpbw_i:.0f}°)")
ax.axvline(src_az8, color="green", linestyle="--", linewidth=1.2, label=f"Source ({src_az8}°)")
ax.set_xlabel("Azimuth (°)")
ax.set_ylabel("dB (normalized)")
ax.set_title(f"Per-frequency D&S maps\n(7 bins within 1 octave of {FC/1000:.0f} kHz)")
ax.legend(fontsize=6.5, loc="upper right", ncol=2)
ax.set_ylim(-40, 2)

# Panel 2: average map vs single-frequency map at band center
ax = axes[1]
p_center = psf_ds_1d(x_ub, FC, az, src_az=src_az8)
ax.plot(az, db_norm(p_center, floor=-40), color="darkorange", linewidth=1.5,
        label=f"Single freq ({FC/1000:.0f} kHz)")
ax.plot(az, db_norm(avg_map, floor=-40), color="steelblue", linewidth=2.0,
        label=f"Incoherent avg (7 bins)")
ax.axvline(src_az8, color="green", linestyle="--", linewidth=1.2, label=f"Source ({src_az8}°)")
ax.set_xlabel("Azimuth (°)")
ax.set_ylabel("dB (normalized)")
ax.set_title("Band-averaged vs single-frequency\n(averaging suppresses sidelobes)")
ax.legend(fontsize=8)
ax.set_ylim(-40, 2)

# Panel 3: noise-floor comparison with Monte Carlo
ax = axes[2]
n_mc = 12
noise_single = []
noise_avg    = []
rng = np.random.default_rng(42)
for _ in range(n_mc):
    p_s = psf_ds_1d(x_ub, FC, az, src_az=src_az8)
    noise_single.append(p_s[np.abs(az) > 35].mean())
    p_a_list = [psf_ds_1d(x_ub, f, az, src_az=src_az8) for f in freqs_band]
    noise_avg.append(np.mean(p_a_list, axis=0)[np.abs(az) > 35].mean())

# noise suppression factor (theory: 1/sqrt(K))
K = len(freqs_band)
suppression = 10 * np.log10(1.0 / np.sqrt(K))
labels_mc = ["Single\nfrequency", f"Avg over\n{K} bins"]
means_mc  = [10 * np.log10(np.mean(noise_single) / np.max(avg_map)),
             10 * np.log10(np.mean(noise_avg)    / np.max(avg_map))]

bars = ax.bar(labels_mc, means_mc, color=["darkorange", "steelblue"], width=0.5)
ax.text(
    0.5, 0.55,
    f"Noise suppression ≈ {abs(means_mc[0]-means_mc[1]):.1f} dB\n"
    f"Theory: 1/√{K} ≈ {suppression:.1f} dB",
    transform=ax.transAxes, ha="center", va="center",
    fontsize=9, color="gray",
    bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.7),
)
ax.set_ylabel("Off-axis noise floor (dB, normalized)")
ax.set_title("Noise-floor improvement\nfrom octave-band averaging")

fig.suptitle(
    f"Incoherent Octave-Band Averaging  —  Underbrink 96-mic @ {FC/1000:.0f} kHz octave band\n"
    "One CSM per freq bin → beamform each → incoherently average power;  "
    "suppresses noise by ≈ 1/√K and broadens PSF across the band",
    fontsize=9,
)
fig.tight_layout()
fig.savefig(OUT / "concept_octave_averaging.png")
plt.close(fig)
print("  done")


print(f"\nAll 8 figures saved to {OUT.resolve()}")
