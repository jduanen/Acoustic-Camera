"""Shared UMA-16 beamforming core (Phase 3+).

Single source of truth for the array geometry, steering matrix, the four
beamformers, and CSM computation. Used by acoustic_camera_p3.py,
benchmark_algos.py, and the pytest suite (tests/).
"""
import numpy as np
from scipy.linalg import inv

# --- UMA-16 v2 array geometry: 4×4 URA, 42 mm spacing, center-referenced ---
# Channel → (x, y) mapping from manual Figure 1 (sound-source side view).
# Mics are grouped as PDM L/R pairs sharing one data line.
# Verify by playing a tone from boresight: peak should land at az≈0°, el≈0°.
C = 343.0
FS = 48000
N_MICS = 16
_d = 0.042   # grid spacing [m]

# positions in units of _d/2 = 21 mm; multiply to get meters
_xy = np.array([
    (-1, -3),   # ch0  MIC1
    (-3, -3),   # ch1  MIC2
    (-1, -1),   # ch2  MIC3
    (-3, -1),   # ch3  MIC4
    (-1, +1),   # ch4  MIC5
    (-3, +1),   # ch5  MIC6
    (-1, +3),   # ch6  MIC7
    (-3, +3),   # ch7  MIC8
    (+3, +3),   # ch8  MIC9
    (+1, +3),   # ch9  MIC10
    (+3, +1),   # ch10 MIC11
    (+1, +1),   # ch11 MIC12
    (+3, -1),   # ch12 MIC13
    (+1, -1),   # ch13 MIC14
    (+3, -3),   # ch14 MIC15
    (+1, -3),   # ch15 MIC16
], dtype=float) * (_d / 2)

MIC_X = -_xy[:, 0]  # negated: Figure 1 is sound-source side; camera side is x-mirrored
MIC_Y = _xy[:, 1]   # (16,) meters
NYQUIST = C / (2 * _d)  # spatial Nyquist ~4082 Hz


# --- Beamforming ---

def _sm(az_grid, el_grid, freq):
    """2D steering matrix, shape (N_mics, N_az * N_el).

    Convention: x = horizontal (right = positive az), y = vertical (up = positive el).
    """
    sin_az = np.sin(np.radians(az_grid))   # (N_az,)
    cos_el = np.cos(np.radians(el_grid))   # (N_el,)
    sin_el = np.sin(np.radians(el_grid))   # (N_el,)
    # ux[i,j] = sin(az[i])*cos(el[j]);  uy[j] = sin(el[j])
    ux = np.outer(sin_az, cos_el).ravel()  # (N_az*N_el,)
    uy = np.tile(sin_el, len(az_grid))     # (N_az*N_el,)
    phase = 2 * np.pi * freq / C * (np.outer(MIC_X, ux) + np.outer(MIC_Y, uy))
    return np.exp(1j * phase) / np.sqrt(N_MICS)   # (N_mics, N_az*N_el)


def beamform_ds(R, freq, az_grid, el_grid):
    H = _sm(az_grid, el_grid, freq)
    return np.real(np.sum(H.conj() * (R @ H), axis=0))


def beamform_mvdr(R, freq, az_grid, el_grid, diag_load=0.01):
    Rl = R + diag_load * np.trace(R) / N_MICS * np.eye(N_MICS)
    Ri = inv(Rl)
    H = _sm(az_grid, el_grid, freq)
    return 1.0 / np.maximum(np.real(np.sum(H.conj() * (Ri @ H), axis=0)), 1e-300)


def beamform_clean(R, freq, az_grid, el_grid, n_iter=20, loop_gain=0.5):
    H = _sm(az_grid, el_grid, freq)
    R_w = R.copy()
    clean = np.zeros(len(az_grid) * len(el_grid))
    for _ in range(n_iter):
        P = np.real(np.sum(H.conj() * (R_w @ H), axis=0))
        k = np.argmax(P)
        h = H[:, k]
        P_src = np.real(h.conj() @ (R_w @ h))
        R_w -= loop_gain * P_src * np.outer(h, h.conj())
        clean[k] += loop_gain * P_src
    return clean


def beamform_music(R, freq, az_grid, el_grid, n_src=1):
    _, V = np.linalg.eigh(R)          # ascending eigenvalues
    En = V[:, :N_MICS - n_src]        # noise subspace
    H = _sm(az_grid, el_grid, freq)
    proj = En.conj().T @ H
    return 1.0 / np.maximum(np.real(np.sum(proj.conj() * proj, axis=0)), 1e-300)


def compute_csm(audio, freq, fs=FS, block_size=256, hop=128):
    n_samp, n_ch = audio.shape
    freqs = np.fft.rfftfreq(block_size, 1.0 / fs)
    f_idx = np.argmin(np.abs(freqs - freq))
    R = np.zeros((n_ch, n_ch), dtype=complex)
    count = 0
    win = np.hanning(block_size)
    for start in range(0, n_samp - block_size, hop):
        block = audio[start:start + block_size] * win[:, np.newaxis]
        F = np.fft.rfft(block, axis=0)[f_idx]
        R += np.outer(F, F.conj())
        count += 1
    return R / max(count, 1)
