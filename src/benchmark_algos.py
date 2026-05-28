#!/usr/bin/env python3
"""Benchmark D&S, MVDR, CLEAN-SC, and MUSIC on a recorded UMA-16 WAV file.

Reports mean CPU time, peak allocated memory, and real-time feasibility
for each algorithm on the actual array geometry and grid.

Usage:
    python src/benchmark_algos.py
    python src/benchmark_algos.py --freq 3000 --iters 100
    python src/benchmark_algos.py --algos ds mvdr --clean_iters 10
    python src/benchmark_algos.py --audio test/UMA16/capture_nb16.wav --fps 30
"""
import argparse
import sys
import time
import tracemalloc
from pathlib import Path

import numpy as np
import scipy.io.wavfile as wavfile
from scipy.linalg import inv

# --- UMA-16 v2 geometry (mirrors acoustic_camera_p3.py) ---
C = 343.0
FS = 48000
N_MICS = 16
_d = 0.042

_xy = np.array([
    (-1, -3), (-3, -3), (-1, -1), (-3, -1),
    (-1, +1), (-3, +1), (-1, +3), (-3, +3),
    (+3, +3), (+1, +3), (+3, +1), (+1, +1),
    (+3, -1), (+1, -1), (+3, -3), (+1, -3),
], dtype=float) * (_d / 2)

MIC_X = -_xy[:, 0]   # negated: Figure 1 is sound-source side; camera side is x-mirrored
MIC_Y = _xy[:, 1]
NYQUIST = C / (2 * _d)


# --- steering matrix and algorithms (identical to acoustic_camera_p3.py) ---

def _sm(az_grid, el_grid, freq):
    sin_az = np.sin(np.radians(az_grid))
    cos_el = np.cos(np.radians(el_grid))
    sin_el = np.sin(np.radians(el_grid))
    ux = np.outer(sin_az, cos_el).ravel()
    uy = np.tile(sin_el, len(az_grid))
    phase = 2 * np.pi * freq / C * (np.outer(MIC_X, ux) + np.outer(MIC_Y, uy))
    return np.exp(1j * phase) / np.sqrt(N_MICS)


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
    _, V = np.linalg.eigh(R)
    En = V[:, :N_MICS - n_src]
    H = _sm(az_grid, el_grid, freq)
    proj = En.conj().T @ H
    return 1.0 / np.maximum(np.real(np.sum(proj.conj() * proj, axis=0)), 1e-300)


def compute_csm(audio, freq, block_size=256, hop=128):
    n_samp, n_ch = audio.shape
    freqs = np.fft.rfftfreq(block_size, 1.0 / FS)
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


# --- benchmarking ---

def bench(fn, n_iters):
    """Return (mean_ms, std_ms, peak_mem_MB).

    peak_mem_MB is the tracemalloc Python-heap peak for one call after warmup.
    C-level LAPACK/BLAS buffers are not captured, so treat it as a lower bound.
    """
    for _ in range(3):
        fn()

    times = []
    for _ in range(n_iters):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1e3)

    tracemalloc.start()
    fn()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return float(np.mean(times)), float(np.std(times)), peak / 1024 / 1024


def fmt_row(label, mean_ms, std_ms, mem_mb, rt_pct, note=''):
    status = '✓' if rt_pct < 100 else '✗'
    return (f'{label:<12}  {mean_ms:>8.1f}  {std_ms:>7.1f}  '
            f'{mem_mb:>8.1f}  {rt_pct:>7.0f}%  {status}  {note}')


def main():
    repo = Path(__file__).parent.parent
    default_audio = str(repo / 'test' / 'UMA16' / 'capture_nb16.wav')
    default_cal   = str(repo / 'test' / 'UMA16' / 'cal.npy')

    ap = argparse.ArgumentParser(description='Benchmark beamforming algorithms on UMA-16 audio')
    ap.add_argument('--audio',       default=default_audio,
                    help=f'WAV file path (default: {default_audio})')
    ap.add_argument('--freq',        type=float, default=3000.0,
                    help='beamforming frequency in Hz (default: 3000)')
    ap.add_argument('--snap',        type=int,   default=128,
                    help='CSM snapshot blocks to average (default: 128)')
    ap.add_argument('--iters',       type=int,   default=50,
                    help='benchmark iterations per algorithm (default: 50)')
    ap.add_argument('--algos',       nargs='+',  default=['ds', 'mvdr', 'clean', 'music'],
                    choices=['ds', 'mvdr', 'clean', 'music'],
                    help='algorithms to benchmark (default: all four)')
    ap.add_argument('--nsrc',        type=int,   default=1,
                    help='signal subspace dimension for MUSIC (default: 1)')
    ap.add_argument('--clean_iters', type=int,   default=20,
                    help='CLEAN-SC loop iterations (default: 20)')
    ap.add_argument('--az_fov',      type=float, default=90.0,
                    help='azimuth FOV in degrees (default: 90)')
    ap.add_argument('--el_fov',      type=float, default=60.0,
                    help='elevation FOV in degrees (default: 60)')
    ap.add_argument('--grid_deg',    type=float, default=0.5,
                    help='grid spacing in degrees (default: 0.5 → 181×121; 1.0 → 91×61)')
    ap.add_argument('--fps',         type=float, default=20.0,
                    help='target display frame rate for RT check (default: 20)')
    ap.add_argument('--cal',         default=default_cal,
                    help=f'calibration vector .npy (default: {default_cal}; pass "" to skip)')
    args = ap.parse_args()

    az_grid = np.linspace(-args.az_fov / 2, args.az_fov / 2,
                          round(args.az_fov / args.grid_deg) + 1)
    el_grid = np.linspace(-args.el_fov / 2, args.el_fov / 2,
                          round(args.el_fov / args.grid_deg) + 1)
    n_grid = len(az_grid) * len(el_grid)
    frame_budget_ms = 1000.0 / args.fps

    # ── load audio ──────────────────────────────────────────────────────────
    sr, raw = wavfile.read(args.audio)
    if raw.ndim == 1:
        sys.exit('ERROR: WAV must be multi-channel (got mono).')
    if raw.shape[1] != N_MICS:
        sys.exit(f'ERROR: expected {N_MICS} channels, got {raw.shape[1]}.')
    if sr != FS:
        print(f'[warn] WAV sample rate {sr} Hz != expected {FS} Hz — proceeding anyway')

    audio = raw.astype(np.float32)
    if np.abs(audio).max() > 1.0:
        audio /= 32768.0

    n_samp = audio.shape[0]
    n_buf  = args.snap * 128 + 256
    if n_samp < n_buf:
        sys.exit(f'ERROR: audio too short — need {n_buf} samples ({n_buf/sr:.2f} s), '
                 f'got {n_samp} ({n_samp/sr:.2f} s).')

    cal_path = args.cal if args.cal else None
    cal_e    = np.load(cal_path) if (cal_path and Path(cal_path).exists()) else None

    # ── header ──────────────────────────────────────────────────────────────
    sm_mb  = N_MICS * n_grid * 16 / 1024 / 1024   # complex128
    csm_kb = N_MICS * N_MICS * 16 / 1024
    print()
    print('UMA-16 Beamforming Benchmark')
    print('─' * 70)
    print(f'Audio    : {Path(args.audio).name}  '
          f'({n_samp/sr:.1f} s, {raw.shape[1]} ch, {sr} Hz)')
    print(f'Freq     : {args.freq:.0f} Hz  '
          f'(spatial Nyquist = {NYQUIST:.0f} Hz'
          + ('  ← operating above Nyquist' if args.freq > NYQUIST else '') + ')')
    print(f'Grid     : {len(az_grid)} az × {len(el_grid)} el = {n_grid:,} points  '
          f'(±{args.az_fov/2:.0f}° az, ±{args.el_fov/2:.0f}° el)')
    print(f'CSM      : {N_MICS}×{N_MICS} complex128 = {csm_kb:.1f} kB')
    print(f'Steer    : {N_MICS}×{n_grid:,} complex128 = {sm_mb:.1f} MB  (allocated per call)')
    print(f'Snaps    : {args.snap}  |  Iters: {args.iters}  |  '
          f'Frame budget: {frame_budget_ms:.0f} ms ({args.fps:.0f} fps)')
    print(f'Cal      : {"applied — " + cal_path if cal_e is not None else "none"}')
    print('─' * 70)

    # ── CSM benchmark ───────────────────────────────────────────────────────
    audio_slice = audio[:n_buf]
    print('Benchmarking CSM...', end=' ', flush=True)
    csm_mean, csm_std, csm_mem = bench(
        lambda: compute_csm(audio_slice, args.freq), args.iters
    )
    print('done')

    # Compute the CSM used for beamforming benchmarks
    R = compute_csm(audio_slice, args.freq)
    if cal_e is not None:
        c = 1.0 / cal_e
        R = np.outer(c, c.conj()) * R

    # ── algorithm benchmarks ─────────────────────────────────────────────────
    algo_fns = {
        'ds':    ('D&S',      lambda: beamform_ds(R, args.freq, az_grid, el_grid)),
        'mvdr':  ('MVDR',     lambda: beamform_mvdr(R, args.freq, az_grid, el_grid)),
        'clean': ('CLEAN-SC', lambda: beamform_clean(R, args.freq, az_grid, el_grid,
                                                      n_iter=args.clean_iters)),
        'music': ('MUSIC',    lambda: beamform_music(R, args.freq, az_grid, el_grid,
                                                      n_src=args.nsrc)),
    }

    print('Benchmarking algorithms...', end=' ', flush=True)
    results = {}
    for key in args.algos:
        label, fn = algo_fns[key]
        results[key] = (label,) + bench(fn, args.iters)
    print('done\n')

    # ── results table ────────────────────────────────────────────────────────
    hdr = f'{"algo":<12}  {"mean ms":>8}  {"std ms":>7}  {"peak MB":>8}  {"RT%":>7}   {"status"}'
    sep = '─' * len(hdr)
    print(hdr)
    print(sep)

    # CSM row
    csm_rt = 100.0 * csm_mean / frame_budget_ms
    print(fmt_row('CSM', csm_mean, csm_std, csm_mem, csm_rt, f'({args.snap} snaps)'))

    for key in args.algos:
        label, mean_ms, std_ms, mem_mb = results[key]
        rt_pct = 100.0 * mean_ms / frame_budget_ms
        notes = []
        if key == 'clean':
            notes.append(f'{args.clean_iters} iters')
        if key == 'music':
            notes.append(f'nsrc={args.nsrc}')
        print(fmt_row(label, mean_ms, std_ms, mem_mb, rt_pct, ', '.join(notes)))

    print(sep)
    # pipeline total = CSM + chosen algo
    for key in args.algos:
        label, mean_ms, std_ms, mem_mb = results[key]
        total = csm_mean + mean_ms
        rt_total = 100.0 * total / frame_budget_ms
        ok = '✓' if rt_total < 100 else '✗'
        print(f'  pipeline ({label:<8}): {total:>7.1f} ms total  '
              f'({rt_total:.0f}% of {frame_budget_ms:.0f} ms budget)  {ok}')

    print()
    print(f'RT% = time / frame budget ({frame_budget_ms:.0f} ms at {args.fps:.0f} fps).  '
          f'< 100% = real-time feasible.')
    print('Peak MB = tracemalloc Python-heap peak per call '
          '(C/LAPACK internals not counted; actual RSS will be higher).')
    if args.freq > NYQUIST:
        print(f'[warn] {args.freq:.0f} Hz exceeds spatial Nyquist '
              f'({NYQUIST:.0f} Hz) — grating lobes are present in the map.')
    print()


if __name__ == '__main__':
    main()
