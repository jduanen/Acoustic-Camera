#!/usr/bin/env python3
"""fir_design.py -- designs the CIC compensation FIR and provides its
bit-exact fixed-point golden model. Single source of truth for both the RTL
coefficient ROM (fir_compensator.v, via $readmemh) and the golden simulator
tb_fir_compensator.v's testbench checks against (fir_bitexact()).

System context (see PHASE4.md): 5-stage CIC, R=64, 3.072 MHz -> 48 kHz has a
sinc^5 passband droop; this FIR corrects it over the 0-8 kHz audio band the
IM72D128 mics/array are designed for (not the full 0-24kHz output Nyquist --
PHASE4.md's compensation target is the passband, not full-spectrum flatness).

Fixed-point convention (v1, not specified anywhere else in the repo -- chosen
here, documented, not hidden):
  - CIC output is 31 bits unsigned (Hogenauer bound for STAGES=5,R=64). FIR
    input is the top 24 bits of that (>> 7) -- a simple truncating shift down
    to a 24-bit working width, matching the system's "24-bit PCM" convention
    used everywhere else in PHASE4.md/DESIGN.md.
  - 32 taps, 18-bit signed coefficients in Q1.17 (unity gain ~ 2**17).
  - 48-bit accumulator -- deliberately matches a Xilinx DSP48E1's native
    accumulator width (this is *why* 18-bit coefficients / 24-bit data were
    chosen: 24x18 fits a DSP48E1's native multiplier, 48-bit ACC is its native
    width -- worst-case 32-tap sum needs ~47 bits, fits with 1 bit to spare).
  - Final output = (accumulator >> 17), then masked to 24 bits. No
    saturation -- v1 wraps on overflow like the CIC stage does, for
    consistency. This is a real limitation for actual audio use (wraparound
    distortion on overload) flagged here rather than silently accepted;
    revisit before hardware bring-up.
"""
import math
import os

import numpy as np
from scipy.signal import firwin2, freqz

FS_IN = 3.072e6      # PDM clock / CIC input rate (Hz)
R = 64                # CIC decimation ratio
STAGES = 5             # CIC stages
FS_OUT = FS_IN / R    # 48 kHz
PASSBAND_HZ = 8000.0   # compensation target band

NTAPS = 32
COEFF_WIDTH = 18
DATA_WIDTH = 24
ACC_WIDTH = 48
FRAC_BITS = COEFF_WIDTH - 1  # Q1.17

VEC_DIR = os.path.join(os.path.dirname(__file__), "..", "vectors")


def cic_response(f, fs_in=FS_IN, r=R, stages=STAGES):
    """CIC decimator magnitude response (normalized to 1 at DC), evaluated at
    output-referred frequency f (Hz)."""
    f = np.asarray(f, dtype=float)
    num = np.sin(np.pi * f * r / fs_in)
    den = np.sin(np.pi * f / fs_in)
    with np.errstate(divide="ignore", invalid="ignore"):
        h = np.where(np.abs(f) < 1e-9, float(r), num / np.where(den == 0, 1e-30, den))
    return (np.abs(h) / r) ** stages


def design_compensation_fir(ntaps=NTAPS, fs_out=FS_OUT, passband_hz=PASSBAND_HZ):
    """scipy.signal.firwin2-designed FIR approximating 1/cic_response(f) over
    the passband, tapered off above it (no attempt to invert the CIC's
    response near/at Nyquist, which is neither necessary nor well-conditioned)."""
    nyq = fs_out / 2.0
    npts = 1024
    freqs = np.linspace(0, nyq, npts)
    cic_mag = cic_response(freqs, fs_in=FS_IN, r=R, stages=STAGES)

    inv = 1.0 / np.maximum(cic_mag, 1e-6)
    pass_idx = np.searchsorted(freqs, passband_hz)
    inv_at_edge = inv[pass_idx]
    # Taper the desired gain from the passband edge down to exactly 0 at
    # Nyquist (raised-cosine) -- an even-length (Type II) linear-phase FIR is
    # mathematically required to have zero gain at Nyquist, and this also
    # avoids chasing the CIC's own response toward its first null (which sits
    # well outside this band anyway, at f=FS_OUT).
    taper = np.where(
        freqs <= passband_hz, 1.0,
        np.cos(0.5 * np.pi * np.clip((freqs - passband_hz) / (nyq - passband_hz), 0, 1)) ** 2,
    )
    desired = np.where(freqs <= passband_hz, inv, inv_at_edge * taper)
    desired[-1] = 0.0

    coeffs = firwin2(ntaps, freqs / nyq, desired)
    return coeffs


def quantize_coeffs(coeffs_float, width=COEFF_WIDTH, frac_bits=FRAC_BITS):
    scale = 2 ** frac_bits
    qmax = 2 ** (width - 1) - 1
    qmin = -(2 ** (width - 1))
    ints = [int(np.clip(round(c * scale), qmin, qmax)) for c in coeffs_float]
    return ints


def fir_bitexact(data_in, coeffs_int, data_width=DATA_WIDTH, acc_width=ACC_WIDTH,
                  frac_bits=FRAC_BITS):
    """Bit-exact fixed-point FIR matching fir_compensator.v's MAC engine.

    data_in: iterable of ints, one data_width-bit *unsigned* sample per
    output-rate cycle (already CIC-output>>7 -- see module docstring).
    coeffs_int: length-N list of signed coeff_width-bit ints, index 0 = the
    tap applied to the newest sample (matches the RTL shift register /
    coefficient ROM addressing in fir_compensator.v).
    Returns a list of data_width-bit outputs, masked (wrapped) not saturated.
    """
    n = len(coeffs_int)
    data_mask = (1 << data_width) - 1
    acc_mask = (1 << acc_width) - 1
    shift_reg = [0] * n
    out = []
    for x in data_in:
        shift_reg = [x & data_mask] + shift_reg[:-1]
        acc = 0
        for i in range(n):
            acc += shift_reg[i] * coeffs_int[i]
        acc &= acc_mask
        if acc & (1 << (acc_width - 1)):
            signed_acc = acc - (1 << acc_width)
        else:
            signed_acc = acc
        shifted = signed_acc >> frac_bits  # arithmetic shift (Python >> on int)
        out.append(shifted & data_mask)
    return out


def combined_response_summary(coeffs_int):
    """Prints the combined CIC+FIR magnitude response flatness over the
    passband -- the "done" check for the filter design itself, independent of
    RTL bit-exactness (that's tb_fir_compensator.v's job)."""
    nyq = FS_OUT / 2.0
    freqs = np.linspace(1.0, PASSBAND_HZ, 500)
    cic_mag = cic_response(freqs)

    coeffs_float = np.array(coeffs_int) / (2 ** FRAC_BITS)
    w, h = freqz(coeffs_float, worN=freqs, fs=FS_OUT)
    fir_mag = np.abs(h)

    combined_db = 20 * np.log10(np.maximum(cic_mag * fir_mag, 1e-12))
    combined_db -= combined_db[0]  # normalize to 0 dB at low frequency
    ripple_db = combined_db.max() - combined_db.min()

    print(f"Combined CIC(stages={STAGES},R={R}) + FIR({NTAPS}-tap) response, "
          f"0-{PASSBAND_HZ:.0f} Hz:")
    print(f"  ripple = {ripple_db:.3f} dB (min={combined_db.min():.3f} dB, "
          f"max={combined_db.max():.3f} dB)")
    return ripple_db


def gen_fir_vectors():
    os.makedirs(VEC_DIR, exist_ok=True)
    coeffs_float = design_compensation_fir()
    coeffs_int = quantize_coeffs(coeffs_float)

    hex_digits = (COEFF_WIDTH + 3) // 4
    path = os.path.join(VEC_DIR, "fir_coeffs.mem")
    with open(path, "w") as f:
        for c in coeffs_int:
            f.write(f"{c & ((1 << COEFF_WIDTH) - 1):0{hex_digits}x}\n")
    print(f"Wrote {len(coeffs_int)} coefficients ({COEFF_WIDTH}-bit, Q1.{FRAC_BITS}) -> {path}")
    return coeffs_int


if __name__ == "__main__":
    coeffs_int = gen_fir_vectors()
    ripple = combined_response_summary(coeffs_int)
    tol_db = 3.0
    status = "OK" if ripple <= tol_db else "OUT OF TOLERANCE"
    print(f"Tolerance: {tol_db} dB -> {status}")
