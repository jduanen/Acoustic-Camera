#!/usr/bin/env python3
"""gen_pdm_stimulus.py -- synthetic multi-tone PDM stimulus + the full
CIC->FIR golden reference per channel, used by fpga/cluster/sim/tb_cluster_top.v
(the end-to-end integration test) via gen_vectors.py's gen_cluster_top_vectors().

This is a functional/structural test, not an audio-quality one: what matters
is that the RTL processes a given PDM bitstream identically to this same
bitstream run through the already-validated cic_bitexact()/fir_bitexact()
golden models (each independently verified against their own testbenches).
The 1st-order noise-shaping modulator below is standard and good enough to
produce a non-trivial, distinguishable-per-channel stimulus; it is not meant
to model the IM72D128's real SNR/noise floor.
"""
import numpy as np

from cic_golden import cic_bitexact
from fir_design import DATA_WIDTH as FIR_DATA_WIDTH, fir_bitexact

FS_IN = 3.072e6  # PDM clock
R = 64
STAGES = 5
N_CH = 24


def delta_sigma_1bit(x):
    """1st-order noise-shaping 1-bit modulator. x: array in (-1, 1).
    Returns a uint8 0/1 array -- density of 1s tracks x's level, matching the
    IM72D128 datasheet's "positive pressure increases density of 1s"."""
    n = len(x)
    bits = np.zeros(n, dtype=np.uint8)
    integ = 0.0
    fb = -1.0
    for i in range(n):
        integ += x[i] - fb
        b = 1 if integ >= 0 else 0
        bits[i] = b
        fb = 1.0 if b else -1.0
    return bits


def gen_multitone_pdm(n_ch=N_CH, n_frames=20, r=R, fs=FS_IN):
    """One distinct tone per channel (500 Hz + 137*c Hz, all comfortably
    within the FIR's 0-8kHz compensation passband), amplitude/phase varied
    per channel too so no two channels are identical -- catches
    channel-order/wiring bugs a single shared tone would hide.

    Returns (pdm_bits, n_samples) where pdm_bits is a list of n_ch uint8
    arrays, each n_frames*r bits long.
    """
    n_samples = n_frames * r
    t = np.arange(n_samples) / fs
    pdm_bits = []
    for c in range(n_ch):
        freq = 500.0 + 137.0 * c
        amp = 0.5 + 0.02 * (c % 5)
        sig = amp * np.sin(2 * np.pi * freq * t + 0.3 * c)
        pdm_bits.append(delta_sigma_1bit(sig))
    return pdm_bits, n_samples


def golden_channel_pipeline(pdm_bits_1ch, coeffs_int, stages=STAGES, r=R, is_r_channel=False,
                             extra_reg_shift=False):
    """One channel's PDM bits -> bit-exact CIC -> bit-exact FIR -> final
    24-bit PCM samples, mirroring cluster_top.v's CIC->FIR wiring exactly
    (top FIR_DATA_WIDTH bits of the CIC's output, see fir_design.py's module
    docstring for why).

    is_r_channel: historical option for the OLD pdm_line_demux.v +
    cic_decimator.v pipeline (still present as standalone-tested modules,
    but no longer instantiated by cluster_top.v -- see its header comment).
    That pipeline captured L on the negedge and R on the posedge of the
    single shared clock, while cic_decimator's own accumulation was
    posedge-triggered -- a posedge-captured signal (R) updates on the *same*
    edge cic_decimator samples it on, and per standard nonblocking-
    assignment semantics a downstream posedge register can never observe an
    upstream posedge register's new value on that same edge (ordinary,
    correct flip-flop-to-flip-flop timing, not a simulation artifact). So
    that pipeline's R channel actually accumulated bit[i-1] where its L
    channel accumulated bit[i]; is_r_channel=True reproduces that by
    shifting the golden R-channel bitstream right by one.

    extra_reg_shift: cluster_top.v's CURRENT pipeline (pdm_line_sync.v +
    cic_decimator_shared.v). cic_decimator_shared reads pdm_line_sync's
    bit_r/phase_r outputs as a plain wire, and both modules' always blocks
    trigger on the same clk edge -- same same-edge-invisibility reasoning as
    above, giving one more registered hop of latency than
    pdm_line_sync's own capture alone. Empirically (verified against RTL
    simulation, not derived by inspection -- see the tb_cluster_top.v
    debugging session that found this), this extra hop's effect is *not*
    symmetric between L and R: L (even channels, phase=0) needs this shift
    (extra_reg_shift=True), R (odd, phase=1) does not
    (extra_reg_shift=False) -- use is_r_channel-style parity
    (extra_reg_shift = channel index even) when generating vectors for this
    pipeline. Ordinary, unavoidable reg-to-reg pipeline latency, not a bug,
    so corrected here rather than in the RTL -- see gen_vectors.py's
    gen_cluster_top_vectors().
    """
    bits = [int(b) for b in pdm_bits_1ch]
    if is_r_channel or extra_reg_shift:
        bits = [0] + bits[:-1]
    cic_out, width = cic_bitexact(bits, stages, r)
    shift = width - FIR_DATA_WIDTH
    mask = (1 << FIR_DATA_WIDTH) - 1
    fir_in = [(v >> shift) & mask for v in cic_out]
    return fir_bitexact(fir_in, coeffs_int)


if __name__ == "__main__":
    from fir_design import design_compensation_fir, quantize_coeffs

    pdm_bits, n_samples = gen_multitone_pdm(n_frames=20)
    coeffs = quantize_coeffs(design_compensation_fir())
    outs = [golden_channel_pipeline(b, coeffs, extra_reg_shift=(c % 2 == 0))
            for c, b in enumerate(pdm_bits)]
    print(f"Generated {len(pdm_bits)} channels x {n_samples} PDM bits -> "
          f"{len(outs[0])} PCM samples/channel each")
    print(f"Channel 0 first 5 PCM samples: {outs[0][:5]}")
    print(f"Channel 23 first 5 PCM samples: {outs[23][:5]}")
    assert outs[0][:5] != outs[23][:5], "channels 0 and 23 must differ (distinct tones)"
    print("Self-check OK: distinct channels produce distinct output.")
