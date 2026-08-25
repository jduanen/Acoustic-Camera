#!/usr/bin/env python3
"""gen_vectors.py -- generates checked-in .mem stimulus/expected-output
fixtures (fpga/single_fpga/vectors/) for tb_gbe_pipeline.v, the end-to-end
integration test covering the real 96-channel CIC+FIR pipeline
(single_fpga_pipeline_top.v, reusing fpga/multi_fpga/cluster/rtl's pdm_line_demux.v/
cic_decimator.v/fir_compensator.v unmodified) feeding the real
gbe_packetizer.v.

Reuses fpga/multi_fpga/cluster/golden's already-independently-verified CIC/FIR golden
models and multi-tone PDM stimulus generator via a path insert below, rather
than duplicating that math -- this pipeline is built from the exact same,
unmodified RTL modules (see single_fpga_pipeline_top.v's header comment), so
the exact same Python reference applies. Source of truth for the packetizer
half is gbe_packetizer_golden.py (this directory).

Re-run this script and re-check-in vectors/ output whenever a golden model
changes. tb_gbe_pipeline.v hardcodes the sizes generated here (N_FRAMES);
keep them in sync if this config changes.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "multi_fpga", "cluster", "golden"))
from fir_design import DATA_WIDTH as FIR_DATA_WIDTH, design_compensation_fir, quantize_coeffs  # noqa: E402
from gen_pdm_stimulus import gen_multitone_pdm  # noqa: E402
from cic_golden import cic_bitexact  # noqa: E402
from fir_design import fir_bitexact  # noqa: E402

from gbe_packetizer_golden import N_CH, gen_packets  # noqa: E402

VEC_DIR = os.path.join(os.path.dirname(__file__), "..", "vectors")

N_FRAMES = 17  # 3 full 5-frame batches + 2 leftover -- exercises multi-packet
               # sequencing and confirms the trailing partial batch is dropped

DST_MAC, SRC_MAC = 0xAABBCCDDEEFF, 0x001122334455
SRC_IP, DST_IP = 0xC0A80002, 0xC0A80001
SRC_PORT, DST_PORT = 50000, 50000


def gen_gbe_pipeline_vectors():
    os.makedirs(VEC_DIR, exist_ok=True)
    coeffs = quantize_coeffs(design_compensation_fir())
    pdm_bits, n_samples = gen_multitone_pdm(n_ch=N_CH, n_frames=N_FRAMES)

    with open(os.path.join(VEC_DIR, "pipeline_pdm_bits.mem"), "w") as f:
        for ch_bits in pdm_bits:
            for b in ch_bits:
                f.write(f"{int(b)}\n")

    # NOT is_r_channel (cluster/golden's built-in option, or extra_reg_shift)
    # -- both model a same-edge relationship between L/R capture and
    # cic_decimator's read that assumes a specific testbench-driving style
    # neither matches here. tb_gbe_pipeline.v drives pdm_d from registered
    # (`always @(posedge/negedge clk)`) processes -- itself a genuine extra
    # clocked hop relative to whatever driving style is_r_channel's 0-vs-1-
    # sample L/R difference was originally derived against. Both shifts below
    # were verified bit-exact (not assumed) against real RTL register traces
    # -- pdm_line_demux.v's own ch_l/ch_r, cic_decimator.v's data_out, and
    # fir_compensator.v's data_out -- captured on two independent physical
    # lines, 1000 samples each, zero mismatches once the boundary case below
    # was accounted for.
    def shifted_channel_pipeline(bits1ch, coeffs_int, is_r, stages=5, r=64):
        b = [int(x) for x in bits1ch]
        if is_r:
            # R (odd) channels: NOT a clean uniform 2-sample shift -- b[0] is
            # captured twice (once at position 1, once feeding the steady-
            # state shift-by-2 relationship from position 2 onward), not
            # once. A real, one-time artifact of this testbench's reset-
            # release mechanics: cur_r_idx stays pinned at 0 for the entire
            # reset period (by design, so whichever edge is first captured
            # after release lands on real sample 0), and the specific
            # negedge/posedge phase at the moment reset releases means
            # r_bit(0) ends up driven -- and captured -- once more than a
            # plain "prepend 2 zeros" model would predict. Found by
            # comparing a real ch_r trace against this exact prediction and
            # seeing every position match except index 1; not something a
            # uniform-shift model can express, so it's spelled out directly
            # rather than folded into a single shift-amount parameter.
            bits = [0, b[0]] + b[:len(b) - 2]
        else:
            # L (even) channels: a clean, uniform 1-sample shift throughout,
            # including the reset boundary -- no duplication needed.
            bits = [0] + b[:len(b) - 1]
        cic_out, width = cic_bitexact(bits, stages, r)
        shift = width - FIR_DATA_WIDTH
        mask = (1 << FIR_DATA_WIDTH) - 1
        fir_in = [(v >> shift) & mask for v in cic_out]
        return fir_bitexact(fir_in, coeffs_int)

    per_channel_out = [shifted_channel_pipeline(b, coeffs, is_r=(c % 2 == 1))
                        for c, b in enumerate(pdm_bits)]

    hex_digits = (FIR_DATA_WIDTH + 3) // 4
    with open(os.path.join(VEC_DIR, "pipeline_expected_channels.mem"), "w") as f:
        for frame_idx in range(N_FRAMES):
            for c in range(N_CH):
                f.write(f"{per_channel_out[c][frame_idx]:0{hex_digits}x}\n")

    # frame-major (per gbe_packetizer_golden.gen_packets()'s expected shape)
    per_channel_frames = [
        [per_channel_out[c][frame_idx] for c in range(N_CH)]
        for frame_idx in range(N_FRAMES)
    ]
    packets = gen_packets(per_channel_frames, DST_MAC, SRC_MAC, SRC_IP, DST_IP,
                           SRC_PORT, DST_PORT)

    with open(os.path.join(VEC_DIR, "pipeline_expected_packets.mem"), "w") as f:
        for pkt in packets:
            for byte in pkt:
                f.write(f"{byte:02x}\n")

    print(f"gbe_pipeline: {N_CH} channels x {n_samples} PDM bits -> {N_FRAMES} frames "
          f"-> {len(packets)} expected packets "
          f"-> pipeline_pdm_bits.mem / pipeline_expected_channels.mem / "
          f"pipeline_expected_packets.mem")


UNIT_N_FRAMES = 22  # 4 full 5-frame batches + 2 leftover


def gen_gbe_packetizer_vectors():
    """Stimulus + expected packets for tb_gbe_packetizer.v -- a standalone
    unit test of gbe_packetizer.v alone (synthetic fir_valid_in/fir_data_in
    stimulus, no real CIC/FIR pipeline), matching this project's convention
    of every RTL module getting its own isolated testbench in addition to
    the end-to-end one (tb_gbe_pipeline.v above). Deterministic, distinct
    per-(frame,channel) values -- not audio-like, no golden CIC/FIR model
    needed here, just gbe_packetizer_golden.py's own header/framing logic.
    """
    os.makedirs(VEC_DIR, exist_ok=True)
    mask = (1 << 24) - 1
    per_channel_frames = [
        [((f * 97 + c * 13 + 5) * 2654435761) & mask for c in range(N_CH)]
        for f in range(UNIT_N_FRAMES)
    ]

    with open(os.path.join(VEC_DIR, "gbe_packetizer_stimulus.mem"), "w") as f:
        for frame in per_channel_frames:
            for v in frame:
                f.write(f"{v:06x}\n")

    packets = gen_packets(per_channel_frames, DST_MAC, SRC_MAC, SRC_IP, DST_IP,
                           SRC_PORT, DST_PORT)
    with open(os.path.join(VEC_DIR, "gbe_packetizer_expected.mem"), "w") as f:
        for pkt in packets:
            for byte in pkt:
                f.write(f"{byte:02x}\n")

    print(f"gbe_packetizer: {UNIT_N_FRAMES} synthetic frames -> {len(packets)} expected packets "
          f"-> gbe_packetizer_stimulus.mem / gbe_packetizer_expected.mem")


if __name__ == "__main__":
    gen_gbe_pipeline_vectors()
    gen_gbe_packetizer_vectors()
