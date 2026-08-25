#!/usr/bin/env python3
"""gbe_packetizer_golden.py -- bit-exact reference for gbe_packetizer.v's v1
UDP framing protocol. See fpga/single_fpga/GBE_FRAMING.md for the protocol
spec this implements; that doc and fpga/single_fpga/rtl/gbe_packetizer.v must
stay in sync with this file (source of truth for testbench comparison)."""

N_CH = 96
FIR_WIDTH = 24
FRAMES_PER_PKT = 5
PAYLOAD_BYTES = N_CH * 3                                  # 288
HDR_FIELD_BYTES = 4 + 8                                    # seq_num + timestamp = 12
PKT_PAYLOAD_BYTES = HDR_FIELD_BYTES + FRAMES_PER_PKT * PAYLOAD_BYTES  # 1452
ETH_HDR_BYTES = 42                                          # Eth(14) + IP(20) + UDP(8)
TOTAL_PKT_BYTES = ETH_HDR_BYTES + PKT_PAYLOAD_BYTES         # 1494

IP_VER_IHL = 0x45
IP_DSCP_ECN = 0x00
IP_TOTAL_LEN = 20 + 8 + PKT_PAYLOAD_BYTES                  # 1480, constant every packet
IP_ID = 0x0000                                              # unused: DF=1, never fragmented
IP_FLAGS_FRAG = 0x4000                                      # DF=1, offset 0
IP_TTL = 64
IP_PROTO = 17                                                # UDP
UDP_LEN = 8 + PKT_PAYLOAD_BYTES                              # 1460, constant every packet


def ip_checksum(words):
    """Standard IPv4 header checksum: ones'-complement sum of 16-bit words
    (checksum field = 0 while summing), carries folded back in, then
    complemented. Mirrors gbe_packetizer.v's ip_checksum() function exactly."""
    s = sum(words)
    s = (s & 0xFFFF) + (s >> 16)
    s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def build_fixed_header(dst_mac, src_mac, src_ip, dst_ip, src_port, dst_port):
    """The 42-byte Ethernet+IP+UDP header, byte-identical on every packet --
    see GBE_FRAMING.md's "IP header checksum is a build-time constant"
    section for why. Returns a 42-byte `bytes` object."""
    checksum = ip_checksum([
        (IP_VER_IHL << 8) | IP_DSCP_ECN, IP_TOTAL_LEN, IP_ID, IP_FLAGS_FRAG,
        (IP_TTL << 8) | IP_PROTO, 0x0000,
        (src_ip >> 16) & 0xFFFF, src_ip & 0xFFFF,
        (dst_ip >> 16) & 0xFFFF, dst_ip & 0xFFFF,
    ])
    hdr = bytearray()
    hdr += dst_mac.to_bytes(6, "big")
    hdr += src_mac.to_bytes(6, "big")
    hdr += (0x0800).to_bytes(2, "big")
    hdr += bytes([IP_VER_IHL, IP_DSCP_ECN])
    hdr += IP_TOTAL_LEN.to_bytes(2, "big")
    hdr += IP_ID.to_bytes(2, "big")
    hdr += IP_FLAGS_FRAG.to_bytes(2, "big")
    hdr += bytes([IP_TTL, IP_PROTO])
    hdr += checksum.to_bytes(2, "big")
    hdr += src_ip.to_bytes(4, "big")
    hdr += dst_ip.to_bytes(4, "big")
    hdr += src_port.to_bytes(2, "big")
    hdr += dst_port.to_bytes(2, "big")
    hdr += UDP_LEN.to_bytes(2, "big")
    hdr += (0x0000).to_bytes(2, "big")
    assert len(hdr) == ETH_HDR_BYTES
    return bytes(hdr)


def pack_packet(fixed_hdr, seq_num, timestamp, frames):
    """frames: list of exactly FRAMES_PER_PKT frames, each a list of N_CH
    FIR_WIDTH-bit unsigned samples (channel-ascending). Returns the full
    TOTAL_PKT_BYTES-byte packet (header + seq_num + timestamp + payload),
    matching gbe_packetizer.v's byte layout exactly (channel 0 first per
    frame, each sample MSB-first)."""
    assert len(frames) == FRAMES_PER_PKT
    mask = (1 << FIR_WIDTH) - 1
    pkt = bytearray(fixed_hdr)
    pkt += (seq_num & 0xFFFFFFFF).to_bytes(4, "big")
    pkt += (timestamp & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "big")
    for frame in frames:
        assert len(frame) == N_CH
        for v in frame:
            pkt += (v & mask).to_bytes(3, "big")
    assert len(pkt) == TOTAL_PKT_BYTES
    return bytes(pkt)


def gen_packets(per_channel_frames, dst_mac, src_mac, src_ip, dst_ip, src_port, dst_port):
    """per_channel_frames: list of frames (frame-major), each a list of N_CH
    samples -- same shape as fpga/multi_fpga/cluster/golden's per-channel pipeline
    output, just frame-major instead of channel-major (see
    gen_vectors.py's gen_gbe_vectors() for the transpose). Mirrors
    gbe_packetizer.v's frame collector exactly: sample_counter increments
    once per input frame, a batch's timestamp is its first frame's
    (pre-increment) sample_counter value, seq_num increments once per
    completed FRAMES_PER_PKT-frame batch. Only full batches are emitted --
    any trailing partial batch (len(per_channel_frames) not a multiple of
    FRAMES_PER_PKT) is silently not sent, matching the RTL.

    Returns a list of TOTAL_PKT_BYTES-byte packets, one per completed batch.
    """
    fixed_hdr = build_fixed_header(dst_mac, src_mac, src_ip, dst_ip, src_port, dst_port)
    n_batches = len(per_channel_frames) // FRAMES_PER_PKT
    packets = []
    for b in range(n_batches):
        batch_frames = per_channel_frames[b * FRAMES_PER_PKT:(b + 1) * FRAMES_PER_PKT]
        timestamp = b * FRAMES_PER_PKT  # sample_counter's value at this batch's frame 0
        packets.append(pack_packet(fixed_hdr, b, timestamp, batch_frames))
    return packets
