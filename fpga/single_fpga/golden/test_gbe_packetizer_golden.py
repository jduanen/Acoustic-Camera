import random

from gbe_packetizer_golden import (
    ETH_HDR_BYTES,
    FIR_WIDTH,
    FRAMES_PER_PKT,
    N_CH,
    TOTAL_PKT_BYTES,
    build_fixed_header,
    gen_packets,
    ip_checksum,
)

DST_MAC, SRC_MAC = 0xAABBCCDDEEFF, 0x001122334455
SRC_IP, DST_IP = 0xC0A80002, 0xC0A80001
SRC_PORT, DST_PORT = 50000, 50000


def _rand_frame(rng):
    return [rng.randint(0, (1 << FIR_WIDTH) - 1) for _ in range(N_CH)]


def test_ip_checksum_known_value():
    # Cross-checked independently by hand against the packet dumped from a
    # real xsim run of gbe_packetizer.v with these same parameters (see
    # session notes) -- both landed on 0xb3d1.
    words = [0x4500, 0x05C8, 0x0000, 0x4000, 0x4011, 0x0000, 0xC0A8, 0x0002, 0xC0A8, 0x0001]
    assert ip_checksum(words) == 0xB3D1


def test_fixed_header_length_and_stability():
    hdr = build_fixed_header(DST_MAC, SRC_MAC, SRC_IP, DST_IP, SRC_PORT, DST_PORT)
    assert len(hdr) == ETH_HDR_BYTES
    hdr2 = build_fixed_header(DST_MAC, SRC_MAC, SRC_IP, DST_IP, SRC_PORT, DST_PORT)
    assert hdr == hdr2  # header is a pure function of the fixed parameters


def test_packet_length():
    rng = random.Random(1)
    frames = [_rand_frame(rng) for _ in range(FRAMES_PER_PKT)]
    pkts = gen_packets([frames[0]] * FRAMES_PER_PKT, DST_MAC, SRC_MAC, SRC_IP, DST_IP,
                        SRC_PORT, DST_PORT)
    assert len(pkts) == 1
    assert len(pkts[0]) == TOTAL_PKT_BYTES


def test_partial_trailing_batch_dropped():
    rng = random.Random(2)
    # 2 full batches + 3 leftover frames -- the leftover must not appear
    frames = [_rand_frame(rng) for _ in range(2 * FRAMES_PER_PKT + 3)]
    pkts = gen_packets(frames, DST_MAC, SRC_MAC, SRC_IP, DST_IP, SRC_PORT, DST_PORT)
    assert len(pkts) == 2


def test_seq_num_and_timestamp_increment():
    rng = random.Random(3)
    frames = [_rand_frame(rng) for _ in range(3 * FRAMES_PER_PKT)]
    pkts = gen_packets(frames, DST_MAC, SRC_MAC, SRC_IP, DST_IP, SRC_PORT, DST_PORT)
    for b, pkt in enumerate(pkts):
        seq_num = int.from_bytes(pkt[ETH_HDR_BYTES:ETH_HDR_BYTES + 4], "big")
        timestamp = int.from_bytes(pkt[ETH_HDR_BYTES + 4:ETH_HDR_BYTES + 12], "big")
        assert seq_num == b
        assert timestamp == b * FRAMES_PER_PKT


def test_payload_channel_order_and_byte_order():
    # only channel 5 of frame 0 non-zero -- must land at payload bytes 15-17
    # (12 header bytes + 5*3), MSB-first.
    frame = [0] * N_CH
    frame[5] = 0xABCDEF
    frames = [frame] + [[0] * N_CH for _ in range(FRAMES_PER_PKT - 1)]
    pkts = gen_packets(frames, DST_MAC, SRC_MAC, SRC_IP, DST_IP, SRC_PORT, DST_PORT)
    payload = pkts[0][ETH_HDR_BYTES:]
    assert payload[12 + 15:12 + 18] == bytes([0xAB, 0xCD, 0xEF])
    # everything else in frame 0's payload region is zero
    assert payload[12:12 + 15] == bytes(15)
    assert payload[12 + 18:12 + 288] == bytes(288 - 18)
