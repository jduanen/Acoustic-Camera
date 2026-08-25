import random

from usb_framer_golden import DATA_WIDTH, HEADER_LEN, N_CH, PAYLOAD_LEN, RECORD_LEN, SYNC0, SYNC1, pack_record


def _rand_channels(rng):
    return [rng.randint(0, (1 << DATA_WIDTH) - 1) for _ in range(N_CH)]


def test_record_length():
    rng = random.Random(1)
    record = pack_record(0, 0, _rand_channels(rng))
    assert len(record) == RECORD_LEN == HEADER_LEN + PAYLOAD_LEN


def test_sync_bytes_fixed():
    rng = random.Random(2)
    record = pack_record(1, 5, _rand_channels(rng))
    assert record[0] == SYNC0
    assert record[1] == SYNC1


def test_spoke_id_and_seq_num_bytes():
    rng = random.Random(3)
    channels = _rand_channels(rng)
    for spoke_id in range(4):
        for seq_num in (0, 1, 255):
            record = pack_record(spoke_id, seq_num, channels)
            assert record[2] == spoke_id
            assert record[3] == seq_num


def test_channel_order_is_ascending():
    # only channel 5 non-zero -- its 3 bytes must land at payload offset 15,16,17
    # (header 4 bytes + 5*3), and nowhere else
    channels = [0] * N_CH
    channels[5] = 0xABCDEF
    record = pack_record(0, 0, channels)
    payload = record[HEADER_LEN:]
    for c in range(N_CH):
        chunk = payload[3 * c:3 * c + 3]
        if c == 5:
            assert chunk == bytes([0xAB, 0xCD, 0xEF])
        else:
            assert chunk == bytes([0, 0, 0])


def test_channel_bytes_are_msb_first():
    channels = [0] * N_CH
    channels[0] = 0x010203
    record = pack_record(0, 0, channels)
    assert record[HEADER_LEN:HEADER_LEN + 3] == bytes([0x01, 0x02, 0x03])


def test_extremes():
    zeros = [0] * N_CH
    ones = [(1 << DATA_WIDTH) - 1] * N_CH
    ramp = list(range(N_CH))
    for channels in (zeros, ones, ramp):
        record = pack_record(3, 42, channels)
        assert len(record) == RECORD_LEN
