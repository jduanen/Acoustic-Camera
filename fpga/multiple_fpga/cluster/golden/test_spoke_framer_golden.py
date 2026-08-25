import random

from spoke_framer_golden import BUSY_CYCLES, DATA_WIDTH, FRAME_CYCLES, N_CH, deframe, spoke_frame


def _rand_channels(rng):
    return [rng.randint(0, (1 << DATA_WIDTH) - 1) for _ in range(N_CH)]


def test_frame_length():
    rng = random.Random(1)
    cycles = spoke_frame(_rand_channels(rng))
    assert len(cycles) == FRAME_CYCLES


def test_strobe_exactly_one_cycle():
    rng = random.Random(2)
    cycles = spoke_frame(_rand_channels(rng))
    strobes = [c[2] for c in cycles]
    assert strobes[0] == 1
    assert sum(strobes) == 1


def test_idle_cycles_are_zero():
    rng = random.Random(3)
    cycles = spoke_frame(_rand_channels(rng))
    for rise, fall, _ in cycles[BUSY_CYCLES:]:
        assert rise == 0 and fall == 0


def test_roundtrip_random():
    rng = random.Random(4)
    for _ in range(20):
        channels = _rand_channels(rng)
        assert deframe(spoke_frame(channels)) == channels


def test_roundtrip_extremes():
    zeros = [0] * N_CH
    ones = [(1 << DATA_WIDTH) - 1] * N_CH
    ramp = list(range(N_CH))
    for channels in (zeros, ones, ramp):
        assert deframe(spoke_frame(channels)) == channels


def test_channel_order_is_ascending():
    # only channel 5 non-zero -- its nibbles must land at cycles 10,11 (2*5, 2*5+1)
    channels = [0] * N_CH
    channels[5] = 0xABCDEF & ((1 << DATA_WIDTH) - 1)
    cycles = spoke_frame(channels)
    for cyc in range(BUSY_CYCLES):
        rise, fall, _ = cycles[cyc]
        if cyc in (10, 11):
            assert (rise, fall) != (0, 0)
        else:
            assert (rise, fall) == (0, 0)
