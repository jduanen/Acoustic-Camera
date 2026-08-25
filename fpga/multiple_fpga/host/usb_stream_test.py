#!/usr/bin/env python3
"""
Breadboard bring-up test: reads the hub FPGA's USB stream from the UM232H
(FT232H, synchronous 245 FIFO mode) and decodes it per fpga/USB_FRAMING.md,
reporting sync-lock/drop/throughput stats -- not a production Pi 5 reader,
just a diagnostic for confirming the USB link works before committing to PCB
fab.

Setup (Ubuntu):
    pip install pyftdi
    # the kernel's ftdi_sio driver auto-attaches to the FT232H (creating
    # /dev/ttyUSB0) and blocks pyftdi's libusb access -- detach it once per
    # plug-in:
    sudo rmmod ftdi_sio
    # optional, avoids needing sudo to open the device:
    #   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="0666", GROUP="plugdev"' \
    #     | sudo tee /etc/udev/rules.d/99-ftdi.rules
    #   sudo udevadm control --reload-rules && sudo udevadm trigger

Usage:
    python3 usb_stream_test.py [--url ftdi://ftdi:232h/1] [--seconds 5]
    # also dump one or more channels to 24-bit mono WAVs, one file per channel
    # named <wav-out base>_chN<ext> (spoke auto-picked as whichever spoke_id
    # shows up first, unless --wav-spoke is given):
    python3 usb_stream_test.py --seconds 5 --wav-out capture.wav --wav-channels 0,3,5
"""
import argparse
import os
import time
import wave

from pyftdi.ftdi import Ftdi

SYNC = bytes([0xA5, 0x5A])
RECORD_LEN = 76  # 2 sync + 1 spoke_id + 1 seq_num + 24 ch * 3 bytes
NUM_CHANNELS = 24
SAMPLE_RATE = 48000


def parse_channels(spec):
    channels = sorted({int(c) for c in spec.split(",")})
    for ch in channels:
        if not 0 <= ch < NUM_CHANNELS:
            raise argparse.ArgumentTypeError(f"channel {ch} out of range 0-{NUM_CHANNELS - 1}")
    return channels


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ftdi://ftdi:232h/1", help="pyftdi device URL")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--wav-out", help="base path for WAV output, e.g. capture.wav "
                                       "-> capture_ch0.wav")
    ap.add_argument("--wav-channels", type=parse_channels, default=[0],
                     help="comma-separated channel indices (0-23) to dump, default: 0")
    ap.add_argument("--wav-spoke", type=int, default=None,
                     help="which spoke_id to dump (default: whichever appears first)")
    args = ap.parse_args()

    ftdi = Ftdi()
    ftdi.open_from_url(args.url)
    ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)

    wavs = {}  # channel -> wave.Wave_write
    if args.wav_out:
        base, ext = os.path.splitext(args.wav_out)
        for ch in args.wav_channels:
            w = wave.open(f"{base}_ch{ch}{ext}", "wb")
            w.setnchannels(1)
            w.setsampwidth(3)  # 24-bit
            w.setframerate(SAMPLE_RATE)
            wavs[ch] = w

    buf = bytearray()
    last_seq = {}   # spoke_id -> last seq_num seen
    counts = {}     # spoke_id -> record count
    resyncs = 0
    total_bytes = 0
    t0 = time.time()
    t_end = t0 + args.seconds

    try:
        while time.time() < t_end:
            chunk = ftdi.read_data(65536)
            if chunk:
                total_bytes += len(chunk)
                buf.extend(chunk)

            while True:
                if len(buf) < RECORD_LEN:
                    break
                if buf[0:2] != SYNC:
                    idx = buf.find(SYNC, 1)
                    if idx == -1:
                        del buf[:len(buf) - 1]  # keep last byte, may be a partial marker
                        break
                    del buf[:idx]
                    resyncs += 1
                    continue

                rec = bytes(buf[:RECORD_LEN])
                del buf[:RECORD_LEN]

                spoke_id = rec[2]
                seq_num = rec[3]
                counts[spoke_id] = counts.get(spoke_id, 0) + 1
                if spoke_id in last_seq:
                    expected = (last_seq[spoke_id] + 1) & 0xFF
                    if seq_num != expected:
                        gap = (seq_num - expected) & 0xFF
                        print(f"DROP: spoke {spoke_id} seq {last_seq[spoke_id]} -> {seq_num} "
                              f"({gap} missed)")
                last_seq[spoke_id] = seq_num

                if wavs:
                    if args.wav_spoke is None:
                        args.wav_spoke = spoke_id  # lock onto the first spoke seen
                    if spoke_id == args.wav_spoke:
                        for ch, w in wavs.items():
                            # channel c = bytes 4+3c .. 4+3c+2, MSB-first two's
                            # complement on the wire (USB_FRAMING.md); WAV
                            # wants little-endian.
                            off = 4 + 3 * ch
                            sample = int.from_bytes(rec[off:off + 3], byteorder="big", signed=True)
                            w.writeframes(sample.to_bytes(3, byteorder="little", signed=True))
    finally:
        ftdi.close()
        for w in wavs.values():
            w.close()

    elapsed = time.time() - t0
    print(f"\n{elapsed:.1f}s: {total_bytes} bytes ({total_bytes * 8 / elapsed / 1e6:.2f} Mbps), "
          f"{resyncs} resyncs")
    for sid, n in sorted(counts.items()):
        print(f"  spoke {sid}: {n} records ({n / elapsed:.1f}/s -- expect ~48000/s while streaming)")


if __name__ == "__main__":
    main()
