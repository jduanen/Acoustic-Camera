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
"""
import argparse
import time

from pyftdi.ftdi import Ftdi

SYNC = bytes([0xA5, 0x5A])
RECORD_LEN = 76  # 2 sync + 1 spoke_id + 1 seq_num + 24 ch * 3 bytes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ftdi://ftdi:232h/1", help="pyftdi device URL")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    ftdi = Ftdi()
    ftdi.open_from_url(args.url)
    ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)

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
    finally:
        ftdi.close()

    elapsed = time.time() - t0
    print(f"\n{elapsed:.1f}s: {total_bytes} bytes ({total_bytes * 8 / elapsed / 1e6:.2f} Mbps), "
          f"{resyncs} resyncs")
    for sid, n in sorted(counts.items()):
        print(f"  spoke {sid}: {n} records ({n / elapsed:.1f}/s -- expect ~48000/s while streaming)")


if __name__ == "__main__":
    main()
