# Running `acoustic_camera_p3.py` on a Raspberry Pi 5

Guide for running the existing Phase 3 live script (`src/acoustic_camera_p3.py`, 16-ch
UMA-16 v2 → 2D beamform → webcam overlay) directly on a Raspberry Pi 5, today, with the
mic array hardware that already exists. This is distinct from the Phase 4 plan
([PHASE4.md](./PHASE4.md)), which uses a Pi 5 as the compute host for a *future* 96-ch
FPGA-fed array — the steps below get the current 16-ch pipeline running standalone in
the meantime, and most of the setup (OS packages, PortAudio, OpenBLAS) carries over
directly to that later phase.

Assumes Raspberry Pi OS, 64-bit, Pi 5 (4 or 8 GB). Written against Debian Bookworm; nothing
here is Bookworm-specific (package names, PortAudio, and the CSI-camera/`picamera2` situation
are all the same on Trixie), but it hasn't been verified hands-on against a Trixie-based image.
On Trixie, do the `numpy.show_config()` and `cv2.imshow` checks below first since package
versions (NumPy 2.x, newer OpenCV) differ from Bookworm even though nothing structural changes.

## 1. System packages

```bash
sudo apt update
sudo apt install -y libportaudio2 portaudio19-dev python3-opencv v4l-utils
```

- `libportaudio2` / `portaudio19-dev` — `sounddevice` is a thin wrapper over PortAudio;
  without the shared lib it fails to import.
- `python3-opencv` — Raspberry Pi OS's apt build of OpenCV includes GTK GUI support, so
  `cv2.imshow` works out of the box. **Prefer this over `pip install opencv-python`**:
  the PyPI wheel is frequently built headless on ARM and silently breaks `cv2.imshow`
  with a "not implemented" backend error at runtime, not at import time.
- `v4l-utils` — gives you `v4l2-ctl --list-devices` to confirm a USB webcam enumerates
  before wiring it into the script.

## 2. Python environment

Reuse the project's existing `requirements.txt` — no Pi-specific fork needed, but skip
the packages the live script doesn't import (`acoular`, `pyroomacoustics`, `acoupipe`
are Phase 1 simulation-only):

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install numpy scipy sounddevice
```

`--system-site-packages` matters here: it lets the venv see the apt-installed
`cv2` (with GUI support) instead of pulling in a headless wheel from PyPI.

Confirm NumPy is linked against OpenBLAS (this is what the beamforming math actually
rides on):

```bash
python3 -c "import numpy; numpy.show_config()"
```

Look for `openblas` in the output. Raspberry Pi OS's apt `python3-numpy` and current
piwheels builds both link OpenBLAS by default, so this should need no extra action —
just worth a one-time check, since a reference-BLAS fallback would make MVDR/MUSIC
(`scipy.linalg.inv`, `numpy.linalg.eigh`) far slower than the estimates below.

## 3. UMA-16 v2 hookup

Identical to desktop — the UMA-16 is a driverless UAC2 USB device, no Pi-specific
driver needed. Verify it enumerates the same way as in
[nb16](./notebooks/16_uma16_capture.ipynb):

```bash
lsusb | grep -i minidsp        # miniDSP UMA16v2
arecord -l                     # card N: UMA16v2, device 0
python3 -c "import sounddevice as sd; print([d for d in sd.query_devices() if 'uma' in d['name'].lower()])"
```

`find_device()` in `acoustic_camera_p3.py` matches on `'uma' in name`, same as the
notebooks, so no `--device` override should be needed. USB bandwidth for 16 channels ×
24-bit × 48 kHz is trivial for any Pi 5 USB port (no hub/bandwidth concerns).

## 4. Camera

The script uses `cv2.VideoCapture(args.video)`, i.e. a V4L2 device index — this only
works with a **USB webcam**, not the Pi Camera Module (CSI). Bookworm removed the
legacy camera stack; the CSI camera is exposed through `libcamera`/`picamera2`, which
`cv2.VideoCapture` cannot open directly. If you only have a Camera Module on hand:

- Easiest: plug in a USB webcam instead — zero code changes.
- Camera Module: would require switching the capture path to `picamera2` (this is
  exactly what [PHASE4.md](./PHASE4.md#why-pi-5-not-pi-4-or-cm4) already plans for
  Phase 4, so if you want this now it's worth doing once rather than twice).

Use `v4l2-ctl --list-devices` to find the right `--video` index — it will very likely
**not** be 0 on a Pi with both a USB webcam and other V4L2-capable hardware present.

## 5. Display

`cv2.imshow` needs a real GUI session:

- HDMI display attached directly to the Pi — works with no extra setup.
- VNC (e.g. RealVNC, built into Raspberry Pi OS) — works.
- Plain SSH with no X forwarding — **will fail** (`cv2.imshow` needs a display; there
  isn't a "just print to terminal" fallback in this script).
- SSH with `-X`/`-Y` forwarding — technically works but X11-forwards every video frame
  over the network; expect it to be far too slow for a live 15-20 fps overlay. Only
  useful for a one-off sanity check, not real use.

## 6. Performance expectations

The Pi 5's Cortex-A76 @ 2.4 GHz with OpenBLAS runs dense BLAS roughly 3-4× slower than a
modern desktop CPU (measured basis for this estimate: [PHASE4.md](./PHASE4.md) scaling
analysis for the 96-ch Phase 4 host). Scaling the existing 16-ch desktop benchmarks from
[PHASE3.md](./PHASE3.md)'s `benchmark_algos.py` results:

| Grid | D&S (desktop measured) | D&S (Pi 5, est.) | MVDR/MUSIC (Pi 5, est.) | CLEAN-SC 20 iter (Pi 5, est.) |
|---|---|---|---|---|
| 0.5°/pt (181×121 = 21,901 pts) | ~21 ms | ~70-85 ms | ~75-90 ms | ~300-350 ms |
| 1.0°/pt (91×61 = 5,551 pts)   | ~7 ms  | ~20-25 ms | ~20-25 ms | ~55-65 ms |

At 1.0°/pt, D&S/MVDR/MUSIC should sustain roughly 15-20 fps on the Pi 5 — comfortably
inside the script's own frame pacing (`time.sleep(0.05)` caps display at 20 fps
regardless). At the default 0.5°/pt, D&S is likely still fine but MVDR/MUSIC and
especially CLEAN-SC will visibly lag.

**Recommended starting command** — coarser grid, cheapest algorithm, confirm it runs
before turning up resolution or trying MVDR/MUSIC/CLEAN-SC:

```bash
python src/acoustic_camera_p3.py --algo ds --grid_deg 1.0
```

If you want real numbers instead of estimates before wiring up the camera/display,
run the existing benchmark script directly on the Pi first — it needs no camera, no
display, and no live mic input (it loads `test/UMA16/capture_nb16.wav`):

```bash
python src/benchmark_algos.py --grid_deg 1.0
```

That gives per-algorithm mean time and RT% on the actual Pi 5 hardware, which is more
trustworthy than the estimates in the table above.

## 7. Known gotchas summary

| Symptom | Cause | Fix |
|---|---|---|
| `sounddevice` import error | missing `libportaudio2` | `apt install libportaudio2` |
| `cv2.imshow` errors "not implemented" | headless OpenCV wheel from `pip` | use apt `python3-opencv`, venv with `--system-site-packages` |
| Camera won't open / wrong device | Pi Camera Module (CSI), not a USB webcam | use a USB webcam, or add `picamera2` support (Phase 4 work) |
| `cv2.imshow` hangs / X errors over SSH | no display session | attach HDMI or use VNC |
| Low fps at default settings | 0.5°/pt grid + MVDR/MUSIC/CLEAN-SC too heavy for Pi 5 | `--grid_deg 1.0`, start with `--algo ds` |
