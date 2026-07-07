# Running `acoustic_camera_p3.py` on a Raspberry Pi 5

Guide for running the existing Phase 3 live script (`src/acoustic_camera_p3.py`, 16-ch
UMA-16 v2 → 2D beamform → webcam overlay) directly on a Raspberry Pi 5, today, with the
mic array hardware that already exists. This is distinct from the Phase 4 plan
([PHASE4.md](./PHASE4.md)), which uses a Pi 5 as the compute host for a *future* 96-ch
FPGA-fed array. The steps below get the current 16-ch pipeline running standalone in
the meantime, and most of the setup (OS packages, PortAudio, OpenBLAS) carries over
directly to that later phase.

Assumes Raspberry Pi OS, 64-bit, Pi 5 (4 or 8 GB). Written against Debian Bookworm and
confirmed hands-on on Trixie as well. Package names, PortAudio, and the CSI-camera/
`picamera2` situation are the same on both. One real difference found on Trixie: OpenBLAS
is **not** installed by default (see §2). Do that check/install regardless of which OS
version you're on.

**Desktop vs. Lite**: §5 (Display) assumes the Desktop image, which auto-starts a GUI
session (X11 or Wayland). On **Raspberry Pi OS Lite**, there is no display server
running at all -- see the Lite-specific fix in §5.

## 1. System Packages

```bash
sudo apt update
sudo apt install -y libportaudio2 portaudio19-dev python3-opencv v4l-utils
```

- `libportaudio2` / `portaudio19-dev`: `sounddevice` is a thin wrapper over PortAudio;
  without the shared lib it fails to import.
- `python3-opencv`: Raspberry Pi OS's apt build of OpenCV includes GTK GUI support, so
  `cv2.imshow` works out of the box. **Prefer this over `pip install opencv-python`**:
  the PyPI wheel is frequently built headless on ARM and silently breaks `cv2.imshow`
  with a "not implemented" backend error at runtime, not at import time.
- `v4l-utils`: gives you `v4l2-ctl --list-devices` to confirm a USB webcam enumerates
  before wiring it into the script.

## 2. Python Environment

Reuse the project's existing `requirements.txt`. No Pi-specific fork needed, but skip
the packages the live script doesn't import (`acoular`, `pyroomacoustics`, `acoupipe`
are Phase 1 simulation-only):

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install numpy scipy sounddevice
```

`--system-site-packages` matters here: it lets the venv see the apt-installed
`cv2` (with GUI support) instead of pulling in a headless wheel from PyPI.

**Confirmed on Raspberry Pi OS Trixie: OpenBLAS is not installed by default.** A stock
Trixie image only has `libblas3`/`liblapack3` (Netlib reference implementations; no
NEON vectorization, single-threaded). `numpy.show_config()` on an unmodified Trixie
install shows generic `name: blas` / `name: lapack` with `openblas configuration:
unknown`, which is the tell that it's riding on the reference implementation rather
than OpenBLAS. This makes MVDR/MUSIC (`scipy.linalg.inv`, `numpy.linalg.eigh`) far
slower than the estimates in §6, so install it explicitly:

```bash
sudo apt install libopenblas0-pthread
```

Debian's `update-alternatives` gives the OpenBLAS build a higher priority (100) than
the reference implementation (10), so this switches automatically. Confirm this with:

```bash
update-alternatives --display libblas.so.3-aarch64-linux-gnu
update-alternatives --display liblapack.so.3-aarch64-linux-gnu
```

Both should report "link currently points to" a path containing `openblas-pthread`.
If not, force it with `sudo update-alternatives --config libblas.so.3-aarch64-linux-gnu`
(and the `liblapack` equivalent) and pick the OpenBLAS entry.

Then confirm from Python:

```bash
python3 -c "import numpy; numpy.show_config()"
```

Note that even with OpenBLAS correctly selected via `update-alternatives`,
`numpy.show_config()` may still print the generic `blas`/`lapack` name with
`openblas configuration: unknown` rather than an explicit `openblas` entry. NumPy's
build-time pkgconfig detection doesn't see through Debian's alternatives indirection.
The `update-alternatives --display` output above is the reliable way to confirm which
implementation is actually linked, not `show_config()`'s `name` field.

This was not something the Bookworm-era assumption in this guide accounted for
(Bookworm's `python3-numpy` was believed to pull in OpenBLAS directly). Treat "is
OpenBLAS actually selected" as a required check on any fresh Pi OS install, not a
formality.

## 3. UMA-16 v2 Hookup

This is dentical to the desktop case -- the UMA-16 is a driverless UAC2 USB device, no Pi-specific
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

The script uses `cv2.VideoCapture(args.video)` (i.e. a V4L2 device index), this only
works with a **USB webcam**, not the Pi Camera Module (CSI). Bookworm removed the
legacy camera stack; the CSI camera is exposed through `libcamera`/`picamera2`, which
`cv2.VideoCapture` cannot open directly. If you only have a Camera Module on hand:

- Easiest: plug in a USB webcam instead; zero code changes.
- Camera Module: would require switching the capture path to `picamera2` (this is
  exactly what [PHASE4.md](./PHASE4.md#why-pi-5-not-pi-4-or-cm4) already plans for
  Phase 4, so if you want this now it's worth doing once rather than twice).

Use `v4l2-ctl --list-devices` to find the right `--video` index. It will very likely
**not** be 0 on a Pi with both a USB webcam and other V4L2-capable hardware present.

## 5. Display

`cv2.imshow` needs a real GUI session. The options below assume the **Desktop** image,
which auto-starts a local GUI session (X11, confirmed by what worked below) on the
HDMI-attached monitor at boot.

**Important distinction confirmed on hardware**: "a desktop session is running on the
Pi" is not the same as "your shell can draw to it." A plain SSH login is a separate
session (`XDG_SESSION_TYPE=tty`, no `DISPLAY`/`XAUTHORITY` set) from the auto-started
local desktop session on `seat0`, even though both exist on the same machine at the
same time (check with `loginctl list-sessions`). Three distinct scenarios:

- **Logged in locally** (keyboard/monitor directly on the Pi, or VNC into that same
  session): just works, `DISPLAY` is already set correctly in that session's own
  terminal.
- **SSH with `-X`/`-Y` forwarding**: works, but *not* by drawing on the Pi's monitor:
  it creates a **new**, separate X11 display on the Pi that tunnels every frame back
  over the network to your own machine's X server. The window appears on your laptop,
  not the Pi's HDMI output. Also far too slow for live 15-20 fps video, it is useful only
  for a one-off sanity check.
- **SSH, no `-Y`, attaching to the Pi's own already-running local desktop session**:
  this is what actually renders on the Pi's physical HDMI monitor while running the
  command remotely. **Confirmed working recipe:**

  ```bash
  export DISPLAY=:0
  export XAUTHORITY=/home/pi/.Xauthority   # your actual home dir
  python acoustic_camera_p3.py --video 0 --grid_deg 1.0
  ```

  Both variables are required. `DISPLAY=:0` alone fails with "could not connect to
  display" (Qt still needs the auth cookie to attach to someone else's X session).
  This is a plain X11 session using the standard per-user `~/.Xauthority`, which is simpler
  than initially expected; no Xwayland-specific rootless auth file hunting needed.

Add `--fullscreen` to fill the whole monitor (borderless window) -- see §6.

### Raspberry Pi OS Lite (partial, not fully validated)

Lite has no display server running at all -- no X11, no Wayland compositor. `cv2.imshow`
fails with:

```
qt.qpa.xcb: could not connect to display
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" ...
```

This looks like a GTK-vs-Qt OpenCV backend problem (the plugin list even offers
`wayland`), but it isn't; there is simply no compositor for any backend to attach to.
The lightest fix, in principle, is a bare X server with no window manager, using
`xinit` to launch the script as the sole X client:

```bash
sudo apt install --no-install-recommends xserver-xorg xinit x11-xserver-utils
xinit /path/to/venv/bin/python /path/to/acoustic_camera_p3.py --video 0 --grid_deg 1.0 -- :0
```

**This was not fully validated**: on real hardware this problem occurred `Fatal server error:
Cannot run in framebuffer mode. Please specify busIDs for all framebuffer devices`.
Xorg is falling back to the `fbdev` driver instead of `modesetting`/KMS (the Pi 5 has no
non-KMS mode). The likely fix is forcing the driver explicitly:

```bash
sudo mkdir -p /etc/X11/xorg.conf.d
sudo tee /etc/X11/xorg.conf.d/10-modesetting.conf <<'EOF'
Section "Device"
    Identifier "Pi5"
    Driver "modesetting"
    Option "kmsdev" "/dev/dri/card0"
EndSection
EOF
```

(`modesetting` is built into `xserver-xorg-core`, not a separate package; no install
needed for the driver itself.) This was not re-tested. The Pi was reinstalled with
the Desktop image instead, which avoids the whole class of problem. If you need a
Lite-based headless deployment later (e.g. a lighter Phase 4 field unit), start here
but expect to need this driver fix, and confirm `/dev/dri/card0` is actually the right
device (`ls /dev/dri/`) before assuming it.

## 6. Performance Expectations

The Pi 5's Cortex-A76 @ 2.4 GHz with OpenBLAS runs dense BLAS roughly 3-4× slower than a
modern desktop CPU (measured basis for this estimate: [PHASE4.md](./PHASE4.md) scaling
analysis for the 96-ch Phase 4 host). Scaling the existing 16-ch desktop benchmarks from
[PHASE3.md](./PHASE3.md)'s `benchmark_algos.py` results:

| Grid | D&S (desktop measured) | D&S (Pi 5, est.) | MVDR/MUSIC (Pi 5, est.) | CLEAN-SC 20 iter (Pi 5, est.) |
|---|---|---|---|---|
| 0.5°/pt (181×121 = 21,901 pts) | ~21 ms | ~70-85 ms | ~75-90 ms | ~300-350 ms |
| 1.0°/pt (91×61 = 5,551 pts)   | ~7 ms  | ~20-25 ms | ~20-25 ms | ~55-65 ms |

At 1.0°/pt, D&S/MVDR/MUSIC should sustain roughly 15-20 fps on the Pi 5, which is comfortably
inside the script's own frame pacing (`time.sleep(0.05)` caps display at 20 fps
regardless). At the default 0.5°/pt, D&S is likely still fine but MVDR/MUSIC and
especially CLEAN-SC will visibly lag.

**Recommended starting command**: coarser grid, cheapest algorithm, confirm it runs
before turning up resolution or trying MVDR/MUSIC/CLEAN-SC:

```bash
python src/acoustic_camera_p3.py --algo ds --grid_deg 1.0
```

For a fixed HDMI-monitor "camera appliance" deployment, add `--fullscreen` to fill the
whole screen with a borderless window (see §5 for how to get the display attached in
the first place):

```bash
python src/acoustic_camera_p3.py --algo ds --grid_deg 1.0 --fullscreen
```

If you want real numbers instead of estimates before wiring up the camera/display,
run the existing benchmark script directly on the Pi first. It needs no camera, no
display, and no live mic input (it loads `test/UMA16/capture_nb16.wav`):

```bash
python src/benchmark_algos.py --grid_deg 1.0
```

That gives per-algorithm mean time and RT% on the actual Pi 5 hardware, which is more
trustworthy than the estimates in the table above.

## 7. Performance Measurements

## 8. Known Issues Summary

| Symptom | Cause | Fix |
|---|---|---|
| `sounddevice` import error | missing `libportaudio2` | `apt install libportaudio2` |
| `cv2.imshow` errors "not implemented" | headless OpenCV wheel from `pip` | use apt `python3-opencv`, venv with `--system-site-packages` |
| Camera won't open / wrong device | Pi Camera Module (CSI), not a USB webcam | use a USB webcam, or add `picamera2` support (Phase 4 work) |
| `cv2.imshow` hangs / X errors over SSH | no display session, or SSH session not attached to the Pi's local desktop session | log in locally/VNC, or export `DISPLAY=:0` + `XAUTHORITY=~/.Xauthority` (see §5) |
| `ssh -Y` window appears on your own machine, not the Pi's monitor | `-Y` tunnels a **new** X display back to your client; it never touches the Pi's HDMI output | use the `DISPLAY`/`XAUTHORITY` attach recipe in §5 instead of `-Y` |
| Low fps at default settings | 0.5°/pt grid + MVDR/MUSIC/CLEAN-SC too heavy for Pi 5 | `--grid_deg 1.0`, start with `--algo ds` |
| MVDR/MUSIC far slower than §6 estimates | OpenBLAS not installed (confirmed default on Trixie); NumPy silently falls back to reference BLAS/LAPACK | `apt install libopenblas0-pthread`, verify with `update-alternatives --display libblas.so.3-aarch64-linux-gnu` |
| Xorg: `Cannot run in framebuffer mode...` (Lite only) | falling back to `fbdev` instead of `modesetting`/KMS | force `modesetting` via `/etc/X11/xorg.conf.d` (see §5, not fully validated) |
