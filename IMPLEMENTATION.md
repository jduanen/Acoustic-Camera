# Implementation Details

# Outline

### Hardware
#### Microphone Elements
#### FPGAs and FPGA Development Boards
##### Candidate FPGA Devices
##### Candidate FPGA Dev Boards
### Software Stack Candidates
#### Python Packages

---

### Hardware

#### Microphone Elements

See [MICS](./MICS.md) for mic element details.

**Chosen Mic**: Infineon IM69D120V01XTSA1 PDM MEMS
  - Newark: $0.755 (unit 1)

#### FPGAs and FPGA Development Boards

Target use: PDM clock distribution, 96-channel CIC+FIR decimation, synchronous sampling, GbE packetization. See [DESIGN](./DESIGN.md) for details.

##### Candidate FPGA Devices

* Lattice ECP5
  - preferred for open-source toolchain (Yosys / nextpnr / openFPGALoader)
  - ECP5-85F: 84K LUTs, 3.4Mb BRAM, abundant I/O
  - low power; actively supported by open-source community
  - dev boards: OrangeCrab, ULX3S, Versa ECP5

* Xilinx Artix-7
  - preferred for resource headroom and mature ecosystem (Vivado)
  - XC7A100T: 101K LUTs, 4.8Mb BRAM, up to 210 user I/O
  - large library of IP cores (GbE MAC, PCIe, etc.)
  - dev boards: Digilent Arty A7-100T, Nexys Video (chosen for Phase 4 — FMC LPC)

* Intel Cyclone 10 LP
  - alternative; good balance of cost and I/O count
  - dev boards: Intel Cyclone 10 LP Evaluation Kit

##### Candidate FPGA Dev Boards

| Device | LUTs | BRAM | Toolchain | Dev board |
|---|---|---|---|---|
| Lattice ECP5-85F | 84K | 3.4 Mb | Yosys/nextpnr (open-source) | OrangeCrab, ULX3S |
| **Xilinx Artix-7 XC7A200T** | **134K** | **13.1 Mb** | **Vivado; chosen for Phase 4** | **Nexys Video (FMC LPC)** |
| Intel Cyclone 10 LP | -- | -- | Quartus | Cyclone 10 LP Eval Kit |

### Software Stack Candidates

| Package | Role |
|---|---|
| acoular | Beamforming core (D&S, MVDR, CLEAN-SC, array geometry) |
| pyroomacoustics | Room acoustics simulation, synthetic source data |
| acoupipe | ML training dataset generation |
| sounddevice | Real-time USB audio capture (Phases 2/3) |
| opencv-python | Video capture, overlay, display |
| scipy / numpy | Signal processing, array math |
| matplotlib / seaborn | Plotting and visualization |
| h5py | HDF5 I/O (Acoular data format) |
| jupyterlab | Notebooks (Phase 1 deliverable) |

#### Python Packages

* acoular: Beamforming core (D&S, MVDR, CLEAN-SC, array geometry)
* pyroomacoustics: Room acoustics simulation, synthetic source data
* acoupipe: ML training dataset generation (from GitHub) 
* numpy: Array math
* scipy: Signal processing
* matplotlib: Plotting
* h5py: HDF5 I/O (Acoular data format)
* jupyterlab: Notebooks (Phase 1 deliverable) 
* pandas: Results tabulation
* seaborn: Visualization
* tqdm: Progress bars

See also: [FOSS](./FOSS.md) for open-source beamforming software survey

**TODO** put in final stack selection here

---

## Phase 3 Application Development Details

????

### Touch UI: Frequency Sliders & Spectrum Pan

Getting the Fhi/Flo sliders right on a real touchscreen (Raspberry Pi Touch Display 2,
see [RASPBERRY_PI.md](./RASPBERRY_PI.md)) took several rounds — worth documenting since
none of these were visible on desktop/mouse testing, only on hardware:

**Touch targets too close together.** The original design had both sliders stacked in
one 44px panel (22px per row) — too small a target for a finger. Fix: split them onto
opposite sides of the spectrum plot, each with its own full 44px strip.

**Displayed slider position lagged the actual touch position.** `flo`/`fhi` were read
from the shared `_sliders` dict once at the top of the main loop (before the ~15-30 ms
of CSM/beamform work), and that stale snapshot was still what got drawn at the bottom
of the loop — so during an active drag, the on-screen handle visibly lagged the live
value by up to one frame, which looked like the handles crossing. Fixed by re-reading
live values immediately before drawing, separate from the early snapshot used for that
frame's beamforming frequency (which intentionally stays tied to what was actually
computed that frame).

**Race condition on `_sliders`.** It's written by `_on_mouse` (OpenCV's Qt backend can
dispatch input callbacks off the main loop's thread) and read by the main loop, with no
lock — unlike `audio_buf` elsewhere in this file, which already uses `buf_lock` for
exactly this reason. Added `_sliders_lock` around all reads/writes, mirroring that
existing pattern.

**Different max ranges made the two sliders behave inconsistently.** `flo`/`fhi`
originally had different maxima (5000 / 8000 Hz), so the same finger position meant a
different Hz value on each — confusing, and made the ordering clamp feel like it was
failing even when it wasn't. Unified to one shared `_FREQ_MAX = 8000` for both.

**Track geometry didn't line up.** Flo's label is on the left, Fhi's on the right (so
the label side visually reinforces which direction is "higher" on the shared scale) —
but that means their tracks need mirrored margins (`[92, w-8]` vs `[8, w-92]`), which
are the *same length* but not the *same interval*, so identical Hz values landed at
different x-pixels on each slider. Fixed by using the intersection of both margins,
`[92, w-92]`, as the single shared track for both — see `_track_geom()`.

**Spectrum-plot pan.** Dragging horizontally on the spectrum plot itself (not the
slider strips) shifts `flo` and `fhi` together by the same amount, preserving the gap
between them — i.e. it pans the visible range rather than resizing it. Clamped at both
ends so the window snaps to sit exactly at 100 Hz or `_FREQ_MAX` rather than partially
clipping. Uses the same `_track_geom()`-derived scale as the sliders, so pan speed
matches slider-drag speed.

**Nyquist marker color.** The vertical line marking spatial Nyquist (~4083 Hz for the
UMA-16) on the spectrum plot was `(255, 100, 0)` in BGR — a blue, not orange as the name
might suggest — which was hard to distinguish against the green spectrum bars. Changed
to magenta `(255, 0, 255)` (zero green component, maximal contrast against a
green-dominated background).

### Touch UI: Energy Threshold Tab & Auto/Manual Range

A small "E" tab in the top-right corner of the video frame (not a new full-width
strip — the video/strip layout budget is unchanged) opens a popup with seven controls
(Nsrc only appears when Algo is set to MUSIC):

- **PAUSE / RESUME** button, at the top of the popup. Freezes the video frame and the
  acoustic overlay in place (camera capture and the CSM/beamform update are both
  skipped while paused; the mic stream itself keeps running in the background, so
  resuming picks back up on live audio rather than a stale buffer). The status line
  in the top-left of the video is prefixed `[PAUSED]` even when the popup is closed,
  so the paused state is visible without opening it.
- **AUTO / MANUAL** toggle. Auto (default) is the original behavior: the color map
  auto-ranges every frame off a slowly-decaying running max of the beamformed power
  (`ref_power`), so relative "hot spots" stay visible regardless of absolute signal
  level. Manual instead measures power against a fixed reference floor and rescales
  the color map against a fixed span starting at an absolute threshold, so the same
  sound always maps to the same color run-to-run instead of adapting to whatever's
  currently loudest.
- **Thresh** slider (0–100 dB, default 30). Only takes effect in Manual mode. Grid
  cells below the threshold fade toward the colormap's coolest color rather than
  disappearing; cells at or above `thresh_db + 30 dB` saturate at the hottest color.
- **Algo** dropdown (`DS` / `MVDR` / `CLEAN` / `MUSIC`; see "Settings Persistence"
  below for how the starting value is chosen). Tapping it expands a 4-row option list
  below the button; tapping an option selects it and collapses the list. Switches the
  live beamformer without restarting the script — same effect as `--algo` at launch,
  just changeable at runtime. The popup grows to fit the expanded list and shrinks
  back when collapsed.
- **Nsrc** slider (1 to `N_MICS - 1` = 15; see "Settings Persistence" for the starting
  value). Only shown when Algo is MUSIC — sets the signal-subspace dimension (number
  of sources) `beamform_music` looks for; irrelevant to the other three algorithms, so
  the row (and the popup's height) only appears while MUSIC is selected.
- **SNAP** button. Saves the exact composite currently on screen (video + overlay +
  the Fhi/spectrum/Flo strips below it, not just the video pane) as a PNG to
  `~/Code/Acoustic-Camera/screengrabs/` (created on startup if missing), named
  `screengrab_<timestamp>.png`; a numeric suffix is appended if a file with that
  second's timestamp already exists, so rapid taps don't silently overwrite each other.
  Confirmation is on-screen, not just the console print: a brief (150 ms) full-screen
  white flash — camera-shutter style — followed by a green "Saved &lt;filename&gt;"
  caption for 1.5 s. Both are drawn only on the live preview *after* the file is
  written, so neither ever ends up baked into the saved PNG itself.
- **EXIT** button, at the bottom of the popup, separated from the other controls (and
  colored a stronger red) since it's destructive and infrequently used. Cleanly shuts
  the script down — same effect as pressing `q`, runs the same `cam.release()`/
  `cv2.destroyAllWindows()` cleanup — just reachable without a keyboard.

Tapping the tab is the only way to open or close the popup — no outside-tap dismiss,
by design, to keep the interaction minimal (the algo dropdown follows the same rule:
only the Algo button or picking an option closes it). Labeled with a plain ASCII "E"
rather than a gear/settings glyph, and the dropdown's caret is a plain "v"/"^", since
OpenCV's Hershey fonts (used everywhere else in this UI) don't reliably render
extended-unicode symbols. The tab and popup are positioned from `frame_w`/`frame_h`
the same way the Flo/Fhi track position is derived in `_track_geom()` — no stored
pixel coordinates — via `_popup_layout()`.

### Power/Throttle Indicator

A background thread polls `vcgencmd get_throttled` every 3 s (bits 0-3 = currently
active undervoltage/throttling/frequency-capping/soft-temp-limit; bits 16-19 = the
same four latched since last boot — see the Raspberry Pi firmware docs). If any are
currently active, a red `! LOW VOLTAGE / THROTTLED !` line is drawn below the main
status line; if none are active now but one occurred earlier in the session, a dimmer
orange `(low-voltage/throttle event occurred)` line is shown instead. Both are drawn
directly on the video frame regardless of whether the settings popup is open, the same
way `[PAUSED]` is. Off a Raspberry Pi (no `vcgencmd`), the poll thread exits quietly on
its first attempt and the indicator simply never appears — see `_poll_throttled()`.

### Settings Persistence

`--config PATH` (default `~/Code/Acoustic-Camera/config.json`) is loaded on startup
and saved on exit (both the `q` key and the popup's EXIT button go through the same
`finally:` cleanup, so either one triggers a save — see `_save_config()`). Only the
touch-adjustable settings round-trip: frequency band (`flo`/`fhi`), Algo, Nsrc,
Auto/Manual, and Thresh — everything else (camera/display/hardware setup) stays
CLI-only, matching the split already drawn by what's in the popup versus what's a
plain CLI flag.

Precedence for the two settings that are *also* CLI flags (`--algo`, `--nsrc`): the
saved config value wins whenever the config file has one; the CLI flag is only the
fallback for a first run, before any config file exists. So once you've picked MVDR
via the popup, `--algo ds` on the next launch still starts as MVDR — the config file
is the source of truth once it exists, not the command line. Delete or hand-edit the
file (see below) to actually change it from outside the app. `flo`/`fhi`/
`auto_range`/`thresh_db` have no CLI equivalents, so the saved value (or its
hardcoded default) always applies to them regardless.

The file is plain JSON and safe to hand-edit or delete; a missing, corrupt, or
partially-invalid file (e.g. an out-of-range `thresh_db`, or an `algo` string that
isn't one of the four real algorithms) is never fatal — each field falls back to its
built-in default independently rather than rejecting the whole file (see
`_load_config()`/`_cfg_int()`).

Running this script on a Raspberry Pi 5 instead of a desktop: see [RASPBERRY_PI.md](./RASPBERRY_PI.md)
for OS packages, PortAudio/OpenCV gotchas, camera/display caveats, and performance expectations.
