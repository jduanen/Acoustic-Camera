#!/usr/bin/env bash
# Build cad/assembly/stack_assembly.blend and render stack_assembly.mp4.
#   ./make_assembly.sh            build + render
#   ./make_assembly.sh --no-render
# Needs: kicad-cli, blender (5.x), ffmpeg, python3 (a venv with cascadio is made in build/).
# Renders with Cycles on every CUDA GPU plus all CPU threads; RENDER_DEVICE=CPU forces CPU only.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$HERE/build
JOBS=$(nproc)
mkdir -p "$B/frames"

if [ ! -x "$B/venv/bin/python" ]; then
    python3 -m venv "$B/venv" && "$B/venv/bin/pip" install -q cascadio numpy
fi

# Boards: GLB origin = centre of the HENC1-4 M4 hole square on each board.
# The four model exports are independent, so run them in parallel.
OPTS="--include-tracks --include-pads --include-silkscreen --include-soldermask --include-zones -f"
kicad-cli pcb export glb $OPTS --user-origin 268.863x205.363mm -o "$B/mic_array.glb" "$ROOT/pcb/single_fpga/mic_array.kicad_pcb" & p1=$!
kicad-cli pcb export glb $OPTS --user-origin 100x100mm         -o "$B/front_end.glb" "$ROOT/pcb/single_fpga/front_end.kicad_pcb" & p2=$!

# Vendor STEP models -> GLB
step2glb() {
    "$B/venv/bin/python" -c 'import sys, cascadio; cascadio.step_to_glb(sys.argv[1], sys.argv[2], tol_linear=float(sys.argv[3]), tol_angular=0.3)' "$@"
}
step2glb "$ROOT/cad/AC7200-step.stp" "$B/ac7200.glb" 0.02 & p3=$!
step2glb "$ROOT/pcb/libraries/AXK5S80347YG/axk5s80347y.stp" "$B/axk5s.glb" 0.01 & p4=$!
for p in $p1 $p2 $p3 $p4; do wait "$p"; done

blender -b -t "$JOBS" --python "$HERE/build_assembly.py" -- "$B" "$HERE/stack_assembly.blend"

[ "${1:-}" = "--no-render" ] && exit 0

# Cycles' device choice is a user preference, not stored in the .blend, so set it here.
# (This Blender build has CUDA but not OptiX; the first GPU render compiles and caches the
# kernels in ~/.cache/cycles/kernels, which takes a few minutes once.)
DEVICES=$(cat <<'PY'
import bpy, os
p = bpy.context.preferences.addons["cycles"].preferences
use_gpu = os.environ.get("RENDER_DEVICE", "GPU").upper() != "CPU"
if use_gpu:
    try:
        p.compute_device_type = "CUDA"
        p.get_devices()
    except TypeError:
        use_gpu = False
gpus = [d for d in p.devices if d.type != "CPU"] if use_gpu else []
for d in p.devices:
    d.use = bool(gpus)                      # every GPU plus the CPU
for s in bpy.data.scenes:
    s.cycles.device = "GPU" if gpus else "CPU"
print("render devices:", [d.name for d in p.devices if d.use] if gpus else "CPU only")
PY
)
blender -b "$HERE/stack_assembly.blend" -t "$JOBS" --python-expr "$DEVICES" -o "$B/frames/f_####" -F PNG -a
ffmpeg -y -loglevel error -threads "$JOBS" -framerate 24 -i "$B/frames/f_%04d.png" -c:v libx264 -pix_fmt yuv420p -crf 18 "$HERE/stack_assembly.mp4"
echo "wrote $HERE/stack_assembly.blend and $HERE/stack_assembly.mp4"
