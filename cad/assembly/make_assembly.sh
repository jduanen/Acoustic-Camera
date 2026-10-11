#!/usr/bin/env bash
# Build cad/assembly/stack_assembly.blend and render stack_assembly.mp4.
#   ./make_assembly.sh            build + render
#   ./make_assembly.sh --no-render
# Needs: kicad-cli, blender (5.x), ffmpeg, python3 (a venv with cascadio is made in build/).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
B=$HERE/build
mkdir -p "$B/frames"

if [ ! -x "$B/venv/bin/python" ]; then
    python3 -m venv "$B/venv" && "$B/venv/bin/pip" install -q cascadio numpy
fi

# Boards: GLB origin = centre of the HENC1-4 M4 hole square on each board
OPTS="--include-tracks --include-pads --include-silkscreen --include-soldermask --include-zones -f"
kicad-cli pcb export glb $OPTS --user-origin 268.863x205.363mm -o "$B/mic_array.glb" "$ROOT/pcb/single_fpga/mic_array.kicad_pcb"
kicad-cli pcb export glb $OPTS --user-origin 100x100mm         -o "$B/front_end.glb" "$ROOT/pcb/single_fpga/front_end.kicad_pcb"

# Vendor STEP models -> GLB
"$B/venv/bin/python" - "$ROOT" "$B" <<'PY'
import sys, cascadio
root, b = sys.argv[1:]
cascadio.step_to_glb(f"{root}/cad/AC7200-step.stp", f"{b}/ac7200.glb", tol_linear=0.02, tol_angular=0.3)
cascadio.step_to_glb(f"{root}/pcb/libraries/AXK5S80347YG/axk5s80347y.stp", f"{b}/axk5s.glb", tol_linear=0.01, tol_angular=0.3)
PY

blender -b --python "$HERE/build_assembly.py" -- "$B" "$HERE/stack_assembly.blend"

[ "${1:-}" = "--no-render" ] && exit 0
blender -b "$HERE/stack_assembly.blend" -o "$B/frames/f_####" -F PNG -a
ffmpeg -y -loglevel error -framerate 24 -i "$B/frames/f_%04d.png" -c:v libx264 -pix_fmt yuv420p -crf 18 "$HERE/stack_assembly.mp4"
echo "wrote $HERE/stack_assembly.blend and $HERE/stack_assembly.mp4"
