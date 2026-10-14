#!/bin/bash
#
# Script to make an animated GIf from the stack assembly animation

BASE_DIR="${HOME}/Code/Acoustic-Camera"
ASSETS_DIR="${BASE_DIR}/assets"
MP4_FILE="${ASSETS_DIR}/stack_assembly.mp4"
GIF_FILE="${ASSETS_DIR}/p4_pcbs.gif"

ffmpeg -i ${MP4_FILE} -vf "fps=12,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" ${GIF_FILE}
