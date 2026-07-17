#!/bin/bash
#
# Sync the files needed to run the Acoustic Camera to the Raspberry Pi appliance.
#
# Only syncs what the running app / systemd unit / desktop icons actually need —
# not docs, notebooks, pcb design files, or dev-only tooling. Does NOT touch
# venv/ (built on-device with --system-site-packages, see RASPBERRY_PI.md §2),
# config.json, or screengrabs/ (Pi-local runtime state) since those paths are
# never passed as rsync sources.
#
# Usage:
#   bin/sync_to_pi.sh              # sync
#   bin/sync_to_pi.sh --dry-run    # preview changes (any rsync flag can be passed through)
#
# Override target with env vars: PI_HOST, PI_USER, PI_DIR.

set -euo pipefail

PI_HOST="${PI_HOST:-acousticcamera.lan}"
PI_USER="${PI_USER:-jdn}"
PI_DIR="${PI_DIR:-/home/jdn/Code/Acoustic-Camera}"

cd "$(dirname "$0")/.."

ssh "${PI_USER}@${PI_HOST}" "mkdir -p '${PI_DIR}'"

rsync -avz --relative --delete \
  --exclude='__pycache__/' \
  src/ \
  ups/UPS_Module_3S_V2/ \
  test/UMA16/cal.npy \
  bin/ \
  etc/ \
  Desktop/ \
  assets/ \
  requirements.txt \
  requirements.lock \
  "$@" \
  "${PI_USER}@${PI_HOST}:${PI_DIR}/"
