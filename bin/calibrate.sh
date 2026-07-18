#!/bin/bash
#
# Launches the UMA-16 calibration app (src/calibrate_uma16.py) in a terminal
# so its prompts and results are visible. Run from the "Calibrate" desktop
# icon (see RASPBERRY_PI.md §8).

cd /home/jdn/Code/Acoustic-Camera
exec /home/jdn/Code/Acoustic-Camera/venv/bin/python /home/jdn/Code/Acoustic-Camera/src/calibrate_uma16.py
