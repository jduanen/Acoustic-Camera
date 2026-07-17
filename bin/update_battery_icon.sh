#!/bin/bash
#
# Renders assets/battery_icon.png with the current battery percentage (see
# bin/battery_icon.py). Run periodically by battery-icon.timer, and by tapping
# the "Battery" desktop icon for an immediate refresh.

exec /home/jdn/Code/Acoustic-Camera/venv/bin/python /home/jdn/Code/Acoustic-Camera/bin/battery_icon.py
