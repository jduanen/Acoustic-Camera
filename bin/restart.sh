#!/bin/bash
#
# Script to restart the Acoustic Camera service
#
# N.B. Must install /etc/sudoers.d/acoustic-camera first to ensure no passwd is required

sudo /usr/bin/systemctl restart acoustic-camera.service
