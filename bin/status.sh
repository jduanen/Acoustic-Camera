#!/bin/bash
#
# Script to get the status of the Acoustic Camera service

systemctl status --no-pager --full acoustic-camera.service
