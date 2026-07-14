#!/bin/bash
#
# Script to get the status of the Acoustic Camera service

sudo /usr/bin/systemctl status --no-pager --full acoustic-camera.service
