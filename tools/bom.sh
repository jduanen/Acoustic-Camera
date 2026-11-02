#!/bin/bash
#
# Script to pretty print the BOMs for all of this project's PCBs


echo "PCB: mic_test"
mlr --icsv --opprint cat ~/Code/Acoustic-Camera/pcb/mic_test/bom/mic_test_bom.csv
echo ""

echo "PCB: mic_array"
mlr --icsv --opprint cat ~/Code/Acoustic-Camera/pcb/single_fpga/bom/mic_array_bom.csv
echo ""

echo "PCB: front_end"
mlr --icsv --opprint cat ~/Code/Acoustic-Camera/pcb/single_fpga/bom/front_end_bom.csv
echo ""
