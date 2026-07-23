#!/usr/bin/env bash

echo "=== Pi power/thermal status ==="
date
echo

echo "Temperature:"
vcgencmd measure_temp
echo

echo "Voltages:"
for rail in core sdram_c sdram_i sdram_p; do
  vcgencmd measure_volts "$rail"
done
echo

echo "Throttled:"
vcgencmd get_throttled
echo

throttled=$(vcgencmd get_throttled | cut -d= -f2)
if [[ "$throttled" == "0x00000" ]]; then
  echo "no voltage or throttling issues seen"
elif [[ "$throttled" == "0x50000" ]]; then
  echo "Warning: undervoltage and throttling occurred earlier in this boot"
elif [[ "$throttled" == "0x50005" ]]; then
  echo "Warning: undervoltage and throttling are happening now"
fi
