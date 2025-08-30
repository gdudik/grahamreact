#!/usr/bin/env zsh

PORT=/dev/tty.usbmodem3133201

for i in {1..50}; do
  echo "Iteration $i of 50…"

  # soft-reset and reconnect
  mpremote connect $PORT exec "import machine; machine.soft_reset()"

  sleep 2

  # import, setup, start logging
  mpremote connect $PORT exec "import fifo_comms_stats; fifo_comms_stats.setup(); fifo_comms_stats.start_loop()"

  # pause 7 seconds
  sleep 1
done

echo "All done."
