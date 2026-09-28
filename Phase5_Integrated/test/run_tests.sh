#!/bin/sh
# Host-side regression tests. No ESP32, no robot: the sketch is compiled
# against the stubs in stub/, with a scripted fake VL53L0X and a clock the
# test drives by hand.
set -e
cd "$(dirname "$0")"
SKETCH=../Phase5_ESP32_Brain/Phase5_ESP32_Brain.ino

sed -e 's/^void setup(/void sketch_setup(/' -e 's/^void loop(/void sketch_loop(/' "$SKETCH" > body.inc
sed -i '1i #include <Arduino.h>' body.inc
trap 'rm -f body.inc t_latch t_junction' EXIT

fail=0

echo "=== near-wall latch: pivot from a 30 mm wall into an open corridor ==="
g++ -I stub -std=gnu++17 -DSKETCH_INC='"body.inc"' -o t_latch test_common.cpp
if ./t_latch; then echo "PASS"; else
  echo "FAIL -- the front sensor never reports the corridor as open, which is"
  echo "        what made the robot turn at every junction after a pivot."
  fail=1
fi

echo
echo "=== junction decision and came-from blocking ==="
g++ -I stub -std=gnu++17 -o t_junction test_junction.cpp
if ./t_junction; then :; else fail=1; fi

exit $fail
