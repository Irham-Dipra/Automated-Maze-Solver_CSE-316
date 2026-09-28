#!/bin/sh
# Host-side regression test for the ToF near-wall latch.
#
# Replays the failure from bluetooth_logs (14:18:17): the robot stops 105 mm
# off a wall, pivots, and the front sensor must then report the open corridor
# ahead instead of staying jammed at "wall touching me".
#
# Runs entirely on a PC -- no ESP32, no robot. The sketch is compiled against
# the stubs in stub/, with a scripted fake VL53L0X and a controllable clock.
set -e
cd "$(dirname "$0")"
SKETCH=../Phase5_ESP32_Brain/Phase5_ESP32_Brain.ino

sed -e 's/^void setup(/void sketch_setup(/' -e 's/^void loop(/void sketch_loop(/' "$SKETCH" > body.inc
sed -i '1i #include <Arduino.h>' body.inc

g++ -I stub -std=gnu++17 -DSKETCH_INC='"body.inc"' -o t_latch test_common.cpp
echo "=== near-wall latch: pivot from a 30 mm wall into an open corridor ==="
if ./t_latch; then
    echo "PASS"
    rm -f body.inc t_latch
else
    echo "FAIL -- the front sensor never reports the corridor as open."
    echo "        This is the bug that made the robot turn right at every"
    echo "        junction after its first pivot."
    rm -f body.inc t_latch
    exit 1
fi
