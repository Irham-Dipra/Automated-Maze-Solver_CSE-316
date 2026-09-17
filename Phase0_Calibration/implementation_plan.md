# Calibration Analysis & Fixes

## 1. The "Ghost Corner" (Why it turned right into a wall)
Look at `06:22:03.608 CORNER DETECTED! [L:31 R:255]`. The car had physically drifted so far to the left (`L:31`) that the car was angled diagonally. Because it was angled, the Front sensor saw the *side* of the left book wall! The code correctly said: "I see a wall in front of me, and the right side is open (R:255). I must turn right!" 
**Solution:** Once we perfectly calibrate the straight-line speeds (and eventually turn PID back on), the car will never drift diagonally into a wall like this.

## 2. The 4-Second Stall (Why the car didn't move)
After your second Right turn, the log shows `BLIND DRIVE` printing perfectly for 4 seconds, but the sensors didn't change. Then, abruptly at `06:22:24.520`, the car shot forward and jerked to the left.
**This is a mechanical issue, not a software or friction issue!** 
When a robot turns in place, the front **caster wheel** swivels sideways. When you try to drive straight again, that sideways wheel acts like a parking brake! The motors stall and vibrate for a few seconds until the wheel finally snaps straight, and the car shoots forward (which is why it jerked left).
**Solution:** I will add a **Kickstart Burst** to the code. Immediately after a turn, the ESP32 will send 100% max power (`255`) for just 100 milliseconds. This violent burst will instantly snap the caster wheel straight, and then it will drop down to your calibrated base speeds to drive smoothly!

## 3. Why did 160/181 drift Right?
You correctly noted that `128:145` drove straight, but scaling it up to `160:181` drifted right. This is because cheap DC motors are non-linear! A motor might be weak at 50% power, but suddenly hit peak efficiency at 70% power. You cannot scale them mathematically. 
**Solution:** You just have to manually find the right pair of numbers (e.g., `160:150` or `170:165`) that drives straight. 

## 4. The Book Maze (Inconsistent Walls)
Because books have gaps and varying thicknesses, the sensors are jumping wildly (e.g. `177 -> 218 -> 8190 -> 180`). 
**Solution:** I will add a **Rolling Average Filter** to the side sensors. It will mathematically smooth out the bumps in the books so the car sees a perfectly flat virtual wall!

## Proposed Code Changes (Phase0_Calibration.ino)
1. Add `F:[dist]` to the Bluetooth `BLIND DRIVE` log so you can monitor the front sensor.
2. Add the 100ms `255` PWM "Kickstart" after every corner to fix the caster wheel jam.
3. Implement a Rolling Average Filter for the Left and Right sensors to smooth out the book walls.
