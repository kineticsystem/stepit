# Stop the motors on a rejected motion command

**Date:** 2026-09-23
**Branch:** `pr-error-stop`, based on `09f5046`
**Status:** ready for review, nothing committed
**Follows:** `2026-09-22_controller_handshake_limits_and_id_validation.md`, §10 and §8.1

---

## 1. Summary

When the firmware rejected a motion command it replied with an error and returned, leaving the motors doing whatever they were doing. Nothing else noticed either: `StepitHardware::write()` assigned the acknowledgement to a local and discarded it, returning `return_type::OK` unconditionally. A rejected command therefore produced an error byte that travelled back over the wire and vanished, while the robot carried on moving.

This implements §10 items 1 and 2 of the previous document, which also supersedes its §8.1.

- The host now reads the acknowledgement and reports the failure.
- The firmware stops the motors when it rejects a motion command, decelerating rather than cutting the motion dead.
- `speedCommand` and `moveCommand` validate the whole packet before applying any of it, so a rejected packet can no longer leave the motors half commanded.

Firmware version **1.0.0 → 1.1.0**: behaviour changed, the wire layout did not.

---

## 2. Why

These are open-loop steppers with no encoders. A goal that the controller refuses, or applies to only some of the motors, leaves the host's model of where the robot is quietly wrong, and nothing downstream can detect it. The handshake work in the previous change made the controller validate what it is asked to do; this change makes a rejection actually mean something on both sides of the link.

The firmware already took exactly this position for a different failure: the 1 s watchdog stops every motor when the host goes quiet. A malformed or invalid command is arguably a stronger signal than silence — silence can be scheduling jitter on a busy host, whereas a bad packet means the host is confused about this machine or the link is corrupting bytes.

---

## 3. What changed

### 3.1 The host reads the acknowledgement

`StepitHardware::write()` inspects the `AcknowledgeResponse` from `set_velocity()` and `set_position()` and returns `hardware_interface::return_type::ERROR` when the controller refuses, so `ros2_control` deactivates the component instead of the control loop carrying on as though the goal had been taken. This is the same class of defect as `on_configure()` ignoring the result of `connect()`, fixed in the previous change.

### 3.2 One place that stops the motors

```cpp
void stopAllMotors()
{
  Guard goalGuard{ writingMotorGoals };
  for (byte i = 0; i < NUMBER_OF_MOTORS; i++)
  {
    motorGoal[i].setSpeed(0);
  }
}
```

Asking for a speed of zero lets the ISR decelerate the motors at their configured acceleration. That matters here: tearing the motion down abruptly is what loses steps, so a safety response that hard-stopped would introduce the very position error it exists to prevent. The 1 s watchdog now calls the same helper rather than repeating the loop, so both paths stop the machine the same way.

### 3.3 Validate the whole packet, then apply it

`speedCommand` and `moveCommand` now follow the structure `configureCommand` was given in the previous change: a first pass parses and validates every entry, a second applies them under the goals guard. Both reject a packet whose length is not a multiple of the entry size, or which carries more entries than there are motors, and both call `stopAllMotors()` before replying with an error.

Previously each entry was applied as it was parsed, so a bad id in the middle of a packet returned an error *after* the preceding motors had already been given new goals — the machine executing half a coordinated move.

### 3.4 Not latched, deliberately

A rejection stops the motors but does not latch a fault. A genuinely malfunctioning host re-triggers the stop on every cycle, so the machine stays stopped; a single bad packet followed by good ones recovers, which is the wanted behaviour there. Phase 3 of the bench test below confirms the recovery. The reasoning for not building a latched fault state now is in §10.3 of the previous document, and §11 there records the ordering problem to solve first if it is ever built.

---

## 4. Files

| File | Change |
|---|---|
| `src/stepit_mcu/src/main.cpp` | `stopAllMotors()`; two-pass `speedCommand` and `moveCommand` with stop-on-rejection; watchdog reuses the helper; version 1.1.0 |
| `src/stepit_hardware/src/stepit_hardware.cpp` | `write()` inspects the acknowledgement and returns `ERROR` |
| `src/stepit_driver/include/stepit_driver/hardware_limits.hpp` | `kFirmwareVersion` 1.1.0, kept in step with the firmware |
| `src/stepit_hardware/tests/test_stepit_hardware.cpp` | two tests for a rejected write |

180 insertions, 38 deletions across 4 files.

---

## 5. Tests

`colcon test` — **65 tests, 0 failures** (`test_stepit_hardware` 19, up from 17).

New: `write_reports_a_rejected_velocity_command` and `write_reports_a_rejected_position_command`. Both were checked to be non-vacuous by removing the `return ERROR` from `write()` and confirming they fail.

There is no unit test for the firmware side; `stepit_mcu` is a PlatformIO project outside the colcon workspace and has no test harness. It is covered by the bench test below.

---

## 6. Hardware verification

Teensy 4.1 with five steppers, flashed with this firmware. Motor 0 was run at 0.25 rot/s and a velocity command naming motor 99 was injected mid-motion.

```
[1] motor 0 spinning at 0.25 rot/s
    running   vel: 0.02 0.06 0.12 0.17 0.22 0.25 0.25 0.25 0.25 0.25   -> pos 0.375 rot

[2] inject a velocity command naming motor 99 (invalid)
    controller replied: Failure
    after bad vel: 0.25 0.20 0.15 0.10 0.04 0.00 0.00 0.00 ...          -> pos 0.513 rot

[3] a valid command must resume motion (the stop is not latched)
    resumed   vel: 0.02 0.06 0.11 0.17 0.22 0.25 0.25 0.25 0.25 0.25   -> pos 0.900 rot

RESULT: rejected=yes, motors stopped=yes, recovered=yes
```

The velocity trace in phase 2 is the result worth reading: `0.25 → 0.20 → 0.15 → 0.10 → 0.04 → 0.00` is a deceleration ramp, not a dead cut, which is what distinguishes this from `setMotorsEnabled(false)` (see §11 of the previous document). Phase 3 confirms the stop is not latched.

`get_info` against the board afterwards reports `STEPIT, firmware 1.1.0` and *"The firmware agrees with hardware_limits.hpp"*.

`pre-commit run -a` in the dev container: all hooks pass.

---

## 7. Known issues and follow-ups

1. **The malformed-length path is untested on hardware.** The bench test injects an invalid motor id, which is reachable through the driver. A packet whose length is not a multiple of the entry size cannot be produced by `DefaultDriver`, so that branch is covered by inspection only.

2. **§10.4 of the previous document is still open.** A corrupted frame never reaches `processBuffer`, so neither the error reply nor any of this stop-on-rejection logic fires; only the 1 s watchdog catches it. That is the realistic failure on a USB serial line and this change does not improve it. It lives in `SerialPort` and belongs in its own change.

3. **A rejected write deactivates the whole component.** That is the blunt instrument `ros2_control` offers, and it is the right default while a rejection means the host and the controller disagree about the machine. If transient rejections ever become expected, this wants revisiting.

4. **Items 2 to 9 of §8 in the previous document remain open**, including the sub-five-joint URDF restriction, which is the natural next piece.

---

## 8. How to build, test and flash

```bash
# Inside the dev container
./bin/build.sh
./bin/test.sh

# Firmware (needs PlatformIO and the Teensy udev rules)
cd src/stepit_mcu && pio run -t upload

# Against a connected board
./build/stepit_hardware_tests/get_info   --port /dev/ttyACM0
./build/stepit_hardware_tests/get_status --port /dev/ttyACM0
```

The driver and the firmware do not have to be flashed together for this change: the wire layout is unchanged and the major version is still 1, so an older board still handshakes. It simply will not stop its motors when it rejects a command.
