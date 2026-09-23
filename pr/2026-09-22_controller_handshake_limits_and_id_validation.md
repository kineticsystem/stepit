# Controller handshake: identity, motor-id validation, motion limits and firmware version

**Date:** 2026-09-22
**Branch:** `main` (uncommitted working tree)
**Status:** ready for review, nothing committed

---

## 1. Summary

The driver used to trust the serial port path. Opening `/dev/ttyACM0` and getting *any* answer was treated as "the robot is connected", and the motor ids reported by the controller were used directly as indices into the joint vector. This change turns the connection into a real handshake and makes the controller the authority on what its motors can physically do.

Five related pieces of work:

1. **Motor-id validation and state routing** — a reported id is no longer used as a vector index.
2. **`configureCommand` implemented in the firmware** — the acceleration and maximum velocity sent by the host were parsed and discarded. They are now applied, bounded by the board's empirical limits.
3. **Motion limits reported in the info response** — the controller tells the host what it tolerates instead of the host guessing.
4. **Firmware version in the info response** — with a protocol-compatibility check.
5. **Tooling** — headless flashing, a `get_info` hardware utility, a printing fix.

> **Breaking change.** The info response layout changed. The driver and the firmware must be updated together. A board running older firmware is refused at connect time.

---

## 2. Issues

The following problems were discovered:

- `StepitHardware::on_configure()` **ignored the return value of `connect()`**, so even a failed handshake proceeded to configure the hardware.
- The URDF declared limits that didn't match those of the MPU i.e. **5 rotations/s** and **0.5 rotations/s²**, while the firmware was hard-coded to **3 rotations/s** and **2 rotations/s²**, so simulation was incorrect. The firmware silently clamped velocity and ignored acceleration entirely.

These are **open-loop steppers with no encoders**: exceeding a limit does not fault, it loses steps, and the reported position silently diverges from reality. That is why the design favours failing loudly at start-up over quietly clamping.

---

## 3. What changed, by area

### 3.1 Motor-id validation and state routing

`src/stepit_hardware/` — `on_init()` now builds `joint_index_by_id_`, a map from configured motor id to joint index, and rejects:

- an id outside `[0, num_joints)`;
- a duplicate id across joints.

`read()` routes each reported state through that map instead of `joints_[motor_id]`. `motor_states_are_valid()` verifies that a status response reports exactly the configured set of ids, each once, and runs both at configure time and on every read cycle. It tracks the joints it has already seen in a local `uint32_t` bitmask, so the read path allocates nothing. That sets an upper bound on the number of joints, which `on_init()` now states and enforces (`kMaxJoints = 32`).

`on_init()` also resets all derived state, so calling it twice no longer reports every id as a duplicate of itself.

**Firmware:** `configureCommand`, `speedCommand` and `moveCommand` reject an id `>= NUMBER_OF_MOTORS` rather than indexing past the end of the motor arrays.

### 3.2 `configureCommand` implemented

`src/stepit_mcu/src/main.cpp`

The empirical constants became an explicit ceiling:

```cpp
constexpr float MAX_ACCELERATION = 6400.0;  // 2 rotations per square second
constexpr float MAX_SPEED = 9600.0;         // 3 rotations per second
```

`motorConfig` starts at those limits and is now genuinely mutable. The command validates in one pass and applies in a second, so a bad entry cannot leave the motors half configured, and it applies under the `writingMotorGoals` guard because the ISR also writes the stepper maximum speed.

Design rule, deliberately asymmetric:

| When | Behaviour | Why |
|---|---|---|
| `configureCommand` (start-up) | **reject** out-of-range values | a one-off step; the host would otherwise plan motions the hardware never performs and, with no encoders, never notice |
| `speedCommand` (control loop) | **clamp** with `min()` | failing a control cycle is worse than slowing it down |

Three latent bugs were fixed alongside: `if (...) { returnCommandError(); };` had **no `return`** in all three command handlers, so a malformed packet got an error reply *and* was then parsed anyway.

`setMotorsEnabled` had its polarity inverted against its own documentation — `enabled == 0` started the interrupt that drives the motors — and now enables on a non-zero byte as the comment says. Nothing on the ROS side sends this command, so the inversion was dormant; it is fixed before anything is built on top of it.

**URDF** (`src/robot_description/urdf/stepit.ros2_control.xacro`) corrected on all five joints to match the board limits:

```xml
<param name="acceleration">12.5663706143592</param> <!-- 2 rotations/s^2 -->
<param name="max_velocity">18.8495559215388</param> <!-- 3 rotations/s -->
```

These round-trip through `STEPS_IN_ONE_ROTATION` to exactly 6400 and 9600 steps.

### 3.3 Limits reported in the info response

The controller now reports, per motor, the maximum acceleration and velocity it tolerates. `StepitHardware::on_configure()` reads them and validates the URDF *before* sending any configuration, so instead of an opaque rejection you get:

```
Motor 0: acceleration 3.141590 rad/s^2 exceeds the controller limit of 1.570795 rad/s^2.
```

Limits are reported **per motor** although they are currently global, so motors with different mechanics can report different limits without a protocol change.

`FakeDriver` mirrors the same validation and answers the same limits from `hardware_limits.hpp`, so a configuration accepted in simulation is one the real controller accepts. It previously accepted anything — including a zero acceleration, which divides by zero in `position_kinematics` (`pow(v_max, 2) / a`).

The firmware remains the authority; `hardware_limits.hpp` is the simulator's copy, and `get_info` (§3.5) checks the two agree against the real board.

### 3.4 Firmware version

```cpp
constexpr byte VERSION_MAJOR = 1;   // wire compatibility of the protocol
constexpr byte VERSION_MINOR = 0;
constexpr byte VERSION_PATCH = 0;
```

`connect()` checks both name and version independently. Convention: **bump major when a packet layout changes**, minor/patch for everything else.

### 3.5 Tooling

- **`src/stepit_mcu/platformio.ini`** — `upload_protocol = teensy-cli`. The default (`teensy-gui`) opens a window and reports `[SUCCESS]` for *launching the loader*, not for flashing, which makes scripted or remote uploads unverifiable. The CLI loader prints `Programming... Booting`.
- **`src/stepit_hardware_tests/src/get_info.cpp`** (new) — asks the real board for its name, version and limits and checks them against `hardware_limits.hpp`, exiting non-zero on disagreement. This is what makes the duplicated constants verifiable rather than comment-enforced.
- **`src/stepit_hardware_tests/src/get_status.cpp`** — motor id was streamed as a `uint8_t`, so ids 0–4 printed as unprintable characters, plus a stray `"rad"` suffix on an id; now cast to `int` and printed without a unit. Display only: the ids were always parsed correctly.

---

## 4. Protocol change

**Info response (`0x76`), before:**

```
status (1) | name (N, ASCII)
```

**After:**

```
status (1) | version major, minor, patch (3) | motor count (1)
           | per motor: id (1), max acceleration (4), max velocity (4)
           | name (N, ASCII)
```

The name stays the **trailing** field so it remains "the remaining bytes" and parsing stays simple. The driver length-checks before reading, so a truncated packet throws instead of reading past the end.

Old firmware against this driver fails the **name** check (the name lands in the wrong place) rather than the version check — 1.0.0 is the first firmware to report a version at all, so the clean diagnostic only exists from this release forward.

---

## 5. Files

| File | Change |
|---|---|
| `src/stepit_mcu/src/main.cpp` | limits as ceiling, `configureCommand` implemented, id checks, missing `return`s, version + limits in info response |
| `src/stepit_mcu/platformio.ini` | `upload_protocol = teensy-cli` |
| `src/robot_description/urdf/stepit.ros2_control.xacro` | acceleration/max_velocity corrected on 5 joints |
| `src/stepit_driver/include/stepit_driver/hardware_limits.hpp` | **new** — simulator's copy of the board limits + version |
| `src/stepit_driver/include/stepit_driver/msgs/info_response.hpp` | `MotorLimits`, `Version` |
| `src/stepit_driver/src/msgs/info_response.cpp` | constructor and accessors |
| `src/stepit_driver/src/default_driver.cpp` | version check in `connect()`, parse limits + version |
| `src/stepit_driver/src/fake/fake_driver.cpp` | mirrored validation, reports limits and version, motion commands fail on unknown id |
| `src/stepit_hardware/include/.../stepit_hardware.hpp` | id map, two predicates |
| `src/stepit_hardware/src/stepit_hardware.cpp` | `connect()` result checked, limit validation, id routing, state validation |
| `src/stepit_hardware_tests/src/get_info.cpp` | **new** — firmware vs `hardware_limits.hpp` check |
| `src/stepit_hardware_tests/src/get_status.cpp` | motor id printing |
| `src/stepit_hardware_tests/CMakeLists.txt` | `get_info` target |
| `src/stepit_driver/tests/test_fake_driver.cpp` | **new** — 7 tests, `FakeDriver` had none |
| `src/stepit_driver/tests/test_default_driver.cpp` | handshake, limits, version, truncated packet |
| `src/stepit_driver/tests/CMakeLists.txt` | `test_fake_driver` target |
| `src/stepit_hardware/tests/test_stepit_hardware.cpp` | id validation, limit validation, re-init |

---

## 6. Tests

`colcon test` — **63 tests, 0 failures**.

| Suite | Tests | New coverage |
|---|---|---|
| `test_default_driver` | 10 | name mismatch, no response, truncated info packet, incompatible version, limits parsing |
| `test_fake_driver` | 7 | **new file** — limits, non-positive/NaN, unknown id, whole-batch rejection |
| `test_fake_motor` | 3 | unchanged |
| `test_stepit_hardware` | 17 | out-of-range/duplicate id at init and at read, id routing with shuffled ids, limit validation, re-init, joint-count bound |
| `cobs_serial` | 19 | unchanged |

Two tests were checked to be non-vacuous by temporarily reverting the fix and confirming they fail: `init_can_run_twice` and the `get_info` drift check.

`read_routes_states_by_configured_id` deliberately uses a shuffled id order, and `configure_accepts_the_declared_limits` hardcodes the URDF literals so that tightening a check can never silently make the shipped description unconfigurable.

---

## 7. Hardware verification

Run against the real Teensy 4.1 (`Teensyduino_USB_Serial_12382150`) with five steppers attached, each carrying a clock hand starting at 0 rad.

**Handshake and limits**

```
Controller: STEPIT, firmware 1.0.0
 - motor 0..4: max acceleration 12.5664rad/s^2, max velocity 18.8496rad/s
The firmware agrees with hardware_limits.hpp.
```

**Negative cases** (configuration only, no motion):

| Case | Result |
|---|---|
| 5 joints at exactly the limit | Success |
| below the limit | Success |
| acceleration 2× over | Failure |
| velocity 2× over | Failure |
| zero acceleration / negative velocity | Failure |
| motor id 99 | Failure |
| batch whose last entry is bad | Failure |

**Version check** — a firmware deliberately flashed as major 2 produced:

```
The StepIt controller runs firmware 2.0.0, which speaks version 2 of the protocol;
this driver speaks version 1. Flash the firmware that matches this workspace.
```

Restored to 1.0.0 afterwards.

**Motion** (run twice, identical results):

| # | Phase | Expected | Measured |
|---|---|---|---|
| 1 | motor 0, one turn at 0.25 rot/s | slow full revolution | 1.000 rot |
| 2 | motor 0 back at 1.0 rot/s | same move, visibly faster | 0.000 rot |
| 3 | all five, half turn | hands 12 → 6 together | all 0.500 rot |
| 4 | staircase | 12 / 3 / 6 / 9 / 12 | 0 / 0.25 / 0.5 / 0.75 / 1.0 |
| 5 | velocity 0.5 rot/s for 4 s | ~2 free revolutions | +2.017 rot, 0.500 rot/s |
| 6 | ask 5 rot/s, configured 1.0 | held at the configured max | 1.000 rot/s |
| 7 | home | all back to 12 | all 0.000 rot |

Phases 1 vs 2 are the proof that the configure change takes effect: identical move, different configuration, visibly different speed. Before this change both would have run at the board's fixed 3 rot/s. Phase 6 confirms the runtime clamp still applies, now against host-supplied configuration. Positions round-tripped exactly and repeated bit for bit across two runs — no lost steps over ~13 commanded revolutions.

Motion is driven by continuous status polling because of the firmware's 1 s watchdog (`TIMEOUT_MS`), so this also exercised the real control-loop pattern.

---

## 8. Known issues and follow-ups

Deliberately left for a decision rather than fixed silently.

**Worth fixing before merge**

1. **Partial application remains in `speedCommand` and `moveCommand`.** `configureCommand` was given a two-pass parse so a bad entry cannot half-apply; the two motion commands were not. A bad id mid-packet returns an error *after* earlier motors in that packet have already had their goals set, so a "rejected" packet can leave motors moving. Hard to trigger now that the driver validates ids, but inconsistent with the function beside it. See §10: the recommended fix also stops the motors on rejection, which supersedes this item.

2. **A URDF with fewer than five joints is now impossible.** `on_init` rejects `id >= num_joints` and `motor_states_are_valid` requires the reported count to equal the joint count, while the firmware always reports five. The count half predates this change. The cleaner formulation, now available because the info response carries the motor count, is to validate ids against the *controller's* count and require "every configured joint appears exactly once" instead of equal counts.

**Minor**

3. **Redundant info query** — `connect()` fetches the info response and `on_configure()` immediately fetches it again.
4. **`get_info` throws where neighbours return a status** — a bare error reply is shorter than the fixed header, so it throws; `on_configure` then returns `ERROR` where the adjacent checks return `FAILURE`, and the `status != Success` branch is effectively unreachable for the real driver.
5. **`FakeDriver` and firmware disagree on duplicate ids** — `motors_.insert` keeps the *first* entry for a repeated id, the firmware applies the *last*. Unreachable through `StepitHardware`, but it is the kind of divergence this work exists to remove.
6. `const Version kFirmwareVersion` in a header has internal linkage — one copy per translation unit. A `constexpr` constructor and `inline constexpr` would be tidier.
7. `CONFIG_TOLERANCE` (0.1 %) turned out not to be load-bearing: the round trip lands exactly on 6400/9600. Kept as insurance against a differently-rounded literal, but it does mean the board accepts marginally above its stated limit.
8. `kMaxJoints = 32` is a bound imposed by the bitmask in `motor_states_are_valid()`, not by the hardware, which drives five motors. It is enforced in `on_init()` and tested, but it is an implementation detail surfacing as a public limit.

**Noticed, untouched**

9. The limits and the version now live in two places (`main.cpp` and `hardware_limits.hpp`). Unavoidable while simulation must run with no board attached, but `get_info` makes the drift detectable. Removing the duplication entirely would mean `FakeDriver` obtaining its limits some other way.

---

## 9. How to build, test and flash

```bash
# Build and test (inside the dev container)
./bin/build.sh
./bin/test.sh

# Flash the firmware (needs PlatformIO and the Teensy udev rules)
cd src/stepit_mcu && pio run -t upload

# Against a connected board
./build/stepit_hardware_tests/get_info   --port /dev/ttyACM0   # identity, version, limits
./build/stepit_hardware_tests/get_status --port /dev/ttyACM0   # positions and velocities
```

PlatformIO is not installed in the dev container; it was run from a virtualenv on the host for this work.

---

## 10. Follow-up: stopping the motors on a rejected motion command

Discussed on 2026-09-23, **not implemented**. Recorded here as a decision to make, not work that has been done.

**The question.** When `speedCommand` or `moveCommand` rejects a packet, the firmware replies with an error and returns, but the motors keep doing whatever they were doing. Should the firmware stop them instead?

**What makes this pressing.** `StepitHardware::write()` assigns the `AcknowledgeResponse` returned by `set_velocity()` / `set_position()` and never inspects it, returning `return_type::OK` unconditionally. So a rejected motion command produces an error byte that travels back over the wire and is discarded: the robot keeps moving and the ROS stack never learns anything. If the firmware does not act, nothing acts. This is the same class of defect as `on_configure()` ignoring the result of `connect()`, which this change already fixes.

**Precedent in the firmware.** The 1 s watchdog already stops everything on a different failure — silence — with `motorGoal[i].setSpeed(0)` under the goals guard. A malformed or invalid command is arguably a stronger signal than silence: silence can be scheduling jitter on a busy host, whereas a bad packet means the host is confused about the machine or the link is corrupting bytes. Reusing the same mechanism also means the motors decelerate at the configured acceleration rather than hard-halting, which on an open-loop stepper would itself lose steps.

### Recommended, in priority order

1. **Make `write()` inspect the acknowledgement** and return `return_type::ERROR` on failure, so `ros2_control` deactivates the component. Roughly three lines, and worth doing whatever is decided about the firmware: without it the firmware can report a problem that nothing hears.
2. **Stop the motors on rejection in `speedCommand` and `moveCommand`, folded into fixing the partial-application defect** (§8.1) rather than treated as a separate change. Give both handlers the two-pass validate-then-apply treatment `configureCommand` received, and on rejection zero every goal through the watchdog's existing path. That yields one coherent behaviour — nothing half-applied, machine stopped — instead of two half-measures, and it supersedes §8.1.
3. **Do not latch the fault for now.** A latched state requiring an explicit clear is the textbook answer, but it needs a fault state in the protocol, a reset command, host-side handling and re-activation semantics: real work for a failure mode that, after this change, a correct host can no longer trigger. A plain non-latched stop already covers the case that matters, because a genuinely malfunctioning host re-triggers it every cycle and it stays stopped; a single bad packet followed by good ones recovers, which is the desired behaviour there. Revisit if these axes ever carry a load. If it is built later, see §11: the mechanism it would naturally use disables without decelerating.
4. **Treat the CRC and framing path separately.** A corrupted frame never reaches `processBuffer`, so neither the error reply nor any stop-on-error logic in these handlers would fire — only the 1 s watchdog catches it, which is a loose reaction time. That is the realistic failure on a USB serial line, but it lives in `SerialPort` and belongs in its own change.

### Why it is not in this change

This change is already large and breaking, and stopping on error is a behavioural change to motion handling: the kind a reviewer wants to see in isolation, with its own test. It is also cheaply verifiable on the bench — start a motion, inject a malformed packet mid-move, and confirm the hands decelerate to a stop rather than continuing — which is a better artefact than the argument above.

### Counter-argument, for the record

On this rig, five free-running steppers with clock hands, stopping is unambiguously safe. On a loaded arm an unexpected stop has its own hazards: a dropped payload, stopping mid-operation, a vertical axis back-driving. That is not an argument against it here, but it is why this tends to be a per-machine policy rather than something a protocol mandates.

---

## 11. Follow-up: `setMotorsEnabled` disables without decelerating

Noted on 2026-09-23 while verifying the polarity fix described in §3.2. **Not implemented**, recorded as a decision to make.

`setMotorsEnabled(false)` calls `timer.end()`, which tears the interrupt down immediately. `stepper[i].run()` then stops being called mid-motion, so the motors halt without the deceleration ramp every other stop in this firmware goes through — the watchdog and `speedCommand` both stop by setting a goal speed of zero and letting the ISR decelerate at the configured acceleration.

On this rig that is harmless and was measured: disabling while motor 0 was turning at 0.25 rot/s froze it after a further 0.013 rot, which is the step already in flight rather than continued motion. Under load, though, an abrupt halt on an open-loop stepper is exactly the condition that loses steps, and the reported position would then diverge from the real one with nothing to detect it.

Arguably it is correct as it stands: a command named "disable the motors" that decelerates first is not really disabling them, and an emergency cut wants to be immediate. The distinction worth drawing is between *disable* (cut the drive now, accept the step loss) and *stop* (come to rest under control, then optionally disable).

**Why it matters for §10.** If the latched fault state discussed there is ever built on this mechanism, it should decelerate first and disable second, otherwise the safety response introduces the very position error it exists to prevent. That ordering is the thing to decide, not the current behaviour of the command in isolation.

**Verification already done.** The polarity is confirmed against the real board: `SET_MOTORS_ENABLED(false)` froze the motor and `(true)` resumed it. Nothing on the ROS side sends `0x7A`, so the command was exercised by writing the raw frame over the COBS layer; there is no automated test for it.
