# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

StepIt is a ROS 2 (Humble) project for controlling stepper motors via a Teensy 4.x microcontroller. It uses `ros2_control` as the hardware abstraction layer. The robot can run in simulation (fake motors) without any physical hardware.

## Build & Development Commands

**Install dependencies:**
```bash
./bin/update.sh        # runs: rosdep install --ignore-src --from-paths . -y -r
```

**Build:**
```bash
./bin/build.sh         # colcon build with Debug, compile_commands.json, symlink-install
```

**Run all tests:**
```bash
./bin/test.sh          # colcon test + test-result --all --verbose
```

**Run a single test package:**
```bash
colcon test --packages-select stepit_driver --return-code-on-test-failure
colcon test-result --all --verbose
```

**Run a specific test binary directly (after build):**
```bash
./build/stepit_driver/tests/test_default_driver
```

**Lint/format:**
```bash
pre-commit run -a                          # run all pre-commit hooks manually
clang-format-14 -i <file>                 # format a single C++ file
```

**Launch the simulation:**
```bash
source install/setup.bash
ros2 launch stepit_description robot.launch.py
```

## Architecture

The project is a ROS 2 workspace with two top-level directories:

- **`src/`** — ROS 2 packages (ament/colcon)
- **`modules/`** — git submodules (non-ROS C++ libraries)

### ROS 2 Packages (`src/`)

| Package | Role |
|---|---|
| `stepit_driver` | Core `ros2_control` hardware interface plugin (`StepitHardware`) |
| `stepit_description` | URDF/xacro robot model, RViz config, controllers config, launch files |
| `stepit_bringup` | Alternative bringup launch (hardware mode) |
| `stepit_teleop` | Teleoperation node |
| `stepit_hardware_tests` | Integration tests requiring real hardware |
| `cobs_serial` | COBS-encoded serial communication library (ROS 2 package) |
| `stepit_mcu` | PlatformIO project for Teensy firmware (not built by colcon) |

### Submodules (`modules/`)

- **`serial`** — Low-level cross-platform serial port C++ library (wjwwood/serial)

### Key Design Pattern: `stepit_driver`

`StepitHardware` (a `hardware_interface::SystemInterface` plugin) delegates all hardware communication to a `Driver` interface:

- **`DefaultDriver`** — real hardware, communicates via `CobsSerial` → COBS-encoded serial → Teensy
- **`FakeDriver`** — simulation, uses `FakeMotor` with `VelocityControl`/`PositionControl` internally
- **`DefaultDriverFactory` / `DriverFactory`** — the factory is injected at construction time; in tests a mock factory is used

The `StepitHardware` reads joint configuration (motor IDs, max velocity, acceleration) from the URDF `<ros2_control>` block via `HardwareInfo`.

### Communication Stack (real hardware)

```
StepitHardware → DefaultDriver → CobsSerial (cobs_serial pkg) → serial (submodule) → USB/serial → Teensy
```

COBS encoding uses zero bytes as packet delimiters. The protocol carries typed request/response messages defined in `stepit_driver/msgs/`.

### Testing Approach

Tests use **GMock** (`ament_add_gmock`). Mocks live alongside tests:
- `tests/mock/mock_cobs_serial.hpp` — mocks the serial layer for `test_default_driver`
- `tests/mock/mock_driver.hpp` / `mock_driver_factory.hpp` — mock the driver for `test_stepit_hardware`
- `tests/fake/fake_hardware_info.hpp` — constructs `HardwareInfo` for unit tests without a URDF

## Code Standards

- C++20, compiled with `-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast`
- Formatting: `clang-format-14` (pre-commit hook, style defined in `.clang-format` if present)
- Python formatting: `black`
- Spell check: `codespell` (pre-commit)
- Pre-commit hooks run automatically on commit; run `pre-commit run -a` to check all files manually

## CI

Three GitHub Actions workflows:
- `industrial_ci.yml` — builds and tests in Docker (Ubuntu 22.04 + ROS Humble) via ros-industrial/industrial_ci
- `ci-format.yml` — pre-commit hooks (clang-format-14, black, codespell)
- `ci-ros-lint.yml` — ROS-specific linters

Local CI can be run with [Nektos `act`](https://github.com/nektos/act).
