// Copyright 2026 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cstddef>
// cpplint's list of C++ headers predates C++20, so it takes <numbers> for a C
// system header and wants it before <cstddef>.
#include <numbers>  // NOLINT(build/include_order)

#include <stepit_driver/msgs/info_response.hpp>

namespace stepit_driver::hardware_limits
{
/**
 * Limits of the StepIt controller. The firmware is the authority: these
 * mirror NUMBER_OF_MOTORS, MAX_ACCELERATION, MAX_SPEED and CONFIG_TOLERANCE
 * in src/stepit_mcu/src/main.cpp, converted from steps to radians.
 *
 * FakeDriver enforces them so that a configuration accepted in simulation is
 * one the real controller accepts too. Without this, a robot description that
 * works in simulation can fail to activate the moment it is run on hardware.
 */

// Number of motors the controller drives.
constexpr std::size_t kMotorCount = 5;

// Maximum acceleration: 2 rotations per square second.
constexpr double kMaxAcceleration = 4.0 * std::numbers::pi;

// Maximum velocity: 3 rotations per second.
constexpr double kMaxVelocity = 6.0 * std::numbers::pi;

// Tolerance applied when comparing against the limits above, so that a value
// stated as the limit itself is not rejected by rounding.
constexpr double kTolerance = 1.001;

// The firmware version the simulated controller reports. Its major number is
// the protocol version this workspace speaks: see VERSION_MAJOR in
// src/stepit_mcu/src/main.cpp.
const Version kFirmwareVersion{ 1, 0, 0 };

}  // namespace stepit_driver::hardware_limits
