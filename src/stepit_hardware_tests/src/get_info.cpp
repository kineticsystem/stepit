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

#include <cmath>
#include <iostream>

#include <stepit_driver/default_driver.hpp>
#include <stepit_driver/hardware_limits.hpp>

#include <cobs_serial/default_serial.hpp>
#include <cobs_serial/default_cobs_serial.hpp>

#include "command_line_utility.hpp"

constexpr auto kUsbPort = "/dev/ttyACM0";
constexpr auto kBaudRate = 9600;
constexpr auto kTimeout = 0.2;

using cobs_serial::DefaultCobsSerial;
using cobs_serial::DefaultSerial;
using stepit_driver::DefaultDriver;
using stepit_driver::MotorLimits;

namespace limits = stepit_driver::hardware_limits;

/**
 * Command-line utility that asks a connected controller for its name and the
 * motion limits of its motors, and checks them against hardware_limits.hpp.
 *
 * The firmware is the authority on those limits, but the simulated driver
 * cannot ask it: FakeDriver answers from the header instead. The two are
 * therefore kept in step by hand, and this utility is what makes that
 * verifiable. Run it against the real board after changing MAX_ACCELERATION
 * or MAX_SPEED in src/stepit_mcu/src/main.cpp.
 *
 * Exits with a non-zero status when they disagree.
 */
int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kUsbPort;
  cli.registerHandler("--port", [&port](const char* value) { port = value; }, false);

  uint32_t baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = static_cast<uint32_t>(std::stoul(value)); }, false);

  double timeout = kTimeout;
  cli.registerHandler("--timeout", [&timeout](const char* value) { timeout = std::stoi(value); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./get_info [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                 Set the com port (default " << kUsbPort << ")\n"
              << "  --baudrate VALUE             Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE              Set the read/write timeout (default " << kTimeout << "s)\n"
              << "  -h                           Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  try
  {
    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(std::chrono::duration<double>{ timeout });

    auto cobs_serial = std::make_unique<DefaultCobsSerial>(std::move(serial));
    auto driver = std::make_unique<DefaultDriver>(std::move(cobs_serial));

    std::cout << "Connecting to " << port << "..." << std::endl;
    if (!driver->connect())
    {
      std::cout << "The driver is not connected." << std::endl;
      return 1;
    }

    const auto response = driver->get_info(rclcpp::Time{});
    std::cout << "Controller: " << response.info() << ", firmware " << response.version().to_string() << std::endl;

    // A tolerance is needed because the controller holds its limits in steps
    // and converts them to radians in single precision.
    constexpr double kTolerance = 1e-4;
    bool matches = true;

    if (response.version().major() != limits::kFirmwareVersion.major())
    {
      std::cout << "MISMATCH: the controller speaks protocol version " << static_cast<int>(response.version().major())
                << ", this workspace speaks " << static_cast<int>(limits::kFirmwareVersion.major()) << "." << std::endl;
      matches = false;
    }

    if (response.limits().size() != limits::kMotorCount)
    {
      std::cout << "MISMATCH: the controller drives " << response.limits().size() << " motors, hardware_limits.hpp "
                << "declares " << limits::kMotorCount << "." << std::endl;
      matches = false;
    }

    for (const MotorLimits& limit : response.limits())
    {
      std::cout << " - motor " << static_cast<int>(limit.id()) << ": max acceleration " << limit.max_acceleration()
                << "rad/s^2" << ", max velocity " << limit.max_velocity() << "rad/s" << std::endl;

      if (std::abs(limit.max_acceleration() - limits::kMaxAcceleration) > kTolerance)
      {
        std::cout << "   MISMATCH: hardware_limits.hpp declares " << limits::kMaxAcceleration << "rad/s^2."
                  << std::endl;
        matches = false;
      }
      if (std::abs(limit.max_velocity() - limits::kMaxVelocity) > kTolerance)
      {
        std::cout << "   MISMATCH: hardware_limits.hpp declares " << limits::kMaxVelocity << "rad/s." << std::endl;
        matches = false;
      }
    }

    if (!matches)
    {
      std::cout << "\nThe firmware and hardware_limits.hpp disagree. Simulation is modelling a robot that does "
                << "not match the hardware." << std::endl;
      return 1;
    }

    std::cout << "\nThe firmware agrees with hardware_limits.hpp." << std::endl;
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the hardware:" << e.what();
    return 1;
  }

  return 0;
}
