/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stepit_driver/default_driver.hpp>
#include <stepit_driver/default_driver_factory.hpp>

#include <cobs_serial/default_serial.hpp>
#include <cobs_serial/default_cobs_serial.hpp>

#include "command_line_utility.hpp"

#include <iostream>

// This is a command to read the status of the gripper.

using namespace stepit_driver;
using namespace cobs_serial;

constexpr auto kUsbPort = "/dev/ttyACM0";
constexpr auto kBaudRate = 9600;
constexpr auto kTimeout = 0.2;

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kUsbPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  double timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stoi(value); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
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
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout)));

    auto cobs_serial = std::make_unique<DefaultCobsSerial>(std::move(serial));

    auto driver = std::make_unique<DefaultDriver>(std::move(cobs_serial));

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeout: " << timeout << "s" << std::endl;

    std::cout << "Checking if the driver is connected..." << std::endl;
    bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The driver is not connected" << std::endl;
      return 1;
    }

    std::cout << "The driver is connected." << std::endl;
    std::cout << "Reading the driver status..." << std::endl;

    auto response = driver->get_status(rclcpp::Time{}, StatusQuery{});

    std::cout << "Status retrieved:" << std::endl;

    for (const MotorState& state : response.motor_states())
    {
      std::cout << " - motor id: " << state.id() << "rad" << std::endl;
      std::cout << " - motor position: " << state.position() << "rad" << std::endl;
      std::cout << " - motor velocity: " << state.velocity() << "rad/s" << std::endl;
      std::cout << " - distance to go: " << state.distance_to_go() << "rad" << std::endl;
    }
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the gripper:" << e.what();
    return 1;
  }
}
