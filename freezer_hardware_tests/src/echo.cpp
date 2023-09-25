// Copyright 2023 Giovanni Remigi
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

#include <chrono>
#include <iostream>
#include <limits>
#include <vector>
#include <thread>

#include <cobs_serial/data_utils.hpp>
#include <cobs_serial/default_serial.hpp>
#include <cobs_serial/default_cobs_serial.hpp>

#include "command_line_utility.hpp"
#include "common_utils.hpp"

constexpr auto kUsbPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 9600;
constexpr auto kTimeout = 1;
constexpr auto kConnectionTimeout = 5;
constexpr uint8_t kReadyMessageId = 0x80;

using cobs_serial::DefaultCobsSerial;
using cobs_serial::DefaultSerial;
using common_utils::safe_convert;

/**
 * Convert a string containing comma separated values of bits and delays
 * into a bitset command.
 * @param sequence A string containing comma separated values of bits and
 * delays.
 * @return A bitset command.
 */
std::vector<uint8_t> to_bytes(const std::string& sequence)
{
  std::vector<std::uint8_t> bytes;

  std::istringstream iss{ sequence };
  std::string token;

  while (std::getline(iss, token, ','))
  {
    token.erase(0, token.find_first_not_of(' '));  // Remove leading spaces.
    uint8_t byte = safe_convert<uint8_t>(token);
    bytes.push_back(byte);
  }

  return bytes;
}

/**
 * Send a sequence of bytes and expect the same sequence echoed back.
 * For example, we can send any of the following sequences:
 *
 * 123,255,128,231
 *
 * Usage:
 * echo --sequence 123,255,128,231
 */
int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kUsbPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  uint32_t baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = static_cast<uint32_t>(std::stoul(value)); }, false);

  double timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stoi(value); }, false);

  double connection_timeout = kConnectionTimeout;
  cli.registerHandler(
      "--connection-timeout", [&connection_timeout](const char* value) { connection_timeout = std::stoi(value); },
      false);

  std::string sequence = "";
  cli.registerHandler(
      "--sequence", [&sequence](const char* value) { sequence = value; }, true);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                    Set the com port (default " << kUsbPort << ")\n"
              << "  --baudrate VALUE                Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE                 Set the read/write timeout (default " << kTimeout << "s)\n"
              << "  --connection-timeout VALUE      Set the max connection timeout (default " << kConnectionTimeout
              << "s)\n"
              << "  --sequence VALUE                A string representing a sequence of bytes.\n"
              << "                                  Example: \"123,255,128,231\"\n"
              << "  -h                              Show this help message\n";
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

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeout: " << timeout << "s" << std::endl;
    std::cout << " - connection timeout: " << connection_timeout << "s" << std::endl;

    // Open a serial connection with Arduino.
    // On connection Arduino resets itself and moves into a bootstrap phase
    // that may last few seconds. When the following method returns true it
    // means the connection has been established, but it doesn't mean Arduino
    // is ready to communicate. When Arduino is ready it sends back a ready
    // message that causes a "requestDataReceived" signal to be emitted.

    std::cout << "Checking if the serial port is open..." << std::endl;
    cobs_serial->open();
    if (!cobs_serial->is_open())
    {
      std::cout << "The serial port is not open" << std::endl;
      return 1;
    }
    std::cout << "The serial port is open." << std::endl;
    std::cout << "Waiting for device..." << std::endl;

    bool connected = false;
    std::chrono::duration<double> max_connection_time(kConnectionTimeout);
    auto start_time = std::chrono::steady_clock::now();
    while (!connected && std::chrono::steady_clock::now() - start_time < max_connection_time)
    {
      try
      {
        if (auto response = cobs_serial->read(); response.size() == 1 && response[0] == kReadyMessageId)
        {
          connected = true;
        }
      }
      catch (const std::exception& e)
      {
        // Expected timeout.
      }
    }

    if (!connected)
    {
      std::cout << "Device not ready" << std::endl;
      return 1;
    }

    std::cout << "Executing an echo command..." << std::endl;

    auto bytes = to_bytes(sequence);

    std::vector<uint8_t> in{ 0x79 };  // Echo command.
    in.insert(in.end(), bytes.begin(), bytes.end());

    std::cout << "Sequence: " << cobs_serial::data_utils::to_hex(in) << std::endl;

    try
    {
      cobs_serial->write(in);
      auto out = cobs_serial->read();
      std::cout << "Echo: " << cobs_serial::data_utils::to_hex(out) << std::endl;
    }
    catch (const std::exception& e)
    {
      std::cout << e.what();
      return 1;
    }

    std::cout << "Echo command executed." << std::endl;
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the hardware:" << e.what();
    return 1;
  }
}
