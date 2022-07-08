#pragma once

#include <stepit_hardware/request.h>
#include <stepit_hardware/response.h>

#include <serial/serial.h>

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class CommandInterface
{
public:
  explicit CommandInterface(serial::Serial* serial);
  Response send(const Request& request);

private:
  /**
   * Escape the given byte according to the PPP specification.
   * @param value The byte to escape.
   */
  static const std::vector<uint8_t> escape(uint8_t byte);

  static const std::vector<uint8_t> create_frame(const std::vector<uint8_t>& data);

  void read();

  uint8_t requestId = 0;

  serial::Serial* serial_ = nullptr;
};
}  // namespace stepit_hardware
