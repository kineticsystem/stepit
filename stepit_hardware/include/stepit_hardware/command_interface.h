#pragma once

#include <stepit_hardware/request.h>
#include <stepit_hardware/response.h>
#include <stepit_hardware/serial_interface.h>
#include <stepit_hardware/data_buffer.h>

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class CommandInterface
{
public:
  explicit CommandInterface(SerialInterface* serial);
  Response write(const Request& request);

private:
  /**
   * Escape the given byte according to the PPP specification.
   * @param value The byte to escape.
   */
  static const std::vector<uint8_t> escape(uint8_t byte);

  static const std::vector<uint8_t> create_frame(const std::vector<uint8_t>& data);

  void read();

  enum class State
  {
    StartReading,
    ReadingMessage,
    ReadingEscapedByte,
    MessageRead
  } state_ = State::StartReading;

  uint8_t requestId = 0;

  /* Circular buffer to read data from the serial port. */
  DataBuffer read_buffer_{ 100 };

  SerialInterface* serial_ = nullptr;
};
}  // namespace stepit_hardware
