#pragma once

#include <stepit_hardware/serial_interface.h>
#include <stepit_hardware/buffer.h>

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class DataInterface
{
public:
  explicit DataInterface(SerialInterface* serial);
  void write(const std::vector<uint8_t>& bytes);
  std::vector<uint8_t> read();

private:
  /**
   * Escape the given byte according to the PPP specification.
   * @param value The byte to escape.
   */
  static const std::vector<uint8_t> escape(uint8_t byte);

  static const std::vector<uint8_t> create_frame(const std::vector<uint8_t>& data);

  enum class State
  {
    StartReading,
    ReadingMessage,
    ReadingEscapedByte,
    MessageRead
  } state_ = State::StartReading;

  uint8_t requestId = 0;

  /* Circular buffer to read data from the serial port. */
  Buffer<uint8_t> read_buffer_{ 100 };

  SerialInterface* serial_ = nullptr;
};
}  // namespace stepit_hardware
