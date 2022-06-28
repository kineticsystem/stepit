#ifndef SERIAL_PORT_H

#include <string>
#include <vector>

#define SERIAL_BUFFER_MAX_SIZE 200
#define FRAME_MAX_SIZE 100

namespace org {
namespace kineticsystem {
namespace stepit {
namespace hardware {
enum class return_type : std::uint8_t { SUCCESS = 0, ERROR = 1 };

struct SerialHdlcFrame {
  uint8_t data[FRAME_MAX_SIZE];
  size_t length;
};

class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  return_type open(const std::string &port_name);
  return_type close();
  return_type read_frames(std::vector<SerialHdlcFrame> &frames);
  return_type write_frame(const uint8_t *data, size_t size);
  bool is_open() const;

protected:
  void encode_byte(uint8_t data);
  void decode_byte(uint8_t data, std::vector<SerialHdlcFrame> &frames);
  uint16_t crc_update(uint16_t crc, uint8_t data);

private:
  int serial_port_;
  uint8_t rx_buffer_[SERIAL_BUFFER_MAX_SIZE];
  uint8_t rx_frame_buffer_[FRAME_MAX_SIZE];
  size_t rx_frame_length_;
  uint16_t rx_frame_crc_;
  bool rx_frame_escape_;
  uint8_t tx_frame_buffer_[FRAME_MAX_SIZE];
  size_t tx_frame_length_;
  uint16_t tx_frame_crc_;
};
} // namespace hardware
} // namespace stepit
} // namespace kineticsystem
} // namespace org

#endif // SERIAL_PORT_H
