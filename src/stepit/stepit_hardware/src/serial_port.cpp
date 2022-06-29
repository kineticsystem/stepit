
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>

#include "serial_port.hpp"

#define DELIMITER_FLAG 0x7E
#define ESCAPE_FLAG 0x7D
#define ESCAPE_XOR 0x20
#define CRC_INIT_VALUE 0xFFFF

using namespace org::kineticsystem::stepit::hardware;

SerialPort::SerialPort()
    : serial_port_(-1), rx_frame_length_(0), rx_frame_crc_(CRC_INIT_VALUE),
      rx_frame_escape_(false), tx_frame_length_(0),
      tx_frame_crc_(CRC_INIT_VALUE) {}

SerialPort::~SerialPort() { close(); }

return_type SerialPort::open(const std::string &port_name) {
  serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);

  if (serial_port_ < 0) {
    fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno),
            errno);
    return return_type::ERROR;
  }

  struct termios tty_config {};
  if (::tcgetattr(serial_port_, &tty_config) != 0) {
    fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n",
            strerror(errno), errno);
    close();
    return return_type::ERROR;
  }

  memset(&tty_config, 0, sizeof(termios));
  tty_config.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
  tty_config.c_cflag &=
      ~PARENB; // Clear parity bit, disabling parity (most common)
  tty_config.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in
                                 // communication (most common)
  tty_config.c_cflag &= ~CSIZE;  // Clear bits per byte
  tty_config.c_cflag |= CS8;     // 8 bit per byte
  tty_config.c_iflag = IGNPAR;
  tty_config.c_oflag = OPOST;
  tty_config.c_lflag = 0;
  tty_config.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning
                               // as soon as any data is received.
  tty_config.c_cc[VMIN] = 0;
  tcflush(serial_port_, TCIFLUSH);

  /*
  if (::cfsetispeed(&tty_config, B9600) != 0 || ::cfsetospeed(&tty_config,
  B9600) != 0) { fprintf(stderr, "Failed to set serial port speed: %s (%d)\n",
  strerror(errno), errno); close(); return return_type::ERROR;
  }
  */

  if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
    fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n",
            strerror(errno), errno);
    close();
    return return_type::ERROR;
  }

  return return_type::SUCCESS;
}

return_type SerialPort::close() {
  if (is_open()) {
    ::close(serial_port_);
    serial_port_ = -1;
  }
  return return_type::SUCCESS;
}

return_type SerialPort::read_frames(std::vector<SerialHdlcFrame> &frames) {
  // Read data from the serial port
  const ssize_t num_bytes = ::read(serial_port_, rx_buffer_, 256);
  if (num_bytes == -1) {
    fprintf(stderr, "Failed to read serial port data: %s (%d)\n",
            strerror(errno), errno);
    return return_type::ERROR;
  }

  for (ssize_t i = 0; i < num_bytes; i++) {
    decode_byte(rx_buffer_[i], frames);
  }

  return return_type::SUCCESS;
}

return_type SerialPort::write_frame(const uint8_t *data, size_t size) {
  if (!is_open()) {
    return return_type::ERROR;
  }

  // Generate the fame
  tx_frame_length_ = 0;
  tx_frame_crc_ = CRC_INIT_VALUE;
  tx_frame_buffer_[tx_frame_length_++] = DELIMITER_FLAG;
  for (size_t i = 0; i < size; i++) {
    tx_frame_crc_ = crc_update(tx_frame_crc_, data[i]);
    encode_byte(data[i]);
  }
  encode_byte((tx_frame_crc_ & 0xFF));
  encode_byte(((tx_frame_crc_ >> 8) & 0xFF));
  tx_frame_buffer_[tx_frame_length_++] = DELIMITER_FLAG;

  if (::write(serial_port_, tx_frame_buffer_, tx_frame_length_) == -1) {
    fprintf(stderr, "Failed to write serial port data: %s (%d)\n",
            strerror(errno), errno);
    return return_type::ERROR;
  }

  return return_type::SUCCESS;
}

bool SerialPort::is_open() const { return serial_port_ >= 0; }

void SerialPort::encode_byte(uint8_t data) {
  if (data == ESCAPE_FLAG || data == DELIMITER_FLAG) {
    tx_frame_buffer_[tx_frame_length_++] = ESCAPE_FLAG;
    data ^= ESCAPE_XOR;
  }
  tx_frame_buffer_[tx_frame_length_++] = data;
}

void SerialPort::decode_byte(uint8_t data,
                             std::vector<SerialHdlcFrame> &frames) {
  if (data == DELIMITER_FLAG) {
    if (rx_frame_escape_) {
      rx_frame_escape_ = false;
    } else if (rx_frame_length_ >= 2 &&
               rx_frame_crc_ == ((rx_frame_buffer_[rx_frame_length_ - 1] << 8) |
                                 rx_frame_buffer_[rx_frame_length_ - 2])) {
      SerialHdlcFrame frame;
      memcpy(frame.data, rx_frame_buffer_, rx_frame_length_ - 2);
      frame.length = rx_frame_length_ - 2;
      frames.push_back(frame);
    }
    rx_frame_length_ = 0;
    rx_frame_crc_ = CRC_INIT_VALUE;
    return;
  }

  if (data == ESCAPE_FLAG) {
    rx_frame_escape_ = true;
    return;
  }

  if (rx_frame_escape_) {
    rx_frame_escape_ = false;
    data ^= ESCAPE_XOR;
  }

  rx_frame_buffer_[rx_frame_length_] = data;
  if (rx_frame_length_ >= 2) {
    rx_frame_crc_ =
        crc_update(rx_frame_crc_, rx_frame_buffer_[rx_frame_length_ - 2]);
  }
  rx_frame_length_++;

  if (rx_frame_length_ == FRAME_MAX_SIZE) {
    rx_frame_length_ = 0;
    rx_frame_crc_ = CRC_INIT_VALUE;
  }
}

uint16_t SerialPort::crc_update(uint16_t crc, uint8_t data) {
  data ^= (uint8_t)(crc & 0xFF);
  data ^= (data << 4);
  return ((((uint16_t)data << 8) | ((uint8_t)(crc >> 8) & 0xFF)) ^
          (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
