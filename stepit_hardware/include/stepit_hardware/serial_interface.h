#pragma once

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class SerialInterface
{
public:
  virtual ~SerialInterface()
  {
  }

  /**
   * Gets the open status of the serial port.
   * @return Returns true if the port is open, false otherwise.
   */
  [[nodiscard]] virtual bool isOpen() const = 0;

  /** Closes the serial port. */
  virtual void close() = 0;

  /**
   * Read a given amount of bytes from the serial port into a give buffer.
   *
   * @param buffer A reference to a std::vector of uint8_t.
   * @param size A size_t defining how many bytes to be read.
   * @return A size_t representing the number of bytes read as a result of the
   *         call to read.
   * @throw serial::PortNotOpenedException
   * @throw serial::SerialException
   */
  virtual std::size_t read(std::vector<uint8_t>& buffer, size_t size = 1) = 0;

  /**
   * Write a sequence of bytes to the serial port.
   * @param  data A const reference containing the data to be written
   * to the serial port.
   * @param  A size_t representing the number of bytes actually written to
   * the serial port.
   * @throw serial::PortNotOpenedException
   * @throw serial::SerialException
   * @throw serial::IOException
   */
  virtual size_t write(const std::vector<uint8_t>& data) = 0;

  /**
   * Sets the serial port identifier.
   * @param port A const std::string reference containing the address of the
   * serial port, which would be something like "COM1" on Windows and
   * "/dev/ttyS0" on Linux.
   * @throw std::invalid_argument
   */
  virtual void setPort(const std::string& port) = 0;

  /**
   * Gets the serial port identifier.
   * @see SerialInterface::setPort
   * @throw std::invalid_argument
   */
  [[nodiscard]] virtual std::string getPort() const = 0;
};
}  // namespace stepit_hardware
