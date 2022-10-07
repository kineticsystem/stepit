#include "serial/serial.h"

#include <stepit_hardware/data_utils.hpp>
#include <stepit_hardware/serial_interface.hpp>

#include <memory>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <array>

void enumerate_ports()
{
  std::vector<serial::PortInfo> devices_found = serial::list_ports();
  auto iter = devices_found.begin();
  while (iter != devices_found.end())
  {
    serial::PortInfo device = *iter++;
    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
{
  stepit_hardware::SerialInterface serial;

  enumerate_ports();

  serial::Serial my_serial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
  if (my_serial.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  //  const std::vector<uint8_t> x = { 1, 2, 3 };
  // my_serial.write(x);

  //  return EXIT_SUCCESS;
}
