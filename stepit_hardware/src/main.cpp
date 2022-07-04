#include "serial/serial.h"
#include <cstdlib>
#include <iostream>
#include <vector>

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
  enumerate_ports();

  serial::Serial my_serial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
  if (my_serial.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  return EXIT_SUCCESS;
}
