#include "serial/serial.h"
#include <cstdlib>
#include <iostream>
#include <vector>

void enumerate_ports() {
  std::vector<serial::PortInfo> devices_found = serial::list_ports();

  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

int main(int argc, char *argv[]) {

  enumerate_ports();
  std::cout << "Hello worldXXX!" << std::endl;

  return EXIT_SUCCESS;
}
