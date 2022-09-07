cmake_minimum_required(VERSION VERSION 3.22.1)
project(stepit_simulation_ignition VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
