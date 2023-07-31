cmake_minimum_required(VERSION 3.8)
project(andino_firmware)

find_package(ament_cmake REQUIRED)

install(
  FILES
    andino_firmware.ino
    platformio.ini
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
