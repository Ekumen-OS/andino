cmake_minimum_required(VERSION 3.8)
project(andino_hardware)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    printing_model
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
