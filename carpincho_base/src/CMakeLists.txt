add_library(motor_driver SHARED motor_driver.cpp)

target_include_directories(motor_driver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Reference: See libserial example project for integration with CMake
target_include_directories(motor_driver PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(motor_driver PUBLIC ${SERIAL_LDFLAGS})


add_library(diffdrive_carpincho SHARED diffdrive_carpincho.cpp wheel.cpp)

target_include_directories(diffdrive_carpincho
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(diffdrive_carpincho
  PUBLIC
    hardware_interface
    pluginlib
    rclcpp
)

target_link_libraries(diffdrive_carpincho PUBLIC motor_driver)

install(
  TARGETS diffdrive_carpincho motor_driver
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(diffdrive_carpincho)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
