add_executable(motor_driver_demo motor_driver_demo.cpp)

target_include_directories(motor_driver_demo
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Reference: See libserial example project for integration with CMake
target_include_directories(motor_driver_demo PUBLIC ${SERIAL_INCLUDE_DIRS})

target_link_libraries(motor_driver_demo
  PUBLIC
    gflags
    motor_driver
    ${SERIAL_LDFLAGS}
)

install(
  TARGETS
    motor_driver_demo
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
