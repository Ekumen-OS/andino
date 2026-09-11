add_executable(serial_mcu_demo serial_mcu_demo.cpp)

target_include_directories(serial_mcu_demo
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(serial_mcu_demo
  PUBLIC
    gflags
    serial_mcu
)

install(
  TARGETS
    serial_mcu_demo
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
