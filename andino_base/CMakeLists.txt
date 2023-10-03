cmake_minimum_required(VERSION 3.8)
project(andino_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# libserial:
# Following linking instructions from: https://github.com/crayzeewulf/libserial/blob/master/examples/example_project/CMakeLists.txt
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

# Include libserial for serial communication
# Typical find_package() does not work for libserial, for reference see:
# https://github.com/crayzeewulf/libserial/issues/113#issuecomment-432245159
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include(CMakePackageConfigHelpers)

add_subdirectory(applications)
add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

# Export library so pluginlib can find it.
pluginlib_export_plugin_description_file(hardware_interface andino_hw.xml)

ament_package()
