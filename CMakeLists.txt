cmake_minimum_required(VERSION 3.0)

project(ObtainRobotino4SensorData VERSION 1.0 DESCRIPTION "Read Robotino2 sensor data and get its location by webcam.")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()

add_subdirectory(src)