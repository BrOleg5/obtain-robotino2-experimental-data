cmake_minimum_required(VERSION 3.0)

project(ObtainRobotino4SensorData VERSION 1.1 DESCRIPTION "Read Robotino2 sensor data and get its location by webcam.")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src)