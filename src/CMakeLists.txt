add_executable(ObtainSensorData main.cpp)

find_package(OpenCV REQUIRED)
find_package(SharedMemory 1.1 REQUIRED)
find_package(Robotino2 1.0 REQUIRED)

target_link_libraries(ObtainSensorData
                      SharedMemoryLib
                      Robotino2Lib
                      ${OpenCV_LIBS})

target_include_directories(ObtainSensorData PUBLIC
                      "${OpenCV_INCLUDE_DIRS}"
                      "${PROJECT_SOURCE_DIR}/include")

install(TARGETS ObtainSensorData
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/ObtainRobotino4SensorData" COMPONENT bin)