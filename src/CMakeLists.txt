add_executable(ObtainSensorData main.cpp)

find_package(SharedMemory 1.2 REQUIRED)
find_package(Robotino2 1.0 REQUIRED)

target_link_libraries(ObtainSensorData
                      SharedMemoryLib
                      Robotino2Lib)

target_include_directories(ObtainSensorData PUBLIC
                      "${PROJECT_SOURCE_DIR}/include")

install(TARGETS ObtainSensorData
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)