add_executable(ObtainSensorData main.cpp)

find_package(SharedMemory 1.2 REQUIRED)
find_package(Robotino2Wrapper 1.1 REQUIRED)

target_link_libraries(ObtainSensorData
                      SharedMemoryLib
                      robotino2wrapper)

string(REPLACE "\\" "/" OPENROBOTINOAPI_DIR "$ENV{OPENROBOTINOAPI_DIR}/1/lib/win32")

target_include_directories(ObtainSensorData PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           ${OPENROBOTINOAPI_DIR}/rec_core_lt.lib)

target_include_directories(ObtainSensorData PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                           $ENV{OPENROBOTINOAPI_DIR}/1/include
                           $ENV{OPENROBOTINOAPI_DIR}/share/include)

install(TARGETS ObtainSensorData
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)