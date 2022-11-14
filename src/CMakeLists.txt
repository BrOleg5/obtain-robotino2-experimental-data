add_executable(ObtainSensorData obtain_data.cpp obtain_data.hpp)

find_package(Robotino2 1.0 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(ArucoLocalization 1.4.0 REQUIRED)

target_link_libraries(ObtainSensorData PUBLIC
                      Robotino2Lib
                      arucolocalization
                      ${OpenCV_LIBS})

target_include_directories(ObtainSensorData PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           ${OpenCV_INCLUDE_DIRS})

install(TARGETS ObtainSensorData
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)