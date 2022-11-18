# Obtaining experimental data from robot's sensors

Program obtain experimental data from robot's sensors and write data to csv file.

## Requirements

- CMake v3.0 or later
- [Robotino2Wrapper](https://github.com/BrOleg5/robotino2-wrapper) v1.1 of later
- [SharedMemory](https://github.com/BrOleg5/boost-shared-memory-wrapper) v1.3 or later
- [ArucoLocalization](https://github.com/BrOleg5/mobile-robot-localization) v2.0 or later
- MSVC v19 or later

## Configure, build and install project

### Windows 10 x86

```bash
# Create build directory
mkdir build

# Configure
cmake -S obtain-robot-experimental-data/ -B build/ -G "Visual Studio 15"

# Build
cmake --build build/

# Install
sudo cmake --install build/
```

## Usage program

```bash
Obtain data from Robotino 2 and external camera.
Usage: ObtainSensorData.exe [params]

        -?, -h, --help, --usage (value:true)
                Print help message
        --ang (value:0)
                Direction of speed in degree
        --ce
                Camera exposure
        --ci
                Camera id
        --circle
                Circular movement
        --cp
                JSON file with camera parameters
        -d
                dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20
        --id
                Marker id
        --ip (value:172.26.1.0)
                Robotino IP address
        --of (value:file42)
                CSV file path without extention
        --omega (value:0)
                Rotational velocity in rad/s
        --ov (value:<none>)
                Output video
        --shm
                Use shared memory to transmit data to other programs
        --square
                Moving in a square
        --st (value:20)
                Sample time
        -t
                Program execution time in ms. If it equals 0, application run until user stop.
        --triang
                Triangular movement
        -v (value:0)
                Amplitude of speed in m/s
        --vx (value:0)
                Speed along X axis in m/s
        --vy (value:0)
                Speed along X axis in m/s
```
