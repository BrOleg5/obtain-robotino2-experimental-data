# Obtaining experimental data from robot's sensors

Program obtain experimental data from robot's sensors and write data to csv file.
Optionally you can get data from [computer vision system](https://github.com/BrOleg5/mobile-robot-localization) by [boost's shared memory](https://github.com/BrOleg5/boost-shared-memory-wrapper).

## Requirements

- CMake v3.0 or later
- [Robotino2 wrapper](https://github.com/BrOleg5/robotino4-lib) v1.0 of later
- [Boost shared memory wrapper](https://github.com/BrOleg5/boost-shared-memory-wrapper) v1.2 or later
- MSVC v19 or later

## Configure, build and install project

### Windows 10 x86

```
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

```
usage: ObtainSensorData [options]

Options:
  --help         Display this information.
  -f <path_to_file>Set <path_to_file>.csv.
  -ip <address>  Set Robotino4 IP-<address>.
  -t <duration>  Set <duration> of program execution in ms.
  -dt <sample_time>Set measure <sample_time> in ms.
  -vx <speed_along_x>Set Robotino4 <speed_along_x> in m/s.
  -vy <speed_along_y>Set Robotino4 <speed_along_y> in m/s.
  -omega <rotation_velocity>Set Robotino4 <rotation_velocity> in rad/s.
  -shared-memory Use shared memory from boost to transfer measurements.
```