#ifndef OBTAIN_DATA_HPP
# define OBTAIN_DATA_HPP

#include "robotino2.hpp"
#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include "read_save_camera_parameters.hpp"
#include "cmdoptionparser.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

//link: https://learn.microsoft.com/en-us/windows/console/registering-a-control-handler-function?source=recommendations
#if  defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#   include <windows.h>
#   include <stdio.h>
std::atomic_bool isExit = false;
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        isExit = TRUE;
        return TRUE;
    default:
        return FALSE;
    }
}
#endif

inline float deg2rad(float deg){
    return deg * static_cast<float>(PI) / 180.0f;
}

inline std::chrono::milliseconds get_interval(std::chrono::steady_clock::time_point t1,
                                              std::chrono::steady_clock::time_point t2) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
}

class Timer {
    public:
        Timer(): start_point(std::chrono::steady_clock::now()) {}
        
        void reset() {
            start_point = std::chrono::steady_clock::now();
        }

        long long time() {
            std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_point);
            return t.count();
        }
    private:
        std::chrono::steady_clock::time_point start_point;
};

/**
 * Sign function.
 * 
 * @tparam T variable type.
 * @param val variable value.
 * @return signum of value.
 */
template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

enum MovementType {
    NONE,
    POLAR,
    SQUARE,
    CIRCLE,
    TRIANGLE
};

class RobotData {
    public:
        //Robot set point speed
        float vx;
        float vy;
        float omega;
        // Robot set speed point in polar coordinate system
        float speed; // in m/s
        float angle; // in degree
        MovementType move_type;

        std::array<float, 3> motor_speed;
        std::array<int, 3> actual_position;
        std::array<float, 3> actual_velocity;
        std::array<float, 3> actual_current;

        RobotData(): vx(0.0f), vy(0.0f), omega(0.0f), speed(0.0f), angle(0.0f),
                         motor_speed({0.0f, 0.0f, 0.0f}), actual_position({0, 0, 0}),
                         actual_velocity({0.0f, 0.0f, 0.0f}), actual_current({0.0f, 0.0f, 0.0f}),
                         move_type(MovementType::NONE) {};

        RobotData& operator=(const RobotData& other) {
            // Guard self assignment
            if(this == &other) {
                return *this;
            }

            this->vx = other.vx;
            this->vy = other.vy;
            this->omega = other.omega;
            this->speed = other.speed;
            this->angle = other.angle;
            this->motor_speed = other.motor_speed;
            this->actual_position = other.actual_position;
            this->actual_velocity = other.actual_velocity;
            this->actual_current = other.actual_current;
            this->move_type = other.move_type;
            return *this;
        }
};

const std::string csv_header = "Time;"
                               "Set robot speed along X axis;Set robot speed along Y axis;Set rotational robot velocity;"
                               "1st motor set speed;2st motor set speed;3st motor set speed;"
                               "1st motor position;2nd motor position;3rd motor position;"
                               "1st motor velocity;2nd motor velocity;3rd motor velocity;"
                               "1st motor current;2nd motor current;3rd motor current;"
                               "X axis global position;Y axis global position;Angle;"
                               "Delta X axis local position;Delta Y axis local position;Delta angle;\n"

                               "ms;"
                               "m/s;m/s;rad/s;"
                               "rad/s;rad/s;rad/s;"
                               "ticks;ticks;ticks;"
                               "rad/s;rad/s;rad/s;"
                               "A;A;A;"
                               "pix;pix;deg;"
                               "mm;mm;deg;\n"

                               "t;"
                               "xsetspeed;ysetspeed;setvel;"
                               "m1setvel;m2setvel;m3setvel;"
                               "m1pos;m2pos;m3pos;"
                               "m1vel;m2vel;m3vel;"
                               "m1cur;m2cur;m3cur;"
                               "xpos;ypos;ang;"
                               "dx;dy;dang;\n";

void read_robot_sensors(Robotino2& robotino, RobotData& robot_buffer, std::mutex& robot_buffer_mutex, Timer& timer,
                        std::mutex& timer_mutex, long long test_duration_ms) {
    robot_buffer_mutex.lock();
    RobotData robot_data = robot_buffer;
    robot_buffer_mutex.unlock();

    const float square_set_speed[4][2] = {
        {robot_data.speed, 0.0f},
        {0.0f, robot_data.speed},
        {-robot_data.speed, 0.0f},
        {0.0f, -robot_data.speed}
    };

    const float triangle_direction_deg[3] = { 30, 150, 270 };

    long long side_time = 0ll;
    long long side_count = 0ll;
    float start_x_speed = 0.0f;
    float start_y_speed = 0.0f;
    float rad = 0.0f;
    switch (robot_data.move_type) {
    case MovementType::SQUARE:
        side_time = test_duration_ms/4;
        break;
    case MovementType::TRIANGLE:
        side_time = test_duration_ms/3;
        break;
    case MovementType::CIRCLE:
        start_x_speed = std::sin(deg2rad(robot_data.angle)) * robot_data.speed;
        start_y_speed = std::cos(deg2rad(robot_data.angle)) * robot_data.speed;
        break;
    case MovementType::POLAR:
        robot_data.vx = robot_data.speed * std::cos(deg2rad(robot_data.angle));
        robot_data.vy = robot_data.speed * std::sin(deg2rad(robot_data.angle));
        break;

    default:
        break;
    }

    while (true) {
        switch (robot_data.move_type) {
        case MovementType::CIRCLE:
            timer_mutex.lock();
            rad = 2.0f * static_cast<float>(PI) * static_cast<float>(timer.time()) / 
                                                  static_cast<float>(test_duration_ms);
            timer_mutex.unlock();
            robot_data.vx = std::cos(rad) * start_x_speed - std::sinf(rad) * start_y_speed;
            robot_data.vy = std::sin(rad) * start_x_speed + std::cosf(rad) * start_y_speed;
            break;
        case MovementType::SQUARE:
            timer_mutex.lock();
            if(timer.time() >= (side_time * (side_count + 1))) {
                    side_count++;
            }
            timer_mutex.unlock();
            robot_data.vx = square_set_speed[side_count][0];
            robot_data.vy = square_set_speed[side_count][1];
            break;
        case MovementType::TRIANGLE:
            timer_mutex.lock();
            if(timer.time() >= (side_time * (side_count + 1))) {
                    side_count++;
            }
            timer_mutex.unlock();
            robot_data.vx = robot_data.speed * std::cos(deg2rad(triangle_direction_deg[side_count]));
            robot_data.vy = robot_data.speed * std::sin(deg2rad(triangle_direction_deg[side_count]));
            break;

        default:
            break;
        }

        try {
            robotino.set_robot_speed(robot_data.vx, robot_data.vy, robot_data.omega);
        }
        catch(const std::invalid_argument& e) {
            std::cout << "Method set_robot_speed() failed with error: " << e.what();
            isExit = true;
        }
        robot_data.motor_speed = robotino.robot_speed_to_motor_speeds(robot_data.vx, robot_data.vy, robot_data.omega);
        
        robot_data.actual_position = robotino.get_actual_positions();
        robot_data.actual_velocity = robotino.get_actual_velocities();
        robot_data.actual_current = robotino.get_actual_currents();

        robot_buffer_mutex.lock();
        robot_buffer = robot_data;
        robot_buffer_mutex.unlock();

        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        if(isExit) {
            break;
        }
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(35));
    }
    robotino.set_robot_speed(0, 0, 0);
    robotino.reset_motor_positions();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void read_camera_data(ArucoLocalization& cv_system, std::mutex& cv_system_mutex, int marker_id,
                      td::TransferData& camera_buffer, std::mutex& camera_buffer_mutex,
                      std::atomic_bool& camera_fault) {
    while (true) {
        cv_system_mutex.lock();
        int status = cv_system.detectMarkers();
        if (status == 0) {
            camera_buffer_mutex.lock();
            camera_fault = !cv_system.estimatePosition(&camera_buffer, marker_id);
            camera_buffer_mutex.unlock();
        }
        else {
            camera_fault = true;
        }
        cv_system_mutex.unlock();

        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        if(isExit) {
            break;
        }
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}  

#endif