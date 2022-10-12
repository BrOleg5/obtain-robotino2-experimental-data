#include "robotino2.hpp"
#include "cmdoptionparser.hpp"
#include "sharedmemory.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>

/**
 * Summing float elements of array
 * 
 * @param array array whose elements should be summarized.
 * @param n array size.
 * @return sum of array elements.
 */
float arr_sum(float* array, size_t n);

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

int main( int argc, char **argv ) {

    // IP-addres Robotino 4
    std::string ip_addr = "172.26.1.0";
    // Path to csv file
    std::string file_path = "file42";
    // Test duration
    unsigned int test_duration = 0;
    // Measure sample time
    unsigned int measure_pause = 0;
    //Robot set point speed
    float vx = 0;
    float vy = 0;
    float omega = 0;
    // Robot set speed point in polar coordinate system
    bool is_polar = false;
    float speed = 0; // in m/s
    float angle = 0; // in degree
    // Motors set point velocities
    float m1_vel = 0;
    float m2_vel = 0;
    float m3_vel = 0;
    bool is_motor_sets = false;
    // Shared memory camera flag
    bool shm_cam_flag = false;
    // Shared memory IMU flag
    bool shm_imu_flag = false;

    //Process command line options
    if(argc > 1) {
        std::streamsize st_size = 25;
        //Help option
        if (cmdOptionExists(argv, argv+argc, "--help")){
            std::cout << "usage: ObtainSensorData [options]\n\nOptions:\n"
                      << std::left << "  " << std::setw(st_size) << "--help" << "Display this information.\n"
                      << "  " << std::setw(st_size) << "-f <path_to_file>" << "Set <path_to_file>.csv.\n"
                      << "  " << std::setw(st_size) << "-ip <address>" << "Set Robotino4 IP-<address>.\n"
                      << "  " << std::setw(st_size) << "-t <duration>" << "Set <duration> of program execution in ms.\n"
                      << "  " << std::setw(st_size) << "-dt <sample_time>" << "Set measure <sample_time> in ms.\n"
                      << "  " << std::setw(st_size) << "-vx <speed_along_x>" << "Set Robotino2 <speed_along_x> in m/s.\n"
                      << "  " << std::setw(st_size) << "-vy <speed_along_y>" << "Set Robotino2 <speed_along_y> in m/s.\n"
                      << "  " << std::setw(st_size) << "-omega <rotation_velocity>" << "Set Robotino2 <rotation_velocity> in rad/s.\n"
                      << "  " << std::setw(st_size) << "-v <speed>" << "Set Robotino2 <speed> in m/s in polar coordinate system.\n"
                      << "  " << std::setw(st_size) << "-ang <angle>" << "Set Robotino2 <angle> of speed vector in degree in polar coordinate system.\n"
                      << "  " << std::setw(st_size) << "-v1 <velocity>" << "Set Robotino2 motor 1 <velocity> in rad/s.\n"
                      << "  " << std::setw(st_size) << "-v2 <velocity>" << "Set Robotino2 motor 2 <velocity> in rad/s.\n"
                      << "  " << std::setw(st_size) << "-v3 <velocity>" << "Set Robotino2 motor 3 <velocity> in rad/s.\n"
                      << "  " << std::setw(st_size) << "-shared-memory-cam" << "Use shared memory from boost to transfer measurements from camera.\n"
                      << "  " << std::setw(st_size) << "-shared-memory-imu" << "Use shared memory from boost to transfer measurements from IMU.\n\n";
            exit(0);
        }
        //Set path to file option
        if (cmdOptionExists(argv, argv+argc, "-f")){
            file_path = getCmdOption(argv, argv+argc, "-f");
        }
        //Set robot ip option
        if (cmdOptionExists(argv, argv+argc, "-ip")){
            ip_addr = getCmdOption(argv, argv+argc, "-ip");    
        }
        //Set test duration option
        if (cmdOptionExists(argv, argv+argc, "-t")){
            test_duration = atoi(getCmdOption(argv, argv+argc, "-t"));
        }
        //Set X axis robot speed
        if (cmdOptionExists(argv, argv+argc, "-vx")){
            vx = (float)atof(getCmdOption(argv, argv+argc, "-vx"));
        }
        //Set Y axis robot speed
        if (cmdOptionExists(argv, argv+argc, "-vy")){
            vy = (float)atof(getCmdOption(argv, argv+argc, "-vy"));
        }
        //Set rotational robot speed
        if (cmdOptionExists(argv, argv+argc, "-omega")){
            omega = (float)atof(getCmdOption(argv, argv+argc, "-omega"));
        }
        //Set robot speed in polar coordinate system
        if (cmdOptionExists(argv, argv+argc, "-v")){
            speed = (float)atof(getCmdOption(argv, argv+argc, "-v"));
            is_polar = true;
        }
        //Set angle of robot speed in polar coordinate system
        if (cmdOptionExists(argv, argv+argc, "-ang")){
            angle = (float)atof(getCmdOption(argv, argv+argc, "-ang"));
            is_polar = true;
        }
        //Set motor 1 speed
        if (cmdOptionExists(argv, argv+argc, "-v1")){
            m1_vel = (float)atof(getCmdOption(argv, argv+argc, "-v1"));
            is_motor_sets = true;
        }
        //Set motor 2 speed
        if (cmdOptionExists(argv, argv+argc, "-v2")){
            m2_vel = (float)atof(getCmdOption(argv, argv+argc, "-v2"));
            is_motor_sets = true;
        }
        //Set motor 3 speed
        if (cmdOptionExists(argv, argv+argc, "-v3")){
            m3_vel = (float)atof(getCmdOption(argv, argv+argc, "-v3"));
            is_motor_sets = true;
        }
        //Set rotational robot speed
        if (cmdOptionExists(argv, argv+argc, "-dt")){
            measure_pause = atoi(getCmdOption(argv, argv+argc, "-dt"));
        }
        // Shared memory camera flag
        shm_cam_flag = cmdOptionExists(argv, argv+argc, "-shared-memory-cam");
        // Shared memory IMU flag
        shm_imu_flag = cmdOptionExists(argv, argv+argc, "-shared-memory-imu");
    }

    Robotino2 robotino(ip_addr);
    rec::core_lt::Timer timer;

    std::string csv_header;
    if (shm_cam_flag){
        if (shm_imu_flag){
            csv_header = "Time;"
                         "Set robot speed along X axis;Set robot speed along Y axis;Set rotational robot velocity;"
                         "1st motor set speed;2st motor set speed;3st motor set speed;"
                         "1st motor position;2nd motor position;3rd motor position;"
                         "1st motor velocity;2nd motor velocity;3rd motor velocity;"
                         "1st motor current;2nd motor current;3rd motor current;"
                         "X axis current;Y axis curent;Rotational current;"
                         "X axis global position;Y axis global position;Angle;"
                         "Delta X axis local position;Delta Y axis local position;Delta angle;"
                         "Gyro X;Gyro Y;Gyro Z;"
                         "Accel X;Accel Y;Accel Z;\n"

                         "ms;"
                         "m/s;m/s;rad/s;"
                         "rad/s;rad/s;rad/s;"
                         "ticks;ticks;ticks;"
                         "rad/s;rad/s;rad/s;"
                         "A;A;A;"
                         "A;A;A;"
                         "pix;pix;deg;"
                         "mm;mm;deg;"
                         "non;non;non;"
                         "non;non;non;\n"
                         
                         "t;"
                         "xsetspeed;ysetspeed;setvel;"
                         "m1setvel;m2setvel;m3setvel;"
                         "m1pos;m2pos;m3pos;"
                         "m1vel;m2vel;m3vel;"
                         "m1cur;m2cur;m3cur;"
                         "xcur;ycur;rotcur;"
                         "xpos;ypos;ang;"
                         "dx;dy;dang;"
                         "gyrox;gyroy;gyroz;"
                         "accelx;accely;accelz;\n";
        }
        else {
            csv_header = "Time;"
                         "Set robot speed along X axis;Set robot speed along Y axis;Set rotational robot velocity;"
                         "1st motor set speed;2st motor set speed;3st motor set speed;"
                         "1st motor position;2nd motor position;3rd motor position;"
                         "1st motor velocity;2nd motor velocity;3rd motor velocity;"
                         "1st motor current;2nd motor current;3rd motor current;"
                         "X axis current;Y axis curent;Rotational current;"
                         "X axis global position;Y axis global position;Angle;"
                         "Delta X axis local position;Delta Y axis local position;Delta angle;\n"

                         "ms;"
                         "m/s;m/s;rad/s;"
                         "rad/s;rad/s;rad/s;"
                         "ticks;ticks;ticks;"
                         "rad/s;rad/s;rad/s;"
                         "A;A;A;"
                         "A;A;A;"
                         "pix;pix;deg;"
                         "mm;mm;deg;\n"
                         
                         "t;"
                         "xsetspeed;ysetspeed;setvel;"
                         "m1setvel;m2setvel;m3setvel;"
                         "m1pos;m2pos;m3pos;"
                         "m1vel;m2vel;m3vel;"
                         "m1cur;m2cur;m3cur;"
                         "xcur;ycur;rotcur;"
                         "xpos;ypos;ang;"
                         "dx;dy;dang;\n";
        }
    }
    else {
        if (shm_imu_flag) {
            csv_header = "Time;"
                         "Set robot speed along X axis;Set robot speed along Y axis;Set rotational robot velocity;"
                         "1st motor set speed;2st motor set speed;3st motor set speed;"
                         "1st motor position;2nd motor position;3rd motor position;"
                         "1st motor velocity;2nd motor velocity;3rd motor velocity;"
                         "1st motor current;2nd motor current;3rd motor current;"
                         "X axis current;Y axis curent;Rotational current;"
                         "Gyro X;Gyro Y;Gyro Z;"
                         "Accel X;Accel Y;Accel Z;\n"

                         "ms;"
                         "m/s;m/s;rad/s;"
                         "rad/s;rad/s;rad/s;"
                         "ticks;ticks;ticks;"
                         "rad/s;rad/s;rad/s;"
                         "A;A;A;"
                         "A;A;A;"
                         "non;non;non;"
                         "non;non;non;\n"
                         
                         "t;"
                         "xsetspeed;ysetspeed;setvel;"
                         "m1setvel;m2setvel;m3setvel;"
                         "m1pos;m2pos;m3pos;"
                         "m1vel;m2vel;m3vel;"
                         "m1cur;m2cur;m3cur;"
                         "xcur;ycur;rotcur;"
                         "gyrox;gyroy;gyroz;"
                         "accelx;accely;accelz;\n";;
        }
        else {
            csv_header = "Time;"
                         "Set robot speed along X axis;Set robot speed along Y axis;Set rotational robot velocity;"
                         "1st motor set speed;2st motor set speed;3st motor set speed;"
                         "1st motor position;2nd motor position;3rd motor position;"
                         "1st motor velocity;2nd motor velocity;3rd motor velocity;"
                         "1st motor current;2nd motor current;3rd motor current;"
                         "X axis current;Y axis curent;Rotational current;\n"

                         "ms;"
                         "m/s;m/s;rad/s;"
                         "rad/s;rad/s;rad/s;"
                         "ticks;ticks;ticks;"
                         "rad/s;rad/s;rad/s;"
                         "A;A;A;"
                         "A;A;A;\n"
                         
                         "t;"
                         "xsetspeed;ysetspeed;setvel;"
                         "m1setvel;m2setvel;m3setvel;"
                         "m1pos;m2pos;m3pos;"
                         "m1vel;m2vel;m3vel;"
                         "m1cur;m2cur;m3cur;"
                         "xcur;ycur;rotcur;\n";
        }
    }

    shm::Receiver<double> receiver_cam;
    shm::Receiver<float> receiver_imu;

    if(shm_cam_flag) {
        receiver_cam = shm::Receiver<double> ("CameraData");
    }

    if(shm_imu_flag) {
        receiver_imu = shm::Receiver<float> ("IMUData");
    }

    // Open or create csv file 
    std::ofstream csv;
    std::string file_name = file_path + ".csv";
    csv.open(file_name, std::ios::out | std::ios::trunc);
	csv << csv_header;

    float actual_current[3];
    int velocity_sign[3];
    float x_current = 0;
    float y_current = 0;
    float rot_current = 0;

    double prev_cam_frame = 0;

    std::vector<float> motor_speed = {0, 0, 0};
    
    timer.start();
    while (timer.msecsElapsed() <= test_duration) {
        if (is_polar){
            vx = speed * std::cos(angle * (float)PI / 180);
            vy = speed * std::sin(angle * (float)PI / 180);
        }
        if (is_motor_sets){
            motor_speed[0] = m1_vel;
            motor_speed[1] = m2_vel;
            motor_speed[2] = m3_vel;
            try {
                robotino.set_motors_speed(motor_speed);
            }
            catch(const std::invalid_argument& e) {
                std::cout << "Method set_motors_speed() failed with error: " << e.what();
                return 1;
            }
        }
        else {
            try {
                robotino.set_robot_speed(vx, vy, omega);
            }
            catch(const std::invalid_argument& e) {
                std::cout << "Method set_robot_speed() failed with error: " << e.what();
                return 1;
            }
        }

        csv << timer.msecsElapsed() << ";";
        std::cout << timer.msecsElapsed() << "\r";

        csv << vx << ";" << vy << ";" << omega << ";";

        if(!is_motor_sets){
            motor_speed = robotino.robot_speed_to_motor_speeds(vx, vy, omega);
        }
        
        for (size_t i = 0; i < 3; i++)
        {
            csv << motor_speed[i] << ";";
        }
        
        for (size_t i = 0; i < 3; i++)
        {
            csv << robotino.get_actual_position(i) << ";";
        }
        
        float buf = 0;
        for (size_t i = 0; i < 3; i++)
        {
            buf = robotino.get_actual_velocity(i);
            csv << buf << ";";
            velocity_sign[i] = sgn(buf);
        }
        
        for (size_t i = 0; i < 3; i++)
        {
            buf = robotino.get_actual_current(i);
            csv << buf << ";";
            actual_current[i] = buf;
        }
        
        float signed_current[3] = [0];
        for(size_t i = 0; i < 3; i++) {
            signed_current[i] = actual_current[i] * velocity_sign[i];
        }
        x_current = -0.577f * signed_current[0] + 0.577f * signed_current[2];
        y_current = 0.333f * signed_current[0] - 0.667f * signed_current[1] + 
                    0.333f * signed_current[2];
        rot_current = arr_sum(signed_current, 3) / 3;
        csv << x_current << ";" << y_current << ";" << rot_current << ";";
        
        if (shm_cam_flag){
            if (receiver_cam.data->at(0) != prev_cam_frame) {
                // std::cout << "|" << std::setw(15) << receiver_cam.data->at(0);
                for (size_t i = 1; i < 7; i++)
                {
                    csv << receiver_cam.data->at(i) << ";";
                    // std::cout << "|" << std::setw(15) << receiver_cam.data->at(i);
                }
                // std::cout << "|\n";
                prev_cam_frame = receiver_cam.data->at(0);
            }
            else {
                for (size_t i = 1; i < 4; i++)
                {
                    csv << receiver_cam.data->at(i) << ";";
                }
                for (size_t i = 0; i < 3; i++)
                {
                    csv << 0 << ";";
                }
            }
        }
        if (shm_imu_flag) {
            // std::cout << "|" << std::setw(15) << receiver_imu.data->at(0);
            for (size_t i = 1; i < 7; i++)
            {
                csv << receiver_imu.data->at(i) << ";";
                // std::cout << "|" << std::setw(15) << receiver_imu.data->at(i);
            }
            // std::cout << "|\n";
        }
        csv << "\n";
        rec::core_lt::msleep(measure_pause);
    }
    robotino.set_robot_speed(0, 0, 0);
    robotino.reset_motors_position();
    rec::core_lt::msleep(1000);
    timer.reset();
    csv.close();
    return 0;
}

float arr_sum(float* array, size_t n){
    float s = 0;
    for (size_t i = 0; i < n; i++)
    {
        s += array[i];
    }
    return s;
}