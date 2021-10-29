#include "robotino2.hpp"
#include "cmdoptionparser.hpp"
#include "sharedmemory.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

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
    // Shared memory flag
    bool shm_flag = false;

    //Process command line options
    if(argc > 1) {
        //Help option
        if (cmdOptionExists(argv, argv+argc, "--help")){
            std::cout << "usage: ObtainSensorData [options]\n\nOptions:\n"
                      << std::left << "  " << std::setw(15) << "--help" << "Display this information.\n"
                      << "  " << std::setw(15) << "-f <path_to_file>" << "Set <path_to_file>.csv.\n"
                      << "  " << std::setw(15) << "-ip <address>" << "Set Robotino4 IP-<address>.\n"
                      << "  " << std::setw(15) << "-t <duration>" << "Set <duration> of program execution in ms.\n"
                      << "  " << std::setw(15) << "-dt <sample_time>" << "Set measure <sample_time> in ms.\n"
                      << "  " << std::setw(15) << "-vx <speed_along_x>" << "Set Robotino4 <speed_along_x> in m/s.\n"
                      << "  " << std::setw(15) << "-vy <speed_along_y>" << "Set Robotino4 <speed_along_y> in m/s.\n"
                      << "  " << std::setw(15) << "-omega <rotation_velocity>" << "Set Robotino4 <rotation_velocity> in rad/s.\n"
                      << "  " << std::setw(15) << "-shared-memory" << "Use shared memory from boost to transfer measurements.\n\n";
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
            vx = atof(getCmdOption(argv, argv+argc, "-vx"));
        }
        //Set Y axis robot speed
        if (cmdOptionExists(argv, argv+argc, "-vy")){
            vy = atof(getCmdOption(argv, argv+argc, "-vy"));
        }
        //Set rotational robot speed
        if (cmdOptionExists(argv, argv+argc, "-omega")){
            omega = atof(getCmdOption(argv, argv+argc, "-omega"));
        }
        //Set rotational robot speed
        if (cmdOptionExists(argv, argv+argc, "-dt")){
            measure_pause = atoi(getCmdOption(argv, argv+argc, "-dt"));
        }
        // Shared memory flag
        shm_flag = cmdOptionExists(argv, argv+argc, "-shared-memory");
    }

    Robotino2 robotino(ip_addr);
    rec::core_lt::Timer timer;

    std::string csv_header;
    if (shm_flag){
        csv_header = "Time;1st motor position;2nd motor position;3rd motor position;1st motor velocity;"
                     "2nd motor velocity;3rd motor velocity;1st motor current;2nd motor current;3rd motor current;"
                     "X axis current;Y axis curent;Rotational current;X axis global position;Y axis global position;Angle;"
                     "Delta X axis local position;Delta Y axis local position;Delta angle;\n"
                     "ms;ticks;ticks;ticks;rad/s;rad/s;rad/s;A;A;A;A;A;A;pix;pix;deg;mm;mm;deg;\n"
                     "t;m1pos;m2pos;m3pos;m1vel;m2vel;m3vel;m1cur;m2cur;m3cur;xcur;ycur;rotcur;xpos;ypos;ang;dx;dy;dang;\n";
    }
    else {
        csv_header = "Time;1st motor position;2nd motor position;3rd motor position;1st motor velocity;"
                     "2nd motor velocity;3rd motor velocity;1st motor current;2nd motor current;3rd motor current;"
                     "X axis current;Y axis curent;Rotational current;\n"
                     "ms;ticks;ticks;ticks;rad/s;rad/s;rad/s;A;A;A;A;A;A;\n"
                     "t;m1pos;m2pos;m3pos;m1vel;m2vel;m3vel;m1cur;m2cur;m3cur;xcur;ycur;rotcur;\n";
    }

    shm::Receiver<double> receiver("SlippageComp");

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

    double prev_frame = 0;
    
    timer.start();
    while (timer.msecsElapsed() <= test_duration) {
        robotino.set_robot_speed(vx, vy, omega);
        csv << timer.msecsElapsed() << ";";
        
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
        
        x_current = -0.577 * velocity_sign[0] * actual_current[0] + 0.577 * velocity_sign[2] * actual_current[2];
        y_current = 0.333 * velocity_sign[0] * actual_current[0] - 0.667 * velocity_sign[1] * actual_current[1] + 
                        0.333 * velocity_sign[2] * actual_current[2];
        rot_current = arr_sum(actual_current, 3) / 3;
        csv << x_current << ";" << y_current << ";" << rot_current << ";";
        
        if (shm_flag){
            if (receiver.data->at(0) != prev_frame) {
                std::cout << "|" << std::setw(15) << receiver.data->at(0);
                for (size_t i = 1; i < 7; i++)
                {
                    csv << receiver.data->at(i) << ";";
                    std::cout << "|" << std::setw(15) << receiver.data->at(i);
                }
                std::cout << "|\n";
                prev_frame = receiver.data->at(0);
            }
            else {
                for (size_t i = 1; i < 4; i++)
                {
                    csv << receiver.data->at(i) << ";";
                }
                for (size_t i = 0; i < 3; i++)
                {
                    csv << 0 << ";";
                }
            }
        }
        csv << "\n";
        rec::core_lt::msleep(measure_pause);
    }
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