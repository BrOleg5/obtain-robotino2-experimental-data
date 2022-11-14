#include "obtain_data.hpp"

const std::string about = "Obtain data from Robotino 2 and external camera.";
const std::string keys  =
        "{h help ? usage |            | Print help message}"
        "{d              |            | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{id             |            | Marker id }"
        "{ci             |            | Camera id }"
        "{t              |            | Program execution time in ms. If it equals 0, application run until user stop.}"
        "{cp             |            | JSON file with camera parameters }"
        "{shm            |            | Use shared memory to transmit data to other programs}"
        "{ov             | <none>     | Output video }"
        "{ce             |            | Camera exposure }"
        "{of             | file42     | CSV file path without extention }"
        "{ip             | 172.26.1.0 | Robotino IP address }"
        "{st             | 20         | Sample time}"
        "{vx             | 0          | Speed along X axis in m/s }"
        "{vy             | 0          | Speed along X axis in m/s }"
        "{omega          | 0          | Rotational velocity in rad/s }"
        "{v              | 0          | Amplitude of speed in m/s }"
        "{ang            | 0          | Direction of speed in degree }"
        "{square         |            | Moving in a square }"
        "{circle         |            | Circular movement }"
        "{triang         |            | Triangular movement }";

int main( int argc, char **argv ) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(parser.has("h") || parser.has("help") || parser.has("?") || parser.has("usage")) {
        parser.printMessage();
        return 0;
    }

    // IP-addres Robotino 2
    std::string ip_addr = parser.get<std::string>("ip");
    Robotino2 robotino(ip_addr);
    // Path to csv file
    std::string file_path = parser.get<std::string>("of");
    // Test duration
    long long test_duration = static_cast<long long>(parser.get<int>("t"));
    // Measure sample time
    long long sample_time = static_cast<long long>(parser.get<int>("st"));

    RobotData robot_buffer;
    robot_buffer.vx = parser.get<float>("vx");
    robot_buffer.vy = parser.get<float>("vy");
    robot_buffer.omega = parser.get<float>("omega");
    robot_buffer.speed = parser.get<float>("v");
    robot_buffer.angle = parser.get<float>("ang");
    
    if(parser.has("v") && parser.has("ang")) {
        robot_buffer.move_type = MovementType::POLAR;
    }
    else if(parser.has("square")) {
        robot_buffer.move_type = MovementType::SQUARE;
    }
    else if(parser.has("circle")) {
        robot_buffer.move_type = MovementType::CIRCLE;
    }
    else if(parser.has("triang")) {
        robot_buffer.move_type = MovementType::TRIANGLE;
    }

    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name;
    if (parser.has("d")) {
        int dictionary_id = parser.get<int>("d");
        dictionary_name = cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id);
    }
    else {
        std::cout << "Dictionary not specified" << std::endl;
        return 1;
    }

    cv::VideoCapture video_capture;
    if(parser.has("ci")) {
        int cam_id = parser.get<int>("ci");
        #ifdef WIN32
            video_capture.open(cam_id, cv::CAP_DSHOW);
        #else
            video_capture.open(cam_id);
        #endif
        video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        video_capture.set(cv::CAP_PROP_FOCUS, 0); // min: 0, max: 255, increment:5
        video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // link: https://stackoverflow.com/a/70074022
        if(parser.has("ce")) {
            video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
            video_capture.set(cv::CAP_PROP_EXPOSURE, parser.get<double>("ce"));
        }
        else {
            video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        }
        #ifdef WIN32
            video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        #endif

        //Checking for the camera to be connected 
        if (video_capture.isOpened()) {
            std::cout << "Camera connected.\n";
        }
        else {
            std::cout << "Camera not connected.\n";
            return 2;
        }

        // For initialization
        video_capture.grab();
    }
    else {
        std::cout << "Camera not specified.\n";
        return 3;
    }

    bool has_marker_id = parser.has("id");
    int markerID = 0;
    if(has_marker_id) {
        markerID = parser.get<int>("id");
    }

    std::string camParamFile;
    if(parser.has("cp")){
        camParamFile = parser.get<std::string>("cp");
    }
    cv::Point2f pixelResolution;
    if (!readCameraParameters(camParamFile, pixelResolution)) {
        std::cout << "Read camera parameters error.\n";
        return 7;
    }
    std::cout << "Camera parameters:\n\tpixel resolution x: " << pixelResolution.x 
            << "\n\tpixel resolution y: " << pixelResolution.y << '\n';
    // validate data
    if((pixelResolution.x == 0) || (pixelResolution.y == 0)) {
        std::cout << "Get invalid camera parameters.\n";
        return 8;
    }
    
    bool writeVideo = parser.has("ov");
    std::string outputFile;
    cv::VideoWriter videoWriter;
    if(writeVideo) {
        outputFile = parser.get<std::string>("ov");
        std::cout << "Write video file.\n";
        bool status = videoWriter.open(outputFile, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(1920, 1080), true);
        if(!status) {
            std::cout << "Video writer not initialized.\n";
            return 11;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return 4;
    }

    td::TransferData camera_buffer(pixelResolution);
	ArucoLocalization cv_system(video_capture, dictionary_name);

    std::mutex robot_buffer_mutex;

    Timer timer;
    std::mutex timer_mutex;

    std::thread robot_thread(read_robot_sensors, std::ref(robotino), std::ref(robot_buffer), std::ref(robot_buffer_mutex),
                            std::ref(timer), std::ref(timer_mutex), test_duration);
    
    std::atomic_bool camera_fault = false;
    std::mutex camera_buffer_mutex;
    std::mutex cv_system_mutex;

    std::thread camera_thread(read_camera_data, std::ref(cv_system), std::ref(cv_system_mutex), markerID,
                              std::ref(camera_buffer), std::ref(camera_buffer_mutex), std::ref(camera_fault));

    robot_thread.detach();
    camera_thread.detach();

    // Open or create csv file
    std::ofstream csv;
    std::string file_name = file_path + ".csv";
    csv.open(file_name, std::ios::out | std::ios::trunc);
	csv << csv_header;
    const char delimeter = ';';

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    if(!SetConsoleCtrlHandler(CtrlHandler, TRUE)) {
        std::cout << "Could not set control handler\n";
        return 7;
    }
    #endif

    long long count = 0ll;

    timer_mutex.lock();
    timer.reset();
    long long current_time = timer.time();
    timer_mutex.unlock();
    while(current_time < test_duration) {
        if(current_time >= (sample_time * count)) {
            robot_buffer_mutex.lock();
            csv << current_time << delimeter << robot_buffer.vx << delimeter <<
                   robot_buffer.vy << delimeter << robot_buffer.omega << delimeter;
            for (int i = 0; i < 3; i++) {
                csv << robot_buffer.motor_speed[i] << delimeter;
            }
            for (int i = 0; i < 3; i++) {
                csv << robot_buffer.actual_position[i] << delimeter;
            }
            for (int i = 0; i < 3; i++) {
                csv << robot_buffer.actual_velocity[i] << delimeter;
            }
            for (int i = 0; i < 3; i++) {
                csv << robot_buffer.actual_current[i] << delimeter;
            } 
            robot_buffer_mutex.unlock();

            camera_buffer_mutex.lock();
            csv << camera_buffer.currGlobalCartesian.x << delimeter << camera_buffer.currGlobalCartesian.y << delimeter <<
                   camera_buffer.currAngle << delimeter << camera_buffer.deltaEigenCartesian.x << delimeter <<
                   camera_buffer.deltaEigenCartesian.y << delimeter << camera_buffer.deltaAngle << '\n';
            camera_buffer_mutex.unlock();

            if(writeVideo) {
                cv_system_mutex.lock();
                videoWriter.write(cv_system.get_frame());
                cv_system_mutex.unlock();
            }
            count++;

            std::cout << current_time << '\r';
        }
        
        if(camera_fault) {
            isExit = true;
        }

        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        if(isExit) {
            break;
        }
        #endif
        timer_mutex.lock();
        current_time = timer.time();
        timer_mutex.unlock();
    }

    isExit = true;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    video_capture.release();
    videoWriter.release();
    csv.close();
    return 0;
} 