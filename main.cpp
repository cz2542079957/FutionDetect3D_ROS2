
#include "lidarNode.h"
#include "serial.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto sllidar_node = std::make_shared<LidarNode>();
    //退出处理函数
    // signal(SIGINT, ExitHandler);
    int ret = 0;
    //多线程实现
    // std::thread thread_a([&]() { ret = sllidar_node->work(); });
    // thread_a.join();

    serial::Serial* serial = new serial::Serial("/dev/ttyUSB0", 115200);
    // std::cout << "available:" << serial->available() << " isOpen:" << serial->open() << std::endl;
    if (!serial->isOpen()) serial->open();
    int i = 100000;
    while (i-- > 0) {
        std::cout << serial->available() << std::endl;
    }

    // std::vector<std::string> lines = serial->readlines();

    // std::cout << lines.size() << std::endl;
    rclcpp::shutdown();
    return ret;
}

/*

    colcon build --packages-select fusionDetect3D

    source install/setup.bash

    ros2 run fusionDetect3D main

*/