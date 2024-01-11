
#include "lidarNode.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto sllidar_node = std::make_shared<LidarNode>();
    //退出处理函数
    // signal(SIGINT, ExitHandler);
    int ret = 0;
    //多线程实现
    std::thread thread_a([&]() { ret = sllidar_node->work(); });
    thread_a.join();
    rclcpp::shutdown();
    return ret;
}

/*

    colcon build --packages-select fusionDetect3D

    source install/setup.bash

    ros2 run fusionDetect3D main

*/