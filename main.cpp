
#include <csignal>
#include <iostream>

#include "deviceManager.h"
#include "imuNode.h"
#include "lidarNode.h"
#include "tasksManager.h"

DeviceManager* dm;
TasksManager* tm;
#define HERARTBEAT_INTERVAL 1000

struct sigaction oldSa;
void signal_handler(int signum) {
    if (signum == SIGINT) {
        //停止所有子线程
        tm->stop();
        sigaction(SIGINT, &oldSa, NULL);
    }
}

int main(int argc, char* argv[]) {
    std::system("clear");
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, &oldSa);

    rclcpp::init(argc, argv);
    // auto sllidar_node = std::make_shared<LidarNode>();
    //多线程实现
    // std::thread thread_a([&]() { ret = sllidar_node->work(); });
    // thread_a.join();

    dm = new DeviceManager();
    int availableDevicesNum = 0;
    int matchDevicesNum = 0;
    while (matchDevicesNum == 0) {
        availableDevicesNum = dm->scanAvailableDevices();
        matchDevicesNum = dm->matchDevices();
        if (matchDevicesNum == 0) {
            RCLCPP_INFO(rclcpp::get_logger("Main"), "正在等待设备接入...");
            std::this_thread::sleep_for(std::chrono::milliseconds(HERARTBEAT_INTERVAL));
        }
    }

    tm = new TasksManager();
    tm->run(dm->devices);

    delete dm, tm;
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("Main"), "结束");
    return 0;
}

/*



    colcon build --packages-select fusion_detect_3d

    source install/setup.bash

    ros2 run fusion_detect_3d main

*/