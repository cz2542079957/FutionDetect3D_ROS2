
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
        rclcpp::shutdown();
        sigaction(SIGINT, &oldSa, NULL);
        delete dm, tm;
        RCLCPP_INFO(rclcpp::get_logger("Main"), "结束");
        exit(EXIT_SUCCESS);
    }
}

int main(int argc, char* argv[]) {
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
    // delete dm;

    // auto imuNode = std::make_shared<ImuNode>();
    // std::thread imuThread([&]() { imuNode->work(); });
    // imuThread.detach();

    rclcpp::shutdown();
    return 0;
}

/*



    colcon build --packages-select fusionDetect3D

    source install/setup.bash

    ros2 run fusionDetect3D main

*/