
#include <csignal>
#include <iostream>

#include "deviceManager.h"
#include "imuNode.h"
#include "lidarNode.h"
#include "tasksManager.h"

DeviceManager* dm;
TasksManager* tm;

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

    // serial::Serial* serial = new serial::Serial("/dev/rosmaster", 115200);
    // if (!serial->isOpen()) {
    //     serial->open();
    // }
    // int check = (0x05 + 0x02 + 0x01 + 0x02) % 256;
    // char* send = new char[7]{0xff, 0xfc, 0x05, 0x02, 0x01, 0x02, check};



    dm = new DeviceManager();
    dm->matchDevices();
    tm = new TasksManager();
    tm->run(dm->getDevices());

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