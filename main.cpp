/*
 * @Author: cz2542079957 2542079957@qq.com
 * @Date: 2024-01-10 11:54:17
 * @LastEditors: cz2542079957 2542079957@qq.com
 * @LastEditTime: 2024-04-14 11:37:02
 * @FilePath: /fusion_detect_3d/main.cpp
 * @Description: fusion detect 3d 项目中部署于[主控开发板]的子项目
 *
 * Copyright (c) 2024 by 2542079957@qq.com, All Rights Reserved.
 */

#include <csignal>
#include <iostream>

#include "deviceManager.h"
#include "imuNode.h"
#include "lidarNode.h"
#include "tasksManager.h"

DeviceManager* deviceManager;
TasksManager* tasksManager;

struct sigaction oldSa;
void signal_handler(int signum) {
    if (signum == SIGINT) {
        // 停止所有子线程
        tasksManager->stop();
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

    RCLCPP_INFO(rclcpp::get_logger("Main"), "开始");
    rclcpp::init(argc, argv);

    deviceManager = new DeviceManager();
    deviceManager->matchDevices();
    tasksManager = new TasksManager();
    tasksManager->run(deviceManager->getDevices());

    delete deviceManager, tasksManager;
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("Main"), "结束");
    return 0;
}

/* 

    colcon build --packages-select fusion_detect_3d

    source install/setup.bash

    export ROS_DOMAIN_ID=1

    ros2 run fusion_detect_3d main

*/