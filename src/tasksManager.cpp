
#include "tasksManager.h"

TasksManager::TasksManager(/* args */) {}

TasksManager::~TasksManager() {}

void TasksManager::run(std::vector<DeviceInfo> devices) {
    if (devices.size() == 0) {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[未启动]：当前没有有效设备，无法启动");
        return;
    }
    running = true;
    RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[启动]");
    for (int i = 0; i < devices.size(); i++) {
        if (devices[i].status != 1) continue;
        Task task = {true, devices[i]};
        tasks.push_back(&task);
        if (devices[i].deviceName == "惯导模块") {
            addImuTask(task);
        } else if (devices[i].deviceName == "激光雷达") {
            addLidarTask(task);
        }
        /* todo */
    }
    while (running) {
    }
    RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[退出]");
}

void TasksManager::stop() {
    for (int i = 0; i < tasks.size(); i++) {
        tasks[i]->running = false;
    }
    running = false;
}

void TasksManager::addLidarTask(Task &task) {
    std::thread lidarThread([&]() {
        auto lidarNode = std::make_shared<LidarNode>();
        lidarNode->work(*this, task);
    });
    lidarThread.detach();
}

void TasksManager::addImuTask(Task &task) {
    std::thread imuThread([&]() {
        auto imuNode = std::make_shared<ImuNode>();
        imuNode->work(*this, task);
    });
    imuThread.detach();
}
