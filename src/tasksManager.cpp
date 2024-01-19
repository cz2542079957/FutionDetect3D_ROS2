
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
        Task *task = new Task();
        task->running = true;
        task->deviceInfo = devices[i];
        tasks.push_back(task);
        if (devices[i].deviceName == "惯导模块") {
            addImuTask(*task);
        } else if (devices[i].deviceName == "激光雷达") {
            addLidarTask(*task);
        }
        /* todo */
    }
    while (running) {
        std::this_thread::yield();
    }
    RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[退出]");
}

void TasksManager::stop() {
    for (int i = 0; i < tasks.size(); i++) {
        tasks[i]->running = false;
        try {
            if (tasks[i]->workThread.joinable()) {
                RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "正在等待“%s”工作线程退出", tasks[i]->deviceInfo.deviceName.c_str());
                tasks[i]->workThread.join();
            }
        } catch (const std::system_error &e) {
            RCLCPP_ERROR(rclcpp::get_logger("TaskManager"), "在Join %s 线程时发生错误: %s", tasks[i]->deviceInfo.deviceName, e.what());
        }
    }
    running = false;
}

void TasksManager::addLidarTask(Task &task) {
    task.workThread = std::thread([&]() {
        RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "激光雷达线程[启动]");
        auto lidarNode = std::make_shared<LidarNode>();
        lidarNode->work(*this, task);
        RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "激光雷达线程[退出]");
    });
    // task.workThread.detach();
}

void TasksManager::addImuTask(Task &task) {
    task.workThread = std::thread([&]() {
        RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块线程[启动]");
        auto imuNode = std::make_shared<ImuNode>();
        imuNode->work(*this, task);
        RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块线程[退出]");
    });
    // task.workThread.detach();
}
