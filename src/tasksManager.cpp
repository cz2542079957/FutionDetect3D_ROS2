
#include "tasksManager.h"

TasksManager::TasksManager(/* args */) {}

TasksManager::~TasksManager() {}

void TasksManager::run(std::vector<DeviceInfo> devices) {
    for (int i = 0; i < devices.size(); i++) {
        if (devices[i].status != 1) continue;
        Task task = {true, devices[i]};
        tasks.push_back(&task);
        if (devices[i].deviceName == "惯导模块") {
            addImuTask(task);
        }
    }
    while (true) {
        /* code */
    }
}

void TasksManager::stop() {
    for (int i = 0; i < tasks.size(); i++) {
        tasks[i]->running = false;
    }
}

void TasksManager::addLidarTask(Task &task) {}

void TasksManager::addImuTask(Task &task) {
    auto imuNode = std::make_shared<ImuNode>();
    std::thread imuThread([&]() { imuNode->work(*this, task); });
    imuThread.detach();
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "惯导模块线程[启动]");
}
