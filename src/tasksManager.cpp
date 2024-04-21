
#include "tasksManager.h"

TasksManager::TasksManager() { RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[初始化]"); }

TasksManager::~TasksManager() {}

void TasksManager::run(std::vector<DeviceInfo> devices) {
    if (devices.size() == 0) {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "任务管理器[未启动]：当前没有有效设备");
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
        if (devices[i].id == DIVECE_ID_IMU) {
            addImuTask(*task);
        } else if (devices[i].id == DIVECE_ID_LIDAR) {
            addLidarTask(*task);
        } else if (devices[i].id == DIVECE_ID_CARMASTER) {
            addRosMasterTask(*task);
        }
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
                RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "正在等待“%s”工作线程退出", tasks[i]->deviceInfo.name.c_str());
                tasks[i]->workThread.join();
            }
        } catch (const std::system_error &e) {
            RCLCPP_ERROR(rclcpp::get_logger("TaskManager"), "在Join %s 线程时发生错误: %s", tasks[i]->deviceInfo.name, e.what());
        }
    }
    running = false;
}

int TasksManager::getMode() { return mode; }

void TasksManager::setMode(int val) { mode = val; }

void TasksManager::addLidarTask(Task &task) {
    task.workThread = std::thread([&]() {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "激光雷达线程[启动]");
        auto lidarNode = std::make_shared<LidarNode>();
        if (lidarNode->work(*this, task) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TasksManager"), "激光雷达线程[故障退出]");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "激光雷达线程[退出]");
    });
    // task.workThread.detach();
}

void TasksManager::addImuTask(Task &task) {
    task.workThread = std::thread([&]() {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "imu模块线程[启动]");
        auto imuNode = std::make_shared<ImuNode>();
        imuNode->work(*this, task);
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "imu模块线程[退出]");
    });
    // task.workThread.detach();
}

void TasksManager::addRosMasterTask(Task &task) {
    task.workThread = std::thread([&]() {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "小车控制板线程[启动]");
        auto carMasterNode = std::make_shared<CarMasterNode>();
        carMasterNode->work(*this, task);
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "小车控制板线程[退出]");
    });
}
