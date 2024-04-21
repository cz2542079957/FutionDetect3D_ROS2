#pragma once
#include <pthread.h>

#include <iostream>
#include <vector>

#include "carMasterNode.h"
#include "deviceManager.h"
#include "imuNode.h"
#include "lidarNode.h"

struct Task {
    // 是否正在运行
    bool running = false;
    // 设备信息
    DeviceInfo deviceInfo;
    // 工作线程
    std::thread workThread;
};

class TasksManager {
   public:
    TasksManager();
    ~TasksManager();

    // 开始主线程事件循环
    void run(std::vector<DeviceInfo> devices);
    // 退出
    void stop();

    // 获取模式（由LidarNode调用）
    int getMode();
    // 设置模式（由carMasterNode调用）
    void setMode(int val);

   private:
    // running
    bool running = false;
    // 当前模式（和小车模式同步）
    int mode = 0;  // 0: 静止 1: 扫描模式 2: 运动模式

    // 当前任务 （指针）
    std::vector<Task *> tasks;
    pthread_mutex_t tasksMutex = PTHREAD_MUTEX_INITIALIZER;
    // 是否可以退出主循环
    pthread_mutex_t exit = PTHREAD_MUTEX_INITIALIZER;

    void addLidarTask(Task &task);
    void addImuTask(Task &task);
    void addRosMasterTask(Task &task);
};
