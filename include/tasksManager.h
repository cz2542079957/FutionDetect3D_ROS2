#pragma once
#include <pthread.h>

#include <iostream>
#include <vector>

#include "deviceManager.h"
#include "imuNode.h"

struct Task {
    //是否正在运行
    bool running = false;
    //设备信息
    DeviceInfo deviceInfo;
};

class TasksManager {
   public:
    TasksManager(/* args */);
    ~TasksManager();
    // 开始主线程时间循环
    void run(std::vector<DeviceInfo> devices);
    //退出
    void stop();

   private:
    //当前任务 （指针）
    std::vector<Task *> tasks;
    pthread_mutex_t tasksMutex = PTHREAD_MUTEX_INITIALIZER;
    //是否可以退出主循环
    pthread_mutex_t exit = PTHREAD_MUTEX_INITIALIZER;

    void addLidarTask(Task &task);
    void addImuTask(Task &task);
};
