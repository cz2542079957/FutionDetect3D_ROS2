#pragma once
#include <dirent.h>
#include <libudev.h>
#include <string.h>
#include <sys/stat.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "utils.h"

// 设备ID
enum DeviceID {
    DIVICE_ID_CARMASTER,
    DIVICE_ID_LIDAR,
    DIVICE_ID_LIDAR_IMU,
    DIVICE_ID_CAR_IMU,
    DIVICE_ID_CAMERA,
    DIVICE_COUNT,
    DIVICE_ID_UNKNOW,
};

extern std::map<DeviceID, std::string> deviceNodes;  // 设备节点映射表
extern std::map<DeviceID, std::string> deviceNames;  // 设备名称映射表
extern std::map<DeviceID, int> deviceBaudRates;      // 设备串口波特率映射表

// 需要的设备
struct DeviceNeed {
    DeviceID id = DIVICE_ID_UNKNOW;
    // 是否强制需要
    bool required;
    // 是否当前有效
    bool available = false;
};

// 设备信息(对外传参使用)
struct DeviceInfo {
    DeviceID id;
    // 设备节点名称
    std::string node = "/dev";
    // 设备名称
    std::string name = "未知设备";
    // 设备波特率
    int baudRate = 0;
    // 设备状态 -1未识别 0断开 1正常
    int status = -1;
};

class DeviceManager : rclcpp::Node {
   public:
    DeviceManager();
    ~DeviceManager();

    // 匹配设备
    int matchDevices();

    std::vector<DeviceInfo>& getDevices() { return this->devices; };

   private:
    // 需要的设备列表
    std::vector<DeviceNeed> devicesNeed = {
        {DIVICE_ID_CARMASTER, false}, {DIVICE_ID_LIDAR, false}, {DIVICE_ID_LIDAR_IMU, false}, {DIVICE_ID_CAR_IMU, false}, {DIVICE_ID_CAMERA, false}};
    // 设备列表
    std::vector<DeviceInfo> devices;
    // 扫描可用设备
    int scanAvailableDevices();
    // 检查
    int checkFailedDevices();
};
