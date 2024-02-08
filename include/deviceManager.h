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

//需要的设备
struct DeviceNeed {
    //端口名称
    std::string node;
    //设备名称
    std::string name;
    //是否强制需要
    bool required;
    //是否当前有效
    bool available = false;
};

//设备信息
// struct DeviceInfo {
//     //设备节点名称
//     std::string node;
//     //设备sn码
//     std::string sn;
//     //设备厂商号
//     std::string vid;
//     //设备产品号
//     std::string pid;
//     //设备名称
//     std::string deviceName = "Unknow";
//     //设备状态 -1未识别 0断开 1正常
//     int status = 0;
// };
struct DeviceInfo {
    //设备节点名称
    std::string node;
    //设备名称
    std::string deviceName = "Unknow";
    //设备状态 -1未识别 0断开 1正常
    int status = -1;
};

class DeviceManager : rclcpp::Node {
   public:
    DeviceManager();
    ~DeviceManager();

    //匹配设备
    int matchDevices();

    std::vector<DeviceInfo>& getDevices() { return this->devices; };

   private:
    // 设备列表
    std::vector<DeviceInfo> devices;
    //需要的设备列表
    std::vector<DeviceNeed> devicesNeed = {{"/dev/rosmaster", "ros扩展板", true}, {"/dev/rplidar", "激光雷达", false}, {"/dev/imu", "imu模块", false}};
    //扫描可用设备
    int scanAvailableDevices();
    //检查
    int checkFailedDevices();

    // 在本项目中 暂时写死
    // std::string lidarSerialNum = "0aebf2bc3113ec119ec8f0ef7a109228";
    // std::string imuSerialNum = "0001";
    //获取sn号
    // std::string getSerialNumber(struct udev_device* dev);
};
