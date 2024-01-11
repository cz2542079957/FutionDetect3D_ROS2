#pragma once
#include <string>

#include "sl_lidar.h"
#include "sl_types.h"
using namespace sl;

class LidarConfig {
   public:
    LidarConfig(std::string _mode = "Standard");
    ~LidarConfig();

    //雷达驱动
    ILidarDriver *driver;
    //通信
    IChannel *channel;
    //通讯方式(默认串口通信)
    std::string channelType = "serial";
    //
    std::string port = "/dev/ttyUSB0";

    //串口波特率
    int baudrate = 1000000;
    /*模式
        0  Standard: 最远扫描距离: 30.0 m, 每秒扫描点数: 16.1K
        1  DenseBoost: 最远扫描距离: 30.0 m, 每秒扫描点数: 32.3K
    */
    //当前扫描模式
    LidarScanMode currentMode;
    //模式名称
    std::string modeName = "Standard";
    //模式id
    int modeId = 0;
    //最远扫描距离 m
    float maxDistance = 30;
    //频率 hz
    int frequency = 10;
};
