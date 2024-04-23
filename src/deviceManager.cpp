
#include "deviceManager.h"

DeviceManager::DeviceManager() : Node("diviceManagerNode") {}

DeviceManager::~DeviceManager() {}

std::map<DeviceID, std::string> deviceNodes = {
    {DIVECE_ID_CARMASTER, "/dev/ros-master"},
    {DIVECE_ID_LIDAR, "/dev/rplidar"},
    {DIVECE_ID_IMU, "/dev/lidar-imu"},
    {DIVECE_ID_UNKNOW, "/dev"},
};

std::map<DeviceID, std::string> deviceNames = {
    {DIVECE_ID_CARMASTER, "小车控制板"},
    {DIVECE_ID_LIDAR, "激光雷达"},
    {DIVECE_ID_IMU, "惯性导航模块"},
    {DIVECE_ID_UNKNOW, "未知设备"},
};

std::map<DeviceID, int> deviceBaudRates = {
    {DIVECE_ID_CARMASTER, 115200},
    {DIVECE_ID_LIDAR, 1000000},
    {DIVECE_ID_IMU, 230400},
    {DIVECE_ID_UNKNOW, 0},
};

int DeviceManager::matchDevices() {
    int availableDevicesNum = 0;
    int failedDevicesNum = -1;
    int epoch = 1;
    while (failedDevicesNum != 0) {
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "正在进行第%d轮设备匹配...", epoch);
        availableDevicesNum = scanAvailableDevices();  // 搜索串口设备数量
        failedDevicesNum = checkFailedDevices();       // 检查连接失败的设备数量
        if (failedDevicesNum > 0) {
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "第%d轮设备匹配失败,还有%d个必要设备连接失败", epoch, failedDevicesNum);
            int sleepTime = std::min(epoch * 100 + 300, 3000);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
            epoch++;
            std::system("clear");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "第%d轮设备匹配成功,一共匹配%d个设备", epoch, availableDevicesNum);
    return devices.size();  // 返回当前匹配成功的设备数量
}

int DeviceManager::scanAvailableDevices() {
    for (int i = 0; i < devicesNeed.size(); i++) {
        struct stat info;
        DeviceID id = devicesNeed[i].id;
        std::string node = deviceNodes[id];
        std::string name = deviceNames[id];
        int baudRate = deviceBaudRates[id];
        if (stat(node.c_str(), &info) == 0) {
            if (S_ISCHR(info.st_mode))  // 检查是否为字符设备
            {
                DeviceInfo di = {id, node, name, baudRate, 1};
                devicesNeed[i].available = true;
                devices.push_back(di);
            }
        }
    }
    return devices.size();
}

int DeviceManager::checkFailedDevices() {
    int countFailed = 0;  // 记录连接失败的设备数量
    for (int i = 0; i < devicesNeed.size(); i++) {
        if (devicesNeed[i].required && !devicesNeed[i].available) {
            // 需要的硬件连接无效
            std::string name = deviceNames[devicesNeed[i].id];
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "该必要硬件连接失败：%s", name.c_str());
            countFailed++;
        }
    }
    return countFailed;
}

// std::string DeviceManager::getSerialNumber(udev_device* dev) {
//     const char* serialNumber = udev_device_get_property_value(dev, "ID_SERIAL_SHORT");
//     if (serialNumber)
//         return std::string(serialNumber);
//     else
//         return "Unknown";
// }
