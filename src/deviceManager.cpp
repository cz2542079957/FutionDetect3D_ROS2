
#include "deviceManager.h"

DeviceManager::DeviceManager() : Node("diviceManagerNode") {}

DeviceManager::~DeviceManager() {}

int DeviceManager::matchDevices() {
    // int count = 0;
    // for (int i = 0; i < devices.size(); i++) {
    //     if (devices[i].sn == lidarSerialNum) {
    //         devices[i].status = 1;
    //         devices[i].deviceName = "激光雷达";
    //         RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功匹配激光雷达：%s", devices[i].node.c_str());
    //         count++;
    //     } else if (devices[i].sn == imuSerialNum) {
    //         devices[i].status = 1;
    //         devices[i].deviceName = "惯导模块";
    //         RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功匹配惯导模块：%s", devices[i].node.c_str());
    //         count++;
    //     } else {
    //         devices[i].status = -1;
    //     }
    // }
    // return count;

    int availableDevicesNum = 0;
    int failedDevicesNum = -1;
    int epoch = 1;
    while (failedDevicesNum != 0) {
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "正在进行第%d轮设备匹配...", epoch);
        availableDevicesNum = scanAvailableDevices();  //搜索串口设备数量
        failedDevicesNum = checkFailedDevices();       //检查连接失败的设备数量
        if (failedDevicesNum > 0) {
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "第%d轮设备匹配失败,还有%d个必要设备连接失败", epoch, failedDevicesNum);
            int sleepTime = std::min(epoch * 100 + 300, 3000);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
            epoch++;
            std::system("clear");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "第%d轮设备匹配成功,一共匹配%d个设备", epoch, availableDevicesNum);
    return devices.size();  //返回当前匹配成功的设备数量
}

int DeviceManager::scanAvailableDevices() {
    // struct udev* udev = udev_new();
    // if (!udev) {
    //     RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), "创建udev上下文失败");
    //     return -1;
    // }
    // // 创建一个udev_enumerate对象，用来枚举指定类型的设备
    // struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    // // 添加匹配规则，只枚举tty子系统下的设备
    // udev_enumerate_add_match_subsystem(enumerate, "tty");
    // // 执行扫描操作，查找满足条件的设备
    // udev_enumerate_scan_devices(enumerate);
    // struct udev_list_entry *_devices = udev_enumerate_get_list_entry(enumerate), *entry;
    // // 遍历设备列表
    // udev_list_entry_foreach(entry, _devices) {
    //     const char* path = udev_list_entry_get_name(entry);
    //     if (std::string(path).find("ttyUSB") != std::string::npos) {
    //         struct udev_device* dev = udev_device_new_from_syspath(udev, path);
    //         std::string devNode = std::string(udev_device_get_devnode(dev));
    //         std::string vid = std::string(udev_device_get_property_value(dev, "ID_VENDOR_ID"));
    //         std::string pid = std::string(udev_device_get_property_value(dev, "ID_MODEL_ID"));
    //         std::string sn = getSerialNumber(dev);
    //         //加入到识别到的设备列表中
    //         DeviceInfo deviceInfo = {devNode, sn, vid, pid};
    //         RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "dev:%s", deviceInfo.node.c_str());
    //         devices.push_back(deviceInfo);
    //         udev_device_unref(dev);
    //     }
    // }
    // RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功找到%d个串口设备", devices.size());
    // //清理
    // udev_enumerate_unref(enumerate);
    // udev_unref(udev);
    for (int i = 0; i < devicesNeed.size(); i++) {
        struct stat info;
        if (stat(devicesNeed[i].node.c_str(), &info) == 0) {
            if (S_ISCHR(info.st_mode))  // 检查是否为字符设备
            {
                DeviceInfo di = {devicesNeed[i].node, devicesNeed[i].name, 1};
                devicesNeed[i].available = true;
                devices.push_back(di);
            }
        }
    }
    return devices.size();
}

int DeviceManager::checkFailedDevices() {
    int countFailed = 0;  //记录连接失败的设备数量
    for (int i = 0; i < devicesNeed.size(); i++) {
        if (devicesNeed[i].required && !devicesNeed[i].available) {
            //需要的硬件连接无效
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "该必要硬件连接失败：%s", devicesNeed[i].name.c_str());
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
