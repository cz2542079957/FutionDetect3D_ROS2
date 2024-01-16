
#include "deviceManager.h"

DeviceManager::DeviceManager() : Node("diviceManagerNode") {}

DeviceManager::~DeviceManager() {}

int DeviceManager::scanAvailableDevices() {
    struct udev* udev = udev_new();
    if (!udev) {
        RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), "创建udev上下文失败");
        return -1;
    }
    // 创建一个udev_enumerate对象，用来枚举指定类型的设备
    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    // 添加匹配规则，只枚举tty子系统下的设备
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    // 执行扫描操作，查找满足条件的设备
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry *_devices = udev_enumerate_get_list_entry(enumerate), *entry;
    // 遍历设备列表
    udev_list_entry_foreach(entry, _devices) {
        const char* path = udev_list_entry_get_name(entry);
        if (std::string(path).find("ttyUSB") != std::string::npos) {
            struct udev_device* dev = udev_device_new_from_syspath(udev, path);
            std::string devNode = std::string(udev_device_get_devnode(dev));
            std::string vid = std::string(udev_device_get_property_value(dev, "ID_VENDOR_ID"));
            std::string pid = std::string(udev_device_get_property_value(dev, "ID_MODEL_ID"));
            std::string sn = getSerialNumber(dev);
            //加入到识别到的设备列表中
            DeviceInfo deviceInfo = {devNode, sn, vid, pid};
            devices.push_back(deviceInfo);
            udev_device_unref(dev);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功找到%d个串口设备", devices.size());
    //清理
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return devices.size();
}

int DeviceManager::matchDevices() {
    int count = 0;
    for (int i = 0; i < devices.size(); i++) {
        if (devices[i].sn == lidarSerialNum) {
            devices[i].status = 1;
            devices[i].deviceName = "激光雷达";
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功匹配激光雷达：%s", devices[i].node.c_str());
            count++;
        } else if (devices[i].sn == imuSerialNum) {
            devices[i].status = 1;
            devices[i].deviceName = "惯导模块";
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "成功匹配惯导模块：%s", devices[i].node.c_str());
            count++;
        } else {
            devices[i].status = -1;
        }
    }
    return count;
}

std::vector<DeviceInfo> DeviceManager::getDevices() { return this->devices; }

std::string DeviceManager::getSerialNumber(udev_device* dev) {
    const char* serialNumber = udev_device_get_property_value(dev, "ID_SERIAL_SHORT");
    if (serialNumber)
        return std::string(serialNumber);
    else
        return "Unknown";
}
