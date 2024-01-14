#include "imuNode.h"

#include "tasksManager.h"

ImuNode ::ImuNode(/* args */) : Node("imuNode") {}

ImuNode ::~ImuNode() {}

int ImuNode::work(TasksManager tm, Task& task) {
    // 115200
    serial::Serial* serial = new serial::Serial(task.deviceInfo.node, 115200);
    // std::cout << "available:" << serial->available() << " isOpen:" << serial->open() << std::endl;
    if (!serial->isOpen()) serial->open();
    int i = 100;
    while (task.running) {
        unsigned long count = serial->available();
        if (count < 1024) continue;
        uint8_t* arr = new uint8_t[count];
        size_t res = serial->read(arr, count);
        // std::cout << i << " " << count << " " << static_cast<unsigned char>(arr[0]) << std::endl;
        // for (int j = 0; j < count; j++) std::cout << static_cast<int>(arr[j]) << " ";

        // std::cout << "Using %hhu format specifier: " << static_cast<uint8_t>(arr[1]) << std::endl;
        std::cout << res << std::endl;
        std::cout.flush();
        // i--;
    }

    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "惯导模块线程[退出]");
    // std::vector<std::string> lines = serial->readlines();
    // std::cout << lines.size() << std::endl;
    return 0;
}
