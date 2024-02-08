#include "carMasterNode.h"

#include "tasksManager.h"

CarMasterNode::CarMasterNode() : Node("carMasterNode") { RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "ros扩展板节点初始化完成"); }

CarMasterNode::~CarMasterNode() { RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "ros扩展板节点销毁"); }

int CarMasterNode::work(TasksManager tm, Task& task) {
    serial::Serial* serial = new serial::Serial(task.deviceInfo.node, baudRate);
    if (!serial->isOpen()) serial->open();

    int i = 100;
    while (i--) {
        unsigned long count = serial->available();
        if (count == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        else {
            //读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            // std::cout << "数量：" << realCount << std::endl;
            for (int i = 0; i < realCount; i++) {
                printf("%02X ", static_cast<int>(arr[i]));
            }
        }
    }
    return 0;
}
