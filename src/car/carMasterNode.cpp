#include "carMasterNode.h"

#include "tasksManager.h"

CarMasterNode::CarMasterNode() : Node("carMasterNode") {
    buzzerController = new BuzzerController();
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "ros扩展板节点初始化完成");
}

CarMasterNode::~CarMasterNode() {
    delete buzzerController;
    delete serial;
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "ros扩展板节点销毁");
}

int CarMasterNode::work(TasksManager tm, Task& task) {
    serial = new serial::Serial(task.deviceInfo.node, task.deviceInfo.baudRate);
    if (!serial->isOpen()) serial->open();
    start();

    while (task.running) {
        unsigned long count = serial->available();
        if (count == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        else {
            // 读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            // std::cout << "数量：" << realCount << std::endl;
            for (int i = 0; i < realCount; i++) {
                printf("%02X ", static_cast<int>(arr[i]));
            }
        }
    }
    end();
    return 0;
}

void CarMasterNode::start() {
    // 关闭自动发送
    std::vector<uint8_t> data = {0xff, 0xfc, 0x05, 0x01, 0x00, 0x00};
    CarMasterBase::send(*serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // data = {0xff, 0xfc, 0x07, 0x10, 100, 100, 100, 100};
    // CarMasterBase::send(*serial, data);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // data = {0xff, 0xfc, 0x07, 0x10, 0, 0, 0, 0};
    // CarMasterBase::send(*serial, data);

    data = {0xff, 0xfc, 0x07, 0x11, 1, 10, 0x00};
    CarMasterBase::send(*serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    data = {0xff, 0xfc, 0x07, 0x11, 0, 0, 0x00};
    CarMasterBase::send(*serial, data);

    // 开机音效
    buzzerController->startSound(*serial);
}

void CarMasterNode::end() { buzzerController->endSound(*serial); }
