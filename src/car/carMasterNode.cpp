#include "carMasterNode.h"

#include "tasksManager.h"

CarMasterNode::CarMasterNode() : Node("carMasterNode") {
    buzzerController = new BuzzerController();
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点初始化完成");
}

CarMasterNode::~CarMasterNode() {
    delete buzzerController;
    delete serial;
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点销毁");
}

int CarMasterNode::work(TasksManager tm, Task& task) {
    serial = new serial::Serial(task.deviceInfo.node, task.deviceInfo.baudRate);
    if (!serial->isOpen()) serial->open();
    start();

    while (task.running) {
        unsigned long count = serial->available();
        if (count != 0) {
            // 读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            // std::cout << "数量：" << realCount << std::endl;
            // for (int i = 0; i < realCount; i++) {
            //     printf("%02X ", static_cast<int>(arr[i]));
            // }
            rawDataBuffer.insert(rawDataBuffer.end(), arr.begin(), arr.end());
        }
        if (rawDataBuffer.size() > 0) {
            // 解析数据
            frameParser();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    end();
    return 0;
}

void CarMasterNode::send(std::vector<uint8_t> data) {
    if (serial->isOpen()) {
        // 计算校验和
        uint8_t checkSum = std::accumulate(data.begin() + 3, data.end(), 0);
        checkSum = checkSum & 0xff;
        // 将校验和添加到数据包的末尾
        data.push_back(checkSum);
        // 插入帧头
        data.insert(data.begin(), data.size());
        data.insert(data.begin(), 0xCC);
        data.insert(data.begin(), 0xFF);
        // 发送帧
        serial->write(data);
    }
}

void CarMasterNode::frameParser() {
    // RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "%d", rawDataBuffer.size());
    // 去除损坏字节
    while (rawDataBuffer.size() > 0) {
        if (rawDataBuffer[0] == 0xFF && rawDataBuffer[1] == 0xCC) {
            break;
        }
        rawDataBuffer.erase(rawDataBuffer.begin());
    }
    // 开始处理数据
    while (rawDataBuffer.size() > MIN_FRAME_LEN) {
        if (rawDataBuffer[0] == 0xFF && rawDataBuffer[1] == 0xCC) {
            // 获取帧长度
            uint8_t frameLen = rawDataBuffer[2];
            if (frameLen + 3 > rawDataBuffer.size()) {
                break;
            }
            // 获取校验和
            uint8_t checkSum = rawDataBuffer[frameLen + 2];
            // 计算校验和
            uint8_t realCheckSum = std::accumulate(rawDataBuffer.begin() + 3, rawDataBuffer.begin() + frameLen + 2, 0);
            realCheckSum = realCheckSum & 0xff;
            // 校验和正确
            if (checkSum == realCheckSum) {
                uint8_t function = rawDataBuffer[3];
                switch (function) {
                    case FRAME_RESPONSE: {
                    }
                    case FRAME_RESPONSE_VOTAGE: {
                        float voltage = (rawDataBuffer[4] | (rawDataBuffer[5] << 8)) / 1000.0;
                        RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "电压：%.2f", voltage);
                        break;
                    }
                    case FRAME_RESPONSE_ENCODER: {
                        // 编码器（带符号）
                        int8_t encoder1 = rawDataBuffer[4];
                        int8_t encoder2 = rawDataBuffer[5];
                        int8_t encoder3 = rawDataBuffer[6];
                        int8_t encoder4 = rawDataBuffer[7];
                        // RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "编码器：%d %d %d %d", encoder1, encoder2, encoder3, encoder4);
                        break;
                    }
                }
            }
            // 清理
            rawDataBuffer.erase(rawDataBuffer.begin(), rawDataBuffer.begin() + frameLen + 3);
        } else {
            break;
        }
    }
}

void CarMasterNode::start() {}

void CarMasterNode::end() {}
