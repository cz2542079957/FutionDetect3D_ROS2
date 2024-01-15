#include "imuNode.h"

#include "tasksManager.h"

ImuNode ::ImuNode(/* args */) : Node("imuNode") {}

ImuNode ::~ImuNode() {}

int ImuNode::work(TasksManager tm, Task& task) {
    // 115200
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "惯导模块线程[启动]");
    serial::Serial* serial = new serial::Serial(task.deviceInfo.node, baudRate);
    if (!serial->isOpen()) serial->open();
    int milliseconds = 1000 / this->frequency;

    while (task.running) {
        unsigned long count = serial->available();
        if (count <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
        } else {
            //读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            //处理原始数据
            // std::cout << "数量：" << realCount << std::endl;
            // for (int i = 0; i < res; i++) {
            //     printf("%02X ", static_cast<int>(arr[i]));
            // }
            rawDataHandler(arr, realCount);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "惯导模块线程[退出]");
    return 0;
}

void ImuNode::rawDataHandler(std::vector<uint8_t> arr, int count) {
    rawDataBuffer.insert(rawDataBuffer.end(), arr.begin(), arr.end());
    //找到开头节点
    int i = 0;
    while (rawDataBuffer[i] != 0x55) i++;
    //删除前面的无效段
    auto start = rawDataBuffer.begin();
    auto end = rawDataBuffer.begin() + i;
    rawDataBuffer.erase(start, end);
    // i要回归0
    i = 0;
    //加速度系数
    double accelerationCoe = 156.8 / 32768;
    //角速度系数
    double angularVelocityCoe = 2000.0 / 32768;
    //角度系数
    double angularCoe = 180.0 / 32768;
    //如果后面段长小于最小帧则跳过
    while (i < rawDataBuffer.size() && rawDataBuffer.size() - i >= 11) {
        if (rawDataBuffer[i] == 0x55) {
            // printf("%d : %d\n", i, rawDataBuffer[i]);
            switch (rawDataBuffer[i + 1]) {
                case 0x50: {
                    //时间
                    // printf("%u年%u月%u日%u时%u分%u秒%u毫秒\n", rawDataBuffer[i + 2], rawDataBuffer[i + 3], rawDataBuffer[i + 4], rawDataBuffer[i + 5],
                    //        rawDataBuffer[i + 6], rawDataBuffer[i + 7], milliseconds);
                    unsigned long long timestemp = (unsigned short)(((unsigned short)rawDataBuffer[9] << 8) | rawDataBuffer[8]) +
                                                   (((rawDataBuffer[i + 4] /*日*/ * 24 + rawDataBuffer[i + 5] /*时*/) * 60 + rawDataBuffer[i + 6] /*分*/) * 60 +
                                                    rawDataBuffer[i + 7] /*秒*/) *
                                                       1000;
                    // printf("time :%u\n", timestemp);
                }
                case 0x51: {
                    //加速度计
                    double ax = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * accelerationCoe;
                    double ay = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * accelerationCoe;
                    double az = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * accelerationCoe;
                    float temperature = ((rawDataBuffer[i + 9] << 8) | rawDataBuffer[i + 8]) / 100;
                    // printf("x:%lf y:%lf z:%lf ,  温度: %f \n", ax, ay, az, temperature);
                    break;
                }
                case 0x52: {
                    //角速度
                    double wx = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * angularVelocityCoe;
                    double wy = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * angularVelocityCoe;
                    double wz = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * angularVelocityCoe;
                    // printf("角速度 x:%lf y:%lf z:%lf\n", wx, wy, wz);
                    break;
                }
                case 0x53: {
                    //角度
                    double roll = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * angularCoe;
                    double pitch = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * angularCoe;
                    double yaw = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * angularCoe;
                    // printf("角度 x:%lf y:%lf z:%lf\n", roll, pitch, yaw);
                    break;
                }
            }
            i += 11;
            continue;
        }
        break;
    }
    rawDataBuffer.erase(rawDataBuffer.begin(), rawDataBuffer.begin() + i);
}
