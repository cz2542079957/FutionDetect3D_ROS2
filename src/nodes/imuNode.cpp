#include "imuNode.h"

#include "tasksManager.h"

ImuNode ::ImuNode() : Node("imuNode") {
    publisher = this->create_publisher<message::msg::ImuData>(nodePrefix + "/imuData", rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块节点初始化完成");
}

ImuNode ::~ImuNode() {
    delete serial;
    RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块节点销毁");
}

int ImuNode::work(TasksManager tm, Task& task) {
    serial = new serial::Serial(task.deviceInfo.node, task.deviceInfo.baudRate);
    if (!serial->isOpen()) serial->open();
    int microseconds = 1000 * 1000 / this->frequency;
    RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块节点开始运行");
    while (task.running) {
        unsigned long count = serial->available();
        if (count <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
        } else {
            // 读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            // std::cout << "数量：" << realCount << std::endl;
            // for (int i = 0; i < realCount; i++) {
            //     printf("%02X ", static_cast<int>(arr[i]));
            // }
            // 脏数据
            if (count != realCount) continue;
            // 处理原始数据
            rawDataHandler(arr, realCount);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("ImuNode"), "惯导模块节点终止运行");
    return 0;
}

void ImuNode::rawDataHandler(std::vector<uint8_t> arr, int count) {
    rawDataBuffer.insert(rawDataBuffer.end(), arr.begin(), arr.end());
    if (rawDataBuffer.size() < 11) return;
    // 找到开头节点
    int i = 0;
    while (!((rawDataBuffer[i] == 0x55) && (rawDataBuffer[i + 1] == 0x52))) i++;
    // 删除前面的无效段
    rawDataBuffer.erase(rawDataBuffer.begin(), rawDataBuffer.begin() + i);
    // i要回归0
    i = 0;
    // 加速度系数
    double accelerationCoe = 156.8 / 32768;
    // 角速度系数
    double angularVelocityCoe = 2000.0 / 32768;
    // 角度系数
    double angularCoe = 180.0 / 32768;
    // 已处理数据
    auto handledData = std::make_shared<message::msg::ImuData>();
    auto dataFrame = message::msg::ImuDataFrame();
    // 如果后面段长小于最小帧则跳过
    while (i < rawDataBuffer.size() && rawDataBuffer.size() - i >= 11) {
        if (rawDataBuffer[i] == 0x55) {
            // printf("%d : %d\n", i, rawDataBuffer[i]);
            switch (rawDataBuffer[i + 1]) {
                // case 0x50: {
                //     //时间
                //     // dataFrame.timestemp = (unsigned short)(((unsigned short)rawDataBuffer[9] << 8) | rawDataBuffer[8]) +
                //     //                       (((rawDataBuffer[i + 4] /*日*/ * 24 + rawDataBuffer[i + 5] /*时*/) * 60 + rawDataBuffer[i + 6] /*分*/) * 60 +
                //     //                        rawDataBuffer[i + 7] /*秒*/) *
                //     //                           1000;
                //     dataFrame.timestemp = this->now().nanoseconds();
                //     // printf("time :%u\n", timestemp);
                //     break;
                // }
                // case 0x51: {
                //     //加速度计
                //     dataFrame.acceleration.acceleration_x = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * accelerationCoe;
                //     dataFrame.acceleration.acceleration_y = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * accelerationCoe;
                //     dataFrame.acceleration.acceleration_z = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * accelerationCoe;
                //     dataFrame.temperature = ((rawDataBuffer[i + 9] << 8) | rawDataBuffer[i + 8]) / 100;
                //     // printf("x:%lf y:%lf z:%lf ,  温度: %f \n", ax, ay, az, temperature);
                //     break;
                // }
                case 0x52: {
                    // 角速度
                    dataFrame.timestemp = this->now().nanoseconds();
                    dataFrame.angular_velocity.angular_velocity_x = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * angularVelocityCoe;
                    dataFrame.angular_velocity.angular_velocity_y = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * angularVelocityCoe;
                    dataFrame.angular_velocity.angular_velocity_z = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * angularVelocityCoe;
                    // printf("角速度 x:%lf y:%lf z:%lf\n", wx, wy, wz);
                    break;
                }
                // case 0x53: {
                //     //角度
                //     dataFrame.angular.roll = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) * angularCoe;
                //     dataFrame.angular.pitch = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) * angularCoe;
                //     dataFrame.angular.yaw = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) * angularCoe;
                //     // printf("角度 x:%lf y:%lf z:%lf\n", dataFrame.angular.roll, dataFrame.angular.pitch, dataFrame.angular.yaw);
                //     //帧最后一段，添加整个帧到待传送容器
                //     handledData->data.push_back(dataFrame);
                //     break;
                // }
                case 0x54: {
                    // 磁场
                    dataFrame.magnetic_field.magnetic_field_x = ((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]);
                    dataFrame.magnetic_field.magnetic_field_y = ((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]);
                    dataFrame.magnetic_field.magnetic_field_z = ((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]);
                    dataFrame.temperature = ((rawDataBuffer[i + 9] << 8) | rawDataBuffer[i + 8]) / 100;
                    // printf("磁场 x:%lf y:%lf z:%lf\n", dataFrame.magnetic_field.magnetic_field_x, dataFrame.magnetic_field.magnetic_field_y,
                    //        dataFrame.magnetic_field.magnetic_field_z);
                    break;
                }
                case 0x59: {
                    // 四元数
                    dataFrame.quaternion.quaternion_0 = (short)((short)(rawDataBuffer[i + 3] << 8) | rawDataBuffer[i + 2]) / 32768.0f;
                    dataFrame.quaternion.quaternion_1 = (short)((short)(rawDataBuffer[i + 5] << 8) | rawDataBuffer[i + 4]) / 32768.0f;
                    dataFrame.quaternion.quaternion_2 = (short)((short)(rawDataBuffer[i + 7] << 8) | rawDataBuffer[i + 6]) / 32768.0f;
                    dataFrame.quaternion.quaternion_3 = (short)((short)(rawDataBuffer[i + 9] << 8) | rawDataBuffer[i + 8]) / 32768.0f;
                    // 帧最后一段，添加整个帧到待传送容器
                    handledData->data.push_back(dataFrame);
                    // printf("四元数 q0:%lf q1:%lf q2:%lf q3:%lf\n", dataFrame.quaternion.quaternion_0, dataFrame.quaternion.quaternion_1,
                    //        dataFrame.quaternion.quaternion_2, dataFrame.quaternion.quaternion_3);
                    break;
                }
            }
            i += 11;
            continue;
        }
        break;
    }
    // RCLCPP_INFO(rclcpp::get_logger("ImuNode"), " %u", publisher);
    publish(handledData);
    rawDataBuffer.erase(rawDataBuffer.begin(), rawDataBuffer.begin() + i);
}

void ImuNode::publish(message::msg::ImuData::SharedPtr& imuData) {
    if (!publisher) {
        // publisher被销毁
        return;
    }
    // 发布
    publisher->publish(*imuData);
}
