#pragma once
#include "message/msg/imu_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial.h"

class TasksManager;
struct Task;

class ImuNode : public rclcpp::Node {
   public:
    ImuNode();
    ~ImuNode();

    int work(TasksManager tm, Task& task);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/imuNode";
    // 发布者
    rclcpp::Publisher<message::msg::ImuData>::SharedPtr publisher;

    // 串口
    serial::Serial* serial;
    // 数据接受频率 hz
    int frequency = 100;
    // buffer = 100;
    std::vector<uint8_t> rawDataBuffer;

    void rawDataHandler(std::vector<uint8_t> arr, int count);

    void publish(message::msg::ImuData::SharedPtr& imuData);
};
