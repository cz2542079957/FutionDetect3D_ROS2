#pragma once
#include "message/msg/imu_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial.h"

class TasksManager;
struct Task;

class CarImuNode : public rclcpp::Node {
   public:
    CarImuNode();
    ~CarImuNode();

    int work(TasksManager& tm, Task& task);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/carImuNode";
    // 发布者
    rclcpp::Publisher<message::msg::ImuData>::SharedPtr publisher;

    // 串口
    serial::Serial* serial;
    // 数据接受频率 hz
    int frequency = 200;
    
    std::vector<uint8_t> rawDataBuffer;

    void rawDataHandler(std::vector<uint8_t> arr, int count);

    void publish(message::msg::ImuData::SharedPtr& imuData);
};
