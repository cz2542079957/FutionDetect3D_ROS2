#pragma once
#include "buzzerController.h"
#include "carMasterBase.h"
#include "rclcpp/rclcpp.hpp"

class TasksManager;
struct Task;

class CarMasterNode : public rclcpp::Node {
   public:
    CarMasterNode();
    ~CarMasterNode();

    int work(TasksManager tm, Task& task);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/carMasterNode";
    // 发布者
    //  rclcpp::Publisher<message::msg::ImuData>::SharedPtr publisher;

    // void rawDataHandler(std::vector<uint8_t> arr, int count);

    // void publish(message::msg::ImuData::SharedPtr& imuData);

    // 串口
    serial::Serial* serial; 
    // 数据接受频率 hz
    int frequency = 100;
    // buffer
    std::vector<uint8_t> rawDataBuffer;

    // 蜂鸣器控制器
    BuzzerController* buzzerController;

    void start();
    void end();
};
