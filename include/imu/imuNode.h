#pragma once
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
    //波特率 230400 115200
    int baudRate = 230400;
    //数据接受频率 hz
    int frequency = 100;
    // buffer = 100;
    std::vector<uint8_t> rawDataBuffer;

    void rawDataHandler(std::vector<uint8_t> arr, int count);
};
