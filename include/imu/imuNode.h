#pragma once
#include "rclcpp/rclcpp.hpp"
#include "serial.h"
class TasksManager;
struct Task;

class ImuNode : public rclcpp::Node {
   private:
   public:
    ImuNode();
    ~ImuNode();

    int work(TasksManager tm, Task& task);
};
