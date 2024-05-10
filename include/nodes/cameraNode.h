#pragma once
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rclcpp/rclcpp.hpp"

class TasksManager;
struct Task;

class CameraNode : public rclcpp::Node {
   public:
    CameraNode();
    ~CameraNode();

    int work(TasksManager& tm, Task& task);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/cameraNode";
    // 是否请求拍照
    bool needTakePhoto = true;
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

    // void publish(message::msg::ImuData::SharedPtr& imuData);
};
