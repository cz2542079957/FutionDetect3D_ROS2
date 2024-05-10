#pragma once
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "message/msg/mode_control.hpp"
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
    bool needTakePhoto = false;
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

    std::thread executorThread;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    // 接收控制
    rclcpp::Subscription<message::msg::ModeControl>::SharedPtr subscriber;

    void cameraControlCallback(const message::msg::ModeControl::SharedPtr msg);

    void start();
    void end();
};
