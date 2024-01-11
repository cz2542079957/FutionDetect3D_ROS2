#pragma once

#include <string>

#include "lidarConfig.h"
#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "utils.h"

class LidarNode : public rclcpp::Node {
   public:
    LidarNode();
    ~LidarNode();

    int work();

   private:
    //发布者
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    //启动服务 ..
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startServer;
    //停止服务
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stopServer;

    //雷达配置属性
    LidarConfig *lidar;

    inline float degToRad(float _deg);

    bool startMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool stopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void clean();
};
