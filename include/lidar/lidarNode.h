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

    std::string nodePrefix = "/lidarNode";
    //雷达配置属性
    LidarConfig* lidar;
    //是否修正数据
    bool needCompensateNodes = true;
    //修正点步长（每一度的修正单位数，越大越精准）
    int compensateMultiple = 1;
    bool inverted = false;
    std::string frameId = "laserFrame";

    inline float degToRad(float _deg);
    inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node);
    bool startMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool stopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void publish(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, sl_lidar_response_measurement_node_hq_t* nodes, size_t node_count,
                 rclcpp::Time start, double scan_time, bool inverted, float angle_min, float angle_max, float max_distance, std::string frame_id);
    void clean();
};
