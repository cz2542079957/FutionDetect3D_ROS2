#include "lidarNode.h"

LidarNode ::LidarNode() : Node("lidarNode") {
    publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)));
    startServer =
        this->create_service<std_srvs::srv::Empty>("StartLidarMotor", std::bind(&LidarNode::startMotor, this, std::placeholders::_1, std::placeholders::_2));
    stopServer =
        this->create_service<std_srvs::srv::Empty>("stopLidarMotor", std::bind(&LidarNode::stopMotor, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "雷达节点初始化完成");
}

LidarNode ::~LidarNode() {
    clean();
    RCLCPP_INFO(this->get_logger(), "雷达节点销毁");
}

int LidarNode::work() {
    lidar = new LidarConfig();
    sl_result res;
    //连接串口
    if (SL_IS_FAIL(lidar->driver->connect(lidar->channel))) {
        //连接失败
        RCLCPP_ERROR(this->get_logger(), "错误，无法绑定到这个USB端口：%s.", lidar->port.c_str());
        clean();
        return -1;
    }
    //启动扫描
    lidar->driver->setMotorSpeed();
    res = lidar->driver->startScanExpress(false /* 不强制扫描 */, 0, 0, &lidar->currentMode);
    if (SL_IS_OK(res)) {
        RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ",
                    lidar->currentMode.scan_mode, (int)(1000 / lidar->currentMode.us_per_sample + 0.5), lidar->maxDistance, lidar->frequency);
    } else {
        RCLCPP_ERROR(this->get_logger(), "启动扫描失败: %08x!", res);
    }

    //雷达开始扫描时间
    rclcpp::Time startTime;
    //雷达结束扫描时间
    rclcpp::Time endTime;
    double duration;
    int c = 10;
    while (c--) {
        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];

        startTime = this->now();
        res = lidar->driver->grabScanDataHq(nodes, count);
        endTime = this->now();
        duration = (endTime - startTime).seconds();
        printf("start: %16d ; end: %16d； duration: %16lf \n ", startTime.nanoseconds(), endTime.nanoseconds(), duration);

        // if (res == SL_RESULT_OK) {
        //     res = lidar->driver->ascendScanData(nodes, count);
        //     float angle_min = degToRad(0.0f);
        //     float angle_max = degToRad(359.0f);
        //     if (res == SL_RESULT_OK) {
        //         if (angle_compensate) {
        //             // const int angle_compensate_multiple = 1;
        //             const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
        //             int angle_compensate_offset = 0;
        //             auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
        //             memset(angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(sl_lidar_response_measurement_node_hq_t));

        //             size_t i = 0, j = 0;
        //             for (; i < count; i++) {
        //                 if (nodes[i].dist_mm_q2 != 0) {
        //                     float angle = getAngle(nodes[i]);
        //                     int angle_value = (int)(angle * angle_compensate_multiple);
        //                     if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
        //                     for (j = 0; j < angle_compensate_multiple; j++) {
        //                         int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
        //                         if (angle_compensate_nodes_index >= angle_compensate_nodes_count)
        //                             angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
        //                         angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
        //                     }
        //                 }
        //             }

        //             publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count, start_scan_time, scan_duration, inverted, angle_min,
        //             angle_max,
        //                          max_distance, frame_id);

        //             if (angle_compensate_nodes) {
        //                 delete[] angle_compensate_nodes;
        //                 angle_compensate_nodes = nullptr;
        //             }
        //         } else {
        //             int start_node = 0, end_node = 0;
        //             int i = 0;
        //             // find the first valid node and last valid node
        //             while (nodes[i++].dist_mm_q2 == 0)
        //                 ;
        //             start_node = i - 1;
        //             i = count - 1;
        //             while (nodes[i--].dist_mm_q2 == 0)
        //                 ;
        //             end_node = i + 1;

        //             angle_min = DEG2RAD(getAngle(nodes[start_node]));
        //             angle_max = DEG2RAD(getAngle(nodes[end_node]));

        //             publish_scan(scan_pub, &nodes[start_node], end_node - start_node + 1, start_scan_time, scan_duration, inverted, angle_min, angle_max,
        //                          max_distance, frame_id);
        //         }
        //     } else if (op_result == SL_RESULT_OPERATION_FAIL) {
        //         // All the data is invalid, just publish them
        //         float angle_min = DEG2RAD(0.0f);
        //         float angle_max = DEG2RAD(359.0f);
        //         publish_scan(scan_pub, nodes, count, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance, frame_id);
        //     }
        // }

        rclcpp::spin_some(shared_from_this());
    }

    lidar->driver->stop();
}

inline float LidarNode::degToRad(float _deg) { return (_deg)*M_PI / 180.0f; }

bool LidarNode::startMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

bool LidarNode::stopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

void LidarNode::clean() {
    lidar->driver->stop();
    delete lidar;
}
