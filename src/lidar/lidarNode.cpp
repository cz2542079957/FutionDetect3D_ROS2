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
    // lidar = new LidarConfig("DenseBoost");
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
    res = lidar->driver->startScanExpress(false /* 不强制扫描 */, lidar->modeId, 0, &lidar->currentMode);
    if (SL_IS_OK(res)) {
        RCLCPP_INFO(this->get_logger(), "当前扫描模式： %s, 采样率: %d Khz, 最大扫描距离: %.1f m, 扫描频率:%d Hz, ", lidar->currentMode.scan_mode,
                    (int)(1000 / lidar->currentMode.us_per_sample + 0.5), lidar->maxDistance, lidar->frequency);
    } else {
        RCLCPP_ERROR(this->get_logger(), "启动扫描失败: %08x!", res);
    }

    //雷达开始扫描时间
    rclcpp::Time startTime;
    //雷达结束扫描时间
    rclcpp::Time endTime;
    double duration;
    int c = 1;
    while (c--) {
        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];
        startTime = this->now();
        res = lidar->driver->grabScanDataHq(nodes, count);
        endTime = this->now();
        duration = (endTime - startTime).seconds();
        // printf("start: %16d ; end: %16d； duration: %16lf \n ", startTime.nanoseconds(), endTime.nanoseconds(), duration);
        if (res == SL_RESULT_OK) {
            res = lidar->driver->ascendScanData(nodes, count);
            float angle_min = degToRad(0.0f);
            float angle_max = degToRad(359.0f);
            if (res == SL_RESULT_OK) {
                if (needCompensateNodes) {
                    const int compensateNodesCount = 360 * compensateMultiple;
                    int compensateOffset = 0;
                    auto compensateNdes = new sl_lidar_response_measurement_node_hq_t[compensateNodesCount];
                    memset(compensateNdes, 0, compensateNodesCount * sizeof(sl_lidar_response_measurement_node_hq_t));
                    size_t i = 0, j = 0;
                    for (; i < count; i++) {
                        // RCLCPP_INFO(this->get_logger(), "index: %10d, dist：%12d, deg: %12d", i, nodes[i].dist_mm_q2, nodes[i].angle_z_q14);
                        // 统计有效点
                        if (nodes[i].dist_mm_q2 != 0) {
                            // RCLCPP_INFO(this->get_logger(), "index: %10d , dist：%12d, deg: %12d, angle: %10f", i, nodes[i].dist_mm_q2, nodes[i].angle_z_q14,
                            //             angle);
                            float angle = getAngle(nodes[i]);
                            int angleValue = (int)(angle * compensateMultiple);
                            if ((angleValue - compensateOffset) < 0) compensateOffset = angleValue;
                            for (j = 0; j < compensateMultiple; j++) {
                                int compensateNodesIndex = angleValue - compensateOffset + j;
                                if (compensateNodesIndex >= compensateNodesCount) compensateNodesIndex = compensateNodesCount - 1;
                                compensateNdes[compensateNodesIndex] = nodes[i];
                            }
                        }
                    }

                    // for (i = 0; i < compensateNodesCount; i++) {
                    //     RCLCPP_INFO(this->get_logger(), "index: %6d,  dist：%12d, deg: %12d", i, compensateNdes[i].dist_mm_q2,
                    //     compensateNdes[i].angle_z_q14);
                    // }

                    publish(publisher, compensateNdes, compensateNodesCount, startTime, duration, inverted, angle_min, angle_max, lidar->maxDistance, frameId);

                    if (compensateNdes) {
                        delete[] compensateNdes;
                        compensateNdes = nullptr;
                    }
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 10;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0)
                        ;
                    start_node = i - 1;
                    i = count - 1;
                    while (nodes[i--].dist_mm_q2 == 0)
                        ;
                    end_node = i + 1;

                    angle_min = degToRad(getAngle(nodes[start_node]));
                    angle_max = degToRad(getAngle(nodes[end_node]));

                    publish(publisher, &nodes[start_node], end_node - start_node + 1, startTime, duration, inverted, angle_min, angle_max, lidar->maxDistance,
                            frameId);
                }
            } else if (res == SL_RESULT_OPERATION_FAIL) {
                // todo 扫描失败处理
                printf("err\n");
                float angle_min = degToRad(0.0f);
                float angle_max = degToRad(359.0f);
                publish(publisher, nodes, count, startTime, duration, inverted, angle_min, angle_max, lidar->maxDistance, frameId);
            }
        }
        rclcpp::spin_some(shared_from_this());
    }

    lidar->driver->stop();
}

inline float LidarNode::degToRad(float _deg) { return (_deg)*M_PI / 180.0f; }

inline float LidarNode::getAngle(const sl_lidar_response_measurement_node_hq_t& node) { return node.angle_z_q14 * 90.f / 16384.f; }

bool LidarNode::startMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

bool LidarNode::stopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

void LidarNode::publish(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, sl_lidar_response_measurement_node_hq_t* _nodes, size_t _nodeCount,
                        rclcpp::Time _startTime, double _duration, bool inverted, float angle_min, float angle_max, float _maxDistance, std::string frame_id) {
    static int scan_count = 0;
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp = _startTime;
    scan_msg->header.frame_id = frame_id;
    scan_count++;

    //捋顺数据顺序
    bool reversed = (angle_max > angle_min);
    if (reversed) {
        scan_msg->angle_min = M_PI - angle_max;
        scan_msg->angle_max = M_PI - angle_min;
    } else {
        scan_msg->angle_min = M_PI - angle_min;
        scan_msg->angle_max = M_PI - angle_max;
    }
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(_nodeCount - 1);

    scan_msg->scan_time = _duration;
    scan_msg->time_increment = _duration / (double)(_nodeCount - 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = _maxDistance;  // 8.0;

    scan_msg->intensities.resize(_nodeCount);
    scan_msg->ranges.resize(_nodeCount);
    //是否需要反转数据
    bool reverseData = (!inverted && reversed) || (inverted && !reversed);
    //计算后的读数
    float ComputedValue = 0;
    if (!reverseData) {
        for (size_t i = 0; i < _nodeCount; i++) {
            ComputedValue = (float)_nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (ComputedValue == 0.0)
                scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[i] = ComputedValue;
            scan_msg->intensities[i] = (float)(_nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < _nodeCount; i++) {
            ComputedValue = (float)_nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (ComputedValue == 0.0)
                scan_msg->ranges[_nodeCount - 1 - i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[_nodeCount - 1 - i] = ComputedValue;
            scan_msg->intensities[_nodeCount - 1 - i] = (float)(_nodes[i].quality >> 2);
        }
    }
    for (size_t i = 0; i < _nodeCount; i++) {
        printf("range: %4d, %10f\n", i, scan_msg->ranges[i]);
    }
    pub->publish(*scan_msg);
}

void LidarNode::clean() {
    lidar->driver->stop();
    delete lidar;
}
