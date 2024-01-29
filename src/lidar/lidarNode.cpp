#include "lidarNode.h"

#include "tasksManager.h"

LidarNode::LidarNode() : Node("lidarNode") {
    publisher = this->create_publisher<message::msg::LidarData>(nodePrefix + "/lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)));
    startServer = this->create_service<std_srvs::srv::Empty>(nodePrefix + "/startLidarMotor",
                                                             std::bind(&LidarNode::startMotor, this, std::placeholders::_1, std::placeholders::_2));
    stopServer = this->create_service<std_srvs::srv::Empty>(nodePrefix + "/stopLidarMotor",
                                                            std::bind(&LidarNode::stopMotor, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "雷达节点初始化完成");
}

LidarNode ::~LidarNode() {
    clean();
    RCLCPP_INFO(this->get_logger(), "激光雷达节点销毁");
}
/*
int LidarNode::work(TasksManager tm, Task& task) {
    lidar = new LidarConfig("DenseBoost");
    lidar->port = task.deviceInfo.node;
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
    res = lidar->driver->startScanExpress(false, lidar->modeId, 0, &lidar->currentMode);
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
    int c = 100;
    while (c--) {
        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];

        startTime = this->now();
        // res = lidar->driver->grabScanDataHq(nodes, count);
        size_t timestemp;
        res = lidar->driver->grabScanDataHqWithTimeStamp(nodes, count, timestemp);
        endTime = this->now();
        // printf("time: %zu\n", timestemp);
        duration = (endTime - startTime).seconds();
        // printf("start: %16d ; end: %16d； duration: %16lf \n ", startTime.nanoseconds(), endTime.nanoseconds(), duration);
        if (res == SL_RESULT_OK) {
            res = lidar->driver->ascendScanData(nodes, count);
            RCLCPP_INFO(get_logger(), "%d ", count);
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
                        // RCLCPP_INFO(this->get_logger(), "index: %10d, dist：%12d, deg: %12f", i, nodes[i].dist_mm_q2, getAngle(nodes[i]));
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
*/

int LidarNode::work(TasksManager tm, Task& task) {
    lidar = new LidarConfig(task.deviceInfo.node, "DenseBoost");
    sl_result res;
    //连接串口
    if (SL_IS_FAIL(lidar->driver->connect(lidar->channel))) {
        //连接失败
        RCLCPP_ERROR(rclcpp::get_logger("LidarNode"), "错误，无法绑定到这个USB端口：%s.", lidar->port.c_str());
        clean();
        return -1;
    }
    //启动扫描
    lidar->driver->setMotorSpeed();
    res = lidar->driver->startScanExpress(false /* 不强制扫描 */, lidar->modeId, 0, &lidar->currentMode);
    if (SL_IS_OK(res)) {
        RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "当前扫描模式： %s, 采样率: %d Khz, 最大扫描距离: %.1f m, 扫描频率:%d Hz, ", lidar->currentMode.scan_mode,
                    (int)(1000 / lidar->currentMode.us_per_sample + 0.5), lidar->maxDistance, lidar->frequency);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("LidarNode"), "启动扫描失败: %08x!", res);
    }

    //雷达开始扫描时间
    rclcpp::Time startTime;
    //雷达结束扫描时间
    rclcpp::Time endTime;
    //扫描时间间隔
    int64_t duration;

    //电机预启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (task.running) {
        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];
        startTime = this->now();
        res = lidar->driver->grabScanDataHq(nodes, count);
        endTime = this->now();
        duration = (endTime - startTime).nanoseconds();
        if (res != SL_RESULT_OK) {
            continue;
        }

        //让数据按照角度值升序排列
        res = lidar->driver->ascendScanData(nodes, count);
        if (res != SL_RESULT_OK) {
            continue;
        }

        //计算每个点的平均事件间隔（ns）
        double timeInterval = (duration / count);

        auto lidarData = std::make_shared<message::msg::LidarData>();
        lidarData->data.resize(count);
        lidarData->start_time = startTime.nanoseconds();
        lidarData->duration = duration;
        // RCLCPP_INFO_STREAM(this->get_logger(), count);
        for (int i = 0; i < count; i++) {
            lidarData->data[i].timestemp = startTime.nanoseconds() + (i * timeInterval);
            lidarData->data[i].angle = getAngle(nodes[i]);
            lidarData->data[i].distance = nodes[i].dist_mm_q2 / 4000.f;
        }
        publish(lidarData);
    }
    lidar->driver->stop();
    if (lidar->driver->isConnected()) lidar->driver->disconnect();
}

bool LidarNode::startMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

bool LidarNode::stopMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { return false; }

void LidarNode::publish(message::msg::LidarData::SharedPtr& lidarData) {
    if (!publisher) {
        return;
    }
    publisher->publish(*lidarData);
}

void LidarNode::clean() {
    lidar->driver->stop();
    delete lidar;
}