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