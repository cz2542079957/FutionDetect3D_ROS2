#include "lidarNode.h"

#include "tasksManager.h"

LidarNode::LidarNode() : Node("lidarNode") {
    publisher = this->create_publisher<message::msg::LidarData>(nodePrefix + "/lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)));
    startServer = this->create_service<std_srvs::srv::Empty>(nodePrefix + "/startLidarMotor",
                                                             std::bind(&LidarNode::startMotor, this, std::placeholders::_1, std::placeholders::_2));
    stopServer = this->create_service<std_srvs::srv::Empty>(nodePrefix + "/stopLidarMotor",
                                                            std::bind(&LidarNode::stopMotor, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "雷达节点初始化完成");
}

LidarNode ::~LidarNode() {
    clean();
    RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "激光雷达节点销毁");
}

std::map<LidarMode, std::string> lidarModeNames = {{STANDARD_MODE, "Standard"}, {DENSE_MODE, "DenseBoost"}};

int LidarNode::work(TasksManager tm, Task& task) {
    driver = *createLidarDriver();                                                       // 创建雷达驱动
    channel = *createSerialPortChannel(task.deviceInfo.node, task.deviceInfo.baudRate);  // 创建串口通道
    sl_result res;
    // 连接串口
    if (SL_IS_FAIL(driver->connect(channel))) {
        // 连接失败
        RCLCPP_ERROR(rclcpp::get_logger("LidarNode"), "错误，无法绑定到这个端口：%s.", task.deviceInfo.node.c_str());
        clean();
        return -1;
    }
    // 启动扫描
    LidarScanMode currentMode;  // 当前扫描模式
    driver->setMotorSpeed();    // 启动电机
    res = driver->startScanExpress(false /* 不强制扫描 */, DENSE_MODE, 0, &currentMode);
    if (SL_IS_OK(res)) {
        RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "当前扫描模式： %s, 采样率: %d Khz, 最大扫描距离: %.1f m, 扫描频率:%d Hz, ", currentMode.scan_mode,
                    (int)(1000 / currentMode.us_per_sample + 0.5), maxDistance, frequency);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("LidarNode"), "启动扫描失败: %08x!", res);
    }

    // 雷达开始扫描时间
    rclcpp::Time startTime;
    // 雷达结束扫描时间
    rclcpp::Time endTime;
    // 扫描时间间隔
    int64_t duration;
    // 电机预启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (task.running) {
        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];
        startTime = this->now();
        res = driver->grabScanDataHq(nodes, count);
        endTime = this->now();
        duration = (endTime - startTime).nanoseconds();
        if (res != SL_RESULT_OK) {
            continue;
        }

        // 让数据按照角度值升序排列
        res = driver->ascendScanData(nodes, count);
        if (res != SL_RESULT_OK) {
            continue;
        }

        // 计算每个点的平均事件间隔（ns）
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
    driver->stop();
    if (driver->isConnected()) driver->disconnect();
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
    driver->stop();
    delete driver;
    delete channel;
}