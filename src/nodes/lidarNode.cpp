#include "lidarNode.h"

#include "tasksManager.h"

LidarNode::LidarNode() : Node("lidarNode") {
    publisher = this->create_publisher<message::msg::LidarData>(nodePrefix + "/lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "雷达节点初始化完成");
}

LidarNode ::~LidarNode() {
    clean();
    RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "激光雷达节点销毁");
}

std::map<LidarMode, std::string> lidarModeNames = {{STANDARD_MODE, "Standard"}, {DENSE_MODE, "DenseBoost"}};

int LidarNode::work(TasksManager& tm, Task& task) {
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
    // 设置电机速度
    driver->setMotorSpeed();

    // 启动扫描
    if (tm.getMode() == 1 && start() != 0) {
        return -1;
    }

    // 雷达开始扫描时间
    rclcpp::Time startTime;
    // 雷达结束扫描时间
    rclcpp::Time endTime;
    // 扫描时间间隔
    int64_t duration;
    // 待发送的ros雷达数据
    auto lidarData = std::make_shared<message::msg::LidarData>();
    while (task.running) {
        if (tm.getMode() != 1) {
            stop();
            while (tm.getMode() != 1 && task.running) {
                // 等待雷达模式切换到扫描模式
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (task.running) {
                // 如果任务还在进行，则开始扫描
                start();
            } else {
                break;
            }
        }

        size_t count = 8192;
        sl_lidar_response_measurement_node_hq_t nodes[count];
        startTime = this->now();
        res = driver->grabScanDataHq(nodes, count);
        endTime = this->now();
        duration = (endTime - startTime).nanoseconds();
        if (res != SL_RESULT_OK) {
            continue;
        }
        // 开始时间结束时间
        // RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "雷达扫描时间：%d, 开始时间: %d, 结束时间: %d", duration / 1000000, startTime.nanoseconds() / 1000000,
        //             endTime.nanoseconds() / 1000000);
        // RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "100  angle: %d, distance: %d", nodes[100].angle_z_q14, nodes[100].dist_mm_q2);
        // RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "200 angle: %d, distance: %d", nodes[200].angle_z_q14, nodes[200].dist_mm_q2);
        // RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "400 angle: %d, distance: %d", nodes[400].angle_z_q14, nodes[400].dist_mm_q2);

        // 让数据按照角度值升序排列
        res = driver->ascendScanData(nodes, count);
        if (res != SL_RESULT_OK) {
            continue;
        }

        // 计算每个点的平均时间间隔（ns）
        double timeInterval = (duration / count);
        lidarData->data.resize(count);
        lidarData->start_time = startTime.nanoseconds();
        lidarData->duration = duration;
        // RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "个数：%d", count);
        for (int i = 0; i < count; i++) {
            lidarData->data[i].timestemp = startTime.nanoseconds() + (i * timeInterval);
            lidarData->data[i].angle = getAngle(nodes[i]);
            lidarData->data[i].distance = nodes[i].dist_mm_q2 / 4000.f;
            // if (i % 100 == 0)
            //     RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "第%d个点，距离：%.2f，角度：%.2f，时间：%ld", i, lidarData->data[i].distance,
            //             lidarData->data[i].angle, lidarData->data[i].timestemp);
        }
        publish(lidarData);
    }
    stop();
    if (driver->isConnected()) driver->disconnect();
    return 0;
}

int LidarNode::start() {
    LidarScanMode currentMode;  // 当前扫描模式
    sl_result res = driver->startScanExpress(false, DENSE_MODE, 0, &currentMode);
    if (SL_IS_OK(res)) {
        // 电机预启动
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "当前扫描模式： %s, 采样率: %d Khz, 最大扫描距离: %.1f m, 扫描频率:%d Hz, ", currentMode.scan_mode,
                    (int)(1000 / currentMode.us_per_sample + 0.5), maxDistance, frequency);
        return 0;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("LidarNode"), "启动扫描失败: %08x!", res);
        return -1;
    }
}

int LidarNode::stop() {
    driver->stop();
    RCLCPP_INFO(rclcpp::get_logger("LidarNode"), "激光雷达停止扫描");
    return 0;
}

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