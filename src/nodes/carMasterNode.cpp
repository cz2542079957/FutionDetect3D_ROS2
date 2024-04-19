#include "carMasterNode.h"

#include "tasksManager.h"

CarMasterNode::CarMasterNode() : Node("carMasterNode") {
    encoderDataPublisher = this->create_publisher<message::msg::CarEncoderData>(nodePrefix + "/encoderData", rclcpp::QoS(rclcpp::KeepLast(10)));
    servoDataPublisher = this->create_publisher<message::msg::CarServoData>(nodePrefix + "/servoData", rclcpp::QoS(rclcpp::KeepLast(10)));
    modeControlSubscriber = this->create_subscription<message::msg::ModeControl>(nodePrefix + "/modeControl", rclcpp::QoS(rclcpp::KeepLast(10)),
                                                                                 std::bind(&CarMasterNode::modeControlCallback, this, std::placeholders::_1));
    motionControlSubscriber = this->create_subscription<message::msg::CarMotionControl>(
        nodePrefix + "/motionControl", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&CarMasterNode::motionControlCallback, this, std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点初始化完成");
}

CarMasterNode::~CarMasterNode() {
    delete serial;
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点销毁");
}

int CarMasterNode::work(TasksManager tm, Task& task) {
    serial = new serial::Serial(task.deviceInfo.node, task.deviceInfo.baudRate);
    if (!serial->isOpen()) serial->open();
    start();
    uint64_t tick = 0;
    while (task.running) {
        // 读取数据
        unsigned long count = serial->available();
        if (count != 0) {
            // 读取这些数据
            std::vector<uint8_t> arr;
            size_t realCount = serial->read(arr, count);
            // std::cout << "数量：" << realCount << std::endl;
            // for (int i = 0; i < realCount; i++) {
            //     printf("%02X ", static_cast<int>(arr[i]));
            // }
            rawDataBuffer.insert(rawDataBuffer.end(), arr.begin(), arr.end());
        }
        // 解析数据
        if (rawDataBuffer.size() > 0) {
            frameParser();
        }
        // 处理和发送控制指令
        if (tick % 40 == 0) motionHandler();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        tick++;
    }
    end();
    return 0;
}

void CarMasterNode::send(std::vector<uint8_t> data) {
    if (serial->isOpen()) {
        // 计算校验和
        uint8_t checkSum = std::accumulate(data.begin() + 3, data.end(), 0);
        checkSum = checkSum & 0xff;
        // 将校验和添加到数据包的末尾
        data.push_back(checkSum);
        // 插入帧头
        data.insert(data.begin(), data.size());
        data.insert(data.begin(), 0xCC);
        data.insert(data.begin(), 0xFF);
        // 发送帧
        serial->write(data);
    }
}

void CarMasterNode::modeControlCallback(const message::msg::ModeControl::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "当前模式：%d\n", msg->mode);
    if (mode != msg->mode) mode = msg->mode;
    motionChanged = true;
}

void CarMasterNode::motionControlCallback(const message::msg::CarMotionControl::SharedPtr msg) {
    // RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "运动控制：%d, %d", msg->state, msg->speed);
    state = msg->state;
    speed = msg->speed;
    motionChanged = true;
}

void CarMasterNode::frameParser() {
    // RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "%d", rawDataBuffer.size());
    // 去除损坏字节
    while (rawDataBuffer.size() > 0) {
        if (rawDataBuffer[0] == 0xFF && rawDataBuffer[1] == 0xCC) {
            break;
        }
        rawDataBuffer.erase(rawDataBuffer.begin());
    }
    // 开始处理数据
    while (rawDataBuffer.size() > MIN_FRAME_LEN) {
        if (rawDataBuffer[0] == 0xFF && rawDataBuffer[1] == 0xCC) {
            // 获取帧长度
            uint8_t frameLen = rawDataBuffer[2];
            if (frameLen + 3 > rawDataBuffer.size()) {
                break;
            }
            // 获取校验和
            uint8_t checkSum = rawDataBuffer[frameLen + 2];
            // 计算校验和
            uint8_t realCheckSum = std::accumulate(rawDataBuffer.begin() + 3, rawDataBuffer.begin() + frameLen + 2, 0);
            realCheckSum = realCheckSum & 0xff;
            // 校验和正确
            if (checkSum == realCheckSum) {
                uint8_t function = rawDataBuffer[3];
                switch (function) {
                    case FRAME_RESPONSE: {
                    }
                    case FRAME_RESPONSE_VOTAGE: {
                        float voltage = (rawDataBuffer[4] | (rawDataBuffer[5] << 8)) / 1000.0;
                        RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "电压：%.2f", voltage);
                        break;
                    }
                    case FRAME_RESPONSE_ENCODER: {
                        // 编码器（带符号）
                        auto message = std::make_shared<message::msg::CarEncoderData>();
                        message->timestemp = this->now().nanoseconds() / 1000000;
                        message->encoder1 = rawDataBuffer[4];
                        message->encoder2 = rawDataBuffer[5];
                        message->encoder3 = rawDataBuffer[6];
                        message->encoder4 = rawDataBuffer[7]; 
                        // RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "编码器：%ld, %d, %d, %d, %d", now.nanoseconds() / 1000000, encoder1, encoder2,
                        //             encoder3, encoder4);
                        encoderDataPublisher->publish(*message);
                        break;
                    }
                }
            }
            // 清理
            rawDataBuffer.erase(rawDataBuffer.begin(), rawDataBuffer.begin() + frameLen + 3);
        } else {
            break;
        }
    }
}

void CarMasterNode::motionHandler() {
    if (mode == 0) {
        // 静止模式
        state = 0;
        speed = 0;
    } else if (mode == 1) {
        // 扫描模式
        if (speed > 30) speed = 30;  // 限制速度

    } else if (mode == 2) {
        // 运动模式
    }

    // 没有接收到控制信号，设置停止
    if (!motionChanged) {
        state = 0, speed = 0;
    }

    // 发送数据
    if (lastState != state || lastSpeed != speed) {
        RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "模式：%d, 状态: %d, 速度：%d", mode, state, speed);
        send(std::vector<uint8_t>{0x61, state, speed});
    }
    lastMode = mode, lastState = state, lastSpeed = speed;
    motionChanged = false;
}

void CarMasterNode::start() {
    executorThread = std::thread([this]() {
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(this->shared_from_this());
        executor->spin();
    });
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点开始运行");
}

void CarMasterNode::end() {
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "正在等待“executor”工作线程退出");
    executor->cancel();
    if (executorThread.joinable()) {
        executorThread.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "“executor”工作线程[退出]");
    RCLCPP_INFO(rclcpp::get_logger("CarMasterNode"), "小车控制板节点终止运行");
}
