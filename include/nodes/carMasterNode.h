#pragma once
#include "message/msg/car_encoder_data.hpp"
#include "message/msg/car_motion_control.hpp"
#include "message/msg/car_servo_data.hpp"
#include "message/msg/mode_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial.h"

class TasksManager;
struct Task;

#define MIN_FRAME_LEN 4

typedef enum {
    FRAME_FUNC_ENABLE_AUTO_SEND = 0x01,  // 自动发送
    FRAME_FUNC_BEEP_ONCE = 0x11,         // 蜂鸣器响一声
    FRAME_FUNC_LED_KEEP_ON = 0x21,       // LED 常亮
    FRAME_FUNC_LED_KEEP_OFF = 0x22,      // LED 熄灭
    FRAME_FUNC_LED_FLASH = 0x23,         // LED 闪烁
    FRAME_FUNC_AUTO_SCAN = 0X41,         // 自动扫描
    FRAME_FUNC_MOTION = 0x61,            // 小车运动控制
    FRAME_FUNC_ENCODER = 0X71,           // 编码器

} FrameRequest;

typedef enum {
    FRAME_RESPONSE = 0xFFu,          // 指令应答帧
    FRAME_RESPONSE_VOTAGE = 0xA1u,   // 电压信息帧
    FRAME_RESPONSE_ENCODER = 0xB1u,  // 编码器信息帧
    FRAME_RESPONSE_SERVO = 0xC1u,    // 舵机信息帧

} FrameResponse;

class CarMasterNode : public rclcpp::Node {
   public:
    CarMasterNode();
    ~CarMasterNode();

    /**
     * @description: 进程执行函数
     * @param {TasksManager} tm
     * @param {Task&} task
     */
    int work(TasksManager& tm, Task& task);

    /**
     * @description: 调用该对象初始化的串口发送数据
     * @param {vector<uint8_t>} data 不包含3字节头部，不包含1字节校验和，只需1字节功能字，若干字节数据字节
     */
    void send(std::vector<uint8_t> data);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/carMasterNode";
    // 编码器数据发布者
    rclcpp::Publisher<message::msg::CarEncoderData>::SharedPtr encoderDataPublisher;
    // 舵机数据发布者
    rclcpp::Publisher<message::msg::CarServoData>::SharedPtr servoDataPublisher;

    std::thread executorThread;
    bool executorRunning = false;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    // 模式控制订阅者
    rclcpp::Subscription<message::msg::ModeControl>::SharedPtr modeControlSubscriber;
    // 运动控制订阅者
    rclcpp::Subscription<message::msg::CarMotionControl>::SharedPtr motionControlSubscriber;

    /**
     * @description: 模式控制回调函数，ros话题收到上位机消息则回调该函数，处理发送给stm32板
     * @param {SharedPtr} msg
     */
    void modeControlCallback(const message::msg::ModeControl::SharedPtr msg);

    /**
     * @description: 运动控制回调函数，ros话题收到上位机消息则回调该函数，处理发送给stm32板
     * @param {SharedPtr} msg
     */
    void motionControlCallback(const message::msg::CarMotionControl::SharedPtr msg);

    // 串口
    serial::Serial* serial;
    // 数据接受频率 hz
    int frequency = 100;
    // buffer
    std::vector<uint8_t> rawDataBuffer;
    // 帧头部
    std::vector<uint8_t> frameHeader = {0xFF, 0xCC};

    /**
     * @description: 对stm32板发送的串口数据进行解析
     */
    void frameParser();

    // 用于保存当前运动状态
    bool motionChanged = false;
    uint8_t lastMode = 0, lastState = 0, lastSpeed = 0;
    uint8_t mode = 0;
    uint8_t state = 0;
    uint8_t speed = 0;

    /**
     * @description: 运动控制处理（放在执行函数中周期性处理）
     * @return {*}
     */
    void motionHandler();

    void start();
    void end();
};
