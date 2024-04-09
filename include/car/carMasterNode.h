#pragma once
#include "buzzerController.h"
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

    int work(TasksManager tm, Task& task);

    /**
     * @description: 调用该对象初始化的串口发送数据
     * @param {vector<uint8_t>} data 不包含3字节头部，不包含1字节校验和，只需1字节功能字，若干字节数据字节
     */
    void send(std::vector<uint8_t> data);

   private:
    // 话题节点前缀
    std::string nodePrefix = "/carMasterNode";
    // 发布者
    //  rclcpp::Publisher<message::msg::ImuData>::SharedPtr publisher;

    // void rawDataHandler(std::vector<uint8_t> arr, int count);

    // void publish(message::msg::ImuData::SharedPtr& imuData);

    // 串口
    serial::Serial* serial;
    // 数据接受频率 hz
    int frequency = 100;
    // buffer
    std::vector<uint8_t> rawDataBuffer;
    // 帧头部
    std::vector<uint8_t> frameHeader = {0xFF, 0xCC};

    // 蜂鸣器控制器
    BuzzerController* buzzerController;

    void frameParser();

    void start();
    void end();
};
