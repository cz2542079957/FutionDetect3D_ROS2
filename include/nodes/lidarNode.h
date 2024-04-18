#pragma once

#include <string>

#include "math.h"
#include "message/msg/lidar_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sl_lidar.h"
#include "sl_types.h"
using namespace sl;

class TasksManager;
struct Task;

#define DEG_TO_RAD_COE (M_PI / 180.0f)
#define GET_ANGLE_COE (90.f / 16384.f)

// 雷达模式
enum LidarMode {
    STANDARD_MODE = 0,  // Standard: 最远扫描距离: 30.0 m, 每秒扫描点数: 16.1K
    DENSE_MODE = 1      // DenseBoost: 最远扫描距离: 30.0 m, 每秒扫描点数: 32.3K
};

extern std::map<LidarMode, std::string> lidarModeNames;

class LidarNode : public rclcpp::Node {
   public:
    static inline float degToRad(float _deg) { return _deg * DEG_TO_RAD_COE; }

    static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node) { return node.angle_z_q14 * GET_ANGLE_COE; }

    LidarNode();
    ~LidarNode();

    // 开始运行
    int work(TasksManager tm, Task& task);

   private:
    // 节点前缀
    std::string nodePrefix = "/lidarNode";
    // 发布者
    rclcpp::Publisher<message::msg::LidarData>::SharedPtr publisher;

    // 雷达驱动
    ILidarDriver* driver;
    // 通信
    IChannel* channel;
    // 模式名称
    std::string modeName = "Standard";
    // 模式id
    int modeId = DENSE_MODE;
    // 最远扫描距离 m
    float maxDistance = 30;
    // 频率 hz
    int frequency = 10;

    void publish(message::msg::LidarData::SharedPtr& lidarData);
    void clean();
};
