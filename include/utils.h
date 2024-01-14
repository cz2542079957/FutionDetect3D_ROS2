#pragma once
// ros2 封装工具
#include "rclcpp/rclcpp.hpp"

namespace Tool {

#define PROGRAM_NAME "FusionDetect3D"

// template <typename... Args>
// void logInfo(const std::string& logger, const Args&... args) {
//     RCLCPP_INFO(rclcpp::get_logger(logger), args);
// }
// template <typename... Args>
// void logError(const std::string& logger, const Args&... args) {
//     RCLCPP_ERROR(rclcpp::get_logger(logger), args);
// }
// template <typename... Args>
// void logDebug(const std::string& logger, const Args&... args) {
//     RCLCPP_DEBUG(rclcpp::get_logger(logger), args);
// }
// template <typename... Args>
// void logWarn(const std::string& logger, const Args&... args) {
//     RCLCPP_WARN(rclcpp::get_logger(logger), args);
// }

// //缺省名称
// template <typename... Args>
// void logInfoDefault(const Args&... args) {
//     RCLCPP_INFO(rclcpp::get_logger(PROGRAM_NAME), args);
// }
// template <typename... Args>
// void logErrorDefault(const Args&... args) {
//     RCLCPP_ERROR(rclcpp::get_logger(PROGRAM_NAME), args);
// }
// template <typename... Args>
// void logDebugDefault(const Args&... args) {
//     RCLCPP_DEBUG(rclcpp::get_logger(PROGRAM_NAME), args);
// }
// template <typename... Args>
// void logWarnDefault(const Args&... args) {
//     RCLCPP_WARN(rclcpp::get_logger(PROGRAM_NAME), args);
// }
}  // namespace Tool