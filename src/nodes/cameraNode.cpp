#include "cameraNode.h"

#include "tasksManager.h"

CameraNode::CameraNode() : Node("cameraNode") {
    publisher = this->create_publisher<sensor_msgs::msg::Image>("/cameraData", 10);

    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点初始化完成");
}
CameraNode::~CameraNode() { RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点销毁"); }

int CameraNode::work(TasksManager& tm, Task& task) {
    cv::Mat frame;
    cv::VideoCapture cap;

    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点开始运行");
    while (task.running) {
        if (needTakePhoto) {
            needTakePhoto = false;
            cap.open("/dev/camera");
            if (cap.isOpened()) {
                cap.read(frame);
                if (imwrite("./capture.jpg", frame)) {
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    publisher->publish(*msg);
                };
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("CameraNode"), "相机模块节点读取图片失败");
            }
            cap.release();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点终止运行");
    return 0;
}
