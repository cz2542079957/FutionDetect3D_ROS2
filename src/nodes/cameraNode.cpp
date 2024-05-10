#include "cameraNode.h"

#include "tasksManager.h"

CameraNode::CameraNode() : Node("cameraNode") {
    publisher = this->create_publisher<sensor_msgs::msg::Image>(nodePrefix + "/cameraData", 10);

    subscriber = this->create_subscription<message::msg::ModeControl>(nodePrefix + "/cameraControl", rclcpp::QoS(rclcpp::KeepLast(10)),
                                                                      std::bind(&CameraNode::cameraControlCallback, this, std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点初始化完成");
}
CameraNode::~CameraNode() { RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点销毁"); }

int CameraNode::work(TasksManager& tm, Task& task) {
    cv::Mat frame;
    cv::VideoCapture cap;

    start();
    while (task.running) {
        if (needTakePhoto) {
            needTakePhoto = false;
            cap.open("/dev/camera");
            if (cap.isOpened()) {
                if (cap.read(frame)) {
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
    end();
    return 0;
}

void CameraNode::cameraControlCallback(const message::msg::ModeControl::SharedPtr msg) {
    this->needTakePhoto = true;
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机控制");
}

void CameraNode::start() {
    executorThread = std::thread([this]() {
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(this->shared_from_this());
        executor->spin();
    });
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点开始运行");
}

void CameraNode::end() {
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "正在等待“executor”工作线程退出");
    executor->cancel();
    if (executorThread.joinable()) {
        executorThread.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "“executor”工作线程[退出]");
    RCLCPP_INFO(rclcpp::get_logger("CameraNode"), "相机模块节点终止运行");
}