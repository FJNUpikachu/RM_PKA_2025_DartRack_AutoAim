#ifndef DART_CAMERA_HIK_CAMERA_NODE_HPP_
#define DART_CAMERA_HIK_CAMERA_NODE_HPP_

// Hik
#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// project
#include "dart_utils/heartbeat.hpp"
// OpenCV (用于图像保存)
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace dart_camera {
class HikCameraNode : public rclcpp::Node {
public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options);
    ~HikCameraNode() override;

private:
    void declareParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    // 保存图像到文件
    void saveImage(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
    
    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;

    int nRet = MV_OK;
    void *camera_handle_;
    MV_IMAGE_BASIC_INFO img_info_;
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;

    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int fail_conut_ = 0;
    std::thread capture_thread_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // Heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_;
    
    // 用于控制只输出一次图像信息的标志位
    bool is_image_info_printed_ = false;

    // 新增：录制相关参数
    bool recording_;                  // 录制开关
    std::string recording_path_;      // 录制存储路径
    uint64_t image_counter_;          // 图像计数器，用于生成唯一文件名
    std::mutex recording_mutex_;      // 线程安全锁
};
} // namespace dart_camera

#endif // DART_CAMERA_HIK_CAMERA_NODE_HPP_
