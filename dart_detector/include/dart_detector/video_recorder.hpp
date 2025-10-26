#ifndef VIDEO_RECORDER_HPP_
#define VIDEO_RECORDER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace dart_detector {

/**
 * @brief 相机内录管理器，负责将图像话题录制为视频文件
 */
class VideoRecorder {
public:
    /**
     * @brief 构造函数
     * @param node 节点指针
     * @param enable_recording 是否启用录制
     * @param output_path 视频保存路径
     * @param frame_rate 视频帧率
     * @param fourcc_codec 视频编码格式
     */
    VideoRecorder(rclcpp::Node* node,
                 bool enable_recording,
                 const std::string& output_path,
                 double frame_rate,
                 const std::string& fourcc_codec);
    
    ~VideoRecorder();

    /**
     * @brief 处理图像帧，用于录制
     * @param image 要录制的图像
     */
    void processImage(const cv::Mat& image);

private:
    /**
     * @brief 初始化视频写入器
     * @param width 图像宽度
     * @param height 图像高度
     * @return 是否初始化成功
     */
    bool initVideoWriter(int width, int height);

    rclcpp::Node* node_; ///< 节点指针
    bool enable_recording_; ///< 是否启用录制
    std::string output_path_; ///< 视频保存路径
    double frame_rate_; ///< 视频帧率
    std::string fourcc_codec_; ///< 视频编码格式
    
    cv::VideoWriter video_writer_; ///< 视频写入器
    bool is_writer_initialized_; ///< 视频写入器是否已初始化
};

}  // namespace dart_detector

#endif  // VIDEO_RECORDER_HPP_
