#include "dart_detector/video_recorder.hpp"
#include <ctime>
#include <sys/stat.h>

namespace dart_detector {

VideoRecorder::VideoRecorder(rclcpp::Node* node,
                           bool enable_recording,
                           const std::string& output_path,
                           double frame_rate,
                           const std::string& fourcc_codec)
    : node_(node),
      enable_recording_(enable_recording),
      output_path_(output_path),
      frame_rate_(frame_rate),
      fourcc_codec_(fourcc_codec),
      is_writer_initialized_(false) {
    
    if (!enable_recording_) {
        RCLCPP_INFO(node_->get_logger(), "视频录制功能已禁用");
        return;
    }

    // 检查并创建输出目录
    struct stat info;
    if (stat(output_path_.c_str(), &info) != 0) {
        if (mkdir(output_path_.c_str(), 0755) != 0) {
            RCLCPP_WARN(node_->get_logger(), "无法创建视频保存目录: %s，使用默认目录", 
                       output_path_.c_str());
            output_path_ = "./recordings";
            mkdir(output_path_.c_str(), 0755);
        }
    } else if (!S_ISDIR(info.st_mode)) {
        RCLCPP_WARN(node_->get_logger(), "视频路径不是目录: %s，使用默认目录", 
                   output_path_.c_str());
        output_path_ = "./recordings";
        mkdir(output_path_.c_str(), 0755);
    }

    RCLCPP_INFO(node_->get_logger(), "视频录制已启用，保存路径: %s，帧率: %.1f",
               output_path_.c_str(), frame_rate_);
}

VideoRecorder::~VideoRecorder() {
    if (video_writer_.isOpened()) {
        video_writer_.release();
        RCLCPP_INFO(node_->get_logger(), "视频录制已停止并保存");
    }
}

bool VideoRecorder::initVideoWriter(int width, int height) {
    if (!enable_recording_) return false;
    
    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    struct tm timeinfo;
    localtime_r(&now_time, &timeinfo);
    
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &timeinfo);
    
    std::string filename = std::string(time_str) + ".mp4";
    std::string full_path = output_path_ + "/" + filename;

    // 初始化视频写入器
    int fourcc = cv::VideoWriter::fourcc(
        fourcc_codec_[0], fourcc_codec_[1], 
        fourcc_codec_[2], fourcc_codec_[3]
    );
    
    video_writer_.open(full_path, fourcc, frame_rate_, cv::Size(width, height));
    
    if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(node_->get_logger(), "无法初始化视频写入器，路径: %s", full_path.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "开始录制视频: %s", full_path.c_str());
    is_writer_initialized_ = true;
    return true;
}

void VideoRecorder::processImage(const cv::Mat& image) {
    if (!enable_recording_ || image.empty()) {
        return;
    }

    // 初始化视频写入器（首次调用时）
    if (!is_writer_initialized_) {
        if (!initVideoWriter(image.cols, image.rows)) {
            enable_recording_ = false; // 初始化失败则禁用录制
            return;
        }
    }

    // 写入视频帧
    video_writer_.write(image);
}

}  // namespace dart_detector
