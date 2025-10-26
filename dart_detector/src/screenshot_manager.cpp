#include "dart_detector/screenshot_manager.hpp"
#include <ctime>
#include <sys/stat.h>
#include <iomanip>

namespace dart_detector {

ScreenshotManager::ScreenshotManager(rclcpp::Node* node,
                                     const std::string& screenshot_path,
                                     bool enable_auto_screenshot,
                                     double screenshot_interval,
                                     bool save_raw_image,
                                     bool save_processed_image)
    : node_(node),
      screenshot_path_(screenshot_path),
      save_raw_image_(save_raw_image),
      save_processed_image_(save_processed_image) {
    
    // 创建截图服务
    screenshot_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "take_screenshot",
        std::bind(&ScreenshotManager::takeScreenshotCallback, this, 
                 std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(node_->get_logger(), "已创建截图服务: take_screenshot");

    // 初始化自动截图定时器
    if (enable_auto_screenshot) {
        double interval = std::max(screenshot_interval, 1.0); // 确保最小间隔为1秒
        auto_screenshot_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(interval),
            std::bind(&ScreenshotManager::autoScreenshotTimerCallback, this)
        );
        RCLCPP_INFO(node_->get_logger(), "自动截图已启用，间隔: %.1f秒", interval);
    }

    // 检查并创建截图目录
    struct stat info;
    if (stat(screenshot_path_.c_str(), &info) != 0) {
        if (mkdir(screenshot_path_.c_str(), 0755) != 0) {
            RCLCPP_WARN(node_->get_logger(), "无法创建截图目录: %s，使用默认目录", 
                       screenshot_path_.c_str());
            screenshot_path_ = "./screenshots";
            mkdir(screenshot_path_.c_str(), 0755);
        }
    } else if (!S_ISDIR(info.st_mode)) {
        RCLCPP_WARN(node_->get_logger(), "截图路径不是目录: %s，使用默认目录", 
                   screenshot_path_.c_str());
        screenshot_path_ = "./screenshots";
        mkdir(screenshot_path_.c_str(), 0755);
    }
}

void ScreenshotManager::setLatestImages(const cv::Mat& raw_image, const cv::Mat& processed_image) {
    if (!raw_image.empty()) {
        latest_raw_image_ = raw_image.clone();
    }
    if (!processed_image.empty()) {
        latest_processed_image_ = processed_image.clone();
    }
}

void ScreenshotManager::saveScreenshot() {
    if (latest_raw_image_.empty() && latest_processed_image_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "没有可用图像进行截图");
        return;
    }

    // 获取当前时间作为文件名
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    struct tm timeinfo;
    localtime_r(&now_time, &timeinfo);
    
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &timeinfo);

    // 保存原始图像
    if (save_raw_image_ && !latest_raw_image_.empty()) {
        std::string filename = std::string(time_str) + "_raw.png";
        std::string full_path = screenshot_path_ + "/" + filename;
        if (cv::imwrite(full_path, latest_raw_image_)) {
            RCLCPP_INFO(node_->get_logger(), "已保存原始图像: %s", full_path.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "无法保存原始图像: %s", full_path.c_str());
        }
    }

    // 保存处理后图像
    if (save_processed_image_ && !latest_processed_image_.empty()) {
        std::string filename = std::string(time_str) + "_processed.png";
        std::string full_path = screenshot_path_ + "/" + filename;
        if (cv::imwrite(full_path, latest_processed_image_)) {
            RCLCPP_INFO(node_->get_logger(), "已保存处理后图像: %s", full_path.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "无法保存处理后图像: %s", full_path.c_str());
        }
    }
}

bool ScreenshotManager::takeScreenshotCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(node_->get_logger(), "收到手动截图请求");
    saveScreenshot();
    response->success = true;
    response->message = "截图已保存";
    return true;
}

void ScreenshotManager::autoScreenshotTimerCallback() {
    RCLCPP_INFO(node_->get_logger(), "自动截图触发");
    saveScreenshot();
}

}  // namespace dart_detector
