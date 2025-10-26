#ifndef SCREENSHOT_MANAGER_HPP_
#define SCREENSHOT_MANAGER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "opencv2/opencv.hpp"

namespace dart_detector {

/**
 * @brief 截图管理器类，负责处理手动和自动截图功能
 */
class ScreenshotManager {
public:
    /**
     * @brief 构造函数
     * @param node 节点指针，用于创建服务和定时器
     * @param screenshot_path 截图保存路径
     * @param enable_auto_screenshot 是否启用自动截图
     * @param screenshot_interval 自动截图间隔(秒)
     * @param save_raw_image 是否保存原始图像
     * @param save_processed_image 是否保存处理后图像
     */
    ScreenshotManager(rclcpp::Node* node,
                     const std::string& screenshot_path,
                     bool enable_auto_screenshot,
                     double screenshot_interval,
                     bool save_raw_image,
                     bool save_processed_image);
    
    ~ScreenshotManager() = default;

    /**
     * @brief 设置最新图像，用于截图
     * @param raw_image 原始图像
     * @param processed_image 处理后图像
     */
    void setLatestImages(const cv::Mat& raw_image, const cv::Mat& processed_image);

private:
    /**
     * @brief 保存截图
     */
    void saveScreenshot();

    /**
     * @brief 手动截图服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     * @return 是否处理成功
     */
    bool takeScreenshotCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief 自动截图定时器回调函数
     */
    void autoScreenshotTimerCallback();

    rclcpp::Node* node_; ///< 节点指针
    std::string screenshot_path_; ///< 截图保存路径
    bool save_raw_image_; ///< 是否保存原始图像
    bool save_processed_image_; ///< 是否保存处理后图像
    
    cv::Mat latest_raw_image_; ///< 最新原始图像
    cv::Mat latest_processed_image_; ///< 最新处理后图像
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr screenshot_service_; ///< 手动截图服务
    rclcpp::TimerBase::SharedPtr auto_screenshot_timer_; ///< 自动截图定时器
};

}  // namespace dart_detector

#endif  // SCREENSHOT_MANAGER_HPP_
