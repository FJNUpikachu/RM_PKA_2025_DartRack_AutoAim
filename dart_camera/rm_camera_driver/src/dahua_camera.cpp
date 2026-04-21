#include "rm_camera_dahua/dahua_camera.hpp"

// std
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <vector>

// Opencv
#include <opencv2/opencv.hpp>

// project
#include "rm_camera_dahua/Camera_Control.h"
#include "dart_utils/heartbeat.hpp"

// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace cv;
using namespace std;

namespace pka::camera_driver
{

// 相机节点构造函数
Dahua_CameraNode::Dahua_CameraNode(const rclcpp::NodeOptions &options)
: Node("camera_driver", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DahuaCameraNode!");

  // 首先声明所有参数，避免重复声明
  declareParameters();

  // 枚举相机设备
  int VideoCheck_LOSTUM = 0;
  while (!camera.videoCheck() && rclcpp::ok()) 
  {
    VideoCheck_LOSTUM++;
    RCLCPP_WARN(this->get_logger(), "No camera found! Attempt: %d", VideoCheck_LOSTUM);
    if (VideoCheck_LOSTUM >= 5) 
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to find camera after %d attempts!", VideoCheck_LOSTUM);
      rclcpp::shutdown();
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // 连接相机
  int VideoOpen_LOSTUM = 0;
  while (!camera.Connect_Camera() && rclcpp::ok()) 
  {
    VideoOpen_LOSTUM++;
    RCLCPP_WARN(this->get_logger(), "Camera connection failed! Attempt: %d", VideoOpen_LOSTUM);
    if (VideoOpen_LOSTUM >= 3) 
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect camera after %d attempts!", VideoOpen_LOSTUM);
      rclcpp::shutdown();
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // 修改部分：创建可靠的QoS配置
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  
  // 创建相机图片发布者
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  RCLCPP_INFO(this->get_logger(), "Camera publisher QoS: Reliability=RELIABLE, Depth=10");

  // Heartbeat心跳节点创建
  heartbeat_ = HeartBeatPublisher::create(this);

  // 设置相机参数
  setupCamera();

  // 开始拉流
  camera.videoStart();
  camera.startGrabbing();

  // 加载相机信息（使用已声明的参数）
  camera_name_ = this->get_parameter("camera_name").as_string();
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  
  auto camera_info_url = this->get_parameter("camera_info_url").as_string();
  
  if (camera_info_manager_->validateURL(camera_info_url)) 
  {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
    RCLCPP_INFO(this->get_logger(), "Camera calibration loaded from: %s", camera_info_url.c_str());
  } 
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

  // 设置相机信息消息的帧ID（使用已声明的参数）
  camera_info_msg_.header.frame_id = this->get_parameter("camera_frame_id").as_string();

  // 参数设置回调
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Dahua_CameraNode::parametersCallback, this, std::placeholders::_1));

  // 初始化图像消息（使用已声明的参数）
  image_msg_.header.frame_id = this->get_parameter("camera_frame_id").as_string();
  image_msg_.encoding = "rgb8";
  image_msg_.is_bigendian = false;

  // 创建图像捕获线程
  capture_thread_ = std::thread{[this]() -> void {
    RCLCPP_INFO(this->get_logger(), "Starting image capture thread!");

    int fail_count = 0;
    cv::Mat frame;

    while (rclcpp::ok()) 
    {
      // 获取图像帧
      if (camera.getFrame(frame)) 
      {
        if (!frame.empty()) 
        {
          // 检查图像格式
          if (frame.channels() != 3) 
          {
            RCLCPP_WARN(this->get_logger(), "Frame has %d channels, expected 3 for rgb8", frame.channels());
            fail_count++;
            continue;
          }
          
          // 更新图像消息
          image_msg_.header.stamp = this->now();
          image_msg_.height = frame.rows;
          image_msg_.width = frame.cols;
          image_msg_.step = frame.cols * 3;  // RGB8格式，每个像素3字节
          
          // 调整数据缓冲区大小
          size_t data_size = frame.total() * frame.elemSize();
          if (image_msg_.data.size() != data_size) 
          {
            image_msg_.data.resize(data_size);
          }
          
          // 复制图像数据
          memcpy(image_msg_.data.data(), frame.data, data_size);

          // 更新相机信息消息时间戳
          camera_info_msg_.header = image_msg_.header;

          // 发布图像
          camera_pub_.publish(image_msg_, camera_info_msg_);

          fail_count = 0;
        }
        else 
        {
          RCLCPP_WARN(this->get_logger(), "Received empty frame!");
          fail_count++;
        }
      }
      else 
      {
        RCLCPP_WARN(this->get_logger(), "Failed to get frame from camera!");
        fail_count++;
      }

      // 检查失败次数
      if (fail_count > 5) 
      {
        RCLCPP_ERROR(this->get_logger(), "Camera failed after %d consecutive errors!", fail_count);
        
        // 尝试重启相机
        camera.stopGrabbing();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        camera.startGrabbing();
        
        RCLCPP_INFO(this->get_logger(), "Camera restarted!");
        fail_count = 0;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Image capture thread stopped!");
  }};

  RCLCPP_INFO(this->get_logger(), "DahuaCameraNode started successfully!");
}
// 相机节点析构函数
Dahua_CameraNode::~Dahua_CameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DahuaCameraNode!");
  
  if (capture_thread_.joinable()) 
  {
    capture_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Capture thread joined!");
  }
  
  // 停止拉流并断开相机连接
  camera.stopGrabbing();
  camera.Disconnect_Camera();
  
  RCLCPP_INFO(this->get_logger(), "Camera disconnected!");
}

void Dahua_CameraNode::declareParameters()
{
  RCLCPP_INFO(this->get_logger(), "Declaring parameters...");

  // 先声明所有参数
  camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
  
  auto camera_info_url = this->declare_parameter(
    "camera_info_url", 
    "package://dart_bringup/config/dahua_camera_info.yaml"
  );
  RCLCPP_INFO(this->get_logger(), "camera calibration URL: %s", camera_info_url.c_str());

  // 帧ID参数
  auto camera_frame_id = this->declare_parameter("camera_frame_id", "camera_optical_frame");
  RCLCPP_INFO(this->get_logger(), "Camera frame ID: %s", camera_frame_id.c_str());

  // 曝光时间参数
  rcl_interfaces::msg::ParameterDescriptor exposure_desc;
  exposure_desc.description = "Exposure time of the camera in microseconds";
  exposure_desc.floating_point_range.resize(1);
  exposure_desc.floating_point_range[0].from_value = 1.0;
  exposure_desc.floating_point_range[0].to_value = 60000.0;
  exposure_desc.floating_point_range[0].step = 1.0;
  exposure_time_ = this->declare_parameter("exposure_time", 3000.0, exposure_desc);

  // 亮度参数
  rcl_interfaces::msg::ParameterDescriptor brightness_desc;
  brightness_desc.description = "Brightness level";
  brightness_desc.integer_range.resize(1);
  brightness_desc.integer_range[0].from_value = 0;
  brightness_desc.integer_range[0].to_value = 100;
  brightness_desc.integer_range[0].step = 1;
  brightness_ = this->declare_parameter("brightness", 50, brightness_desc);

  // 增益参数
  rcl_interfaces::msg::ParameterDescriptor gain_desc;
  gain_desc.description = "Gain value";
  gain_desc.floating_point_range.resize(1);
  gain_desc.floating_point_range[0].from_value = 0.0;
  gain_desc.floating_point_range[0].to_value = 20.0;
  gain_desc.floating_point_range[0].step = 0.1;
  gain_ = this->declare_parameter("gain", 2.0, gain_desc);

  // 其他参数
  auto_white_balance_ = this->declare_parameter("auto_white_balance", 1);
  frame_rate_ = this->declare_parameter("frame_rate", 100.0);
  resolution_width_ = this->declare_parameter("resolution_width", 800);
  resolution_height_ = this->declare_parameter("resolution_height", 600);
  offset_x_ = this->declare_parameter("offset_x", 0);
  offset_y_ = this->declare_parameter("offset_y", 0);

  RCLCPP_INFO(this->get_logger(), "Parameters declared successfully!");
}

void Dahua_CameraNode::setupCamera()
{
  RCLCPP_INFO(this->get_logger(), "Setting up camera parameters...");

  // 设置相机模式 - 连续拉流
  camera.CameraChangeTrig(Camera_Control::ETrigType::trigContinous);
  RCLCPP_INFO(this->get_logger(), "Set trigger mode to continuous");

  // 设置曝光时间
  if (camera.SetExposeTime(exposure_time_)) 
  {
    RCLCPP_INFO(this->get_logger(), "Exposure time set to: %.1f us", exposure_time_);
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set exposure time!");
  }

  // 设置增益
  if (camera.SetAdjustPlus(gain_)) 
  {
    RCLCPP_INFO(this->get_logger(), "Gain set to: %.1f", gain_);
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set gain!");
  }

  // 设置亮度
  if (camera.setBrightness(brightness_)) 
  {
    RCLCPP_INFO(this->get_logger(), "Brightness set to: %d", brightness_);
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set brightness!");
  }

  // 设置ROI
  if (camera.setROI(offset_x_, offset_y_, resolution_width_, resolution_height_)) 
  {
    RCLCPP_INFO(this->get_logger(), "ROI set to: %dx%d at (%d, %d)", 
                resolution_width_, resolution_height_, offset_x_, offset_y_);
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set ROI!");
  }

  // 设置自动白平衡
  if (camera.autoBalance(auto_white_balance_)) 
  {
    if (auto_white_balance_ == 1) 
    {
      RCLCPP_INFO(this->get_logger(), "Auto white balance enabled");
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Auto white balance disabled");
    }
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set auto white balance!");
  }

  // 设置帧率
  if (camera.setFrameRate(frame_rate_)) 
  {
    RCLCPP_INFO(this->get_logger(), "Frame rate set to: %.1f fps", frame_rate_);
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to set frame rate!");
  }

  RCLCPP_INFO(this->get_logger(), "Camera parameters set successfully!");
}

rcl_interfaces::msg::SetParametersResult Dahua_CameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto &param : parameters) 
  {
    if (param.get_name() == "exposure_time") 
    {
      try 
      {
        double new_exposure = param.as_double();
        if (camera.SetExposeTime(new_exposure)) 
        {
          exposure_time_ = new_exposure;
          RCLCPP_INFO(this->get_logger(), "Exposure time set to: %.1f us", exposure_time_);
        }
        else 
        {
          result.successful = false;
          result.reason = "Failed to set exposure time";
        }
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException &e) 
      {
        result.successful = false;
        result.reason = "Invalid parameter type for exposure_time";
      }
    } 
    else if (param.get_name() == "gain") 
    {
      try 
      {
        double new_gain = param.as_double();
        if (camera.SetAdjustPlus(new_gain)) 
        {
          gain_ = new_gain;
          RCLCPP_INFO(this->get_logger(), "Gain set to: %.1f", gain_);
        }
        else 
        {
          result.successful = false;
          result.reason = "Failed to set gain";
        }
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException &e) 
      {
        result.successful = false;
        result.reason = "Invalid parameter type for gain";
      }
    } 
    else if (param.get_name() == "brightness") 
    {
      try 
      {
        int new_brightness = param.as_int();
        if (camera.setBrightness(new_brightness)) 
        {
          brightness_ = new_brightness;
          RCLCPP_INFO(this->get_logger(), "Brightness set to: %d", brightness_);
        }
        else 
        {
          result.successful = false;
          result.reason = "Failed to set brightness";
        }
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException &e) 
      {
        result.successful = false;
        result.reason = "Invalid parameter type for brightness";
      }
    } 
    else 
    {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }

  return result;
}

}  // namespace pka::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::camera_driver::Dahua_CameraNode)