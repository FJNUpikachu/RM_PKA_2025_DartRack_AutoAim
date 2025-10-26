#include "dart_camera/hik_camera_node.hpp"
#include <rclcpp/utilities.hpp>
#include <thread>
#include <filesystem>
#include <iomanip>

namespace dart_camera {
namespace fs = std::filesystem;

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options) : Node("light_camera", options),
    image_counter_(0) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);

    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;

    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    RCLCPP_INFO(this->get_logger(), "Created camera publisher: image_raw and camera_info topics will be published");

    heartbeat_ = HeartBeatPublisher::create(this);

    declareParameters();  // 声明参数（包括新增的日志开关）

    std::lock_guard<std::mutex> lock(recording_mutex_);
    if (!recording_path_.empty() && !fs::exists(recording_path_)) {
        try {
            fs::create_directories(recording_path_);
            RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", recording_path_.c_str());
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create recording directory: %s", e.what());
            recording_ = false;
        }
    }

    MV_CC_StartGrabbing(camera_handle_);

    // 加载相机内参
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
        this->declare_parameter("camera_info_url", "package://light_bringup/config/camera_info.yaml");
    
    // 根据开关控制内参日志输出
    if (camera_info_manager_->validateURL(camera_info_url)) {
        if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
            camera_info_msg_ = camera_info_manager_->getCameraInfo();
            
            // 新增：仅当开关开启时才输出详细内参信息
            if (enable_camera_info_log_) {
                RCLCPP_INFO(this->get_logger(), "Successfully loaded camera info from: %s", camera_info_url.c_str());
                RCLCPP_INFO(this->get_logger(), "Camera intrinsic parameters:");
                RCLCPP_INFO(this->get_logger(), "fx: %.2f, fy: %.2f", camera_info_msg_.k[0], camera_info_msg_.k[4]);
                RCLCPP_INFO(this->get_logger(), "cx: %.2f, cy: %.2f", camera_info_msg_.k[2], camera_info_msg_.k[5]);
                RCLCPP_INFO(this->get_logger(), "Distortion coefficients: [%f, %f, %f, %f, %f]",
                          camera_info_msg_.d[0], camera_info_msg_.d[1], 
                          camera_info_msg_.d[2], camera_info_msg_.d[3], 
                          camera_info_msg_.d[4]);
            } else {
                RCLCPP_INFO(this->get_logger(), "Camera info loaded from: %s (detailed log disabled)", camera_info_url.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load camera info from URL: %s", camera_info_url.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT out_frame;

        RCLCPP_INFO(this->get_logger(), "Starting image capture loop");

        image_msg_.header.frame_id = "camera_optical_frame";
        image_msg_.encoding = "rgb8";

        while (rclcpp::ok()) {
            nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
            if (MV_OK == nRet) {
                if (!is_image_info_printed_) {
                    RCLCPP_INFO(this->get_logger(), "Image width: %d pixels, height: %d pixels",
                              out_frame.stFrameInfo.nWidth, out_frame.stFrameInfo.nHeight);
                    is_image_info_printed_ = true;
                }
                
                convert_param_.pDstBuffer = image_msg_.data.data();
                convert_param_.nDstBufferSize = image_msg_.data.size();
                convert_param_.pSrcData = out_frame.pBufAddr;
                convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
                convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

                MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

                image_msg_.header.stamp = this->now();
                image_msg_.height = out_frame.stFrameInfo.nHeight;
                image_msg_.width = out_frame.stFrameInfo.nWidth;
                image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
                image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

                camera_info_msg_.header = image_msg_.header;
                camera_pub_.publish(image_msg_, camera_info_msg_);

                std::lock_guard<std::mutex> lock(recording_mutex_);
                if (recording_) {
                    saveImage(std::make_shared<sensor_msgs::msg::Image>(image_msg_));
                }

                MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                fail_conut_ = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_StartGrabbing(camera_handle_);
                fail_conut_++;
            }

            if (fail_conut_ > 5) {
                RCLCPP_FATAL(this->get_logger(), "Camera failed!");
                rclcpp::shutdown();
            }
        }
    }};
}

HikCameraNode::~HikCameraNode() override {
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

void HikCameraNode::declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    
    // 曝光时间
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 11000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // 增益
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", 10.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // 录制参数
    param_desc.description = "Whether to record images";
    param_desc.integer_range.clear();
    recording_ = this->declare_parameter("recording", false, param_desc);

    param_desc.description = "Path to save recorded images";
    recording_path_ = this->declare_parameter("recording_path", "./recorded_images", param_desc);
    RCLCPP_INFO(this->get_logger(), "Recording path set to: %s", recording_path_.c_str());

    // 新增：内参日志开关参数
    param_desc.description = "Enable detailed camera info logging (fx, fy, cx, cy, distortion coefficients)";
    enable_camera_info_log_ = this->declare_parameter("enable_camera_info_log", true, param_desc);
    RCLCPP_INFO(this->get_logger(), "Camera info detailed logging: %s", 
                enable_camera_info_log_ ? "enabled" : "disabled");
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
        if (param.get_name() == "exposure_time") {
            int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
            if (MV_OK != status) {
                result.successful = false;
                result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "gain") {
            int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
            if (MV_OK != status) {
                result.successful = false;
                result.reason = "Failed to set gain, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "recording") {
            std::lock_guard<std::mutex> lock(recording_mutex_);
            recording_ = param.as_bool();
            if (recording_) {
                RCLCPP_INFO(this->get_logger(), "Started recording images to: %s", recording_path_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Stopped recording images");
            }
        } else if (param.get_name() == "recording_path") {
            std::lock_guard<std::mutex> lock(recording_mutex_);
            recording_path_ = param.as_string();
            
            if (!fs::exists(recording_path_)) {
                try {
                    fs::create_directories(recording_path_);
                    RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", recording_path_.c_str());
                } catch (const fs::filesystem_error& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create recording directory: %s", e.what());
                    result.successful = false;
                    result.reason = "Failed to create recording directory";
                }
            }
            RCLCPP_INFO(this->get_logger(), "Recording path updated to: %s", recording_path_.c_str());
        } else if (param.get_name() == "enable_camera_info_log") {
            // 允许动态更新日志开关
            enable_camera_info_log_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Camera info detailed logging updated: %s", 
                        enable_camera_info_log_ ? "enabled" : "disabled");
        } else {
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
    }
    return result;
}

void HikCameraNode::saveImage(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
        cv::Mat bgr_image;
        cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
        
        std::stringstream ss;
        ss << recording_path_ << "/" 
           << std::fixed << std::setprecision(0) << image_msg->header.stamp.sec 
           << "_" << image_counter_++ << ".jpg";
        std::string filename = ss.str();
        
        if (cv::imwrite(filename, bgr_image)) {
            RCLCPP_DEBUG(this->get_logger(), "Saved image: %s", filename.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to save image: %s", filename.c_str());
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
}

} // namespace dart_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dart_camera::HikCameraNode)
