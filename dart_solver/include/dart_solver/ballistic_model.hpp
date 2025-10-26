#ifndef DART_SOLVER_BALLISTIC_MODEL_HPP_
#define DART_SOLVER_BALLISTIC_MODEL_HPP_

#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"
#include "dart_solver/solver_method.hpp"

namespace dart_solver {

// 弹道模型类
class BallisticModel {
public:
    BallisticModel();
    
    void setParameters(const BallisticParameters& params);
    const BallisticParameters& getParameters() const { return params_; }
    
    // 计算弹道轨迹，返回轨迹点集合
    std::vector<Eigen::Vector3d> calculateTrajectory(
        const Eigen::Vector3d& launch_point,
        const Eigen::Vector3d& target_point,
        double pitch, double yaw,
        uint8_t mode);
    
private:
    BallisticParameters params_;
};

// TF广播器类
class TFBroadcaster {
public:
    TFBroadcaster(rclcpp::Node* node);
    
    geometry_msgs::msg::TransformStamped createTransform(
        const std::string& frame_id,
        const std::string& child_frame_id,
        const Eigen::Vector3d& position);
    
    void publishTransforms(
        const Eigen::Vector3d& launch_point,
        const Eigen::Vector3d& target_point,
        const std::vector<Eigen::Vector3d>& trajectory,
        bool enable_tf, bool enable_rviz);
    
private:
    rclcpp::Node* node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace dart_solver

#endif  // DART_SOLVER_BALLISTIC_MODEL_HPP_
