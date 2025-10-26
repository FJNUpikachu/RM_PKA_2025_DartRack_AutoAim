#include "dart_solver/ballistic_model.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace dart_solver {

// 弹道模型实现
BallisticModel::BallisticModel() : params_() {}

void BallisticModel::setParameters(const BallisticParameters& params) {
    params_ = params;
}

std::vector<Eigen::Vector3d> BallisticModel::calculateTrajectory(
    const Eigen::Vector3d& launch_point,
    const Eigen::Vector3d& target_point,
    double pitch, double yaw,
    uint8_t mode) {
    
    // 如果弹道解算被禁用，返回空轨迹
    if (!params_.enable) {
        return std::vector<Eigen::Vector3d>();
    }
    
    std::vector<Eigen::Vector3d> trajectory;
    trajectory.push_back(launch_point);
    
    // 将角度转换为弧度
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;
    
    // 根据模式获取距离参数
    double horizontal_distance = (mode == 1) ? params_.sentinel_distance : params_.base_distance;
    
    // 计算初始速度向量
    Eigen::Vector3d velocity;
    velocity.x() = params_.muzzle_velocity * cos(pitch_rad) * cos(yaw_rad);
    velocity.y() = params_.muzzle_velocity * cos(pitch_rad) * sin(yaw_rad);
    velocity.z() = params_.muzzle_velocity * sin(pitch_rad);
    
    // 模拟轨迹（简化模型）
    Eigen::Vector3d current_pos = launch_point;
    double time_step = 0.05;  // 时间步长 50ms
    double max_time = 5.0;    // 最大模拟时间 5秒
    double current_time = 0.0;
    
    while (current_time < max_time) {
        // 如果弹道解算被禁用，提前退出
        if (!params_.enable) {
            break;
        }
        
        // 计算空气阻力 (简化模型)
        Eigen::Vector3d drag = -params_.drag_coeff * velocity.norm() * velocity;
        
        // 计算加速度
        Eigen::Vector3d acceleration = drag;
        acceleration.z() -= params_.gravity;  // 重力加速度
        
        // 更新速度和位置
        velocity += acceleration * time_step;
        current_pos += velocity * time_step;
        
        trajectory.push_back(current_pos);
        current_time += time_step;
        
        // 计算到目标点的水平距离
        Eigen::Vector2d current_xy(current_pos.x(), current_pos.y());
        Eigen::Vector2d target_xy(target_point.x(), target_point.y());
        double dist_to_target = (current_xy - target_xy).norm();
        
        // 如果接近目标点或低于发射点高度，停止模拟
        if (dist_to_target < 0.5 || current_pos.z() < launch_point.z() - 0.5) {
            break;
        }
    }
    
    return trajectory;
}

// TF广播器实现
TFBroadcaster::TFBroadcaster(rclcpp::Node* node) : node_(node) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
}

geometry_msgs::msg::TransformStamped TFBroadcaster::createTransform(
    const std::string& frame_id,
    const std::string& child_frame_id,
    const Eigen::Vector3d& position) {
    
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  // 不旋转
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    return transform;
}

void TFBroadcaster::publishTransforms(
    const Eigen::Vector3d& launch_point,
    const Eigen::Vector3d& target_point,
    const std::vector<Eigen::Vector3d>& trajectory,
    bool enable_tf, bool enable_rviz) {
    
    // 如果TF发布被禁用，直接返回
    if (!enable_tf) {
        return;
    }
    
    // 发布发射点变换
    auto launch_tf = createTransform("world", "launch_position", launch_point);
    tf_broadcaster_->sendTransform(launch_tf);
    
    // 发布目标点变换
    auto target_tf = createTransform("world", "target_position", target_point);
    tf_broadcaster_->sendTransform(target_tf);
    
    // 如果RViz可视化被启用，发布轨迹点
    if (enable_rviz && !trajectory.empty()) {
        for (size_t i = 0; i < trajectory.size(); i += 5) {  // 每隔5个点发布一个
            std::string frame_name = "trajectory_point_" + std::to_string(i/5);
            auto trajectory_tf = createTransform("world", frame_name, trajectory[i]);
            tf_broadcaster_->sendTransform(trajectory_tf);
        }
    }
}

}  // namespace dart_solver
