#include "dart_solver/solver_method.hpp"
#include <cmath>
#include <algorithm>

namespace dart_solver {

SolverMethod::SolverMethod(const SolverParameters& params) : params_(params) {}

void SolverMethod::updateParameters(const SolverParameters& params) {
    params_ = params;
}

void SolverMethod::updateMode(uint8_t mode) {
    // 只接受模式1和模式2
    if (mode == 1 || mode == 2) {
        params_.current_mode = mode;
    }
}

void SolverMethod::solveAngles(double x, double y, double& pitch, double& yaw) {
    // 计算yaw角度（使x轴对齐图像中心）
    double x_diff = x - params_.cx;
    yaw = atan2(x_diff, params_.fx) * 180.0 / M_PI;  // 转换为度
    
    // 这里可以根据实际需求实现pitch角解算逻辑
    // 暂时简单实现为基于y坐标的比例计算
    double y_diff = y - params_.cy;
    pitch = atan2(y_diff, params_.fy) * 180.0 / M_PI;
}

uint8_t SolverMethod::determineFireAdvice(double x, double y) {
    // 检查x是否在允许范围内
    bool x_ok = std::abs(x - params_.cx) <= params_.x_threshold;
    
    // 根据当前模式检查y是否在允许范围内
    bool y_ok = false;
    if (params_.current_mode == 1) {  // 前哨站模式
        y_ok = (y >= params_.sentinel_y_min) && (y <= params_.sentinel_y_max);
    } else if (params_.current_mode == 2) {  // 基地模式
        y_ok = (y >= params_.base_y_min) && (y <= params_.base_y_max);
    }
    
    return (x_ok && y_ok) ? 1 : 0;
}

Eigen::Vector3d SolverMethod::calculateTargetPoint(double x, double y) {
    Eigen::Vector3d target_point;
    
    // 使用相机内参将图像坐标转换为3D坐标
    // 这里简化处理，实际应根据距离计算
    target_point.x() = (x - params_.cx) / params_.fx;
    target_point.y() = (y - params_.cy) / params_.fy;
    target_point.z() = 1.0;  // 单位距离，实际应根据模式使用不同距离
    
    return target_point;
}

}  // namespace dart_solver
