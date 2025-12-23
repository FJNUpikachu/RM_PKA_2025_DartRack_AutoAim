#include "dart_solver/solver_method.hpp"
#include <cmath>

namespace pka {

SolverMethod::SolverMethod(const SolverParameters& params) : params_(params) {}

void SolverMethod::updateParameters(const SolverParameters& params) {
    params_ = params;
}

double SolverMethod::calculateYawAngle(double x_pixel) {
    // 计算图像中心点x坐标
    double image_center_x = params_.image_width / 2.0;
    
    // 计算像素坐标系下的x方向差值（相对于图像中心）
    double x_diff = x_pixel - image_center_x;
    
    // 将像素差值转换为角度（弧度）
    double yaw_rad = atan2(x_diff, params_.fx);
    
    // 转换为度
    double yaw_deg = yaw_rad * 180.0 / M_PI;
    
    return yaw_deg;
}

uint8_t SolverMethod::determineFireAdvice(double yaw_angle) {
    // 判断yaw角度绝对值是否在阈值范围内
    if (std::abs(yaw_angle) <= params_.yaw_threshold) {
        return 1;  // 可发射
    }
    return 0;  // 不可发射
}

}  // namespace pka