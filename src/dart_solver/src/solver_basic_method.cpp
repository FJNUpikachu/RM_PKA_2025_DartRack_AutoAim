#include "dart_solver/solver_basic_method.hpp"
#include <cmath>

namespace pka {

SolverMethod::SolverMethod(const SolverParameters& params)
    : params_(params)
{}

void SolverMethod::updateParameters(const SolverParameters& params) {
    params_ = params;
}

double SolverMethod::calculateYawAngle(double x_pixel) {
    // 优先使用 camera_info 中的主点 cx 作为 yaw 解算中心。
    // 如果没有收到有效 camera_info，则回退到图像几何中心 image_width / 2。
    const double image_center_x =
        params_.use_camera_center ? params_.cx : params_.image_width / 2.0;

    const double error_pixel = x_pixel - image_center_x;
    const double focal_length = params_.fx > 1e-6 ? params_.fx : 1.0;
    const double yaw_rad = std::atan2(error_pixel, focal_length);
    return yaw_rad * 180.0 / M_PI;
}

uint8_t SolverMethod::determineFireAdvice(double yaw_angle) {
    if (std::abs(yaw_angle) <= params_.yaw_threshold) {
        return 1;
    }
    return 0;
}

}  // namespace pka