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
    double image_center_x = params_.image_width / 2.0;
    double error_pixel = x_pixel - image_center_x;
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