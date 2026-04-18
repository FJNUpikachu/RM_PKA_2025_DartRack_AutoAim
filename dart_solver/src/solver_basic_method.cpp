#include "dart_solver/solver_basic_method.hpp"
#include <cmath>

namespace pka {

SolverMethod::SolverMethod(const SolverParameters& params) : params_(params) {}

void SolverMethod::updateParameters(const SolverParameters& params) {
    params_ = params;
}

double SolverMethod::calculateYawAngle(double x_pixel) {
    double image_center_x = params_.image_width / 2.0;
    double x_diff = x_pixel - image_center_x;
    double yaw_rad = std::atan2(x_diff, params_.fx);
    double yaw_deg = yaw_rad * 180.0 / std::acos(-1.0);
    return yaw_deg;
}

uint8_t SolverMethod::determineFireAdvice(double yaw_angle) {
    if (std::abs(yaw_angle) <= params_.yaw_threshold) {
        return 1;
    }
    return 0;
}

}  // namespace pka
