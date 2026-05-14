#ifndef DART_SOLVER_SOLVER_BASIC_METHOD_HPP_
#define DART_SOLVER_SOLVER_BASIC_METHOD_HPP_

#include <cstdint>

namespace pka {

struct SolverParameters {
    double image_width = 0.0;
    double image_height = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    // 相机主点，来自 camera_info.k[2] / camera_info.k[5]
    // 如果 camera_info 未提供有效主点，则回退到 image_width/2, image_height/2。
    double cx = 0.0;
    double cy = 0.0;
    bool use_camera_center = false;
    double yaw_threshold = 1.0;
};

class SolverMethod {
public:
    explicit SolverMethod(const SolverParameters& params);

    void updateParameters(const SolverParameters& params);
    double calculateYawAngle(double x_pixel);
    uint8_t determineFireAdvice(double yaw_angle);
    const SolverParameters& getParameters() const { return params_; }

private:
    SolverParameters params_;
};

}  // namespace pka

#endif  // DART_SOLVER_SOLVER_BASIC_METHOD_HPP_