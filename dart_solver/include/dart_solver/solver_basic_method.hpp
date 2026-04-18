#ifndef DART_SOLVER_SOLVER_BASIC_METHOD_HPP_
#define DART_SOLVER_SOLVER_BASIC_METHOD_HPP_

#include <cstdint>

namespace pka {

struct SolverParameters {
    double image_width = 0.0;
    double image_height = 0.0;
    double fx = 0.0;
    double fy = 0.0;
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
