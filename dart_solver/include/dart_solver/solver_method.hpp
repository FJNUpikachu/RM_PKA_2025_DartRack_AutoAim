#ifndef DART_SOLVER_SOLVER_METHOD_HPP_
#define DART_SOLVER_SOLVER_METHOD_HPP_

#include <cstdint>

namespace pka {

// 解算器参数结构体
struct SolverParameters {
    // 图像尺寸
    double image_width = 0.0;   // 图像宽度
    double image_height = 0.0;  // 图像高度
    
    // 相机内参
    double fx = 0.0;  // 焦距x
    double fy = 0.0;  // 焦距y
    
    // yaw角度判断阈值（度）
    double yaw_threshold = 1.0;  // 默认±1度范围内可发射
};

// 解算方法类
class SolverMethod {
public:
    SolverMethod(const SolverParameters& params);
    
    void updateParameters(const SolverParameters& params);
    
    // 计算yaw角度
    double calculateYawAngle(double x_pixel);
    
    // 判断是否可发射
    uint8_t determineFireAdvice(double yaw_angle);
    
    const SolverParameters& getParameters() const { return params_; }
    
private:
    SolverParameters params_;
};

}  // namespace pka

#endif  // DART_SOLVER_SOLVER_METHOD_HPP_