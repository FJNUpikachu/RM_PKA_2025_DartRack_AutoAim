#ifndef DART_SOLVER_SOLVER_METHOD_HPP_
#define DART_SOLVER_SOLVER_METHOD_HPP_

#include <Eigen/Dense>
#include <cstdint>

namespace dart_solver {

// 解算器参数结构体（与新配置文件对应）
struct SolverParameters {
    // 相机内参
    double fx = 0.0;  // 焦距x
    double fy = 0.0;  // 焦距y
    double cx = 0.0;  // 光心x
    double cy = 0.0;  // 光心y

    // 发射判断阈值
    double x_threshold = 3.0;        // x轴允许的最大偏差（像素）
    double sentinel_y_min = 100.0;   // 前哨站y轴最小阈值
    double sentinel_y_max = 500.0;   // 前哨站y轴最大阈值
    double base_y_min = 80.0;        // 基地y轴最小阈值
    double base_y_max = 450.0;       // 基地y轴最大阈值

    // 发射点位置
    Eigen::Vector3d launch_position = Eigen::Vector3d::Zero();

    // 当前模式 (1: 前哨站, 2: 基地)
    uint8_t current_mode = 1;
};

// 弹道参数结构体
struct BallisticParameters {
    bool enable = true;              // 弹道解算总开关
    double gravity = 9.81;           // 重力加速度 (m/s²)
    double drag_coeff = 0.01;        // 阻力系数
    double muzzle_velocity = 10.0;   // 枪口初速度 (m/s)
    double sentinel_distance = 5.0;  // 到前哨站的水平距离 (m)
    double base_distance = 8.0;      // 到基地的水平距离 (m)
    double vertical_offset = 1.2;    // 竖直距离 (m)
};

// 解算方法类
class SolverMethod {
public:
    SolverMethod(const SolverParameters& params);
    
    void updateParameters(const SolverParameters& params);
    void updateMode(uint8_t mode);
    
    void solveAngles(double x, double y, double& pitch, double& yaw);
    uint8_t determineFireAdvice(double x, double y);
    Eigen::Vector3d calculateTargetPoint(double x, double y);
    
    const SolverParameters& getParameters() const { return params_; }
    
private:
    SolverParameters params_;
};

}  // namespace dart_solver

#endif  // DART_SOLVER_SOLVER_METHOD_HPP_
