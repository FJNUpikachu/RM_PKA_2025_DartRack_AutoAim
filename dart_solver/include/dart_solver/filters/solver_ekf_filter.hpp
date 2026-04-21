#ifndef DART_SOLVER_FILTERS_SOLVER_EKF_FILTER_HPP_
#define DART_SOLVER_FILTERS_SOLVER_EKF_FILTER_HPP_

#include <Eigen/Dense>

#include "dart_solver/filters/extended_kalman_filter.hpp"

namespace pka {

class Ekf2DFilter {
public:
  using StateVector = Eigen::Matrix<double, 2, 1>;

  Ekf2DFilter(
    double process_noise_x = 8.0,
    double process_noise_y = 8.0,
    double measurement_noise_x = 12.0,
    double measurement_noise_y = 12.0,
    double initial_covariance = 500.0);

  StateVector filter(double measurement_x, double measurement_y);
  void reset();

private:
  struct PredictModel {
    template<typename T>
    void operator()(const T x[2], T x_pred[2]) const
    {
      x_pred[0] = x[0];
      x_pred[1] = x[1];
    }
  };

  struct MeasureModel {
    template<typename T>
    void operator()(const T x[2], T z_pred[2]) const
    {
      z_pred[0] = x[0];
      z_pred[1] = x[1];
    }
  };

  using FilterCore = ExtendedKalmanFilter<2, 2, PredictModel, MeasureModel>;

  bool initialized_;
  double process_noise_x_;
  double process_noise_y_;
  double measurement_noise_x_;
  double measurement_noise_y_;
  double initial_covariance_;
  FilterCore filter_;
};

}  // namespace pka

#endif  // DART_SOLVER_FILTERS_SOLVER_EKF_FILTER_HPP_
