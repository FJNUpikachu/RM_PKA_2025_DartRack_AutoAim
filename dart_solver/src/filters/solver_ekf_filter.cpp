#include "dart_solver/filters/solver_ekf_filter.hpp"

namespace pka {

Ekf2DFilter::Ekf2DFilter(
  double process_noise_x,
  double process_noise_y,
  double measurement_noise_x,
  double measurement_noise_y,
  double initial_covariance)
: initialized_(false),
  process_noise_x_(process_noise_x),
  process_noise_y_(process_noise_y),
  measurement_noise_x_(measurement_noise_x),
  measurement_noise_y_(measurement_noise_y),
  initial_covariance_(initial_covariance),
  filter_(
    PredictModel{},
    MeasureModel{},
    [this]() {
      FilterCore::MatrixXX q = FilterCore::MatrixXX::Zero();
      q(0, 0) = process_noise_x_;
      q(1, 1) = process_noise_y_;
      return q;
    },
    [this](const FilterCore::MatrixZ1 &) {
      FilterCore::MatrixZZ r = FilterCore::MatrixZZ::Zero();
      r(0, 0) = measurement_noise_x_;
      r(1, 1) = measurement_noise_y_;
      return r;
    },
    FilterCore::MatrixXX::Identity() * initial_covariance)
{
}

Ekf2DFilter::StateVector Ekf2DFilter::filter(double measurement_x, double measurement_y)
{
  StateVector z;
  z << measurement_x, measurement_y;

  if (!initialized_) {
    filter_.setState(z);
    initialized_ = true;
    return z;
  }

  filter_.predict();
  return filter_.update(z);
}

void Ekf2DFilter::reset()
{
  initialized_ = false;
}

}  // namespace pka
