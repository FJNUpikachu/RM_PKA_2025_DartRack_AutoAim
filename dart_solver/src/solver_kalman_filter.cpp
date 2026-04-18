#include "dart_solver/solver_kalman_filter.hpp"

namespace pka {

KalmanFilter1D::KalmanFilter1D(double q, double r, double init_p)
: initialized_(false), x_hat_(0.0), p_(init_p), q_(q), r_(r), init_p_(init_p)
{
}

double KalmanFilter1D::filter(double measurement, double) {
    if (!initialized_) {
        x_hat_ = measurement;
        p_ = init_p_;
        initialized_ = true;
        return x_hat_;
    }

    p_ = p_ + q_;
    const double k = p_ / (p_ + r_);
    x_hat_ = x_hat_ + k * (measurement - x_hat_);
    p_ = (1.0 - k) * p_;
    return x_hat_;
}

void KalmanFilter1D::reset() {
    initialized_ = false;
    x_hat_ = 0.0;
    p_ = init_p_;
}

void KalmanFilter1D::setProcessNoise(double q) {
    q_ = q;
}

void KalmanFilter1D::setMeasurementNoise(double r) {
    r_ = r;
}

void KalmanFilter1D::setInitialCovariance(double init_p) {
    init_p_ = init_p;
    if (!initialized_) {
        p_ = init_p_;
    }
}

}  // namespace pka
