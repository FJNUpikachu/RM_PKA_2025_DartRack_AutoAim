#ifndef DART_SOLVER_SOLVER_KALMAN_FILTER_HPP_
#define DART_SOLVER_SOLVER_KALMAN_FILTER_HPP_

namespace pka {

class KalmanFilter1D {
public:
    KalmanFilter1D(double q = 8.0, double r = 12.0, double init_p = 500.0);

    double filter(double measurement, double stamp_sec = 0.0);
    void reset();

    void setProcessNoise(double q);
    void setMeasurementNoise(double r);
    void setInitialCovariance(double init_p);

private:
    bool initialized_;
    double x_hat_;
    double p_;
    double q_;
    double r_;
    double init_p_;
};

}  // namespace pka

#endif  // DART_SOLVER_SOLVER_KALMAN_FILTER_HPP_
