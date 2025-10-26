#include "dart_solver/one_euro_filter.hpp"
#include <cmath>

namespace dart_solver {

OneEuroFilter::OneEuroFilter(double freq, double min_cutoff, double beta, double d_cutoff)
    : freq_(freq), min_cutoff_(min_cutoff), beta_(beta), d_cutoff_(d_cutoff),
      x_prev_(0.0), dx_prev_(0.0), t_prev_(-1.0) {}

double OneEuroFilter::alpha(double cutoff) {
    double te = 1.0 / freq_;
    double tau = 1.0 / (2 * M_PI * cutoff);
    return 1.0 / (1.0 + tau / te);
}

double OneEuroFilter::filter(double x, double t) {
    if (t_prev_ < 0) {
        x_prev_ = x;
        t_prev_ = t;
        return x;
    }
    
    double dt = t - t_prev_;
    if (dt <= 0) return x_prev_;
    
    double freq = 1.0 / dt;
    setFrequency(freq);
    
    double dx = (x - x_prev_) / dt;
    double edx = dx_prev_ + alpha(d_cutoff_) * (dx - dx_prev_);
    
    double cutoff = min_cutoff_ + beta_ * std::abs(edx);
    double x_filtered = x_prev_ + alpha(cutoff) * (x - x_prev_);
    
    x_prev_ = x_filtered;
    dx_prev_ = edx;
    t_prev_ = t;
    
    return x_filtered;
}

void OneEuroFilter::setFrequency(double freq) {
    freq_ = freq;
}

void OneEuroFilter::setMinCutoff(double min_cutoff) {
    min_cutoff_ = min_cutoff;
}

void OneEuroFilter::setBeta(double beta) {
    beta_ = beta;
}

void OneEuroFilter::setDCutoff(double d_cutoff) {
    d_cutoff_ = d_cutoff;
}

}  // namespace dart_solver
