#ifndef DART_SOLVER_ONE_EURO_FILTER_HPP_
#define DART_SOLVER_ONE_EURO_FILTER_HPP_

namespace pka {

// 一欧元滤波器实现
class OneEuroFilter {
public:
    OneEuroFilter(double freq, double min_cutoff = 1.0, double beta = 0.0, double d_cutoff = 1.0);
    
    double filter(double x, double t);
    
    void setFrequency(double freq);
    void setMinCutoff(double min_cutoff);
    void setBeta(double beta);
    void setDCutoff(double d_cutoff);

private:
    double x_prev_;
    double dx_prev_;
    double t_prev_;
    double freq_;
    double min_cutoff_;
    double beta_;
    double d_cutoff_;
    
    double alpha(double cutoff);
};

}  // namespace pka

#endif  // DART_SOLVER_ONE_EURO_FILTER_HPP_
