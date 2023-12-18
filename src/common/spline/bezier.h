#ifndef _SPLINE_BEZIER_H_
#define _SPLINE_BEZIER_H_

#include <assert.h>

#include <vector>

#include "basics/basics.h"
namespace jarvis {
namespace decision_lib {
namespace common {

// forward declaration
template <int N_DEG>
class BezierUtils;

/**
 * @brief Bezier spline class
 * The the j-th segment evaluation is given by
 * B_j(t) = s_j*\sum_{i=0}^{N_DEG} c_j^i * b_{N_DEG}^i(t-Tj/s_j)
 * the s_j is the time scaling factor of the j-the segment
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
public:
    BezierSpline() {}
    /**
     * @brief Set the vector domain
     * @param vec_domain:input vec domain
     */
    void set_vec_domain(const std::vector<decimal_t> &vec_domain) {
        assert(vec_domain.size() > 1);
        vec_domain_ = vec_domain;
        ctrl_pts_.resize(vec_domain.size() - 1);
        for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
            ctrl_pts_[j].setZero();
        }
    }

private:
    vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts_;
    std::vector<decimal_t> vec_domain_;
};
}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis
#endif