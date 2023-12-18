#ifndef _COMMON_STATE_STATE_H__
#define _COMMON_STATE_STATE_H__

#include "basics/basics.h"
namespace jarvis {
namespace decision_lib {
namespace common {
struct State {
    decimal_t time_stamp{0.0};
    Vecf<2> vec_position{Vecf<2>::Zero()};
    decimal_t angle{0.0};
    decimal_t curvature{0.0};
    decimal_t velocity{0.0};
    decimal_t acceleration{0.0};
    decimal_t steer{0.0};
    void print() const {
        sprintf("State:\n");
        sprintf(" -- time_stamp: %lf.\n", time_stamp);
        sprintf(" -- vec_position: (%lf, %lf).\n", vec_position[0],
                vec_position[1]);
        sprintf(" -- angle: %lf.\n", angle);
        sprintf(" -- curvature: %lf.\n", curvature);
        sprintf(" -- velocity: %lf.\n", velocity);
        sprintf(" -- acceleration: %lf.\n", acceleration);
        sprintf(" -- steer: %lf.\n", steer);
    }

    Vec3f ToXYTheta() const {
        return Vec3f(vec_position(0), vec_position(1), angle);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace common
}  // namespace decision_lib

}  // namespace jarvis

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__