#ifndef _COMMON_INC_COMMON_STATE_WAYPOINT_H__
#define _COMMON_INC_COMMON_STATE_WAYPOINT_H__

#include "basics/basics.h"
namespace jarvis {
namespace decision_lib {
namespace common {

template <int N_DIM>
struct Waypoint {
    Vecf<N_DIM> pos;
    Vecf<N_DIM> vel;
    Vecf<N_DIM> acc;
    Vecf<N_DIM> jrk;
    decimal_t t{0.0};
    bool fix_pos = false;
    bool fix_vel = false;
    bool fix_acc = false;
    bool fix_jrk = false;
    bool stamped = false;
};

typedef Waypoint<2> Waypoint2D;
typedef Waypoint<3> Waypoint3D;
}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis
#endif