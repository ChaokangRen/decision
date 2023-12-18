#include "lane_generator.h"

namespace jarvis {
namespace decision_lib {
namespace common {

ErrorType LaneGenerator::GetLaneBySampleInterpolation(
    const vec_Vecf<LaneDim> &samples, const std::vector<decimal_t> &para,
    Lane *lane) {
    Spline<LaneDegree, LaneDim> spline;

    return kSuccess;
}
}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis