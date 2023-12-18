#ifndef _LANE_LANE_GENERATOR_H__
#define _LANE_LANE_GENERATOR_H__

#include "basics/basics.h"
#include "basics/config.h"
#include "lane.h"
namespace jarvis {
namespace decision_lib {
namespace common {
class LaneGenerator {
public:
    static ErrorType GetLaneBySampleInterpolation(
        const vec_Vecf<LaneDim> &samples, const std::vector<decimal_t> &para,
        Lane *lane);

    static ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim> &samples,
                                           Lane *lane);

    /**
     * @brief get lane by least square fitting with continuity constraints
     * @note  regulator recommendation: [1e6, 1e8)
     */

    static ErrorType GetLaneBySampleFitting(const vec_Vecf<LaneDim> &samples,
                                            const std::vector<decimal_t> &para,
                                            const Eigen::ArrayXf &breaks,
                                            const decimal_t regulator,
                                            Lane *lane);
};
}  // namespace common
}  // namespace decision_lib

}  // namespace jarvis

#endif