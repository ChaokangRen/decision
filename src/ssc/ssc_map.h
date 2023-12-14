#ifndef _SSC_MAP_H_
#define _SSC_MAP_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "basics/semantics.h"
#include "state/frenet_state.h"
namespace jarvis {
namespace decision_lib {
using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

class SscMap {
public:
    void print();
    using GridMap3D = common::GridMapND<ObstacleMapType, 3>;

    struct Config {
        std::array<int, 3> map_size = {{1000, 100, 81}};
        std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};
        std::array<std::string, 3> axis_name = {{"s", "d", "t"}};

        decimal_t s_back_len = 0.0;
        decimal_t kMaxLongitudinalVel = 50.0;
        decimal_t kMinLongitudinalVel = 0.0;
        decimal_t kMaxLongitudinalAcc = 3.0;
        decimal_t kMaxLongitudinalDecel = -8.0;
        decimal_t kMaxLateralVel = 3.0;
        decimal_t kMaxLateralAcc = 2.5;

        int kMaxNumOfGridAlongTime = 2;

        std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};
    };
    SscMap(){};
    SscMap(const Config &config);
    ~SscMap(){};

    GridMap3D *p_3d_grid() const {
        return p_3d_grid_;
    }

private:
    common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
    common::GridMapND<SscMapDataType, 3> *p_3d_inflate_grid_;

    std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

    Config config_;

    decimal_t start_time_;

    vec_E<double> ve;

    common::FrenetState initial_fs_;

    bool map_vaild_ = false;

    // vec_E<common::DrivingCorridor> driving_cooridor_vec_;

    std::vector<int> if_corridor_valid_;

    // vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>>
    // final_corridor_vec_;
};

}  // namespace decision_lib
}  // namespace jarvis
#endif