/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <memory>
#include <vector>

#include "basics/basics.h"
#include "basics/semantics.h"
#include "state/free_state.h"
#include "state/state.h"
namespace jarvis {
namespace decision_lib {
namespace semantic_map_manager {

struct AgentConfigInfo {
    common::GridMapMetaInfo obstacle_map_meta_info;
    decimal_t surrounding_search_radius;
    bool enable_openloop_prediction{false};
    bool enable_tracking_noise{false};
    bool enable_log{false};
    bool enable_fast_lane_lut{true};
    std::string log_file;

    void PrintInfo() {
        obstacle_map_meta_info.print();
        sprintf("surrounding_search_radius: %f\n", surrounding_search_radius);
        sprintf("enable_openloop_prediction: %d\n", enable_openloop_prediction);
        sprintf("enable_tracking_noise: %d\n", enable_tracking_noise);
        sprintf("enable_log: %d\n", enable_log);
        sprintf("enable_fast_lane_lut: %d\n", enable_fast_lane_lut);
        sprintf("log_file: %s\n", log_file.c_str());
    }
};
}  // namespace semantic_map_manager
}  // namespace decision_lib

}  // namespace jarvis

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_