#ifndef _SEMANTIC_MAP_MANAGER_H_
#define _SEMANTIC_MAP_MANAGER_H_

#include <algorithm>
#include <list>
#include <memory>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include "basics.h"
#include "basics/semantics.h"
#include "lane/lane.h"
#include "state/state.h"

namespace jarvis {
namespace decision_lib {
namespace semantic_map_manager {

class SemanticMapManager {
public:
    using ObstacleMapType = uint8_t;
    using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
    using State = common::State;
    using Lane = common::Lane;
    using LateralBehavior = common::LateralBehavior;
    using SemanticLane = common::SemanticLane;

    SemanticMapManager() {}

private:
    double time_stamp_{0.0};
    decimal_t pred_time_ = 5.0;
    decimal_t pred_step_ = 0.2;

    decimal_t nearest_lane_range_ = 1.5;
    decimal_t lane_range_ = 10.0;

    decimal_t max_distance_to_lane_ = 2.0;

    bool has_fast_lut_ = false;
    std::unordered_set<int, common::Lane> local_lanes_;
    std::unordered_set<int, std::vector<int>> local_to_segment_lut_;
    std::unordered_set<int, std::set<int>> segment_to_local_lut_;

    decimal_t local_lane_length_forward_ = 250.0;
    decimal_t local_lane_length_backward_ = 150.0;

    int ego_id_;
    std::string agent_config_path_;
    AgentConfigInfo agent_config_info_;
    bool use_right_hand_axis_ = true;
};
}  // namespace semantic_map_manager
}  // namespace decision_lib
}  // namespace jarvis
#endif