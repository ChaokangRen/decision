#include "semantic_map_manager.h"
namespace jarvis {
namespace decision_lib {
namespace semantic_map_manager {
SemanticMapManager::SemanticMapManager(const int &id,
                                       const std::string &agent_config_path)
    : ego_id_(id), agent_config_path_(agent_config_path) {
    p_config_loader_ = new ConfigLoader();
    p_config_loader_->set_ego_id(ego_id_);
    p_config_loader_->set_agent_config_path(agent_config_path_);
    p_config_loader_->ParseAgentConfig(&agent_config_info_);
    global_timer_.tic();
}
SemanticMapManager::SemanticMapManager(
    const int &id, const decimal_t surrounding_search_radius,
    bool enable_openloop_prediction, bool use_right_hand_axis) {
    // default constructor
    ego_id_ = id;
    agent_config_info_.surrounding_search_radius = surrounding_search_radius;
    agent_config_info_.enable_openloop_prediction = enable_openloop_prediction;
    agent_config_info_.enable_tracking_noise = false;
    agent_config_info_.enable_log = false;
    agent_config_info_.enable_fast_lane_lut = true;
    use_right_hand_axis_ = use_right_hand_axis;
    is_simple_lane_structure_ = true;
}

ErrorType SemanticMapManager::CheckCollisionUsingGlobalPosition(
    const Vec2f &p_w, bool *res) const {
    std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
    return obstacle_map_.CheckIfEqualUsingGlobalPosition(p, GridMap2D::OCCUPIED,
                                                         res);
}

ErrorType SemanticMapManager::GetObstacleMapValueUsingGlobalPosition(
    const Vec2f &p_w, ObstacleMapType *res) {
    std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
    return obstacle_map_.GetValueUsingGlobalPosition(p, res);
}

ErrorType SemanticMapManager::CheckCollisionUsingStateAndVehicleParam(
    const common::VehicleParam &vehicle_param, const common::State &state,
    bool *res) {
    // check static collision
    {
        common::Vehicle vehicle;
        vehicle.set_state(state);
        vehicle.set_param(vehicle_param);
        vec_E<Vec2f> vertices;
        common::ShapeUtils::GetVerticesOfOrientedBoundingBox(
            vehicle.RetOrientedBoundingBox(), &vertices);
        bool is_collision = false;
        for (auto &v : vertices) {
            CheckCollisionUsingGlobalPosition(v, &is_collision);
            if (is_collision) {
                *res = is_collision;
                return kSuccess;
            }
        }
    }

    // check dynamic collision
    if (agent_config_info_.enable_openloop_prediction) {
        for (const auto &v : semantic_surrounding_vehicles_.semantic_vehicles) {
            auto state_stamp = state.time_stamp;
            auto obstacle_init_stamp = v.second.vehicle.state().time_stamp;
            auto openloop_pred_traj = openloop_pred_trajs_.at(v.first);
            int access_index =
                std::round((state_stamp - obstacle_init_stamp) / pred_step_);
            int num_pred_states = static_cast<int>(openloop_pred_traj.size());
            if (access_index < 0 || access_index >= num_pred_states) continue;

            bool is_collision = false;
            CheckCollisionUsingState(
                vehicle_param, state, v.second.vehicle.param(),
                openloop_pred_traj[access_index], &is_collision);
            if (is_collision) {
                *res = is_collision;
                return kSuccess;
            }
        }
    }
    *res = false;
    return kSuccess;
}

ErrorType SemanticMapManager::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
    common::OrientedBoundingBox2D obb_a, obb_b;
    common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
        param_a, state_a, &obb_a);
    common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
        param_b, state_b, &obb_b);
    *res =
        common::ShapeUtils::CheckIfOrientedBoundingBoxIntersect(obb_a, obb_b);
    return kSuccess;
}

ErrorType SemanticMapManager::GetDistanceToLanesUsing3DofState(
    const Vec3f &state,
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const {
    for (const auto &p : semantic_lane_set_.semantic_lanes) {
        decimal_t arc_len;
        p.second.lane.GetArcLengthByVecPosition(Vec2f(state(0), state(1)),
                                                &arc_len);

        Vec2f pt;
        p.second.lane.GetPositionByArcLength(arc_len, &pt);

        double dist = std::hypot((state(0) - pt(0)), (state(1) - pt(1)));
        if (dist > lane_range_) {
            continue;
        }
        decimal_t lane_angle;
        p.second.lane.GetOrientationByArcLength(arc_len, &lane_angle);
        decimal_t angle_diff = normalize_angle(lane_angle - state(2));

        res->insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
            dist, arc_len, angle_diff, p.second.id));
    }

    return kSuccess;
}

ErrorType SemanticMapManager::UpdateSemanticMap(
    const double &time_stamp, const common::Vehicle &ego_vehicle,
    const common::LaneNet &whole_lane_net,
    const common::LaneNet &surrounding_lane_net,
    const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
    const std::set<std::array<decimal_t, 2>> &obstacle_grids,
    const common::VehicleSet &surrounding_vehicles) {
    TicToc timer;
    time_stamp_ = time_stamp;
    set_ego_vehicle(ego_vehicle);
    set_whole_lane_net(whole_lane_net);
    set_surrounding_lane_net(surrounding_lane_net);
    set_obstacle_map(obstacle_map);
    set_obstacle_grids(obstacle_grids);
    set_surrounding_vehicles(surrounding_vehicles);

    // * update lanes and topologies
    UpdateSemanticLaneSet();

    // * update key lanes and its LUT
    if (agent_config_info_.enable_fast_lane_lut) {
        UpdateLocalLanesAndFastLut();
    }

    // // * update semantic info for vehicles
    // UpdateSemanticVehicles();

    // // * update selected key vehicles
    // UpdateKeyVehicles();

    // // * openloop prediction for all semantic vehicles
    // if (agent_config_info_.enable_openloop_prediction) {
    //     OpenloopTrajectoryPrediction();
    // }

    // if (agent_config_info_.enable_log) {
    //     SaveMapToLog();
    // }
    return kSuccess;
}
ErrorType SemanticMapManager::UpdateSemanticLaneSet() {
    semantic_lane_set_.clear();
    // Update semantic lane set
    {
        for (const auto &pe : surrounding_lane_net_.lane_set) {
            common::SemanticLane semantic_lane;
            semantic_lane.id = pe.second.id;
            semantic_lane.dir = pe.second.dir;
            semantic_lane.child_id = pe.second.child_id;
            semantic_lane.father_id = pe.second.father_id;
            semantic_lane.l_lane_id = pe.second.l_lane_id;
            semantic_lane.l_change_avbl = pe.second.l_change_avbl;
            semantic_lane.r_lane_id = pe.second.r_lane_id;
            semantic_lane.r_change_avbl = pe.second.r_change_avbl;
            semantic_lane.behavior = pe.second.behavior;
            semantic_lane.length = pe.second.length;
            vec_Vecf<2> smaples;
            for (const auto &pt : pe.second.lane_points) {
                smaples.push_back(pt);
            }
            // if (common::LaneGenerator::GetLaneBySamplePoints(
            //         samples, &semantic_lane.lane) != kSuccess) {
            //     continue;
            // }
        }
        return kSuccess;
    }
}
}  // namespace semantic_map_manager
}  // namespace decision_lib
}  // namespace jarvis