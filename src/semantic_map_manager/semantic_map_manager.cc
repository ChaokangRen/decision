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
            vec_Vecf<2> samples;
            for (const auto &pt : pe.second.lane_points) {
                samples.push_back(pt);
            }
            //这里获取了原始的车道点，重新进行拟合
            if (common::LaneGenerator::GetLaneBySamplePoints(
                    samples, &semantic_lane.lane) != kSuccess) {
                continue;
            }

            semantic_lane_set_.semantic_lanes.insert(
                std::pair<int, common::SemanticLane>(semantic_lane.id,
                                                     semantic_lane));
        }
    }
    // Check the consistency of semantic map
    {
        for (auto &semantic_lane : semantic_lane_set_.semantic_lanes) {
            if (semantic_lane.second.l_change_avbl) {
                auto l_it = semantic_lane_set_.semantic_lanes.find(
                    semantic_lane.second.l_lane_id);
                if (l_it == semantic_lane_set_.semantic_lanes.end()) {
                    semantic_lane.second.l_change_avbl = false;
                    semantic_lane.second.l_lane_id = kInvalidLaneId;
                }
            }
            if (semantic_lane.second.r_change_avbl) {
                auto r_it = semantic_lane_set_.semantic_lanes.find(
                    semantic_lane.second.r_lane_id);
                if (r_it == semantic_lane_set_.semantic_lanes.end()) {
                    semantic_lane.second.r_change_avbl = false;
                    semantic_lane.second.r_lane_id = kInvalidLaneId;
                }
            }
            for (auto it = semantic_lane.second.father_id.begin();
                 it < semantic_lane.second.father_id.end();) {
                auto father_it = semantic_lane_set_.semantic_lanes.find(*it);
                if (father_it == semantic_lane_set_.semantic_lanes.end()) {
                    it = semantic_lane.second.father_id.erase(it);
                } else {
                    ++it;
                }
            }
            for (auto it = semantic_lane.second.child_id.begin();
                 it < semantic_lane.second.child_id.end();) {
                auto child_it = semantic_lane_set_.semantic_lanes.find(*it);
                if (child_it == semantic_lane_set_.semantic_lanes.end()) {
                    it = semantic_lane.second.child_id.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
    return kSuccess;
}

ErrorType SemanticMapManager::UpdateLocalLanesAndFastLut() {
    common::State ego_state = ego_vehicle_.state();
    int cur_lane_id;
    decimal_t dist_tmp, arc_len_tmp;
    //得到离自车最近(先考虑角度，在考虑距离)的那条车道，
    if (kSuccess != GetNearestLaneIdUsingState(ego_state.ToXYTheta(),
                                               std::vector<int>(), &cur_lane_id,
                                               &dist_tmp, &arc_len_tmp)) {
        return kWrongStatus;
    }

    local_lanes_.clear();
    local_to_segment_lut_.clear();
    segment_to_local_lut_.clear();

    //* currently we consider ego lane and its adjacent lanes (trunk lanes)
    std::vector<int> root_lane_ids;
    {
        root_lane_ids.push_back(cur_lane_id);
        // ~ left
        if (whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id > 0) {
            int l_id = whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id;
            root_lane_ids.push_back(l_id);
            // ~ left -> left
            if (whole_lane_net_.lane_set.at(l_id).l_lane_id > 0) {
                int ll_id = whole_lane_net_.lane_set.at(l_id).l_lane_id;
                root_lane_ids.push_back(ll_id);
            }
        }
        // ~ right
        if (whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id > 0) {
            int r_id = whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id;
            root_lane_ids.push_back(r_id);
            // ~ right -> right
            if (whole_lane_net_.lane_set.at(r_id).r_lane_id > 0) {
                int rr_id = whole_lane_net_.lane_set.at(r_id).r_lane_id;
                root_lane_ids.push_back(rr_id);
            }
        }
    }

    std::vector<std::vector<int>> lane_ids_expand_front;
    for (const auto root_id : root_lane_ids) {
        decimal_t arc_len;
        semantic_lane_set_.semantic_lanes.at(root_id)
            .lane.GetArcLengthByVecPosition(ego_state.vec_position, &arc_len);
        decimal_t length_remain =
            semantic_lane_set_.semantic_lanes.at(root_id).length - arc_len;

        // Get forward lane paths
        std::vector<std::vector<int>> all_paths_forward;
        GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
            root_id, length_remain, 0.0, std::vector<int>(),
            &all_paths_forward);

        // ~ Get backward lane paths
        std::vector<std::vector<int>> all_paths_backward;
        GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
            root_id, arc_len, 0.0, std::vector<int>(), &all_paths_backward);

        // ~ assemble forward and backward
        std::vector<std::vector<int>> assembled_paths;
        for (const auto &f_path : all_paths_forward) {
            for (const auto &b_path : all_paths_backward) {
                std::vector<int> path = f_path;
                path.erase(path.begin());
                path.insert(path.begin(), b_path.begin(), b_path.end());
                assembled_paths.push_back(path);
            }
        }

        // ~ Fit local lanes and construct LUTs

        int local_lane_cnt = local_lanes_.size();
        for (const auto &path : assembled_paths) {
            common::Lane lane;
            if (kSuccess != GetLocalLaneUsingLaneIds(
                                ego_state, path, local_lane_length_forward_,
                                local_lane_length_backward_, true, &lane)) {
                continue;
            };
            int local_id = local_lane_cnt++;
            local_lanes_.insert(std::pair<int, common::Lane>(local_id, lane));

            local_to_segment_lut_.insert(
                std::pair<int, std::vector<int>>(local_id, path));

            for (const auto &id : path) {
                segment_to_local_lut_[id].insert(local_id);
            }
        }
    }
    has_fast_lut_ = true;
    return kSuccess;
}

void SemanticMapManager::GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
    // Traverse Forward
    decimal_t node_aggre_length = node_length + aggre_length;
    auto path = path_to_node;
    path.push_back(node_id);

    if (node_aggre_length >= local_lane_length_forward_ ||
        whole_lane_net_.lane_set.at(node_id).child_id.empty()) {
        // stop recursion
        all_paths->push_back(path);
        return;
    } else {
        // expand
        auto child_ids = whole_lane_net_.lane_set.at(node_id).child_id;
        for (const auto child_id : child_ids) {
            decimal_t child_length =
                whole_lane_net_.lane_set.at(child_id).length;
            GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
                child_id, child_length, node_aggre_length, path, all_paths);
        }
    }
}

void SemanticMapManager::GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
    // * Traverse BACKWARD
    // check cycle here?

    decimal_t node_aggre_length = node_length + aggre_length;
    auto path = path_to_node;
    path.push_back(node_id);

    if (node_aggre_length >= local_lane_length_backward_ ||
        whole_lane_net_.lane_set.at(node_id).father_id.empty()) {
        // stop recursion
        // backward, reverse path
        std::reverse(path.begin(), path.end());
        all_paths->push_back(path);
        return;
    } else {
        // expand
        auto father_ids = whole_lane_net_.lane_set.at(node_id).father_id;
        for (const auto father_id : father_ids) {
            decimal_t father_length =
                whole_lane_net_.lane_set.at(father_id).length;
            GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
                father_id, father_length, node_aggre_length, path, all_paths);
        }
    }
}

ErrorType SemanticMapManager::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) const {
    // tuple:dist,arc_len,angle_diff,id
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> lanes_in_dist;
    if (GetDistanceToLanesUsing3DofState(state, &lanes_in_dist) != kSuccess) {
        return kWrongStatus;
    }

    if (lanes_in_dist.empty()) {
        sprintf("[GetNearestLaneIdUsingState]No nearest lane found.\n");
        return kWrongStatus;
    }

    //   if (navi_path.empty()) {
    //     *id = std::get<3>(*lanes_in_dist.begin());
    //     *distance = std::get<0>(*lanes_in_dist.begin());
    //     *arc_len = std::get<1>(*lanes_in_dist.begin());
    //   } else {
    // #if 0
    //     // bool has_nearest_lane_found = false;
    //     std::map<decimal_t, std::pair<int, decimal_t>> surrounding_lanes;
    //     for (auto &t : lanes_in_dist) {
    //       int lane_id = std::get<3>(t);
    //       decimal_t cur_distance = std::get<0>(t);
    //       bool is_reachable = false;
    //       int num_lane_changes;
    //       if (IsTopologicallyReachable(lane_id, navi_path, &num_lane_changes,
    //                                    &is_reachable) != kSuccess) {
    //         continue;
    //       }
    //       if (is_reachable) {
    //         const decimal_t lane_change_dist_cost = 0.1;
    //         decimal_t cost =
    //             num_lane_changes * lane_change_dist_cost + cur_distance;
    //         surrounding_lanes.emplace(cost, std::make_pair(lane_id,
    //         cur_distance));
    //       }
    //     }
    //     if (surrounding_lanes.empty()) {
    //       return kWrongStatus;
    //     }
    //     *id = surrounding_lanes.begin()->second.first;
    //     *distance = surrounding_lanes.begin()->second.second;
    // #else
    //     *id = std::get<3>(*lanes_in_dist.begin());
    //     *distance = std::get<0>(*lanes_in_dist.begin());
    //     *arc_len = std::get<1>(*lanes_in_dist.begin());
    // #endif
    //   }

    // * Get candidate lanes within a small range, then sort by angle_diff
    // tuple: angle_diff, dist, arc_len, id
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>>
        lanes_in_angle_diff;
    for (const auto &ele : lanes_in_dist) {
        if (std::get<0>(ele) > nearest_lane_range_) {
            break;
        }
        lanes_in_angle_diff.insert(
            std::tuple<decimal_t, decimal_t, decimal_t, int>(
                fabs(std::get<2>(ele)), std::get<0>(ele), std::get<1>(ele),
                std::get<3>(ele)));
    }
    if (lanes_in_angle_diff.empty() ||
        std::get<0>(*lanes_in_angle_diff.begin()) > kPi / 2) {
        // use the nearest lane with suitable angle diff
        for (const auto &ele : lanes_in_dist) {
            for (const auto &ele : lanes_in_dist) {
                if (std::get<2>(ele) < kPi / 2) {
                    *id = std::get<3>(ele);
                    *distance = std::get<0>(ele);
                    *arc_len = std::get<1>(ele);
                    return kSuccess;
                }
            }
        }
    }
    // Otherwise, use the nearest lane
    *id = std::get<3>(*lanes_in_dist.begin());
    *distance = std::get<0>(*lanes_in_dist.begin());
    *arc_len = std::get<1>(*lanes_in_dist.begin());
    // sprintf(
    //     "[GetNearestLaneIdUsingState]No suitable lane in %f m, use the
    //     nearest " "one, dist: %lf, id: %d\n", nearest_lane_range_, *distance,
    //     *id);
    return kSuccess;
}
ErrorType SemanticMapManager::GetLocalLaneUsingLaneIds(
    const common::State &state, const std::vector<int> &lane_ids,
    const decimal_t forward_length, const decimal_t backward_length,
    const bool &is_high_quality, common::Lane *lane) {
    vec_Vecf<2> raw_samples;
    for (const auto &id : lane_ids) {
        //避免重复的点
        if (raw_samples.empty() &&
            whole_lane_net_.lane_set.at(id).lane_points.size() > 0) {
            raw_samples.push_back(
                whole_lane_net_.lane_set.at(id).lane_points[0]);
        }
        for (int i = 1; i < whole_lane_net_.lane_set.at(id).lane_points.size();
             ++i) {
            raw_samples.push_back(
                whole_lane_net_.lane_set.at(id).lane_points[i]);
        }
    }
    common::Lane long_lane;
    if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
        kSuccess) {
        return kWrongStatus;
    }
    decimal_t arc_len;
    long_lane.GetArcLengthByVecPosition(state.vec_position, &arc_len);

    vec_Vecf<2> samples;
    decimal_t acc_dist_tmp;
    decimal_t sample_start = std::max(0.0, arc_len - backward_length);
    decimal_t sample_end = std::min(arc_len + forward_length, long_lane.end());
    SampleLane(long_lane, sample_start, sample_end, 1.0, &samples,
               &acc_dist_tmp);

    if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
        return kWrongStatus;
    }

    return kSuccess;
}

ErrorType SemanticMapManager::GetLaneBySampledPoints(
    const vec_Vecf<2> &samples, const bool &is_high_quality,
    common::Lane *lane) const {
    if (is_high_quality) {
        double d = 0.0;
        std::vector<decimal_t> para;
        para.push_back(d);

        int num_samples = static_cast<int>(samples.size());
        for (int i = 1; i < num_samples; i++) {
            double dx = samples[i](0) - samples[i - 1](0);
            double dy = samples[i](1) - samples[i - 1](1);
            d += std::hypot(dx, dy);
            para.push_back(d);
        }

        const int num_segments = 20;
        Eigen::ArrayXf breaks =
            Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

        const decimal_t regulator = (double)1e6;
        if (common::LaneGenerator::GetLaneBySampleFitting(
                samples, para, breaks, regulator, lane) != kSuccess) {
            return kWrongStatus;
        }
    } else {
        if (common::LaneGenerator::GetLaneBySamplePoints(samples, lane) !=
            kSuccess) {
            return kWrongStatus;
        }
    }
    return kSuccess;
}
ErrorType SemanticMapManager::SampleLane(const common::Lane &lane,
                                         const decimal_t &s0,
                                         const decimal_t &s1,
                                         const decimal_t &step,
                                         vec_E<Vecf<2>> *samples,
                                         decimal_t *accum_dist) const {
    Vecf<2> pt;
    for (decimal_t s = s0; s < s1; s += step) {
        lane.GetPositionByArcLength(s, &pt);
        samples->push_back(pt);
        (*accum_dist) += step;
    }
    return kSuccess;
}
}  // namespace semantic_map_manager
}  // namespace decision_lib
}  // namespace jarvis