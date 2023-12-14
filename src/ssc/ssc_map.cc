#include "ssc_map.h"
namespace jarvis {
namespace decision_lib {

SscMap::SscMap(const SscMap::Config &config) : config_(config) {
    config_.Print();

    p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
        config_.map_size, config_.map_resolution, config_.axis_name);

    p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
        config_.map_size, config_.map_resolution, config_.axis_name);
}

void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
    initial_fs_ = ori_fs;
    std::array<decimal_t, 3> map_origin;

    map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
    map_origin[1] =
        -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;
    map_origin[2] = ori_fs.time_stamp;

    p_3d_grid_->set_origin(map_origin);
    p_3d_inflated_grid_->set_origin(map_origin);
}

void SscMap::print() {
    std::array<int, 3> map_size = {{10, 10, 8}};                   // s, d, t
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    common::GridMapND<uint8_t, 3> grid_map_3d(map_size, map_resolution,
                                              axis_name);
    Config config;
    config.Print();
}

ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
    for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
        if (obs_grid_fs[i][0] <= 0) {
            continue;
        }
        //由于是静态的，所以在sdt坐标系中表现为柱子
        for (int k = 0; k < config_.map_size[2]; ++k) {
            std::array<decimal_t, 3> pt = {
                {obs_grid_fs[i](0), obs_grid_fs[i](1),
                 (double)k * config_.map_resolution[2]}};
            auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
            if (p_3d_grid_->CheckCoordInRange(coord)) {
                p_3d_grid_->SetValueUsingCoordinate(coord, 100);
            }
        }
    }
    return kSuccess;
}

ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids) {
    p_3d_grid_->clear_data();
    p_3d_inflated_grid_->clear_data();

    FillStaticPart(obstacle_grids);
    FillDynamicPart(sur_vehicle_trajs_fs);

    return kSuccess;
}

ErrorType SscMap::FillDynamicPart(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs) {
    for (auto it = sur_vehicle_trajs_fs.begin();
         it != sur_vehicle_trajs_fs.end(); ++it) {
        FillMapWithFsVehicleTraj(it->second);
    }

    return kSuccess;
}

ErrorType SscMap::FillMapWithFsVehicleTraj(
    const vec_E<common::FsVehicle> traj) {
    if (traj.size() == 0) {
        sprintf_error("[ssc] SscMap - Trajectory is empty.");
        return kWrongStatus;
    }

    for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
        bool is_valid = true;
        for (const auto v : traj[i].vertices) {
            if (v(0) <= 0) {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) {
            continue;
        }
        decimal_t z = traj[i].frenet_state.time_stamp;
        int t_idx = 0;
        std::vector<common::Point2i> v_coord;
        std::array<decimal_t, 3> p_w;
        for (const auto v : traj[i].vertices) {
            p_w = {v(0), v(1), z};
            auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
            t_idx = coord[2];
            if (!p_3d_grid_->CheckCoordInRange(coord)) {
                is_valid = false;
                break;
            }
            v_coord.push_back(common::Point2i(coord[0], coord[1]));
        }
        if (!is_valid) {
            continue;
        }
        std::vector<std::vector<cv::Point2i>> vv_coord_cv;
        std::vector<cv::Point2i> v_coord_cv;
        common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord,
                                                                 &v_coord_cv);
        vv_coord_cv.push_back(v_coord_cv);
        int w = p_3d_grid_->dims_size()[0];
        int h = p_3d_grid_->dims_size()[1];
        int layer_offset = t_idx * w * h;
        cv::Mat layer_mat =
            cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                    p_3d_grid_->get_data_ptr() + layer_offset);
        cv::fillPoly(layer_mat, vv_coord_cv, 100);
    }

    return kSuccess;
}

ErrorType SscMap::InflateObstacleGrid(const common::VehicleParam &param) {
    decimal_t s_p_inflate_len = param.length() / 2.0 - param.d_cr();
    decimal_t s_n_inflate_len = param.length() - s_p_inflate_len;
    int num_s_p_inflate_grids =
        std::floor(s_p_inflate_len / config_.map_resolution[0]);
    int num_s_n_inflate_grids =
        std::floor(s_n_inflate_len / config_.map_resolution[0]);
    int num_d_inflate_grids =
        std::floor((param.width() - 0.5) / 2.0 / config_.map_resolution[1]);
    bool is_free = false;

    for (int i = 0; i < config_.map_size[0]; ++i) {
        for (int j = 0; j < config_.map_size[1]; ++j) {
            for (int k = 0; k < config_.map_size[2]; ++k) {
                std::array<int, 3> coord = {i, j, k};
                p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
                if (!is_free) {
                    for (int s = -num_s_n_inflate_grids;
                         s < num_s_n_inflate_grids; s++) {
                        for (int d = -num_d_inflate_grids;
                             d < num_d_inflate_grids; d++) {
                            coord = {i + s, j + d, k};
                            p_3d_inflated_grid_->SetValueUsingCoordinate(coord,
                                                                         100);
                        }
                    }
                }
            }
        }
    }
    return kSuccess;
}

ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
    // Stage1:get seeds
    vec_E<Vec3i> traj_seeds;
    int num_states = static_cast<int>(trajs.size());
    if (num_states > 1) {
        bool first_seed_determined = false;
        for (int k = 0; k < num_states; ++k) {
            std::array<decimal_t, 3> p_w = {};
            if (!first_seed_determined) {
                //这里把initial_fs_和trajs的第一个点都塞进了traj_seeds
                decimal_t s_0 = initial_fs_.vec_s[0];
                decimal_t d_0 = initial_fs_.vec_dt[0];
                decimal_t t_0 = initial_fs_.time_stamp;
                std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
                auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

                decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
                decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
                decimal_t t_1 = trajs[k].frenet_state.time_stamp;

                std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
                auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
                // * remove the states out of range
                if (!p_grid->CheckCoordInRange(coord_1)) {
                    continue;
                }
                if (coord_1[2] <= 0) {
                    continue;
                }
                first_seed_determined = true;
                traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
                traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
            } else {
                decimal_t s = trajs[k].frenet_state.vec_s[0];
                decimal_t d = trajs[k].frenet_state.vec_dt[0];
                decimal_t t = trajs[k].frenet_state.time_stamp;
                p_w = {s, d, t};
                auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
                if (!p_grid->CheckCoordInRange(coord)) {
                    continue;
                }
                traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
            }
        }
    }
    // stage2:inflate cubes
    common::DrivingCorridor driving_corridor;
    bool is_valid = true;
    auto seed_num = static_cast<int>(traj_seeds.size());
    if (seed_num < 2) {
        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        return kWrongStatus;
    }

    for (int i = 0; i < seed_num; ++i) {
        if (i == 0) {
            common::AxisAlignedCubeNd<int, 3> cube;
            GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
            if (!CheckIfCubeIsFree(p_grid, cube)) {
                sprintf_error(
                    "[Ssc] SccMap - Initial cube is not free, seed id: %d", i);
                common::DrivingCube driving_cube;
                driving_cube.cube = cube;
                driving_cube.seeds.push_back(traj_seeds[i]);
                driving_cube.seeds.push_back(traj_seeds[i + 1]);
                driving_corridor.cubes.push_back(driving_cube);

                driving_corridor.is_valid = false;
                driving_corridor_vec_.push_back(driving_corridor);
                is_valid = false;
                break;
            }

            std::array<bool, 6> dirs_disabled = {false, false, false,
                                                 false, false, false};
        }
    }
}

//检测在p_grid中cube所在位置是否是空的
bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
    int f0_min = cube.lower_bound[0];
    int f0_max = cube.upper_bound[0];
    int f1_min = cube.lower_bound[1];
    int f1_max = cube.upper_bound[1];
    int f2_min = cube.lower_bound[2];
    int f2_max = cube.upper_bound[2];

    int i, j, k;
    std::array<int, 3> coord;
    bool is_free;
    for (i = f0_min; i <= f0_max; ++i) {
        for (j = f1_min; j <= f1_max; ++j) {
            for (k = f2_min; k < f2_max; ++k) {
                coord = {i, j, k};
                p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
                if (!is_free) {
                    return false;
                }
            }
        }
    }
    return true;
}

ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
    std::array<int, 3> lb;
    std::array<int, 3> ub;
    lb[0] = std::min(seed_0(0), seed_1(0));
    lb[1] = std::min(seed_0(1), seed_1(1));
    lb[2] = std::min(seed_0(2), seed_1(2));
    ub[0] = std::max(seed_0(0), seed_1(0));
    ub[1] = std::max(seed_0(1), seed_1(1));
    ub[2] = std::max(seed_0(2), seed_1(2));

    *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
}

}  // namespace decision_lib
}  // namespace jarvis