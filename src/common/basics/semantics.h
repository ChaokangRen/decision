#ifndef _COMMON_SEMANTIC_H_
#define _COMMON_SEMANTIC_H_
#include <cmath>
#include <iostream>

#include "basics.h"
#include "lane/lane.h"
#include "shapes.h"
#include "state/frenet_state.h"
namespace jarvis {
namespace decision_lib {
namespace common {
class VehicleParam {
public:
    inline double width() const {
        return width_;
    }
    inline double length() const {
        return length_;
    }
    inline double wheel_base() const {
        return wheel_base_;
    }
    inline double front_suspension() const {
        return front_suspension_;
    }
    inline double rear_suspension() const {
        return rear_suspension_;
    }
    inline double max_steering_angle() const {
        return max_steering_angle_;
    }
    inline double max_longitudinal_acc() const {
        return max_longitudinal_acc_;
    }
    inline double max_lateral_acc() const {
        return max_lateral_acc_;
    }
    inline double d_cr() const {
        return d_cr_;
    }

    inline void set_width(const double val) {
        width_ = val;
    }
    inline void set_length(const double val) {
        length_ = val;
    }
    inline void set_wheel_base(const double val) {
        wheel_base_ = val;
    }
    inline void set_front_suspension(const double val) {
        front_suspension_ = val;
    }
    inline void set_rear_suspension(const double val) {
        rear_suspension_ = val;
    }
    inline void set_max_steering_angle(const double val) {
        max_steering_angle_ = val;
    }
    inline void set_max_longitudinal_acc(const double val) {
        max_longitudinal_acc_ = val;
    }
    inline void set_max_lateral_acc(const double val) {
        max_lateral_acc_ = val;
    }
    inline void set_d_cr(const double val) {
        d_cr_ = val;
    }

    /**
     * @brief Print info
     */
    void print() const;

private:
    double width_ = 1.90;
    double length_ = 4.88;
    double wheel_base_ = 2.85;
    double front_suspension_ = 0.93;
    double rear_suspension_ = 1.10;
    double max_steering_angle_ = 45.0;

    double max_longitudinal_acc_ = 2.0;
    double max_lateral_acc_ = 2.0;

    double d_cr_ = 1.34;  // length between geometry center and rear axle
};

enum class LongitudinalBehavior {
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    kStopping
};

enum class LateralBehavior {
    kUndefined = 0,
    kLaneKeeping,
    kLaneChangeLeft,
    kLaneChangeRight,
};

struct GridMapMetaInfo {
    int width = 0;
    int height = 0;
    double resolution = 0;
    double w_metric = 0;
    double h_metric = 0;

    /**
     * @brief Construct a new Grid Map Meta Info object
     */
    GridMapMetaInfo();

    /**
     * @brief Construct a new Grid Map Meta Info object
     *
     * @param w width
     * @param h height
     * @param res resolution
     */
    GridMapMetaInfo(const int w, const int h, const double res);

    /**
     * @brief Print info
     */
    void print() const;
};

struct SemanticLane {
    int id;
    int dir;

    std::vector<int> child_id;
    std::vector<int> father_id;

    int l_lane_id;
    bool l_change_avbl;
    int r_lane_id;
    bool r_change_avbl;

    std::string behavior;
    decimal_t length;

    Lane lane;
};

template <typename T, int N_DIM>
class GridMapND {
public:
    enum ValType {
        OCCUPIED = 70,
        FREE = 0,
        SCANNED_OCCUPIED = 128,
        UNKNOWN = 0
    };
    /**
     * @brief Construct a new GridMapND object
     *
     */
    GridMapND();
    /**
     * @brief Construct a new GridMapND object
     *
     * @param dims_size size of each dimension
     * @param dims_resolution resolution of each dimension
     * @param dims_name name of each dimension
     */
    GridMapND(const std::array<int, N_DIM> &dims_size,
              const std::array<decimal_t, N_DIM> &dims_resolution,
              const std::array<std::string, N_DIM> &dims_name);

    inline std::array<int, N_DIM> dims_size() const {
        return dims_size_;
    }
    inline int dims_size(const int &dim) const {
        return dims_size_.at(dim);
    }
    inline std::array<int, N_DIM> dims_step() const {
        return dims_step;
    }
    inline int dims_step(const int &dim) const {
        return dims_step_.at(dim);
    }
    inline std::array<decimal_t, N_DIM> dims_resolution() const {
        return dims_resolution_;
    }
    inline decimal_t dims_resolution(const int &dim) const {
        return dims_resolution_.at(dim);
    }
    inline std::array<std::string, N_DIM> dims_name() const {
        return dims_name_;
    }
    inline std::string dims_name(const int &dim) const {
        return dims_name_.at(dim);
    }
    inline std::array<decimal_t, N_DIM> origin() const {
        return origin_;
    }
    inline int data_size() const {
        return data_size_;
    }
    // Pointer to the underlying element storage. For non-empty containers, the
    // returned pointer compares equal to the address of the first element.
    inline const std::vector<T> *data() const {
        return &data_;
    }
    inline T data(const int &i) const {
        return data_[i];
    };
    inline T *get_data_ptr() {
        return data_.data();
    }
    inline const T *data_ptr() const {
        return data_.data();
    }

    inline void set_origin(const std::array<decimal_t, N_DIM> &origin) {
        origin_ = origin;
    }
    inline void set_dims_size(const std::array<int, N_DIM> &dims_size) {
        dims_size_ = dims_size;
        SetNDimSteps(dims_size);
        SetDataSize(dims_size);
    }
    inline void set_dims_resolution(
        const std::array<decimal_t, N_DIM> &dims_resolution) {
        dims_resolution_ = dims_resolution;
    }
    inline void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
        dims_name_ = dims_name;
    }
    inline void set_data(const std::vector<T> &in) {
        data_ = in;
    }
    /**
     * @brief Set all data in array to 0
     */
    inline void clear_data() {
        data_ = std::vector<T>(data_size_, 0);
    }
    /**
     * @brief Fill all data in array using value
     *
     * @param val input value
     */
    inline void fill_data(const T &val) {
        data_ = std::vector<T>(data_size_, val);
    }
    /**
     * @brief Get the Value Using Coordiante
     *
     * @param coord Input coordinate
     * @param val Output val
     * @return Errortype
     */
    ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                      T *val) const;

    /**
     * @brief Get the Value Using Global Position
     *
     * @param p_w Input global position
     * @param val Output value
     * @return ErrorType
     */
    ErrorType GetValueUsingGlobalPosition(
        const std::array<decimal_t, N_DIM> &p_w, T *val) const;

    /**
     * @brief Get the Coordinate Using Global Position
     *
     * @param p_w Input global position
     * @return std::array<int, N_DIM> Output coordinate
     */
    std::array<int, N_DIM> GetCoordUsingGlobalPosition(
        const std::array<decimal_t, N_DIM> &p_w) const;
    /**
     * @brief Check if the input value is equal to the value in map
     *
     * @param p_w Input global position
     * @param val_in Input value
     * @param res Result
     * @return ErrorType
     */
    ErrorType CheckIfEqualUsingGlobalPosition(
        const std::array<decimal_t, N_DIM> &p_w, const T &val_in,
        bool *res) const;
    /**
     * @brief Check if the input value is equal to the value in map
     *
     * @param coord Input coordinate
     * @param val_in Input value
     * @param res Result
     * @return ErrorType
     */
    ErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                          const T &val_in, bool *res) const;

    /**
     * @brief Set the Value Using Coordinate
     *
     * @param coord Coordinate of the map
     * @param val Input value
     * @return ErrorType
     */
    ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                      const T &val);

    /**
     * @brief Set the Value Using Global Position
     *
     * @param p_w Global position
     * @param val Input value
     * @return ErrorType
     */
    ErrorType SetValueUsingGlobalPosition(
        const std::array<decimal_t, N_DIM> &p_w, const T &val);

    /**
     * @brief Get the Rounded Position Using Global Position object
     *
     * @param p_w Input global position
     * @return std::array<decimal_t, N_DIM> Output global position
     */
    std::array<decimal_t, N_DIM> GetRoundedPosUsingGlobalPosition(
        const std::array<decimal_t, N_DIM> &p_w) const;

    /**
     * @brief Get the Global Position Using Coordinate
     *
     * @param coord Input coordinate
     * @param p_w Output global position
     * @return ErrorType
     */
    ErrorType GetGlobalPositionUsingCoordinate(
        const std::array<int, N_DIM> &coord,
        std::array<decimal_t, N_DIM> *p_w) const;

    /**
     * @brief Get the Coordinate Using Global Metric On Single Dimension
     *
     * @param metric Input global 1-dim position
     * @param i Dimension
     * @param idx Output 1-d coordinate
     * @return ErrorType
     */
    ErrorType GetCoordUsingGlobalMetricOnSingleDim(const decimal_t &metric,
                                                   const int &i,
                                                   int *idx) const;

    /**
     * @brief Get the Global Metric Using Coordinate On Single Dim object
     *
     * @param idx Input 1-d coordinate
     * @param i Dimension
     * @param metric Output 1-d position
     * @return ErrorType
     */
    ErrorType GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                                   decimal_t *metric) const;

    /**
     * @brief Check if the input coordinate is in map range
     *
     * @param coord Input coordinate
     * @return true In range
     * @return false Out of range
     */
    bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

    /**
     * @brief Check if the input 1-d coordinate is in map range
     *
     * @param idx Input 1-d coordinate
     * @param i Dimension
     * @return true In range
     * @return false Out of range
     */
    bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

    /**
     * @brief Get the mono index using N-dim index
     *
     * @param idx Input N-dim index
     * @return int Output 1-dim index
     */
    int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

    /**
     * @brief Get N-dim index using mono index
     *
     * @param idx Input mono index
     * @return std::array<int, N_DIM> Output N-dim index
     */
    std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

private:
    /**
     * @brief Set the steps of N-dim array
     * @brief E.g. A x-y-z map's steps are {1,x,x*y}
     *
     * @param dims_size Input the size of dimension
     * @return ErrorType
     */
    ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

    /**
     * @brief Get the Data size
     *
     * @param dims_size Input the size of dimension
     * @return ErrorType
     */
    ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

    std::array<int, N_DIM> dims_size_;
    std::array<int, N_DIM> dims_step_;
    std::array<decimal_t, N_DIM> dims_resolution_;
    std::array<std::string, N_DIM> dims_name_;
    std::array<decimal_t, N_DIM> origin_;

    int data_size_{0};
    std::vector<T> data_;
};

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<decimal_t, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
    dims_size_ = dims_size;
    dims_resolution_ = dims_resolution;
    dims_name_ = dims_name;
    SetNDimSteps(dims_size_);
    SetDataSize(dims_size_);
    data_ = std::vector<T>(data_size_, 0);
    origin_.fill(0);
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
    int step = 1;
    for (int i = 0; i < N_DIM; ++i) {
        dims_step_[i] = step;
        step = step * dims_size[i];
    }
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
    int total_ele_num = 1;
    for (int i = 0; i < N_DIM; ++i) {
        total_ele_num = total_ele_num * dims_size_[i];
    }
    data_size_ = total_ele_num;
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, T *val) const {
    if (!CheckCoordInRange(coord)) {
        return kWrongStatus;
    }
    int idx = GetMonoIdxUsingNDimIdx(coord);
    *val = data_[idx];
    return kSuccess;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM> &coord) const {
    for (int i = 0; i < N_DIM; ++i) {
        if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
            return false;
        }
    }
    return true;
}

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM> &idx) const {
    int mono_idx = 0;
    for (int i = 0; i < N_DIM; ++i) {
        mono_idx += dims_step_[i] * idx[i];
    }
    return mono_idx;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, T *val) const {
    std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
    GetValueUsingCoordinate(coord, val);
    return kSuccess;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
    std::array<int, N_DIM> coord = {};
    for (int i = 0; i < N_DIM; ++i) {
        coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
    }
    return coord;
}
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val_in, bool *res) const {
    std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
    T val;
    if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
        *res = false;
    } else {
        *res = (val == val_in);
    }
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val_in, bool *res) const {
    T val;
    if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
        *res = false;
    } else {
        *res = (val == val_in);
    }
    return kSuccess;
}
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val) {
    if (!CheckCoordInRange(coord)) {
        // printf("[GridMapND] Out of range\n");
        return kWrongStatus;
    }
    int idx = GetMonoIdxUsingNDimIdx(coord);
    data_[idx] = val;
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val) {
    std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
    SetValueUsingCoordinate(coord, val);
    return kSuccess;
}

template <typename T, int N_DIM>
std::array<decimal_t, N_DIM>
GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
    std::array<int, N_DIM> coord = {};
    for (int i = 0; i < N_DIM; ++i) {
        coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
    }
    std::array<decimal_t, N_DIM> round_pos = {};
    for (int i = 0; i < N_DIM; ++i) {
        round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
    }
    return round_pos;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> &coord,
    std::array<decimal_t, N_DIM> *p_w) const {
    auto ptr = p_w->data();
    for (int i = 0; i < N_DIM; ++i) {
        *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
    }
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const decimal_t &metric, const int &i, int *idx) const {
    *idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
    return kSuccess;
}
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int &idx, const int &i, decimal_t *metric) const {
    *metric = idx * dims_resolution_[i] + origin_[i];
    return kSuccess;
}
template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int &idx,
                                                       const int &i) const {
    return (idx >= 0) && (idx < dims_size_[i]);
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int &idx) const {
    std::array<int, N_DIM> idx_nd = {};
    int tmp = idx;
    for (int i = 0; i < N_DIM - 1; --i) {
        idx_nd[i] = tmp / dims_step_[i];
        tmp = tmp % dims_step_[i];
    }
    return idx_nd;
}

struct DrivingCube {
    vec_E<Vec3i> seeds;
    AxisAlignedCubeNd<int, 3> cube;
};

struct DrivingCorridor {
    int id;
    bool is_valid;
    vec_E<DrivingCube> cubes;
};

template <int N_DIM>
struct SpatioTemporalSemanticCubeNd {
    decimal_t t_lb, t_ub;
    std::array<decimal_t, N_DIM> p_lb, p_ub;
    std::array<decimal_t, N_DIM> v_lb, v_ub;
    std::array<decimal_t, N_DIM> a_lb, a_ub;

    SpatioTemporalSemanticCubeNd() {
        FillDefaultBounds();
    }

    void FillDefaultBounds() {
        // ~ for optimization, infinity bound will cause numerical
        // ~ unstable. So we put some loose bounds by default
        t_lb = 0.0;
        t_ub = 1.0;

        const decimal_t default_pos_lb = -1;
        const decimal_t default_pos_ub = 1;
        const decimal_t default_vel_lb = -50.0;
        const decimal_t default_vel_ub = 50.0;
        const decimal_t default_acc_lb = -20.0;
        const decimal_t default_acc_ub = 20.0;

        p_lb.fill(default_pos_lb);
        v_lb.fill(default_vel_lb);
        a_lb.fill(default_acc_lb);

        p_ub.fill(default_pos_ub);
        v_ub.fill(default_vel_ub);
        a_ub.fill(default_acc_ub);
    }
};

/**
 * @brief Vehicle info under Frenet-frame
 */
struct FsVehicle {
    FrenetState frenet_state;
    vec_E<Vec2f> vertices;
};

}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis
#endif
