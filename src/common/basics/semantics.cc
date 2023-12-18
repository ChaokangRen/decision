#include "basics.h"
#include "semantics.h"
namespace jarvis {
namespace decision_lib {
namespace common {
void VehicleParam::print() const {
    sprintf("VehicleParam:\n");
    sprintf(" -- width:\t %lf.\n", width_);
    sprintf(" -- length:\t %lf.\n", length_);
    sprintf(" -- wheel_base:\t %lf.\n", wheel_base_);
    sprintf(" -- front_suspension:\t %lf.\n", front_suspension_);
    sprintf(" -- rear_suspension:\t %lf.\n", rear_suspension_);
    sprintf(" -- d_cr:\t %lf.\n", d_cr_);
    sprintf(" -- max_steering_angle:\t %lf.\n", max_steering_angle_);
    sprintf(" -- max_longitudinal_acc:\t %lf.\n", max_longitudinal_acc_);
    sprintf(" -- max_lateral_acc:\t %lf.\n", max_lateral_acc_);
}

Vehicle::Vehicle() {}

Vehicle::Vehicle(const VehicleParam &param, const State &state)
    : param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const VehicleParam &param, const State &state)
    : id_(id), param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const std::string &subclass,
                 const VehicleParam &param, const State &state)
    : id_(id), subclass_(subclass), param_(param), state_(state) {}

Vec3f Vehicle::Ret3DofState() const {
    return Vec3f(state_.vec_position(0), state_.vec_position(1), state_.angle);
}

ErrorType Vehicle::Ret3DofStateAtGeometryCenter(Vec3f *state) const {
    decimal_t cos_theta = cos(state_.angle);
    decimal_t sin_theta = sin(state_.angle);
    decimal_t x = state_.vec_position(0) + param_.d_cr() * cos_theta;
    decimal_t y = state_.vec_position(1) + param_.d_cr() * sin_theta;
    (*state)(0) = x;
    (*state)(1) = y;
    (*state)(2) = state_.angle;
    return kSuccess;
}

void Vehicle::print() const {
    sprintf("\nVehicle:\n");
    sprintf(" -- ID:\t%d\n", id_);
    sprintf(" -- Subclass:\t%s\n", subclass_.c_str());
    param_.print();
    state_.print();
}

void SemanticLaneSet::print() const {
    sprintf("SemanticLaneSet:\n");
    sprintf(" -- Number of lanes:\t%d\n",
            static_cast<int>(semantic_lanes.size()));
}
TrafficSignal::TrafficSignal()
    : start_point_(Vec2f::Zero()),
      end_point_(Vec2f::Zero()),
      valid_time_(Vec2f(0.0, std::numeric_limits<decimal_t>::max())),
      vel_range_(Vec2f::Zero()),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

TrafficSignal::TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                             const Vec2f &valid_time, const Vec2f &vel_range)
    : start_point_(start_point),
      end_point_(end_point),
      valid_time_(valid_time),
      vel_range_(vel_range),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

void TrafficSignal::set_start_point(const Vec2f &start_point) {
    start_point_ = start_point;
}

void TrafficSignal::set_end_point(const Vec2f &end_point) {
    end_point_ = end_point;
}

void TrafficSignal::set_valid_time_til(const decimal_t max_valid_time) {
    valid_time_(1) = max_valid_time;
}

void TrafficSignal::set_valid_time_begin(const decimal_t min_valid_time) {
    valid_time_(0) = min_valid_time;
}

void TrafficSignal::set_valid_time(const Vec2f &valid_time) {
    valid_time_ = valid_time;
}

void TrafficSignal::set_vel_range(const Vec2f &vel_range) {
    vel_range_ = vel_range;
}

void TrafficSignal::set_lateral_range(const Vec2f &lateral_range) {
    lateral_range_ = lateral_range;
}

void TrafficSignal::set_max_velocity(const decimal_t max_velocity) {
    vel_range_(1) = max_velocity;
}

Vec2f TrafficSignal::start_point() const {
    return start_point_;
}
Vec2f TrafficSignal::end_point() const {
    return end_point_;
}
Vec2f TrafficSignal::valid_time() const {
    return valid_time_;
}
Vec2f TrafficSignal::vel_range() const {
    return vel_range_;
}
Vec2f TrafficSignal::lateral_range() const {
    return lateral_range_;
}
decimal_t TrafficSignal::max_velocity() const {
    return vel_range_(1);
}
SpeedLimit::SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
                       const Vec2f &vel_range)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    vel_range) {}

StoppingSign::StoppingSign(const Vec2f &start_point, const Vec2f &end_point)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    Vec2f::Zero()) {}

void TrafficLight::set_type(const Type &type) {
    type_ = type;
}

TrafficLight::Type TrafficLight::type() const {
    return type_;
}

ErrorType SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
    const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb) {
    double cos_theta = cos(s.angle);
    double sin_theta = sin(s.angle);
    obb->x = s.vec_position[0] + param.d_cr() * cos_theta;
    obb->y = s.vec_position[1] + param.d_cr() * sin_theta;
    obb->angle = s.angle;
    obb->length = param.length();
    obb->width = param.width();
    return kSuccess;
}

ErrorType SemanticsUtils::GetVehicleVertices(const VehicleParam &param,
                                             const State &state,
                                             vec_E<Vec2f> *vertices) {
    decimal_t angle = state.angle;

    decimal_t cos_theta = cos(angle);
    decimal_t sin_theta = sin(angle);

    decimal_t c_x = state.vec_position(0) + param.d_cr() * cos_theta;
    decimal_t c_y = state.vec_position(1) + param.d_cr() * sin_theta;

    decimal_t d_wx = param.width() / 2 * sin_theta;
    decimal_t d_wy = param.width() / 2 * cos_theta;
    decimal_t d_lx = param.length() / 2 * cos_theta;
    decimal_t d_ly = param.length() / 2 * sin_theta;

    // Counterclockwise from left-front vertex
    vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
    // vertices->push_back(Vec2f(c_x - d_wx, c_y + d_wy));
    vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
    // vertices->push_back(Vec2f(c_x - d_lx, c_y - d_ly));
    vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
    // vertices->push_back(Vec2f(c_x + d_wx, c_y - d_wy));
    vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
    // vertices->push_back(Vec2f(c_x + d_lx, c_y + d_ly));

    return kSuccess;
}

ErrorType SemanticsUtils::InflateVehicleBySize(const Vehicle &vehicle_in,
                                               const decimal_t delta_w,
                                               const decimal_t delta_l,
                                               Vehicle *vehicle_out) {
    common::Vehicle inflated_vehicle = vehicle_in;
    common::VehicleParam vehicle_param = vehicle_in.param();
    vehicle_param.set_width(vehicle_param.width() + delta_w);
    vehicle_param.set_length(vehicle_param.length() + delta_l);
    inflated_vehicle.set_param(vehicle_param);
    *vehicle_out = inflated_vehicle;
    return kSuccess;
}
}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis