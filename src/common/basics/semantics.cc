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
}  // namespace common
}  // namespace decision_lib
}  // namespace jarvis