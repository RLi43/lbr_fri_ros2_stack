#include "lbr_fri_ros2/interfaces/base_joint_command.hpp"

namespace lbr_fri_ros2 {

BaseJointCommandInterface::BaseJointCommandInterface(const double &joint_position_tau,
                                           const CommandGuardParametersJoint &command_guard_parameters,
                                           const std::string &command_guard_variant)
    : joint_position_filter_(joint_position_tau) {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
  };

void BaseJointCommandInterface::log_info() const {
  command_guard_->log_info();
  joint_position_filter_.log_info();
}

void BaseJointCommandInterface::init_command(const_idl_state_t_ref state) {
  command_target_.joint_position = state.measured_joint_position; // TODO: should we use measured or interpolated?
  command_target_.torque.fill(0.);
  command_target_.wrench.fill(0.);
  // command_target_.cartesian_pose = state.measured_cartesian_pose;
  // command_target_.cartesian_matrix.fill(0.);
  // command_target_.redundancy_commanded = false;
  // command_target_.redundancy_value = 0.;
  command_ = command_target_;
}
} // namespace lbr_fri_ros2
