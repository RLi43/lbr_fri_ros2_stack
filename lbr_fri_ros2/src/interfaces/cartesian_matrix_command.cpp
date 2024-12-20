#include "lbr_fri_ros2/interfaces/cartesian_matrix_command.hpp"

namespace lbr_fri_ros2 {
CartesianMatrixCommandInterface::CartesianMatrixCommandInterface(
  const double &cart_pose_tau,
  const CommandGuardParameters &command_guard_parameters,
  const std::string &command_guard_variant
)
    : BaseCartesianCommandInterface(
      cart_pose_tau, command_guard_parameters, command_guard_variant) {}

void CartesianMatrixCommandInterface::init_command(const_idl_state_t_ref state) {
  command_target_.cartesian_matrix = state.measured_cartesian_matrix

  // command_target_.redundancy_commanded = false;
  // command_target_.redundancy_value = 0.;
  command_ = command_target_;
}

void CartesianMatrixCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {
  BaseCartesianCommandInterface::buffered_command_to_fri(command, state);

  if (std::any_of(command_target_.cartesian_pose.cbegin(), command_target_.cartesian_pose.cend(),
                  [](const double &v) { return std::isnan(v); })) {
    this->init_command(state);
  }

  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // // exponential smooth
  // if (!joint_position_filter_.is_initialized()) {
  //   joint_position_filter_.initialize(state.sample_time);
  // }
  // joint_position_filter_.compute(command_target_.joint_position, command_.joint_position);

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // write cartesian transform matrix to output
  double mat[3][4];
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < 4; ++j){
      mat[i][j] = command_.cartesian_matrix[i*4 + j];
    }
  }
  double* redundancy_value = nullptr;
  if(command_.redundancy_commanded) redundancy_value = &command_.redundancy_value;
  command.setCartesianPoseAsMatrix(mat, redundancy_value);
}
} // namespace lbr_fri_ros2
