#include "lbr_fri_ros2/interfaces/cartesian_pose_command.hpp"

namespace lbr_fri_ros2 {
CartesianPoseCommandInterface::CartesianPoseCommandInterface(
  // const double &cart_pose_tau,
  const CommandGuardParametersCartesian &command_guard_parameters,
  const std::string &command_guard_variant
)
    : BaseCartesianCommandInterface(
      // cart_pose_tau, 
      command_guard_parameters, command_guard_variant) {}

void CartesianPoseCommandInterface::init_command(const_idl_state_t_ref state) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
    "Initializing Command by IPO Pose"
  );
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
    "MEASURED " << state.measured_cartesian_pose[0] 
    << ", " << state.measured_cartesian_pose[1] 
    << ", " << state.measured_cartesian_pose[2] 
    << ", " << state.measured_cartesian_pose[3] 
    << ", " << state.measured_cartesian_pose[4] 
    << ", " << state.measured_cartesian_pose[5] 
    << ", " << state.measured_cartesian_pose[6] 
    );
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
    "IPO_CART_POSE " << state.ipo_cartesian_pose[0] 
    << ", " << state.ipo_cartesian_pose[1] 
    << ", " << state.ipo_cartesian_pose[2] 
    << ", " << state.ipo_cartesian_pose[3] 
    << ", " << state.ipo_cartesian_pose[4] 
    << ", " << state.ipo_cartesian_pose[5] 
    << ", " << state.ipo_cartesian_pose[6] 
    );

  command_target_.cartesian_pose = state.ipo_cartesian_pose;
  // command_target_.cartesian_matrix.fill(0.);
  // command_target_.redundancy_commanded = false;
  // command_target_.redundancy_value = 0.;
  command_ = command_target_;
}

void CartesianPoseCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {
  // check command mode 
  BaseCartesianCommandInterface::buffered_command_to_fri(command, state);

  if (std::any_of(command_target_.cartesian_pose.cbegin(), command_target_.cartesian_pose.cend(),
                  [](const double &v) { return std::isnan(v); })) {

    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        "Empty command target, initializing with ipo pose");
    this->init_command(state);
  }

  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }


  // // exponential smooth
  // TODO
  // if (!joint_position_filter_.is_initialized()) {
  //   joint_position_filter_.initialize(state.sample_time);
  // }
  // joint_position_filter_.compute(command_target_.joint_position, command_.joint_position);

  // TODO: Dangerous! command_target may be initialized as 0.0 ... 
  command_ = command_target_;

  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
    "Validating Command " << command_.cartesian_pose[0] 
    << ", " << command_.cartesian_pose[1] 
    << ", " << command_.cartesian_pose[2] 
    << ", " << command_.cartesian_pose[3] 
    << ", " << command_.cartesian_pose[4] 
    << ", " << command_.cartesian_pose[5] 
    << ", " << command_.cartesian_pose[6] 
    );

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string err = "Invalid command. Using IPO Cartesian Pose";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    // TODO: give the command projected to the workspace
    // throw std::runtime_error(err);
    // TODO: should we just exit?
    if (std::any_of(state.ipo_cartesian_pose.cbegin(), state.ipo_cartesian_pose.cend(),
                    [](const double &v) { return std::isnan(v); })) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR 
                        << "IPO is not available, using measured"
                        << ColorScheme::ENDC);
                      command_.cartesian_pose = state.measured_cartesian_pose;
    }else command_.cartesian_pose = state.ipo_cartesian_pose;
  }

  // RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
  //   "Buffering Command " << command_.cartesian_pose[0] 
  //   << ", " << command_.cartesian_pose[1] 
  //   << ", " << command_.cartesian_pose[2] 
  //   << ", " << command_.cartesian_pose[3] 
  //   << ", " << command_.cartesian_pose[4] 
  //   << ", " << command_.cartesian_pose[5] 
  //   << ", " << command_.cartesian_pose[6] 
  //   );

  // write cartesian pose to output
  // double* redundancy_value = nullptr;
  // if(command_.redundancy_commanded) redundancy_value = &command_.redundancy_value;
  // command.setCartesianPose(command_.cartesian_pose.data(), redundancy_value);
  command.setCartesianPose(command_.cartesian_pose.data());
}
} // namespace lbr_fri_ros2
