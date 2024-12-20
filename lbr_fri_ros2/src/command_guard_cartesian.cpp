#include "lbr_fri_ros2/command_guard_cartesian.hpp"

namespace lbr_fri_ros2 {
CommandGuardCartesian::CommandGuardCartesian(
  const CommandGuardParametersCartesian &command_guard_parameters, bool as_matrix)
    : parameters_(command_guard_parameters), 
    prev_measured_pose_position_init_(false), 
    as_matrix(as_matrix) {

    };

bool CommandGuardCartesian::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_idl_state_t_ref lbr_state) {
  if (!command_in_position_limits_(lbr_command, lbr_state)) {
    return false;
  }
  if (!command_in_velocity_limits_(lbr_state)) {
    return false;
  }
  return true;
}

void CommandGuardCartesian::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  for (std::size_t i = 0; i < CART_POSE_TRANS_NUM; ++i) {
    RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_NAME),
        "*   Axis %s limits: Position: [%.1f, %.1f] mm, velocity: %.1f mm/s, acceleration: %.1f mm/s2",
        CART_POSE_TRANS_NAME[i].data(), 
        parameters_.min_positions[i], parameters_.max_positions[i], 
        parameters_.max_trans_vel, parameters_.max_trans_acc);
  }
  RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_NAME),
      "*   Rotational limits: velocity: %.1f rad/s, acceleration: %.1f rad/s2",
      parameters_.max_rot_vel, parameters_.max_rot_acc);

}

bool CommandGuardCartesian::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                               const_idl_state_t_ref /*lbr_state*/) const {
  for (std::size_t i = 0; i < CART_POSE_TRANS_NUM; ++i) {
    if (lbr_command.cartesian_pose[i] < parameters_.min_positions[i] ||
        lbr_command.cartesian_pose[i] > parameters_.max_positions[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for axis '"
                                             << CART_POSE_TRANS_NAME[i].data() << "' "
                                             << lbr_command.cartesian_pose[i] << " is not in range["
                                             << parameters_.min_positions[i] << ", "
                                             << parameters_.max_positions[i] << "]"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool CommandGuardCartesian::command_in_velocity_limits_(const_idl_state_t_ref lbr_state) {
  // translation
  if (!prev_measured_pose_position_init_) {
    prev_measured_pose_position_init_ = true;
    prev_measured_pose_position_ = lbr_state.measured_cartesian_pose;
    return true;
  }
  const double &dt = lbr_state.sample_time;
  for (std::size_t i = 0; i < CART_POSE_TRANS_NUM; ++i) {
    double diff = std::abs(prev_measured_pose_position_[i] - lbr_state.measured_cartesian_pose[i]);
    if ((diff > 1e-6) && (diff / dt > parameters_.max_trans_vel)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Velocity not in limits for axis '"
                                             << CART_POSE_TRANS_NAME[i].data() << "'"
                                             << " prev: " << prev_measured_pose_position_[i]
                                             << " curr: " << lbr_state.measured_cartesian_pose[i]
                                             << " dt: " << dt
                                             << " diff: " << diff
                                             << " -> vel=" << diff / dt
                                             << " > limit(" << parameters_.max_trans_vel << ")"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  prev_measured_pose_position_ = lbr_state.measured_cartesian_pose;

  // TODO: calculate the rotation
  
  return true;
  /*
  // TODO: support for as matrix
  const double &dt = lbr_state.sample_time;
  if (!prev_measured_joint_position_init_) {
    prev_measured_joint_position_init_ = true;
    prev_measured_joint_position_ = lbr_state.measured_joint_position;
    return true;
  }
  for (std::size_t i = 0; i < lbr_state.measured_joint_position.size(); ++i) {
    if (std::abs(prev_measured_joint_position_[i] - lbr_state.measured_joint_position[i]) / dt >
        parameters_.max_velocities[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Velocity not in limits for pose '"
                                             << parameters_.pose_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  prev_measured_pose_position_ = lbr_state.measured_cartesian_pose;
  return true;
  */
}

// bool CommandGuardCartesian::command_in_torque_limits_(const_idl_command_t_ref lbr_command,
//                                              const_idl_state_t_ref lbr_state) const {
//   for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
//     if (std::abs(lbr_command.torque[i] + lbr_state.external_torque[i]) >
//         parameters_.max_torques[i]) {
//       RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME), ColorScheme::ERROR
//                                                                << "Torque not in limits for joint '"
//                                                                << parameters_.joint_names[i].c_str()
//                                                                << "'" << ColorScheme::ENDC);
//       return false;
//     }
//   }
//   return true;
// }

bool SafeStopCommandGuardCartesian::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                                       const_idl_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < CART_POSE_TRANS_NUM; ++i) {
    if (lbr_command.cartesian_pose[i] <
            parameters_.min_positions[i] + parameters_.max_trans_vel* lbr_state.sample_time ||
        lbr_command.cartesian_pose[i] >
            parameters_.max_positions[i] - parameters_.max_trans_vel * lbr_state.sample_time) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for axis '"
                                             << CART_POSE_TRANS_NAME[i].data() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

std::unique_ptr<CommandGuardCartesian>
command_guard_factory(const CommandGuardParametersCartesian &command_guard_parameters,
                      const std::string &variant) {
  constexpr char LOGGER_NAME[] = "lbr_fri_ros2::command_guard_factory";
  if (variant == "default") {
    return std::make_unique<CommandGuardCartesian>(command_guard_parameters);
  }
  if (variant == "safe_stop") {
    return std::make_unique<SafeStopCommandGuardCartesian>(command_guard_parameters);
  }
  std::string err = "Invalid CommandGuard variant provided.";
  RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                      ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
  throw std::runtime_error(err);
}
} // namespace lbr_fri_ros2
