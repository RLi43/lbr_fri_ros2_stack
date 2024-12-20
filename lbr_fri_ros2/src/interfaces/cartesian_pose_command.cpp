#include "lbr_fri_ros2/interfaces/cartesian_pose_command.hpp"

namespace lbr_fri_ros2 {
CartesianPoseCommandInterface::CartesianPoseCommandInterface(
  const double &cart_pose_tau,
  const CommandGuardParametersCartesian &command_guard_parameters,
  const std::string &command_guard_variant
)
    : BaseCartesianCommandInterface(
      cart_pose_tau, 
      command_guard_parameters, command_guard_variant) {
        last_valid_command_.fill(std::numeric_limits<double>::quiet_NaN());
      }

void CartesianPoseCommandInterface::init_command(const_idl_state_t_ref state) {
  
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

  
  if (std::any_of(last_valid_command_.cbegin(), last_valid_command_.cend(),
                  [](const double &v) { return std::isnan(v); })) {
    // TODO: check if it's too far away from measured pose
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
      "Initializing Command by IPO Pose"
    );
    command_target_.cartesian_pose = state.ipo_cartesian_pose;
  }else{
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
      "Initializing Command by last valid command"
    );
    command_target_.cartesian_pose = last_valid_command_;
  }
                  
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


  // exponential smooth
  if (!cartesian_pose_filter_.is_initialized()) {
    cartesian_pose_filter_.initialize(state.sample_time);
  }
  cartesian_pose_filter_.compute(command_target_.cartesian_pose, command_.cartesian_pose);

  // // TODO: Dangerous! command_target may be initialized as 0.0 ... 
  // command_ = command_target_;

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
    std::string err = "Invalid command. Using Measured Cartesian Pose";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    // TODO: give the command projected to the workspace
    // TODO: should we just exit?
    // throw std::runtime_error(err);


    if (std::any_of(last_valid_command_.cbegin(), last_valid_command_.cend(),
                    [](const double &v) { return std::isnan(v); })) {
      // TODO: check if it's too far away from measured pose
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
        "Giving measured pose"
      );
      command_.cartesian_pose = state.measured_cartesian_pose;
    }else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
        "Giving last valid command"
      );
      command_.cartesian_pose = last_valid_command_;
    }
    

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
      "MEASURED " << state.measured_cartesian_pose[0] 
      << ", " << state.measured_cartesian_pose[1] 
      << ", " << state.measured_cartesian_pose[2] 
      << ", " << state.measured_cartesian_pose[3] 
      << ", " << state.measured_cartesian_pose[4] 
      << ", " << state.measured_cartesian_pose[5] 
      << ", " << state.measured_cartesian_pose[6] 
      );    
  }else{
    last_valid_command_ = command_.cartesian_pose;
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
