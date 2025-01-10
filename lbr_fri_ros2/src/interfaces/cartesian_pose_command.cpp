#include "lbr_fri_ros2/interfaces/cartesian_pose_command.hpp"

namespace lbr_fri_ros2 {
CartesianPoseCommandInterface::CartesianPoseCommandInterface(
  const double cart_pose_tau,
  const CommandGuardParametersCartesian &command_guard_parameters,
  const std::string &command_guard_variant
)
    : BaseCartesianCommandInterface(
      cart_pose_tau, 
      command_guard_parameters, command_guard_variant) {
        
      }

void CartesianPoseCommandInterface::init_command(const_idl_state_t_ref state) {

  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
    "init cmd as MEASURED " << state.measured_cartesian_pose[0] 
    << ", " << state.measured_cartesian_pose[1] 
    << ", " << state.measured_cartesian_pose[2] 
    << ", " << state.measured_cartesian_pose[3] 
    << ", " << state.measured_cartesian_pose[4] 
    << ", " << state.measured_cartesian_pose[5] 
    << ", " << state.measured_cartesian_pose[6] 
    );
  command_target_.cartesian_pose = state.measured_cartesian_pose;
  command_target_.redundancy_value = state.ipo_redundancy_value;
  std::copy(command_target_.cartesian_pose.begin(), command_target_.cartesian_pose.end(), command_.cartesian_pose.begin());
  command_.redundancy_value = command_target_.redundancy_value;
}

void CartesianPoseCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {
  // // check command mode 
  // BaseCartesianCommandInterface::buffered_command_to_fri(command, state);
  
  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // sanity check -- make sure we have a valid command applied
  assert(std::all_of(command_.cartesian_pose.cbegin(), command_.cartesian_pose.cend(),
                  [](const double &v) { return !std::isnan(v); }) 
                  && command_guard_->is_valid_command(command_, state));
  
  bool use_command_directly = false;
  // check command target
  if (std::any_of(command_target_.cartesian_pose.cbegin(), command_target_.cartesian_pose.cend(),
                  [](const double &v) { return std::isnan(v); })){
    use_command_directly = true;
  }else{ // we have command target ordered
    // set target redundancy value
    command_target_.redundancy_value = (
      abs(command_target_.redundancy_value) > 3.0 ? state.ipo_redundancy_value : command_target_.redundancy_value);
    // TODO: check if IK solution exist, redundancy value
    if(!command_guard_->is_valid_command(command_target_, state, false)){
      use_command_directly = true;
      // TODO: project the target to a valid one?

      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
        "cmd target is not valid " << std::fixed << std::setprecision(2)
        << command_target_.cartesian_pose[0] 
        << ", " << command_target_.cartesian_pose[1] 
        << ", " << command_target_.cartesian_pose[2] 
        << std::fixed << std::setprecision(4)
        << ", " << command_target_.cartesian_pose[3] 
        << ", " << command_target_.cartesian_pose[4] 
        << ", " << command_target_.cartesian_pose[5] 
        << ", " << command_target_.cartesian_pose[6]
        << ". R=" << command_target_.redundancy_value 
        );

    }else{
      // check if target is very close to current sent command, 
      // if so, directly apply the target

      // Translation
      bool very_close = true;
      for(size_t idx = 0; idx < 3; ++idx){
        if(abs(command_.cartesian_pose[idx] - command_target_.cartesian_pose[idx]) > 1e-2){
          very_close = false;
          break;
        }
      }
      // Orientation // TODO: rot_angle?
      for(size_t idx = 3; idx < 7; ++idx){
        if(abs(command_.cartesian_pose[idx] - command_target_.cartesian_pose[idx]) > 1e-3){
          very_close = false;
          break;
        }
      }
      // redundancy value
      if(abs(command_.redundancy_value - command_target_.redundancy_value) > 1e-2/* rad */)
      {
        very_close = false;
      }

      if(very_close){
        std::copy(command_target_.cartesian_pose.begin(), command_target_.cartesian_pose.end(), 
                  command_.cartesian_pose.begin());
        command_.redundancy_value = command_target_.redundancy_value;
        use_command_directly = true;
      }
    }
  }
  
  if(!use_command_directly){    
    // exponential smooth
    if (!cartesian_pose_filter_.is_initialized()) {
      cartesian_pose_filter_.initialize(state.sample_time);
    }
    
    // smooth
    cartesian_pose_filter_.compute(command_target_.cartesian_pose, command_.cartesian_pose);
    // redundancy value    
    command_.redundancy_value = cartesian_pose_filter_.compute(
        command_target_.redundancy_value, command_.redundancy_value);
    
    // clamp
    // Translation
    double max_translation = command_guard_->get_trans_vel_limit() * state.sample_time;
    for(size_t idx = 0; idx < 3; ++idx){
      if(command_.cartesian_pose[idx] > state.measured_cartesian_pose[idx] + max_translation){        
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
          "clamp pose[" << idx << "] from " << command_.cartesian_pose[idx] 
          << " to " << state.measured_cartesian_pose[idx] << " + " << max_translation
        );
        command_.cartesian_pose[idx] = state.measured_cartesian_pose[idx] + max_translation;
      }
      else if(command_.cartesian_pose[idx] < state.measured_cartesian_pose[idx] - max_translation){
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
          "clamp pose[" << idx << "] from " << command_.cartesian_pose[idx] 
          << " to " << state.measured_cartesian_pose[idx] << " - " << max_translation
        );
        command_.cartesian_pose[idx] = state.measured_cartesian_pose[idx] - max_translation;
      }
    }
    // TODO: Orientation

    // Redundancy Value
    double max_movement = command_guard_->get_joint_vel_limit() * state.sample_time;
    if(command_.redundancy_value > state.measured_redundancy_value + max_movement){
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
          "clamp redundancy value from " << command_.redundancy_value 
          << " to " << state.measured_redundancy_value << " + " << max_movement
        );
      command_.redundancy_value = state.measured_redundancy_value + max_movement;
    }else if(command_.redundancy_value < state.measured_redundancy_value - max_movement){      
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
          "clamp redundancy value from " << command_.redundancy_value 
          << " to " << state.measured_redundancy_value << " - " << max_movement
        );
      command_.redundancy_value = state.measured_redundancy_value - max_movement;
    }    

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
      "cmd smooth " << std::fixed << std::setprecision(2)
      << command_.cartesian_pose[0] 
      << ", " << command_.cartesian_pose[1] 
      << ", " << command_.cartesian_pose[2] 
      << std::fixed << std::setprecision(4)
      << ", " << command_.cartesian_pose[3] 
      << ", " << command_.cartesian_pose[4] 
      << ", " << command_.cartesian_pose[5] 
      << ", " << command_.cartesian_pose[6] 
      << ". R=" << command_.redundancy_value
      );

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
      "measured   " << std::fixed << std::setprecision(2)
      << state.measured_cartesian_pose[0] 
      << ", " << state.measured_cartesian_pose[1] 
      << ", " << state.measured_cartesian_pose[2] 
      << std::fixed << std::setprecision(4)
      << ", " << state.measured_cartesian_pose[3] 
      << ", " << state.measured_cartesian_pose[4] 
      << ", " << state.measured_cartesian_pose[5] 
      << ", " << state.measured_cartesian_pose[6] 
      << ". R=" << state.measured_redundancy_value
      );

      // hold the assumption that if previous and current are valid, the filtered value in between is also valid
  }

  // write cartesian pose to output
  // double* redundancy_value = nullptr;
  // if(command_.redundancy_commanded) redundancy_value = &command_.redundancy_value;
  // command.setCartesianPose(command_.cartesian_pose.data(), redundancy_value);
  if(abs(command_.redundancy_value) > 3.0){
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()), "Redundancy value should be specified");
    command.setCartesianPose(command_.cartesian_pose.data());
  }else{
    command.setCartesianPose(command_.cartesian_pose.data(), &command_.redundancy_value);
  }
}
} // namespace lbr_fri_ros2
