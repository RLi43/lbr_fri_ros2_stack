#include "lbr_fri_ros2/interfaces/state.hpp"

namespace lbr_fri_ros2 {
StateInterface::StateInterface(
  const StateInterfaceParameters &state_interface_parameters,
  const bool use_joint_overlay)
    : state_initialized_(false), 
    parameters_(state_interface_parameters), 
    joint_overlay(use_joint_overlay) {}

void StateInterface::set_state(const_fri_state_t_ref state) {
#if FRI_CLIENT_VERSION_MAJOR == 1
  std::memcpy(state_.commanded_joint_position.data(), state.getCommandedJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
#endif
  std::memcpy(state_.commanded_torque.data(), state.getCommandedTorque(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  state_.connection_quality = state.getConnectionQuality();
  
  state_.client_command_mode = state.getClientCommandMode();
  state_.control_mode = state.getControlMode();

  state_.drive_state = state.getDriveState();
  // double mat[3][4];

  // TODO: no neeed to update ipo when there's no state change
  if (state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {      

    if(joint_overlay){      
      std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                  sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
    }else{
      std::memcpy(state_.ipo_cartesian_pose.data(), state.getIpoCartesianPose(),
                  sizeof(double) * CARTESIAN_QUAT_DOF);
    state_.ipo_redundancy_value = state.getIpoRedundancyValue();
    }
  }
  std::memcpy(state_.measured_joint_position.data(), state.getMeasuredJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  std::memcpy(state_.measured_cartesian_pose.data(), state.getMeasuredCartesianPose(),
              sizeof(double) * CARTESIAN_QUAT_DOF);
              
  state_.measured_redundancy_value = state.getMeasuredRedundancyValue();
  
  state_.redundancy_strategy = state.getRedundancyStrategy();  
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();
  state_.redundancy_strategy = state.getRedundancyStrategy();

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize state_.sample_time is available
    init_filters_();
  }

  // only compute after state_.sample_time is available
  external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);
  measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);
  // external_force_filter_.compute(state.getExternalForce(), state_.external_force);

  state_initialized_ = true;
};

// TODO: update this function for cartesian control
// in open loop mode, the measured joint position is refered as the commanded joint position
void StateInterface::set_state_open_loop(const_fri_state_t_ref state,
                                         const_idl_command_t_ref command,
                                         bool joint_overlay) {
  state_.client_command_mode = state.getClientCommandMode();
#if FRI_CLIENT_VERSION_MAJOR == 1
  std::memcpy(state_.commanded_joint_position.data(), state.getCommandedJointPosition(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
#endif
  std::memcpy(state_.commanded_torque.data(), state.getCommandedTorque(),
              sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  state_.connection_quality = state.getConnectionQuality();
  state_.control_mode = state.getControlMode();
  state_.drive_state = state.getDriveState();
  if (state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
      state.getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
    // TODO: switch by compile parameters
    if(joint_overlay){
      std::memcpy(state_.ipo_joint_position.data(), state.getIpoJointPosition(),
                  sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
    }else{
      std::memcpy(state_.ipo_cartesian_pose.data(), state.getIpoCartesianPose(),
                  sizeof(double) * CARTESIAN_QUAT_DOF);
    }
  }
  if(joint_overlay){
    std::memcpy(state_.measured_joint_position.data(), command.joint_position.data(),
                sizeof(double) * fri_state_t::NUMBER_OF_JOINTS);
  }else{
    std::memcpy(state_.measured_cartesian_pose.data(), command.cartesian_pose.data(),
                sizeof(double) * CARTESIAN_QUAT_DOF);
  }
  state_.operation_mode = state.getOperationMode();
  state_.overlay_type = state.getOverlayType();
  state_.safety_state = state.getSafetyState();
  state_.sample_time = state.getSampleTime();
  state_.session_state = state.getSessionState();
  state_.time_stamp_nano_sec = state.getTimestampNanoSec();
  state_.time_stamp_sec = state.getTimestampSec();
  state_.tracking_performance = state.getTrackingPerformance();
  state_.redundancy_strategy = state.getRedundancyStrategy();

  if (!external_torque_filter_.is_initialized() || !measured_torque_filter_.is_initialized()) {
    // initialize state_.sample_time is available
    init_filters_();
  }

  // only compute after state_.sample_time is available
  external_torque_filter_.compute(state.getExternalTorque(), state_.external_torque);
  measured_torque_filter_.compute(state.getMeasuredTorque(), state_.measured_torque);

  state_initialized_ = true;
}

void StateInterface::init_filters_() {
  external_torque_filter_.initialize(parameters_.external_torque_tau, state_.sample_time);
  measured_torque_filter_.initialize(parameters_.measured_torque_tau, state_.sample_time);
}

void StateInterface::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   external_torque_tau: %.5f s",
              parameters_.external_torque_tau);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   measured_torque_tau: %.5f s",
              parameters_.measured_torque_tau);
}
} // namespace lbr_fri_ros2
