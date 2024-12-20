#include "lbr_fri_ros2/interfaces/base_cartesian_command.hpp"

namespace lbr_fri_ros2 {

BaseCartesianCommandInterface::BaseCartesianCommandInterface(
                      //  const double &cart_pose_tau,
                       const CommandGuardParametersCartesian &command_guard_parameters,
                       const std::string &command_guard_variant)
                       {
  command_guard_ = command_guard_factory(command_guard_parameters, command_guard_variant);
 };

void BaseCartesianCommandInterface::log_info() const {
  command_guard_->log_info();
}

void BaseCartesianCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                       const_idl_state_t_ref state) {

#if FRI_CLIENT_VERSION_MAJOR >= 2
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::CARTESIAN_POSE) {
    std::string err =
        "Expected robot in " +
        EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::CARTESIAN_POSE) +
        " command mode.";
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME()), err.c_str());
    throw std::runtime_error(err);
  }
#endif

}

} // namespace lbr_fri_ros2
