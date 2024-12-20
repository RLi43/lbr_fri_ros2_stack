#ifndef LBR_FRI_ROS2__INTERFACES__CARTESIAN_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__CARTESIAN_COMMAND_HPP_

#include <algorithm>

#include "lbr_fri_ros2/interfaces/base_command.hpp"
#include "lbr_fri_ros2/command_guard_cartesian.hpp"

namespace lbr_fri_ros2 {
class BaseCartesianCommandInterface : public BaseCommandInterface {
protected:
  virtual std::string LOGGER_NAME() const = 0;

public:
  BaseCartesianCommandInterface() = delete;
  BaseCartesianCommandInterface(
                       const double &cart_pose_tau,
                       const CommandGuardParametersCartesian &command_guard_parameters,
                       const std::string &command_guard_variant = "default");

  void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state);

  void log_info() const override;
  // virtual void init_command(const_idl_state_t_ref state) = 0;
  // TODO: deal with the redundancy value
  CartesianExponentialFilterArray cartesian_pose_filter_;
  std::unique_ptr<CommandGuardCartesian> command_guard_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__CARTESIAN_COMMAND_HPP_
