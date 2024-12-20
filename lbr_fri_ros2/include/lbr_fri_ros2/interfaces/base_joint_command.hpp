#ifndef LBR_FRI_ROS2__INTERACES__BASE_JOINT_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERACES__BASE_JOINT_COMMAND_HPP_

#include "lbr_fri_ros2/interfaces/base_command.hpp"
#include "lbr_fri_ros2/command_guard_joint.hpp"

namespace lbr_fri_ros2 {
class BaseJointCommandInterface: public BaseCommandInterface {
protected:
  virtual std::string LOGGER_NAME() const = 0;

public:
  BaseJointCommandInterface() = delete;
  BaseJointCommandInterface(const double &joint_position_tau,
                       const CommandGuardParametersJoint &command_guard_parameters,
                       const std::string &command_guard_variant = "default");

  void log_info() const override;
  void init_command(const_idl_state_t_ref state) override;
protected:
  JointExponentialFilterArray joint_position_filter_;
  std::unique_ptr<CommandGuardJoint> command_guard_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERACES__BASE_JOINT_COMMAND_HPP_
