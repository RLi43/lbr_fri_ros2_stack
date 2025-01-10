#ifndef LBR_FRI_ROS2__COMMAND_GUARD_JOINT_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_JOINT_HPP_

#include "command_guard.hpp"

namespace lbr_fri_ros2 {
class CommandGuardParametersJoint: public CommandGuardParameters{
public:
  jnt_name_array_t joint_names;                           /**< Joint names.*/
  jnt_array_t min_positions{0., 0., 0., 0., 0., 0., 0.};  /**< Minimum joint positions [rad].*/
  jnt_array_t max_positions{0., 0., 0., 0., 0., 0., 0.};  /**< Maximum joint positions [rad].*/
  jnt_array_t max_velocities{0., 0., 0., 0., 0., 0., 0.}; /**< Maximum joint velocities [rad/s].*/
  jnt_array_t max_torques{0., 0., 0., 0., 0., 0., 0.};    /**< Maximum joint torque [Nm].*/
};

class CommandGuardJoint : public CommandGuard {

public:
  CommandGuardJoint(const CommandGuardParametersJoint &command_guard_parameters);
  // virtual bool is_valid_command(const_idl_command_t_ref lbr_command,
  //                               const_idl_state_t_ref lbr_state);

  void log_info() const;

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref /*lbr_state*/) const;
  virtual bool command_in_velocity_limits_(const_idl_command_t_ref lbr_command,
                                         const_idl_state_t_ref lbr_state);
  virtual bool command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                         const_idl_state_t_ref lbr_state) const;

  CommandGuardParametersJoint parameters_;
  bool prev_measured_joint_position_init_;
  jnt_array_t prev_measured_joint_position_;
};

class SafeStopCommandGuardJoint : public CommandGuardJoint {
public:
  SafeStopCommandGuardJoint(const CommandGuardParametersJoint &command_guard_parameters)
      : CommandGuardJoint(command_guard_parameters) {};

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref lbr_state) const override;
};

std::unique_ptr<CommandGuardJoint>
command_guard_factory(const CommandGuardParametersJoint &command_guard_parameters,
                      const std::string &variant = "default");
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_GUARD_JOINT_HPP_
