#ifndef LBR_FRI_ROS2__COMMAND_GUARD_CARTEISAN_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_CARTEISAN_HPP_

#include "command_guard.hpp"

namespace lbr_fri_ros2 {

class CommandGuardParametersCartesian: public CommandGuardParameters{
public:
  cart_trans_array_t min_positions{0., 0., 0.};  /**< Minimum pose positions [mm].*/
  cart_trans_array_t max_positions{0., 0., 0.};  /**< Maximum pose positions [mm].*/
  double max_trans_vel; /**< Maximum pose velocities [mm/s].*/
  double max_trans_acc;    /**< Maximum pose accelerations [mm/s2].*/
  double max_rot_vel; /**< Maximum pose velocities [mm/s].*/
  double max_rot_acc;    /**< Maximum pose accelerations [mm/s2].*/
};

class CommandGuardCartesian: public CommandGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandGuard";

public:
  CommandGuardCartesian(const CommandGuardParametersCartesian &command_guard_parameters,
                        bool as_matrix = false);
  bool is_valid_command(const_idl_command_t_ref lbr_command,
                                const_idl_state_t_ref lbr_state) override;

  void log_info() const;

protected:
  bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref /*lbr_state*/) const;
  bool command_in_velocity_limits_(const_idl_state_t_ref lbr_state);
  // TODO
  // bool command_in_acceleration_limits_(const_idl_command_t_ref lbr_command,
  //                                        const_idl_state_t_ref lbr_state) const;

  CommandGuardParametersCartesian parameters_;
  bool prev_measured_pose_position_init_;
  const bool as_matrix; // TODO: better seperate them
  cart_pose_array_t prev_measured_pose_position_;
};

class SafeStopCommandGuardCartesian : public CommandGuardCartesian {
public:
  SafeStopCommandGuardCartesian(const CommandGuardParametersCartesian &command_guard_parameters)
      : CommandGuardCartesian(command_guard_parameters) {};

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref lbr_state) const override;
};

std::unique_ptr<CommandGuardCartesian>
command_guard_factory(const CommandGuardParametersCartesian &command_guard_parameters,
                      const std::string &variant = "default");
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_GUARD_CARTEISAN_HPP_
