#ifndef LBR_FRI_ROS2__COMMAND_GUARD_HPP_
#define LBR_FRI_ROS2__COMMAND_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"
#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_command.hpp"
#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
class CommandGuardParameters{
public:
  CommandGuardParameters(){};
};

class CommandGuard {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::CommandGuard";

public:
  CommandGuard() = default;
  // CommandGuard(const CommandGuardParameters &command_guard_parameters);
  virtual bool is_valid_command(const_idl_command_t_ref lbr_command,
                                const_idl_state_t_ref lbr_state,
                                bool vel_check = true);

  virtual void log_info() const = 0;

protected:
  virtual bool command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref /*lbr_state*/) const = 0;
  virtual bool command_in_velocity_limits_(const_idl_command_t_ref lbr_command,
                                           const_idl_state_t_ref lbr_state) = 0;
  // virtual bool command_in_torque_limits_(const_idl_command_t_ref lbr_command,
  //                                        const_idl_state_t_ref lbr_state) const = 0;

};

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant = "default");
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__COMMAND_GUARD_HPP_
