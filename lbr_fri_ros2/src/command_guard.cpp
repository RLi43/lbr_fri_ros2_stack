#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {
bool CommandGuard::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_idl_state_t_ref lbr_state,
                                    bool vel_check) {
  if (!command_in_position_limits_(lbr_command, lbr_state)) {
    return false;
  }
  if (vel_check && !command_in_velocity_limits_(lbr_command, lbr_state)) {
    return false;
  }
  return true;
}
} // namespace lbr_fri_ros2
