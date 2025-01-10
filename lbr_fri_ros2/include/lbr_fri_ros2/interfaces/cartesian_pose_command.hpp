#ifndef LBR_FRI_ROS2__INTERFACES__CARTESIAN_POSE_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__CARTESIAN_POSE_COMMAND_HPP_

#include <algorithm>

#include "lbr_fri_ros2/interfaces/base_cartesian_command.hpp"

namespace lbr_fri_ros2 {
class CartesianPoseCommandInterface : public BaseCartesianCommandInterface {
protected:
  std::string LOGGER_NAME() const override { return "lbr_fri_ros2::CartesianPoseCommandInterface"; }

public:
  CartesianPoseCommandInterface() = delete;
  CartesianPoseCommandInterface(
    const double cart_pose_trans_tau,
    const CommandGuardParametersCartesian &command_guard_parameters,
    const std::string &command_guard_variant = "default");

  void init_command(const_idl_state_t_ref state) override;

  void buffered_command_to_fri(fri_command_t_ref command, const_idl_state_t_ref state) override;

};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__CARTESIAN_POSE_COMMAND_HPP_
