#ifndef LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
#define LBR_FRI_ROS2__ASYNC_CLIENT_HPP_

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "friClientVersion.h"
#include "friLBRClient.h"

#include "lbr_fri_ros2/formatting.hpp"
#include "lbr_fri_ros2/interfaces/base_command.hpp"
#include "lbr_fri_ros2/interfaces/position_command.hpp"
#include "lbr_fri_ros2/interfaces/state.hpp"
#include "lbr_fri_ros2/interfaces/torque_command.hpp"
#include "lbr_fri_ros2/interfaces/wrench_command.hpp"
#include "lbr_fri_ros2/interfaces/cartesian_pose_command.hpp"
// #include "lbr_fri_ros2/interfaces/cartesian_matrix_command.hpp"

namespace lbr_fri_ros2 {
class AsyncClient : public KUKA::FRI::LBRClient {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::AsyncClient";

public:
  AsyncClient() = delete;
  // Joint Overlay
  AsyncClient(const KUKA::FRI::EClientCommandMode &client_command_mode,
              const double &joint_position_tau, 
              const CommandGuardParametersJoint &command_guard_parameters,
              const std::string &command_guard_variant,
              const StateInterfaceParameters &state_interface_parameters = {0.04, 0.04},
              const bool &open_loop = true  // TODO: remove this `open_loop` or add support to cartesian commands 
              );

  // Cartesian Overlay
  AsyncClient(
              // TODO: also apply for postion filtering
              const CommandGuardParametersCartesian &command_guard_parameters,
              const std::string &command_guard_variant,
              const bool use_matrix_for_cartesian_pose = false,
              const StateInterfaceParameters &state_interface_parameters = {0.04, 0.04},
              const bool &open_loop = false  // TODO: remove this `open_loop` or add support to cartesian commands 
              );

  inline std::shared_ptr<BaseCommandInterface> get_command_interface() {
    return command_interface_ptr_;
  }
  inline std::shared_ptr<StateInterface> get_state_interface() { return state_interface_ptr_; }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

protected:
  std::shared_ptr<BaseCommandInterface> command_interface_ptr_;
  std::shared_ptr<StateInterface> state_interface_ptr_;

  bool joint_overlay_;
  bool open_loop_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__ASYNC_CLIENT_HPP_
