#include "lbr_ros2_control/controllers/lbr_cartesian_pose_command_controller.hpp"

namespace lbr_ros2_control {
LBRCartesianPoseCommandController::LBRCartesianPoseCommandController()
    : rt_lbr_cartesian_pose_command_ptr_(nullptr),
      lbr_cartesian_pose_command_subscription_ptr_(nullptr) {}

controller_interface::InterfaceConfiguration
LBRCartesianPoseCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_X);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_Y);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_Z);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_QW);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_QX);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_QY);
  interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_GPIO_PREFIX) + "/" + HW_IF_CARTESIAN_POSE_QZ);
  // interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_SENSOR_PREFIX) + "/" + HW_IF_REDUNDANCY_VALUE_CONTROLLED);
  // interface_configuration.names.push_back(std::string(HW_IF_CARTESIAN_SENSOR_PREFIX) + "/" + HW_IF_REDUNDANCY_VALUE);
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRCartesianPoseCommandController::state_interface_configuration() const {
  // TODO: should we add measured values?
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LBRCartesianPoseCommandController::on_init() {
  try {
    lbr_cartesian_pose_command_subscription_ptr_ =
        this->get_node()->create_subscription<lbr_fri_idl::msg::LBRCartesianPoseCommand>(
            "command/cartesian_pose", 1,
            [this](const lbr_fri_idl::msg::LBRCartesianPoseCommand::SharedPtr msg) {
              rt_lbr_cartesian_pose_command_ptr_.writeFromNonRT(msg);
            });
    if (!this->get_node()->has_parameter("robot_name")) {
      this->get_node()->declare_parameter("robot_name", "lbr");
    }
    configure_cartesian_names_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize LBR cartesian pose command controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRCartesianPoseCommandController::update(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration & /*period*/) {
  auto lbr_cartesian_pose_command = rt_lbr_cartesian_pose_command_ptr_.readFromRT();
  if (!lbr_cartesian_pose_command || !(*lbr_cartesian_pose_command)) {
    return controller_interface::return_type::OK;
  }
  // RCLCPP_INFO_STREAM(
  //       this->get_node()->get_logger(),
  //       "Giving Command: "
  //       << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[0]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[1]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[2]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[3]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[4]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[5]
  //       << ", " << (*lbr_cartesian_pose_command)->cartesian_pose_quaternion[6]
  //       );
  for(std::size_t idx = 0; idx < lbr_fri_ros2::CARTESIAN_QUAT_DOF; ++idx){
    command_interfaces_[idx].set_value((*lbr_cartesian_pose_command)->cartesian_pose_quaternion[idx]);
  }
  // command_interfaces_[lbr_fri_ros2::CARTESIAN_QUAT_DOF].set_value(
  //   (*lbr_cartesian_pose_command)->redundancy_cmd
  // );
  // command_interfaces_[lbr_fri_ros2::CARTESIAN_QUAT_DOF + 1].set_value(
  //   (*lbr_cartesian_pose_command)->reduncancy_value
  // );

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LBRCartesianPoseCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRCartesianPoseCommandController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LBRCartesianPoseCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void LBRCartesianPoseCommandController::configure_cartesian_names_() {
  if (cart_names_.size() != lbr_fri_ros2::CARTESIAN_QUAT_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of cartesian names (%ld) does not match the number of cartesian cmd in the robot (%d).",
        cart_names_.size(), lbr_fri_ros2::CARTESIAN_QUAT_DOF);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  
  cart_names_[0] = robot_name + "_C.X";
  cart_names_[1] = robot_name + "_C.Y";
  cart_names_[2] = robot_name + "_C.Z";
  cart_names_[3] = robot_name + "_C.QW";
  cart_names_[0] = robot_name + "_C.QX";
  cart_names_[1] = robot_name + "_C.QY";
  cart_names_[2] = robot_name + "_C.QZ";
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::LBRCartesianPoseCommandController,
                       controller_interface::ControllerInterface)
