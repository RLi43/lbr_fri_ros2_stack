#ifndef LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_
#define LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_

// see
// https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp
namespace lbr_ros2_control {
// additional LBR state interfaces, reference KUKA::FRI::LBRState
constexpr char HW_IF_SAMPLE_TIME[] = "sample_time";
constexpr char HW_IF_SESSION_STATE[] = "session_state";
constexpr char HW_IF_CONNECTION_QUALITY[] = "connection_quality";
constexpr char HW_IF_SAFETY_STATE[] = "safety_state";
constexpr char HW_IF_OPERATION_MODE[] = "operation_mode";
constexpr char HW_IF_DRIVE_STATE[] = "drive_state";
constexpr char HW_IF_CLIENT_COMMAND_MODE[] = "client_command_mode";
constexpr char HW_IF_OVERLAY_TYPE[] = "overlay_type";
constexpr char HW_IF_CONTROL_MODE[] = "control_mode";
constexpr char HW_IF_REDUNDANCY_STRATEGY[] = "redundancy_strategy";

constexpr char HW_IF_TIME_STAMP_SEC[] = "time_stamp_sec";
constexpr char HW_IF_TIME_STAMP_NANO_SEC[] = "time_stamp_nano_sec";

constexpr char HW_IF_COMMANDED_JOINT_POSITION[] = "commanded_joint_position";
constexpr char HW_IF_COMMANDED_TORQUE[] = "commanded_torque";

constexpr char HW_IF_EXTERNAL_TORQUE[] = "external_torque";

constexpr char HW_IF_IPO_JOINT_POSITION[] = "ipo_joint_position";
constexpr char HW_IF_TRACKING_PERFORMANCE[] = "tracking_performance";

// additional cartesian interface
// constexpr char HW_IF_REDUNDANCY_VALUE_CONTROLLED[] = "redundancy_value_controlled";
// constexpr char HW_IF_REDUNDANCY_VALUE[] = "redundancy_value";
// constexpr char HW_IF_IPO_REDUNDANCY_VALUE[] = "ipo_redundancy_value";
// constexpr char HW_IF_MEASURED_REDUNDANCY_VALUE[] = "measured_redundancy_value";

constexpr char HW_IF_IPO_CARTESIAN_POSE[] = "ipo_cartesian";
constexpr char HW_IF_IPO_CARTESIAN_POSE_X[] = "ipo_pose.x";
constexpr char HW_IF_IPO_CARTESIAN_POSE_Y[] = "ipo_pose.y";
constexpr char HW_IF_IPO_CARTESIAN_POSE_Z[] = "ipo_pose.z";
constexpr char HW_IF_IPO_CARTESIAN_POSE_QW[] = "ipo_pose.qw";
constexpr char HW_IF_IPO_CARTESIAN_POSE_QX[] = "ipo_pose.qx";
constexpr char HW_IF_IPO_CARTESIAN_POSE_QY[] = "ipo_pose.qy";
constexpr char HW_IF_IPO_CARTESIAN_POSE_QZ[] = "ipo_pose.qz";

constexpr char HW_IF_MEASURED_CARTESIAN_POSE[] = "measured_cartesian";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_X[] = "measured_pose.x";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_Y[] = "measured_pose.y";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_Z[] = "measured_pose.z";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_QW[] = "measured_pose.qw";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_QX[] = "measured_pose.qx";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_QY[] = "measured_pose.qy";
constexpr char HW_IF_MEASURED_CARTESIAN_POSE_QZ[] = "measured_pose.qz";

constexpr char HW_IF_CARTESIAN_POSE_X[] = "pose.x";
constexpr char HW_IF_CARTESIAN_POSE_Y[] = "pose.y";
constexpr char HW_IF_CARTESIAN_POSE_Z[] = "pose.z";
constexpr char HW_IF_CARTESIAN_POSE_QW[] = "pose.qw";
constexpr char HW_IF_CARTESIAN_POSE_QX[] = "pose.qx";
constexpr char HW_IF_CARTESIAN_POSE_QY[] = "pose.qy";
constexpr char HW_IF_CARTESIAN_POSE_QZ[] = "pose.qz";

// TODO pose as matrix
// constexpr char HW_IF_IPO_CARTESIAN_MATRIX[] = "ipo_cartesian_matrix";
// constexpr char HW_IF_MEASURED_CARTESIAN_MATRIX[] = "measured_cartesian_matrix";
// constexpr char HW_IF_CARTESIAN_MATRIX_11[] = "matrix.11";

// additional force-torque command and state interfaces
constexpr char HW_IF_FORCE_X[] = "force.x";
constexpr char HW_IF_FORCE_Y[] = "force.y";
constexpr char HW_IF_FORCE_Z[] = "force.z";
constexpr char HW_IF_TORQUE_X[] = "torque.x";
constexpr char HW_IF_TORQUE_Y[] = "torque.y";
constexpr char HW_IF_TORQUE_Z[] = "torque.z";

// additional LBR command interfaces, reference KUKA::FRI::LBRCommand
constexpr char HW_IF_WRENCH_PREFIX[] = "wrench";
constexpr char HW_IF_AUXILIARY_PREFIX[] = "auxiliary_sensor";
constexpr char HW_IF_ESTIMATED_FT_PREFIX[] = "estimated_ft_sensor";
constexpr char HW_IF_CARTESIAN_SENSOR_PREFIX[] = "cartesian_sensor";
constexpr char HW_IF_CARTESIAN_GPIO_PREFIX[] = "cartesian_pose_quat"; // TODO: support as matrix
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__SYSTEM_INTERFACE_TYPE_VALUES_HPP_
