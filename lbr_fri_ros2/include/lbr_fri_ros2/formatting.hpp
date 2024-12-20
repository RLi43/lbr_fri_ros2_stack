#ifndef LBR_FRI_ROS2__FORMATTING_HPP_
#define LBR_FRI_ROS2__FORMATTING_HPP_

#include <string>

#include "friClientVersion.h"
#include "friLBRClient.h"

namespace lbr_fri_ros2 {
struct ColorScheme {
  // refer https://stackoverflow.com/a/287944
  static constexpr char HEADER[] = "\033[95m";
  static constexpr char OKBLUE[] = "\033[94m";
  static constexpr char OKCYAN[] = "\033[96m";
  static constexpr char OKGREEN[] = "\033[92m";
  static constexpr char WARNING[] = "\033[93m";
  static constexpr char ERROR[] = "\033[91m";
  static constexpr char ENDC[] = "\033[0m";
  static constexpr char BOLD[] = "\033[1m";
  static constexpr char UNDERLINE[] = "\033[4m";
};

struct EnumMaps {
  static std::string session_state_map(const int &session_state) {
    switch (session_state) {
    case KUKA::FRI::ESessionState::IDLE:
      return "IDLE";
    case KUKA::FRI::ESessionState::MONITORING_WAIT:
      return "MONITORING_WAIT";
    case KUKA::FRI::ESessionState::MONITORING_READY:
      return "MONITORING_READY";
    case KUKA::FRI::ESessionState::COMMANDING_WAIT:
      return "COMMANDING_WAIT";
    case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
      return "COMMANDING_ACTIVE";
    default:
      return "UNKNOWN";
    }
  };

  static std::string connection_quality_map(const int &connection_quality){
    switch(connection_quality){
      case KUKA::FRI::EConnectionQuality::POOR:
        return "POOR";
      case KUKA::FRI::EConnectionQuality::FAIR:
        return "FAIR";
      case KUKA::FRI::EConnectionQuality::GOOD:
        return "GOOD";
      case KUKA::FRI::EConnectionQuality::EXCELLENT:
        return "EXCELLENT";
      default:
        return "UNKNOWN";
    }
  };

  static std::string safety_state_map(const int &safty_state){
    switch(safty_state){
      case KUKA::FRI::ESafetyState::NORMAL_OPERATION:
        return "NORMAL";
      case KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_0:
        return "STOP0_or_STOP1";
      case KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_1:
        return "STOP1_on_path";
      case KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_2:
        return "STOP2";
      default:
        return "UNKNOWN";
    }
  };

  static std::string operation_mode_map(const int &operation_mode){
    switch(operation_mode){
      case KUKA::FRI::EOperationMode::TEST_MODE_1:
        return "T1";
      case KUKA::FRI::EOperationMode::TEST_MODE_2:
        return "T2";
      case KUKA::FRI::EOperationMode::AUTOMATIC_MODE:
        return "AUT";
      default:
        return "UNKNOWN";
    }
  }

  static std::string drive_state_map(const int &drive_state){
    switch(drive_state){
      case KUKA::FRI::EDriveState::OFF:
        return "OFF";
      case KUKA::FRI::EDriveState::TRANSITIONING:
        return "TRANSITIONING";
      case KUKA::FRI::EDriveState::ACTIVE:
        return "ACTIVE";
      default:
        return "UNKNOWN";
    }
  }

  static std::string control_mode_map(const int &control_mode) {
    switch (control_mode) {
    case KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE:
      return "CART_IMP_CONTROL_MODE";
    case KUKA::FRI::EControlMode::JOINT_IMP_CONTROL_MODE:
      return "JOINT_IMP_CONTROL_MODE";
    case KUKA::FRI::EControlMode::NO_CONTROL:
      return "NO_CONTROL";
    case KUKA::FRI::EControlMode::POSITION_CONTROL_MODE:
      return "POSITION_CONTROL_MODE";
    default:
      return "UNKNOWN";
    }
  };

  static std::string client_command_mode_map(const int &client_command_mode) {
    switch (client_command_mode) {
    case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
      return "NO_COMMAND_MODE";
#if FRI_CLIENT_VERSION_MAJOR == 1
    case KUKA::FRI::EClientCommandMode::POSITION:
      return "POSITION";
#endif
#if FRI_CLIENT_VERSION_MAJOR >= 2
    case KUKA::FRI::EClientCommandMode::JOINT_POSITION:
      return "JOINT_POSITION";
    case KUKA::FRI::EClientCommandMode::CARTESIAN_POSE:
      return "CARTESIAN_POSE";
#endif
    case KUKA::FRI::EClientCommandMode::TORQUE:
      return "TORQUE";
    case KUKA::FRI::EClientCommandMode::WRENCH:
      return "WRENCH";
    default:
      return "UNKNOWN";
    }
  };
  
  static std::string overlay_type_map(const int &overlay_type) {
    switch (overlay_type)
    {
    case KUKA::FRI::EOverlayType::NO_OVERLAY:
      return "NO_OVERLAY";
    case KUKA::FRI::EOverlayType::JOINT:
      return "JOINT";
    case KUKA::FRI::EOverlayType::CARTESIAN:
      return "CARTESIAN";
    default:
      return "UNKNOWN";
    }
  }

  static std::string redundancy_strategy_map(const int &redundancy_strategy) {
    switch (redundancy_strategy)
    {
    case KUKA::FRI::ERedundancyStrategy::E1:
      return "E1";
    case KUKA::FRI::ERedundancyStrategy::NO_STRATEGY:
      return "NO_STRATEGY";    
    default:
      return "UNKNOWN";
    }
  }
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__ENUM_MAPS_HPP
