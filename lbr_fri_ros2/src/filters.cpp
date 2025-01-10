#include "lbr_fri_ros2/filters.hpp"
#include "lbr_fri_ros2/quaternion.hpp"

namespace lbr_fri_ros2 {
ExponentialFilter::ExponentialFilter()
    : tau_(std::numeric_limits<double>::quiet_NaN()),
      sample_time_(std::numeric_limits<double>::quiet_NaN()),
      alpha_(std::numeric_limits<double>::quiet_NaN()) {}

ExponentialFilter::ExponentialFilter(const double &tau) : tau_(tau) {}

void ExponentialFilter::initialize(const double &sample_time) {
  if (std::isnan(tau_)) {
    throw std::runtime_error("Time constant must be set before initializing.");
  }
  return initialize(tau_, sample_time);
}

void ExponentialFilter::initialize(const double &tau, const double &sample_time) {
  if (tau <= 0.0) {
    throw std::runtime_error("Time constant must be positive and greater zero.");
  }
  if (sample_time < 0.0) {
    throw std::runtime_error("Sample time must be positive.");
  }
  double alpha = 1.0 - std::exp(-sample_time / tau);
  if (!validate_alpha_(alpha)) {
    throw std::runtime_error("Alpha is not within [0, 1]");
  }
  tau_ = tau;
  sample_time_ = sample_time;
  alpha_ = alpha;
}

bool ExponentialFilter::validate_alpha_(const double &alpha) { return alpha <= 1. && alpha >= 0.; }

JointExponentialFilterArray::JointExponentialFilterArray(const double &tau)
    : exponential_filter_(tau) {}

void JointExponentialFilterArray::compute(const double *const current, jnt_array_t_ref previous) {
  std::for_each(current, current + N_JNTS, [&, i = 0](const auto &current_i) mutable {
    previous[i] = exponential_filter_.compute(current_i, previous[i]);
    ++i;
  });
}

void JointExponentialFilterArray::compute(const_jnt_array_t_ref current, jnt_array_t_ref previous) {
  compute(current.data(), previous);
}

void JointExponentialFilterArray::initialize(const double &sample_time) {
  exponential_filter_.initialize(sample_time);
  initialized_ = true;
}

void JointExponentialFilterArray::initialize(const double &tau, const double &sample_time) {
  exponential_filter_.initialize(tau, sample_time);
  initialized_ = true;
}

void JointExponentialFilterArray::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   tau: %.5f s", exponential_filter_.get_tau());
}


CartesianExponentialFilterArray::CartesianExponentialFilterArray(const double &tau)
    : exponential_filter_(tau) {}

void CartesianExponentialFilterArray::compute(const double *const current, cart_pose_array_t_ref previous) {
  // translation
  std::for_each(current, current + CART_POSE_TRANS_NUM, [&, i = 0](const auto &current_i) mutable {
    previous[i] = exponential_filter_.compute(current_i, previous[i]);
    ++i;
  });
  // rotation
  Quaternion prev = Quaternion(previous[3], previous[4], previous[5], previous[6]);
  Quaternion curr = Quaternion(current[3], current[4], current[5], current[6]);
  Quaternion filtered = Quaternion::slerp(prev, curr, exponential_filter_.get_alpha());
  previous[3] = filtered.get_w();
  previous[4] = filtered.get_x();
  previous[5] = filtered.get_y();
  previous[6] = filtered.get_z();
}

void CartesianExponentialFilterArray::compute(const_cart_pose_array_t_ref current, cart_pose_array_t_ref previous) {
  compute(current.data(), previous);
}

double CartesianExponentialFilterArray::compute(double current, double previous)
{
    return exponential_filter_.compute(current, previous);;
}
void CartesianExponentialFilterArray::initialize(const double &sample_time)
{
    exponential_filter_.initialize(sample_time);
    initialized_ = true;
}

void CartesianExponentialFilterArray::initialize(const double &tau, const double &sample_time) {
  exponential_filter_.initialize(tau, sample_time);
  initialized_ = true;
}

void CartesianExponentialFilterArray::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*   tau: %.5f s", exponential_filter_.get_tau());
}
} // namespace lbr_fri_ros2
