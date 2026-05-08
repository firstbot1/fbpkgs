#include "vio_bridge/utils.h"

namespace vio_bridge::utils {

bool forbid_stamp_correction(const rclcpp::Clock & clock, bool disable_when_ros_time)
{
  const bool ros_clock = clock.get_clock_type() == RCL_ROS_TIME;
  return disable_when_ros_time && ros_clock;
}

rclcpp::Time now_nonzero(const rclcpp::Node & node)
{
  rclcpp::Time t = node.now();
  if (t.seconds() == 0 && t.nanoseconds() == 0) {
    t = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }
  if (t.seconds() == 0 && t.nanoseconds() == 0) {
    t = rclcpp::Time(1, 0, RCL_SYSTEM_TIME);
  }
  return t;
}

void symmetrize_and_clamp(std::array<double, 36> & C)
{
  for (int i = 0; i < 6; ++i) {
    for (int j = i + 1; j < 6; ++j) {
      const double a = C[i * 6 + j];
      const double b = C[j * 6 + i];
      const double m = 0.5 * (a + b);
      C[i * 6 + j] = m;
      C[j * 6 + i] = m;
    }
  }
  for (int i = 0; i < 6; ++i) {
    double & d = C[i * 6 + i];
    if (!(d >= 0.0)) d = 0.0;
  }
}

}  // namespace vio_bridge::utils

