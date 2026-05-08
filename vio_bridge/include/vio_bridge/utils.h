//
// @file utils.h
// @brief Utilities (timestamp policy and covariance helpers).
//
#ifndef VIO_BRIDGE__UTILS_H_
#define VIO_BRIDGE__UTILS_H_

#include <array>
#include <rclcpp/rclcpp.hpp>

namespace vio_bridge::utils {

/**
 * @brief Decide if zero-stamp correction should be suppressed.
 * @param clock Node clock
 * @param disable_when_ros_time If true, disable correction under ROS time
 */
bool forbid_stamp_correction(const rclcpp::Clock & clock, bool disable_when_ros_time);

/**
 * @brief Get a non-zero timestamp.
 * Attempts node's clock, then system clock, then a small constant.
 */
rclcpp::Time now_nonzero(const rclcpp::Node & node);

/**
 * @brief Symmetrize 6x6 covariance and clamp diagonal to non-negative.
 */
void symmetrize_and_clamp(std::array<double, 36> & C);

}  // namespace vio_bridge::utils

#endif  // VIO_BRIDGE__UTILS_H_
