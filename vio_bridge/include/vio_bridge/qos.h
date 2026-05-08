//
// @file qos.h
// @brief QoS utilities extracted from VioBridgeNode.
//
#ifndef VIO_BRIDGE__QOS_H_
#define VIO_BRIDGE__QOS_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace vio_bridge::qos {

/**
 * @brief Build QoS from string parameters.
 * @param history    "keep_all" or "keep_last"
 * @param depth      depth used when history == keep_last (clamped to >= 0)
 * @param reliability "best_effort" or "reliable"
 * @param durability  "transient_local" or "volatile"
 * @throws std::invalid_argument on invalid values
 */
rclcpp::QoS build_qos(const std::string & history,
                      int depth,
                      const std::string & reliability,
                      const std::string & durability);

} // namespace vio_bridge::qos

#endif // VIO_BRIDGE__QOS_H_
