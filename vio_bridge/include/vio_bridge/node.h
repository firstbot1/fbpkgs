//
// @file node.h
// @brief VioBridgeNode declaration and minimal test helpers.
//
// The node subscribes to a VIO Odometry topic and republishes
// to a MAVROS-compatible topic, applying optional field overrides
// and sanitization policies.
//
// This header is intentionally lightweight; implementation details
// live in src/vio_bridge/node.cc.
//
#ifndef VIO_BRIDGE__VIO_BRIDGE_NODE_H_
#define VIO_BRIDGE__VIO_BRIDGE_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace vio_bridge {
/**
 * @class VioBridgeNode
 * @brief ROS 2 node bridging VIO odometry to MAVROS-compatible topic.
 *
 * Responsibilities:
 * - Subscribe to input odometry.
 * - Optionally normalize quaternion, symmetrize covariance.
 * - Optionally override frame_id/child_frame_id.
 * - Republish to output odometry with configured QoS.
 */
class VioBridgeNode: public rclcpp::Node {
public:
    /**
     * @brief Construct node with options.
     * @param options Node options (parameters, use_sim_time, etc.).
     */
    explicit VioBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~VioBridgeNode() = default;

    /** @brief Test helper: expose input subscription QoS. */
    rclcpp::QoS get_input_qos() const { return subscription_->get_actual_qos(); }
    /** @brief Test helper: expose output publisher QoS. */
    rclcpp::QoS get_output_qos() const { return publisher_->get_actual_qos(); }

  private:
    /**
     * @brief Callback for incoming odometry.
     * @param msg Incoming message.
     */
    void vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr subscription_;
    rclcpp::Publisher < nav_msgs::msg::Odometry > ::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    std::string input_topic_;
    std::string output_topic_;
    // Optional field overrides
    std::string force_frame_id_;
    std::string force_child_frame_id_;
    // Sanitization/validation toggles
    bool normalize_quaternion_ {false};
    bool symmetrize_covariance_ {false};
    bool fill_header_stamp_if_zero_ {false};
    bool strict_tf_ {false};
    // Timestamp policy: when ROS time is enabled but unstarted (0), do not correct zero stamps
    bool disable_stamp_correction_when_ros_time_zero_ {true};
    bool use_introspection_for_fields_ {false};
  };
}  // namespace vio_bridge

#endif  // VIO_BRIDGE__VIO_BRIDGE_NODE_H_
