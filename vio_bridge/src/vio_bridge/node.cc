#include "vio_bridge/node.h"
#include "vio_bridge/qos.h"
#include "vio_bridge/constants.h"
#include "vio_bridge/utils.h"
#include "vio_bridge/introspection.h"
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <cmath>

namespace vio_bridge
{
  VioBridgeNode::VioBridgeNode(const rclcpp::NodeOptions & options)
  : Node("vio_bridge_node", options)
  {
    using namespace vio_bridge::constants;
    input_topic_ = this->declare_parameter<std::string>(kParamInputTopic, std::string(kDefaultInputTopic));
    output_topic_ = this->declare_parameter<std::string>(kParamOutputTopic, std::string(kDefaultOutputTopic));

    // New QoS selection parameters
    const auto history = this->declare_parameter<std::string>(kParamHistory, std::string(kDefaultHistory));
    const auto depth = this->declare_parameter<int>(kParamDepth, kDefaultDepth);
    const auto reliability = this->declare_parameter<std::string>(kParamReliability, std::string(kDefaultReliability));
    const auto durability = this->declare_parameter<std::string>(kParamDurability, std::string(kDefaultDurability));

    if (input_topic_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'input_topic' must be non-empty.");
      throw std::invalid_argument("parameter 'input_topic' must be non-empty");
    }
    if (output_topic_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'output_topic' must be non-empty.");
      throw std::invalid_argument("parameter 'output_topic' must be non-empty");
    }

    // Optional field override parameters (empty means pass-through)
    force_frame_id_ = this->declare_parameter<std::string>(kParamForceFrameId, std::string(""));
    force_child_frame_id_ = this->declare_parameter<std::string>(kParamForceChildFrameId, std::string(""));
    // Sanitization/validation toggles
    normalize_quaternion_ = this->declare_parameter<bool>(kParamNormalizeQuat, false);
    symmetrize_covariance_ = this->declare_parameter<bool>(kParamSymmetrizeCov, false);
    fill_header_stamp_if_zero_ = this->declare_parameter<bool>(kParamFillStampIfZero, false);
    disable_stamp_correction_when_ros_time_zero_ = this->declare_parameter<bool>(
      kParamDisableStampWhenRosZero, true);
    strict_tf_ = this->declare_parameter<bool>(kParamStrictTf, false);
    use_introspection_for_fields_ = this->declare_parameter<bool>(constants::kParamUseIntrospectionForFields, false);

    // Build QoS based on parameters (extracted utility)
    rclcpp::QoS qos = qos::build_qos(history, depth, reliability, durability);

    // Validate history value
    if (!(history == "keep_all" || history == "keep_last")) {
      RCLCPP_ERROR(this->get_logger(), "invalid value for parameter 'history': '%s'", history.c_str());
      throw std::invalid_argument("invalid value for parameter 'history'");
    }

    subscription_ = this->create_subscription < nav_msgs::msg::Odometry > (
      input_topic_,
      qos,
      std::bind(&VioBridgeNode::vio_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher < nav_msgs::msg::Odometry > (output_topic_, qos);

    RCLCPP_INFO(this->get_logger(), "VioBridgeNode has been started.");

    // Runtime parameter change support
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        // Start with current values
        auto history = this->get_parameter(constants::kParamHistory).as_string();
        auto depth = static_cast<int>(this->get_parameter(constants::kParamDepth).as_int());
        auto reliability = this->get_parameter(constants::kParamReliability).as_string();
        auto durability = this->get_parameter(constants::kParamDurability).as_string();
        auto force_frame_id = this->get_parameter(constants::kParamForceFrameId).as_string();
        auto force_child_frame_id = this->get_parameter(constants::kParamForceChildFrameId).as_string();
        auto normalize_quaternion = this->get_parameter(constants::kParamNormalizeQuat).as_bool();
        auto symmetrize_covariance = this->get_parameter(constants::kParamSymmetrizeCov).as_bool();
        auto fill_header_stamp_if_zero = this->get_parameter(constants::kParamFillStampIfZero).as_bool();
        auto strict_tf = this->get_parameter(constants::kParamStrictTf).as_bool();
        auto use_introspection_for_fields = this->get_parameter(constants::kParamUseIntrospectionForFields).as_bool();
        auto disable_stamp_correction_when_ros_time_zero =
          this->get_parameter(constants::kParamDisableStampWhenRosZero).as_bool();

        // Apply incoming updates
        for (const auto & p : params) {
          if (p.get_name() == constants::kParamHistory && p.get_type() == rclcpp::PARAMETER_STRING) history = p.as_string();
          if (p.get_name() == constants::kParamDepth && p.get_type() == rclcpp::PARAMETER_INTEGER) depth = static_cast<int>(p.as_int());
          if (p.get_name() == constants::kParamReliability && p.get_type() == rclcpp::PARAMETER_STRING) reliability = p.as_string();
          if (p.get_name() == constants::kParamDurability && p.get_type() == rclcpp::PARAMETER_STRING) durability = p.as_string();
          if (p.get_name() == constants::kParamForceFrameId && p.get_type() == rclcpp::PARAMETER_STRING) force_frame_id = p.as_string();
          if (p.get_name() == constants::kParamForceChildFrameId && p.get_type() == rclcpp::PARAMETER_STRING) force_child_frame_id = p.as_string();
          if (p.get_name() == constants::kParamNormalizeQuat && p.get_type() == rclcpp::PARAMETER_BOOL) normalize_quaternion = p.as_bool();
          if (p.get_name() == constants::kParamSymmetrizeCov && p.get_type() == rclcpp::PARAMETER_BOOL) symmetrize_covariance = p.as_bool();
          if (p.get_name() == constants::kParamFillStampIfZero && p.get_type() == rclcpp::PARAMETER_BOOL) fill_header_stamp_if_zero = p.as_bool();
          if (p.get_name() == constants::kParamStrictTf && p.get_type() == rclcpp::PARAMETER_BOOL) strict_tf = p.as_bool();
          if (p.get_name() == constants::kParamDisableStampWhenRosZero && p.get_type() == rclcpp::PARAMETER_BOOL)
            disable_stamp_correction_when_ros_time_zero = p.as_bool();
        }

        rcl_interfaces::msg::SetParametersResult result;
        try {
          auto qos = qos::build_qos(history, depth, reliability, durability);
          // Recreate subscription/publisher with new QoS
          subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, qos,
            std::bind(&VioBridgeNode::vio_callback, this, std::placeholders::_1));
          publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
          // Apply overrides atomically on success
          force_frame_id_ = force_frame_id;
          force_child_frame_id_ = force_child_frame_id;
          normalize_quaternion_ = normalize_quaternion;
          symmetrize_covariance_ = symmetrize_covariance;
          fill_header_stamp_if_zero_ = fill_header_stamp_if_zero;
          strict_tf_ = strict_tf;
          disable_stamp_correction_when_ros_time_zero_ = disable_stamp_correction_when_ros_time_zero;
          use_introspection_for_fields_ = use_introspection_for_fields;
          result.successful = true;
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
        }
        return result;
      });
  }

  void VioBridgeNode::vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto out_msg = nav_msgs::msg::Odometry(*msg);
    // Optional: read frame ids via introspection (no-op if unsupported)
    if (use_introspection_for_fields_) {
      using vio_bridge::introspection::try_read_string_field;
      const rosidl_message_type_support_t * ts = rosidl_typesupport_cpp::get_message_type_support_handle<nav_msgs::msg::Odometry>();
      std::string frame, child;
      if (try_read_string_field(msg.get(), ts, constants::kFieldFrameId, frame)) {
        out_msg.header.frame_id = frame;
      }
      if (try_read_string_field(msg.get(), ts, constants::kFieldChildFrameId, child)) {
        out_msg.child_frame_id = child;
      }
    }
    // Timestamp fix if requested and zero
    if (fill_header_stamp_if_zero_ && out_msg.header.stamp.sec == 0 && out_msg.header.stamp.nanosec == 0u) {
      if (!utils::forbid_stamp_correction(*this->get_clock(), disable_stamp_correction_when_ros_time_zero_)) {
        out_msg.header.stamp = utils::now_nonzero(*this);
      }
    }
    // Enforce frame ids if configured
    if (!force_frame_id_.empty()) {
      out_msg.header.frame_id = force_frame_id_;
    }
    if (!force_child_frame_id_.empty()) {
      out_msg.child_frame_id = force_child_frame_id_;
    }
    // Quaternion normalization / validation
    auto & q = out_msg.pose.pose.orientation;
    const bool any_nan = std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w);
    if (any_nan && strict_tf_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Dropping message: NaN quaternion in strict_tf mode");
      return; // drop
    }
    double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (normalize_quaternion_ && norm > 0.0 && std::isfinite(norm)) {
      q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    } else if (strict_tf_ && (norm == 0.0 || !std::isfinite(norm))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Dropping message: invalid quaternion norm in strict_tf mode");
      return; // drop
    }
    // Covariance symmetrization/clamp diagonal (pose and twist)
    if (symmetrize_covariance_) {
      utils::symmetrize_and_clamp(out_msg.pose.covariance);
      utils::symmetrize_and_clamp(out_msg.twist.covariance);
    }
    publisher_->publish(out_msg);
  }

  // QoS builder moved to vio_bridge::qos::build_qos (structural refactor)
}  // namespace vio_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vio_bridge::VioBridgeNode)
