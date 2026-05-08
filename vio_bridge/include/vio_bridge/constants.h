// Constants for parameter names and default topics
#ifndef VIO_BRIDGE__CONSTANTS_H_
#define VIO_BRIDGE__CONSTANTS_H_

namespace vio_bridge::constants {

// Default topics
inline constexpr const char kDefaultInputTopic[] = "/rtabmap/odom";
inline constexpr const char kDefaultOutputTopic[] = "/mavros/odometry/out";

// Parameter names
inline constexpr const char kParamInputTopic[] = "input_topic";
inline constexpr const char kParamOutputTopic[] = "output_topic";
inline constexpr const char kParamHistory[] = "history";
inline constexpr const char kParamDepth[] = "depth";
inline constexpr const char kParamReliability[] = "reliability";
inline constexpr const char kParamDurability[] = "durability";
inline constexpr const char kParamForceFrameId[] = "force_frame_id";
inline constexpr const char kParamForceChildFrameId[] = "force_child_frame_id";
inline constexpr const char kParamNormalizeQuat[] = "normalize_quaternion";
inline constexpr const char kParamSymmetrizeCov[] = "symmetrize_covariance";
inline constexpr const char kParamFillStampIfZero[] = "fill_header_stamp_if_zero";
inline constexpr const char kParamStrictTf[] = "strict_tf";
inline constexpr const char kParamDisableStampWhenRosZero[] = "disable_stamp_correction_when_ros_time_zero";
inline constexpr const char kParamUseIntrospectionForFields[] = "use_introspection_for_fields";

// Parameter default values
inline constexpr const char kDefaultHistory[] = "keep_last";
inline constexpr int kDefaultDepth = 10;
inline constexpr const char kDefaultReliability[] = "reliable";
inline constexpr const char kDefaultDurability[] = "volatile";

// Message field keys (optional helpers for clarity)
inline constexpr const char kFieldFrameId[] = "frame_id";
inline constexpr const char kFieldChildFrameId[] = "child_frame_id";

}  // namespace vio_bridge::constants

#endif  // VIO_BRIDGE__CONSTANTS_H_
