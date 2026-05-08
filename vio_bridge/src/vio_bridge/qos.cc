#include "vio_bridge/qos.h"

namespace vio_bridge::qos {

rclcpp::QoS build_qos(const std::string & history,
                      int depth,
                      const std::string & reliability,
                      const std::string & durability)
{
  size_t keep_last_depth = 0;
  if (history == "keep_all") {
    // KeepAll
  } else if (history == "keep_last") {
    long d = depth;
    if (d < 0) d = 0;
    keep_last_depth = static_cast<size_t>(d);
  } else {
    throw std::invalid_argument("invalid value for parameter 'history'");
  }

  rclcpp::QoS qos = (history == "keep_all") ? rclcpp::QoS(rclcpp::KeepAll())
                                             : rclcpp::QoS(rclcpp::KeepLast(keep_last_depth));

  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  } else {
    throw std::invalid_argument("invalid value for parameter 'reliability'");
  }

  if (durability == "transient_local") {
    qos.transient_local();
  } else if (durability == "volatile") {
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  } else {
    throw std::invalid_argument("invalid value for parameter 'durability'");
  }

  return qos;
}

} // namespace vio_bridge::qos

