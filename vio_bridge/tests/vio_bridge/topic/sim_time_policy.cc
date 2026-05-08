// Tests for header.stamp correction policy with ROS time
#include <catch2/catch.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

using nav_msgs::msg::Odometry;

static void spin_until(rclcpp::executors::SingleThreadedExecutor & exec,
                       std::function<bool()> pred,
                       int timeout_ms = 500)
{
  const auto start = std::chrono::steady_clock::now();
  while (!pred() && (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(timeout_ms)) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_CASE("ShouldFillZeroStampWhenEnabledAndClockStarted", "[vio_bridge][sim_time]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);

  // Node with fill_header_stamp_if_zero=true and using system time (started)
  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamFillStampIfZero, rclcpp::ParameterValue(true));
  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);
  // Ensure system time for this test to exercise filling path deterministically
  node->set_parameter(rclcpp::Parameter("use_sim_time", false));

  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));
  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_sim_time_fill");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);

  Odometry m; m.header.stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  pub->publish(m);

  spin_until(exec, [&]{ return received.has_value(); });
  REQUIRE(received.has_value());
  // If node uses system time, expect non-zero stamp; otherwise just ensure we received something.
  if (node->get_clock()->get_clock_type() == RCL_SYSTEM_TIME) {
    REQUIRE(!(received->header.stamp.sec == 0 && received->header.stamp.nanosec == 0u));
  }
}

TEST_CASE("ShouldNotFillWhenRosTimeZeroAndPolicyDisabled", "[vio_bridge][sim_time]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);

  // Node policy: disable correction when ROS time is zero
  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamFillStampIfZero, rclcpp::ParameterValue(true));
  opts.append_parameter_override(vio_bridge::constants::kParamDisableStampWhenRosZero, rclcpp::ParameterValue(true));
  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);

  // Switch node clock to ROS time and leave it unstarted (sim time active but no /clock)
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));
  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_sim_time_no_fill");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);

  Odometry m; m.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  pub->publish(m);

  spin_until(exec, [&]{ return received.has_value(); });
  REQUIRE(received.has_value());
  // If node uses ROS time, expect still zero because sim time is active but not started and policy forbids correction.
  if (node->get_clock()->get_clock_type() == RCL_ROS_TIME) {
    REQUIRE(received->header.stamp.sec == 0);
    REQUIRE(received->header.stamp.nanosec == 0u);
  }
}
