// Tests for frame_id/child_frame_id mapping via parameters
#include <catch2/catch.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

using nav_msgs::msg::Odometry;

TEST_CASE("ShouldMapFrameIdsWhenForced", "[vio_bridge][topic]") {
  int argc = 0; char ** argv = nullptr;
  if (!rclcpp::ok()) rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamForceFrameId, rclcpp::ParameterValue(std::string("map")));
  opts.append_parameter_override(vio_bridge::constants::kParamForceChildFrameId, rclcpp::ParameterValue(std::string("base")));

  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);

  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));

  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_listener_frame");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(sub_node);

  Odometry m; m.header.frame_id = "odom"; m.child_frame_id = "base_link";
  pub->publish(m);

  const auto start = std::chrono::steady_clock::now();
  while (!received.has_value() && (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500)) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  REQUIRE(received.has_value());
  REQUIRE(received->header.frame_id == std::string("map"));
  REQUIRE(received->child_frame_id == std::string("base"));

  rclcpp::shutdown();
}
