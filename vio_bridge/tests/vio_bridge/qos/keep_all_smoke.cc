// Smoke test for keep_all with large depth parameter (depth ignored under keep_all)
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

using nav_msgs::msg::Odometry;

TEST_CASE("ShouldWorkWithKeepAllAndLargeDepth", "[qos][smoke]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamHistory, rclcpp::ParameterValue(std::string("keep_all")));
  opts.append_parameter_override(vio_bridge::constants::kParamDepth, rclcpp::ParameterValue(10000));
  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);

  // Use simple QoS; keep_all publisher/subscriber should still deliver in single process
  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));
  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_keep_all");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);
  pub->publish(Odometry());

  const auto start = std::chrono::steady_clock::now();
  while (!received.has_value() && (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500)) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(received.has_value());
  rclcpp::shutdown();
}
