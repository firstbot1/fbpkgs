// Runtime QoS change tests for VioBridgeNode
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

using nav_msgs::msg::Odometry;

TEST_CASE("ShouldApplyRuntimeQosChanges", "[qos][runtime]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  auto node = std::make_shared<vio_bridge::VioBridgeNode>();

  // Change QoS parameters at runtime
  auto results = node->set_parameters({
    rclcpp::Parameter(vio_bridge::constants::kParamHistory, std::string("keep_last")),
    rclcpp::Parameter(vio_bridge::constants::kParamDepth, 1),
    rclcpp::Parameter(vio_bridge::constants::kParamReliability, std::string("best_effort")),
    rclcpp::Parameter(vio_bridge::constants::kParamDurability, std::string("volatile"))
  });
  for (const auto & res : results) REQUIRE(res.successful);

  // Publish/subscribe round-trip still works after QoS change
  // Match publisher QoS to node's subscription (best_effort, depth=1, volatile)
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", pub_qos);
  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_qos_runtime");
  // Match subscriber QoS to node's publisher QoS (best_effort, depth=1, volatile)
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", pub_qos,
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
