// Tests for quaternion normalization/strict_tf and covariance symmetrization
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

TEST_CASE("ShouldNormalizeQuaternionWhenEnabled", "[vio_bridge][sanitize]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamNormalizeQuat, rclcpp::ParameterValue(true));

  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);
  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));

  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_sanitize_norm");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);

  Odometry m; m.pose.pose.orientation.x = 10.0; m.pose.pose.orientation.w = 0.0; // norm 10
  pub->publish(m);

  spin_until(exec, [&]{ return received.has_value(); });
  REQUIRE(received.has_value());
  const auto & q = received->pose.pose.orientation;
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  REQUIRE(norm == Approx(1.0).margin(1e-9));
}

TEST_CASE("ShouldDropOnInvalidQuaternionWhenStrict", "[vio_bridge][sanitize]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamStrictTf, rclcpp::ParameterValue(true));

  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);
  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));

  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_sanitize_strict");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);

  Odometry m; m.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  pub->publish(m);

  spin_until(exec, [&]{ return received.has_value(); }, /*timeout_ms*/ 200);
  REQUIRE_FALSE(received.has_value()); // should drop
}

TEST_CASE("ShouldSymmetrizeAndClampCovariance", "[vio_bridge][sanitize]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamSymmetrizeCov, rclcpp::ParameterValue(true));

  auto node = std::make_shared<vio_bridge::VioBridgeNode>(opts);
  auto pub = node->create_publisher<Odometry>("/rtabmap/odom", rclcpp::QoS(10));

  std::optional<Odometry> received;
  auto sub_node = std::make_shared<rclcpp::Node>("test_sanitize_cov");
  auto sub = sub_node->create_subscription<Odometry>(
    "/mavros/odometry/out", rclcpp::QoS(10),
    [&](const Odometry & msg){ received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.add_node(sub_node);

  Odometry m;
  // Asymmetric values
  m.pose.covariance[0*6 + 1] = 2.0; // (0,1)
  m.pose.covariance[1*6 + 0] = -6.0; // (1,0)
  // Negative diagonal
  m.pose.covariance[2*6 + 2] = -1.0;

  pub->publish(m);

  spin_until(exec, [&]{ return received.has_value(); });
  REQUIRE(received.has_value());
  const auto & C = received->pose.covariance;
  REQUIRE(C[0*6 + 1] == Approx(C[1*6 + 0]));
  REQUIRE(C[2*6 + 2] >= 0.0);
}
