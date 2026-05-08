// Parameter validation tests for VioBridgeNode
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

TEST_CASE("ShouldRejectInvalidHistoryOnConstruction", "[param][validation]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamHistory, rclcpp::ParameterValue(std::string("invalid")));
  REQUIRE_THROWS_AS(std::make_shared<vio_bridge::VioBridgeNode>(opts), std::invalid_argument);
  rclcpp::shutdown();
}

TEST_CASE("ShouldRejectEmptyTopicsOnConstruction", "[param][validation]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override(vio_bridge::constants::kParamInputTopic, rclcpp::ParameterValue(std::string("")));
  REQUIRE_THROWS_AS(std::make_shared<vio_bridge::VioBridgeNode>(opts), std::invalid_argument);
  rclcpp::shutdown();
}

TEST_CASE("ShouldRejectInvalidReliabilityAtRuntime", "[param][validation]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  auto node = std::make_shared<vio_bridge::VioBridgeNode>();

  // Attempt to set invalid reliability; callback should fail
  auto results = node->set_parameters({ rclcpp::Parameter(vio_bridge::constants::kParamReliability, std::string("maybe")) });
  REQUIRE(results.size() == 1);
  REQUIRE_FALSE(results[0].successful);
  rclcpp::shutdown();
}
