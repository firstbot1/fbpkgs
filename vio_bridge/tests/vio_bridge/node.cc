// Basic skeleton test for VioBridgeNode using Catch2
#include <catch2/catch.hpp>

#include <rclcpp/rclcpp.hpp>
#include "vio_bridge/node.h"

TEST_CASE("ShouldConstructNode", "[vio_bridge]") {
  int argc = 0;
  char ** argv = nullptr;
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared < vio_bridge::VioBridgeNode > ();
  REQUIRE(node != nullptr);
  REQUIRE(node->get_name() == std::string("vio_bridge_node"));

  // Keep tests deterministic and fast; no spinning required here.
  rclcpp::shutdown();
}
