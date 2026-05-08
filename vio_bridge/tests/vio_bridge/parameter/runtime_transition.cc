// Runtime parameter transition: invalid -> valid should recover cleanly
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vio_bridge/node.h"
#include "vio_bridge/constants.h"

TEST_CASE("ShouldRecoverFromInvalidRuntimeParamToValid", "[param][runtime]") {
  int argc = 0; char ** argv = nullptr; if (!rclcpp::ok()) rclcpp::init(argc, argv);
  auto node = std::make_shared<vio_bridge::VioBridgeNode>();

  // Set invalid durability first
  auto res_bad = node->set_parameters({ rclcpp::Parameter(vio_bridge::constants::kParamDurability, std::string("forever")) });
  REQUIRE(res_bad.size() == 1);
  REQUIRE_FALSE(res_bad[0].successful);

  // Then set to valid value; should succeed
  auto res_good = node->set_parameters({ rclcpp::Parameter(vio_bridge::constants::kParamDurability, std::string("transient_local")) });
  REQUIRE(res_good.size() == 1);
  REQUIRE(res_good[0].successful);

  rclcpp::shutdown();
}
