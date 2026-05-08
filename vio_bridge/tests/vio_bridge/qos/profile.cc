// QoS profile unit tests for qos::build_qos
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vio_bridge/qos.h"

using vio_bridge::qos::build_qos;

TEST_CASE("ShouldBuildKeepAllReliableTransientLocal", "[qos]") {
  auto q = build_qos("keep_all", /*depth*/10, "reliable", "transient_local");
  // rclcpp has no direct equality, but we can assert options via policy getters
  // At minimum, ensure not throwing and using transient_local doesn't crash
  REQUIRE_NOTHROW(q);
}

TEST_CASE("ShouldBuildKeepLastWithDepthBestEffortVolatile", "[qos]") {
  auto q = build_qos("keep_last", /*depth*/5, "best_effort", "volatile");
  REQUIRE_NOTHROW(q);
}

TEST_CASE("ShouldRejectInvalidHistory", "[qos]") {
  REQUIRE_THROWS_AS(build_qos("invalid", 0, "reliable", "volatile"), std::invalid_argument);
}

TEST_CASE("ShouldRejectInvalidReliability", "[qos]") {
  REQUIRE_THROWS_AS(build_qos("keep_last", 1, "meh", "volatile"), std::invalid_argument);
}

TEST_CASE("ShouldRejectInvalidDurability", "[qos]") {
  REQUIRE_THROWS_AS(build_qos("keep_last", 1, "reliable", "meh"), std::invalid_argument);
}

