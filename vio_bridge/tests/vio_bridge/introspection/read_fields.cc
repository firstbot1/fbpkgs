// Introspection tests: read child_frame_id/frame_id via reflection
#include <catch2/catch.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "vio_bridge/introspection.h"
#include "vio_bridge/constants.h"

using nav_msgs::msg::Odometry;

TEST_CASE("ShouldFallbackWhenFieldMissing", "[introspection]") {
  Odometry msg; // frame_id empty by default
  const rosidl_message_type_support_t* ts = nullptr; // unavailable in some environments
  std::string out;
  const bool ok = vio_bridge::introspection::try_read_string_field(
    &msg, ts, "no_such_field", out);
  REQUIRE_FALSE(ok);
}
