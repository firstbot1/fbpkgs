// Minimal executable entrypoint for vio_bridge_node (Option B)
#include <rclcpp/rclcpp.hpp>
#include "vio_bridge/node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared < vio_bridge::VioBridgeNode > ();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
