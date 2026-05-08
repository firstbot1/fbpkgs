cmake_minimum_required(VERSION 3.8)
project(vio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include_directories(include)

# Build an executable with explicit main() (Option B)
add_executable(vio_bridge_node
  src/main.cc
  src/vio_bridge/node.cc
  src/vio_bridge/qos.cc
  src/vio_bridge/utils.cc
  src/vio_bridge/introspection.cc
)

ament_target_dependencies(vio_bridge_node
  rclcpp
  rclcpp_components
  nav_msgs
  rosidl_typesupport_introspection_cpp
)

install(TARGETS vio_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Keep tests minimal and deterministic; register only existing test sources for now
  find_package(ament_cmake_catch2 REQUIRED)

  ament_add_catch2(vio_bridge_tests
    tests/main.cc
    tests/vio_bridge/node.cc
    tests/vio_bridge/topic/behavior.cc
    tests/vio_bridge/topic/field_transform.cc
    tests/vio_bridge/topic/sanitization.cc
    tests/vio_bridge/topic/sim_time_policy.cc
    tests/vio_bridge/qos/profile.cc
    tests/vio_bridge/qos/runtime.cc
    tests/vio_bridge/qos/keep_all_smoke.cc
    tests/vio_bridge/parameter/validation.cc
    tests/vio_bridge/parameter/runtime_transition.cc
    tests/vio_bridge/introspection/read_fields.cc
    src/vio_bridge/node.cc
    src/vio_bridge/qos.cc
    src/vio_bridge/utils.cc
    src/vio_bridge/introspection.cc
  )
  ament_target_dependencies(vio_bridge_tests
    rclcpp
    rclcpp_components
    nav_msgs
    rosidl_typesupport_introspection_cpp
  )
endif()

ament_package()
