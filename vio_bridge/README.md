# vio_bridge

ROS 2 node that bridges VIO Odometry to a MAVROS‑compatible topic with optional field overrides and sanitization.

![CI](https://github.com/${GITHUB_REPOSITORY:-owner/repo}/actions/workflows/ci.yaml/badge.svg)

## Build & Test

- Build: `colcon build --symlink-install --cmake-args -D CMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to vio_bridge`
- Test: `colcon test --packages-select vio_bridge && colcon test-result --verbose`

## Run

```
source install/local_setup.bash
ros2 run vio_bridge vio_bridge_node
```

## Parameters

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `input_topic` | string | `/rtabmap/odom` | Input odometry topic |
| `output_topic` | string | `/mavros/odometry/out` | Output odometry topic |
| `history` | string | `keep_last` | QoS history (`keep_all` or `keep_last`) |
| `depth` | int | `10` | QoS depth for `keep_last` |
| `reliability` | string | `reliable` | QoS reliability (`reliable` or `best_effort`) |
| `durability` | string | `volatile` | QoS durability (`volatile` or `transient_local`) |
| `force_frame_id` | string | `` | If non‑empty, overrides `header.frame_id` |
| `force_child_frame_id` | string | `` | If non‑empty, overrides `child_frame_id` |
| `normalize_quaternion` | bool | `false` | Normalize orientation quaternion |
| `symmetrize_covariance` | bool | `false` | Symmetrize 6×6 covariance and clamp diagonals ≥ 0 |
| `fill_header_stamp_if_zero` | bool | `false` | Fill zero `header.stamp` with current time |
| `disable_stamp_correction_when_ros_time_zero` | bool | `true` | Under ROS time, do not fill zero stamps |
| `use_introspection_for_fields` | bool | `false` | (Experimental) Read frame fields via introspection |

## Notes

- Timestamp policy: with `fill_header_stamp_if_zero=true`, the node fills zero timestamps. If ROS time is in use and `disable_stamp_correction_when_ros_time_zero=true`, the node does not fill.
- QoS: parameters are validated; invalid values throw or are rejected at runtime.

## Development

- Style: see `docs/STYLE.md`
- Tests: Catch2 via `ament_cmake_catch2`
- CI: GitHub Actions using `ros-tooling/action-ros-ci`

