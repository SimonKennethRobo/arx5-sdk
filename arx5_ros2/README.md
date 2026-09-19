* [ ]

# arx5_ros2

C++ ROS 2 Foxy wrapper node for the ARX5 SDK, mirroring the topic/service
layout of [`python/communication/ros2_node.py`](../python/communication/ros2_node.py).
Targets Jetson Orin (aarch64) running ROS 2 Foxy.

Note: it compiles `arx5-sdk`'s controller sources directly (like the Python
pybind module does), rather than linking `arx5-sdk`'s own `build/` output, so
it has no dependency on that directory having been built first — only on the
prebuilt `lib/aarch64/{libhardware,libsolver}.so` shipped in this repo.

Verified building and running (`ros2 run arx5_ros2 arx5_ros2_node`) on ROS 2
Foxy / Ubuntu 20.04 aarch64 on a Jetson Orin.

### Dependency resolution — do NOT `conda activate` before building

Eigen3, orocos_kdl, kdl_parser, fmt, spdlog and Boost must resolve to ROS 2
Foxy's / the system's own copies (`ros-foxy-orocos-kdl`, `ros-foxy-kdl-parser`,
`libfmt-dev`, `libspdlog-dev`, `libboost-dev`, all normally already present
with a Foxy install), **not** the SDK's conda environment. `rclcpp` itself is
built against the system's fmt/spdlog ABI, so pointing `CMAKE_PREFIX_PATH` at
a `conda activate`d env (which ships much newer fmt/spdlog) makes the linker
pick two incompatible copies and fail with `undefined reference to fmt::v12::...` or similar.

The one dependency genuinely only available via conda is `soem` (EtherCAT
master; the SDK's `conda_environments/*.yaml` pull it in, and it has no
apt/ROS package). Point the build at just its lib directory with
`-DARX5_SOEM_DIR=...`, or `export CONDA_PREFIX=/path/to/env` (without
activating) and it's picked up automatically. The prebuilt
`libhardware.so`/`libsolver.so` also need a `libstdc++.so.6` newer than
Ubuntu 20.04's system one (built with a newer GCC); the same conda env's copy
is reused for that (via a runtime `RPATH`, ordered so it's only a fallback —
same-named libraries the conda env also happens to ship, like `liburdf.so`,
are resolved from ROS's own lib dir first).

## Topics & services

The Go2-X5 integration uses a request/target split. This node is the actuator
endpoint and never publishes back onto a request topic:

- Publishes `/go2_x5/arm/state` (`sensor_msgs/msg/JointState`) with names `x5_joint1` ... `x5_joint6`.
- Publishes `/go2_x5/arm/driver/mode` (`std_msgs/msg/String`) as the SDK execution state.
- Subscribes `/go2_x5/arm/command/target` (`trajectory_msgs/msg/JointTrajectory`).
- Subscribes `/go2_x5/arm/mode/target` (`std_msgs/msg/String`) with `HOME`, `HOLD`, `DAMPING`, or `OCS2`.

The private EEF/gripper endpoints and reset/float services remain available for
standalone arm tooling. Use `control_mode:=joint` for the Go2-X5 graph.

## Parameters

| Name                       | Type   | Default       | Notes                                        |
| -------------------------- | ------ | ------------- | -------------------------------------------- |
| `model`                  | string | `X5`        | `X5`, `X5_umi`, `L5`, `X7_left`, ... |
| `interface`              | string | `can0`      | CAN interface name                           |
| `sdk_config_file`        | string | empty       | SDK YAML path; empty uses `config/arx5.yaml` |
| `control_mode`           | string | `joint` | `cartesian` or `joint`                   |
| `publish_rate`           | double | `50.0`      | Hz, state publish rate                       |
| `auto_home`              | bool   | `false`     | Move to home on startup                      |
| `gravity_compensation`   | bool   | `true`      |                                              |
| `base_frame`             | string | `base_link` | `frame_id` for `~/eef_state`             |
| `joint_command_duration` | double | `0.0`       | seconds; preview time for joint commands     |

## Build

Set up a colcon workspace and symlink (or copy) this directory in as a
package:

```sh
mkdir -p ~/arx5_ros2_ws/src
ln -s /path/to/arx5-sdk/ros2 ~/arx5_ros2_ws/src/arx5_ros2

source /opt/ros/foxy/setup.bash   # do NOT also `conda activate` — see above

cd ~/arx5_ros2_ws
colcon build --packages-select arx5_ros2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DARX5_SOEM_DIR=/home/unitree/miniconda3/envs/arx-py310/lib \
  -DARX5_SDK_DIR=/home/unitree/Projects/Simon/WBC/Ctrl/arx5-sdk
```

If this `ros2/` directory is copied somewhere other than `<arx5-sdk>/ros2`,
also point the build at the SDK checkout explicitly with
`-DARX5_SDK_DIR=/path/to/arx5-sdk`.

## Run

```sh
source ~/arx5_ros2_ws/install/setup.bash
ros2 launch arx5_ros2 arx5_ros2.launch.py model:=X5 interface:=can0 control_mode:=cartesian sdk_config_file:=/path/to/arx5.yaml
```

or directly:

```sh
ros2 run arx5_ros2 arx5_ros2_node --ros-args -p model:=X5 -p interface:=can0
```
