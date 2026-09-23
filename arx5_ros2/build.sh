colcon build --packages-select arx5_ros2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DARX5_SOEM_DIR=/opt/miniconda3/envs/arx-py310/lib \
  -DARX5_SDK_DIR=$HOME/Projects/Simon/WBC/Ctrl/arx5-sdk