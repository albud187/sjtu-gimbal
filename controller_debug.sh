#!/bin/bash
# ros2 run controller_manager ros2_control_node --ros-args --params-file /workdir/src/sjtu_drone_description/config/ros2_controllers.yaml --log-level debug

ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file /workdir/src/sjtu_drone_description/config/ros2_controllers.yaml \
  --param robot_description:="$(ros2 param get /drone1/robot_state_publisher robot_description)" \
  --log-level debug
