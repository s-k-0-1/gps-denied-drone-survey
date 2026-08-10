#!/bin/bash
# MAVROS launch for Team Viman Rakshak drone
# Uses corrected px4_config.yaml + ds_config.yaml params-file workaround

source ~/drone_ws/install/setup.bash

ros2 run mavros mavros_node \
  --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760 \
  -p tgt_system:=1 \
  -p tgt_component:=1 \
  -p config_yaml:=/home/jetson/drone_ws/px4_config.yaml \
  -p pluginlists_yaml:=/home/jetson/drone_ws/px4_pluginlists.yaml \
  --params-file /home/jetson/drone_ws/ds_config.yaml
