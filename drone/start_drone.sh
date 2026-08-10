#!/bin/bash
tmux new-session -d -s drone -x 220 -y 50

# T1 - Pipeline
tmux send-keys -t drone:0 'source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/rs_pipeline_node.py' Enter

# T2 - SLAM
tmux split-window -h -t drone:0
tmux send-keys -t drone:0 'sleep 3 && export LD_LIBRARY_PATH=/home/jetson/ORB_SLAM3/lib:$LD_LIBRARY_PATH && export PANGOLIN_WINDOW_URI="headless://" && unset DISPLAY && ~/ros2_ws/install/orbslam3/lib/orbslam3/rgbd_inertial /home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/jetson/ORB_SLAM3/Examples/RGB-D-Inertial/RealSense_D455.yaml 2>&1 | grep -E "State=|VIBA|New Map|Fail"' Enter

# T3 - MAVROS
tmux split-window -v -t drone:0
tmux send-keys -t drone:0 'sleep 5 && source /opt/ros/humble/setup.bash && ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600' Enter

# T4 - Vision bridge
tmux split-window -v -t drone:0
tmux send-keys -t drone:0 'sleep 8 && source /opt/ros/humble/setup.bash && python3 ~/mavros_vision_bridge.py' Enter

tmux attach -t drone
