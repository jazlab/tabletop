#!/bin/bash

OUTPUT_FILE=${1:-graph.md}

NODES_TO_IGNORE="/rosbag2_recorder
/rviz2
/forward_position_controller
/forward_velocity_controller
/force_mode_controller
/force_torque_sensor_broadcaster
/freedrive_mode_controller
/joint_trajectory_controller
/passthrough_trajectory_controller"

NODES=$(ros2 node list | grep -vE "$NODES_TO_IGNORE" | tr '\n' ' ')
echo Nodes: $NODES

chmod 755 /root

su ubuntu -c "source /opt/ros/jazzy/setup.bash; \
    source /root/ws/install/setup.bash; \
    source /root/venv/bin/activate; \
    cd /root/ws/src/tabletop; \
    ros2_graph $NODES \
    -o $OUTPUT_FILE \
    --styleConfig ros/graph_style.yaml"
