#!/bin/bash

_output_file=${1:-graph.md}

_nodes_to_ignore=""
# /rosbag2_recorder
# /rviz2
# /forward_position_controller
# /forward_velocity_controller
# /force_mode_controller
# /force_torque_sensor_broadcaster
# /freedrive_mode_controller
# /joint_trajectory_controller
# /passthrough_trajectory_controller"

_nodes=$(ros2 node list | grep -vE "$_nodes_to_ignore" | tr '\n' ' ')
echo Nodes: $_nodes

chmod 755 /root

su ubuntu -c "source /opt/ros/jazzy/setup.bash; \
    source /root/ws/install/setup.bash; \
    source /root/venv/bin/activate; \
    cd /root/ws/src/tabletop; \
    ros2_graph $_nodes \
    -o $_output_file \
    --styleConfig ros/graph_style.yaml"
