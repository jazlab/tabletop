FROM ros:jazzy

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip \
    python-is-python3 \
    ros-jazzy-ur \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rmw-cyclonedds-cpp \
    xarclock

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> $HOME/.bashrc


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV PYTHONUNBUFFERED 1

# CMD ["bash", "-c", "\
#     source /root/ws/src/tabletop/scripts/build.sh  && \
#     ros2 launch tabletop_server server.launch.py"]