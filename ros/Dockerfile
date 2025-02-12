FROM ros:jazzy AS server

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    python3-pip \
    python3-venv \
    python-is-python3 \
    ros-jazzy-ur \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-ros2controlcli \
    ros-jazzy-cv-bridge \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    gdb \
    iputils-ping \
    net-tools \
    xarclock

WORKDIR $HOME
SHELL ["/bin/bash", "-c"]

RUN curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 /tmp/get-platformio.py \
    && mkdir -p $HOME/.local/bin \
    && ln -s $HOME/.platformio/penv/bin/platformio $HOME/.local/bin/platformio \
    && ln -s $HOME/.platformio/penv/bin/pio $HOME/.local/bin/pio \
    && ln -s $HOME/.platformio/penv/bin/piodebuggdb $HOME/.local/bin/piodebuggdb

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN echo "source $HOME/ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "PATH=$HOME/.local/bin:\$PATH" >> $HOME/.bashrc

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED 1
