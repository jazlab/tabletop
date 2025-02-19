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
    vim \
    psmisc \
    iputils-ping \
    net-tools \
    gdb \
    xarclock

WORKDIR $HOME
SHELL ["/bin/bash", "-c"]

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y

RUN curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 /tmp/get-platformio.py \
    && mkdir -p $HOME/.local/bin \
    && ln -s $HOME/.platformio/penv/bin/platformio $HOME/.local/bin/platformio \
    && ln -s $HOME/.platformio/penv/bin/pio $HOME/.local/bin/pio \
    && ln -s $HOME/.platformio/penv/bin/piodebuggdb $HOME/.local/bin/piodebuggdb

ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN pip install \
    trimesh \
    pyfqmr \
    "pyglet<2" \
    ros2-graph

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN echo "source $HOME/ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "PATH=$HOME/.local/bin:\$PATH" >> $HOME/.bashrc
RUN echo 'eval "$(starship init bash)"' >> $HOME/.bashrc

RUN mkdir -p $HOME/.config
COPY ./starship.toml $HOME/.config/starship.toml

ENV ROS_LOG_DIR=/root/ws/src/tabletop/ros/log
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED=1