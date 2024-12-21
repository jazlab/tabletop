FROM ros:jazzy

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y python3-pip \
    python-is-python3 \
    ros-jazzy-ur \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-ros2controlcli \
    ros-jazzy-cv-bridge \ 
    ros-jazzy-rmw-cyclonedds-cpp \
    gdb \
    iputils-ping \
    xarclock

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> $HOME/.bashrc

WORKDIR /root

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init bash)"' >> /root/.bashrc

COPY ./starship.toml /root/.config/starship.toml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED 1

CMD ["bash", "-c", "\
    source /root/ws/src/tabletop/scripts/build.sh  && \
    sleep infinity"]