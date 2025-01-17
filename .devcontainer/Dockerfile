FROM ros:jazzy

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
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    gdb \
    iputils-ping \
    xarclock

WORKDIR /root
SHELL ["/bin/bash", "-c"]

RUN mkdir -p venv \
    && python3 -m venv venv \
    && source venv/bin/activate \
    && pip install \
    ros2-graph \
    numpy

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init bash)"' >> /root/.bashrc

COPY ./starship.toml /root/.config/starship.toml

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/venv/bin/activate" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED 1

CMD ["bash", "-c", "\
    /root/ws/src/tabletop/scripts/build.sh \
    && sleep infinity"]