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

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init bash)"' >> /root/.bashrc

RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 get-platformio.py

RUN mkdir -p venv \
    && python3 -m venv venv \
    && source venv/bin/activate \
    && pip install \
    ros2-graph \
    numpy \
    && deactivate

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
RUN echo "PATH=\$PATH:/root/.platformio/penv/bin" >> /root/.bashrc


COPY ../ros/tabletop_server/00-teensy.rules /etc/udev/rules.d/00-teensy.rules


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED 1

CMD ["bash", "-c", "sleep infinity"]
