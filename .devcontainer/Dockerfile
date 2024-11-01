FROM ros:jazzy

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip \
    python-is-python3 \
    ros-jazzy-ur \
    iproute2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /${HOME}/.bashrc

ENV SHELL /bin/bash
ENV PYTHONUNBUFFERED 1

CMD ["/bin/bash"]