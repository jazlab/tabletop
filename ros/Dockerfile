FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

COPY ./apt.list /tmp/apt.list

RUN apt-get update \
    && apt-get install -y ca-certificates software-properties-common \
    && apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key \
    && add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    $(cat /tmp/apt.list) \
    && rm /tmp/apt.list

WORKDIR $HOME

RUN curl -fsSL -o /tmp/get-platformio.py \
    https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 /tmp/get-platformio.py \
    && mkdir -p $HOME/.local/bin \
    && ln -s $HOME/.platformio/penv/bin/platformio $HOME/.local/bin/platformio \
    && ln -s $HOME/.platformio/penv/bin/pio $HOME/.local/bin/pio \
    && ln -s $HOME/.platformio/penv/bin/piodebuggdb $HOME/.local/bin/piodebuggdb

ENV PIP_BREAK_SYSTEM_PACKAGES=1
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN echo "source $HOME/ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "export PATH=$HOME/.local/bin:\$PATH" >> $HOME/.bashrc
RUN echo 'eval "$(starship init bash)"' >> $HOME/.bashrc

COPY ./starship.toml /tmp/starship.toml
RUN mkdir -p $HOME/.config \
    && mv /tmp/starship.toml $HOME/.config/starship.toml

ENV ROS_LOG_DIR=/root/ws/src/tabletop/ros/logs
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED=1
