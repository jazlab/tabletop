FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# Install apt dependencies
RUN apt-get update \
    && apt-get install -y ca-certificates software-properties-common \
    && apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key \
    && add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'

# Install apt packages
COPY ./apt.list /tmp/apt.list
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    $(cat /tmp/apt.list) \
    && rm /tmp/apt.list

# Set workspace directory
WORKDIR $HOME

# Install platformio
RUN curl -fsSL -o /tmp/get-platformio.py \
    https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 /tmp/get-platformio.py \
    && mkdir -p $HOME/.local/bin \
    && ln -s $HOME/.platformio/penv/bin/platformio $HOME/.local/bin/platformio \
    && ln -s $HOME/.platformio/penv/bin/pio $HOME/.local/bin/pio \
    && ln -s $HOME/.platformio/penv/bin/piodebuggdb $HOME/.local/bin/piodebuggdb

# Install pip packages
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# Install starship
RUN curl -sS https://starship.rs/install.sh | sh -s -- -y

# Copy starship config
COPY ./starship.toml /tmp/starship.toml
RUN mkdir -p $HOME/.config \
    && mv /tmp/starship.toml $HOME/.config/starship.toml

# Update .bashrc
RUN echo 'export PATH=$HOME/.local/bin:$PATH' >> $HOME/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> $HOME/.bashrc
RUN echo 'source $HOME/ws/install/setup.bash' >> $HOME/.bashrc
RUN echo 'source $HOME/ws/src/tabletop/bin/setup.bash' >> $HOME/.bashrc
RUN echo 'eval "$(starship init bash)"' >> $HOME/.bashrc
