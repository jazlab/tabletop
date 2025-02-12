FROM tabletop-server:latest

RUN curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init bash)"' >> $HOME/.bashrc

RUN mkdir -p venv \
    && python3 -m venv venv \
    && source venv/bin/activate \
    && pip install \
    ros2-graph \
    numpy \
    && deactivate
