FROM debian:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install git, supervisor, VNC, & X11 packages
RUN set -ex; \
    apt-get update; \
    apt-get install -y \
    bash \
    dbus-x11 \
    dwm \
    fluxbox \
    i3 \
    net-tools \
    novnc \
    suckless-tools \
    supervisor \
    xfce4 \
    xfce4-goodies \
    x11vnc \
    xterm \
    xvfb

# Setup environment variables
ENV HOME=/root \
    USER=root \
    DISPLAY=:0.0
# SESSION_MANAGER= \
# DBUS_SESSION_BUS_ADDRESS=

COPY ./config/i3.config /root/.config/i3/config
COPY . /app
RUN chmod +x /app/entrypoint.sh
RUN chmod +x /app/healthcheck.sh
ENTRYPOINT ["/app/entrypoint.sh"]
CMD []
EXPOSE 8080
