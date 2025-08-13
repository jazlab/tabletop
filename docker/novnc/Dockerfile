FROM debian:latest

ENV DEBIAN_FRONTEND=noninteractive

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install git, supervisor, VNC, & X11 packages
RUN --mount=type=cache,target=/var/cache/apt,id=novnc-apt-cache,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,id=novnc-apt-lib,sharing=locked <<EOT
set -ex
apt-get update && apt-get upgrade -y
apt-get install --no-install-recommends -y \
    bash \
    dbus-x11 \
    dwm \
    net-tools \
    novnc \
    suckless-tools \
    supervisor \
    x11vnc \
    xterm
EOT

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
