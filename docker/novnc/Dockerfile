FROM debian:latest

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install git, supervisor, VNC, & X11 packages
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache-novnc,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,id=apt-lib-novnc,sharing=locked <<EOT
set -ex
apt-get update && apt-get upgrade -y
apt-get install -y \
    bash \
    dbus-x11 \
    dwm \
    net-tools \
    novnc \
    suckless-tools \
    supervisor \
    x11vnc \
    xterm \
    xvfb
EOT

# Set user to root
USER root

# Copy app files
COPY . /app
RUN chmod +x /app/entrypoint.sh /app/healthcheck.sh

# Set up entrypoint,
ENTRYPOINT ["/app/entrypoint.sh"]
CMD []

# Set up healthcheck
HEALTHCHECK --interval=10s --timeout=2s --retries=3 --start-period=10s --start-interval=2s CMD ["/app/healthcheck.sh"]

# Setup environment variables
ENV PORT=8080
ENV DISPLAY=:0.0
ENV DISPLAY_WIDTH=1920
ENV DISPLAY_HEIGHT=1080
ENV WINDOW_MANAGER=dwm
