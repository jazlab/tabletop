# DWM's workspace count is compiled in. Keep the existing 6.5 version, with
# exactly six numbered workspaces and no shortcuts/rules for removed tags.
FROM debian:trixie AS dwm-build
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential libx11-dev libxft-dev libxinerama-dev
WORKDIR /build
ADD --checksum=sha256:21d79ebfa9f2fb93141836c2666cb81f4784c69d64e7f1b2352f9b970ba09729 \
    https://dl.suckless.org/dwm/dwm-6.5.tar.gz /tmp/dwm.tar.gz
RUN tar -xzf /tmp/dwm.tar.gz --strip-components=1 && \
    sed -i \
      -e 's/"6", "7", "8", "9"/"6"/' \
      -e 's/1 << 8/0/' \
      -e 's/"st", NULL/"xterm", NULL/' \
      -e '/TAGKEYS(.*XK_[789],/d' config.def.h && \
    make

FROM debian:trixie

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install git, supervisor, VNC, & X11 packages
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache-novnc,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,id=apt-lib-novnc,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
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

COPY --from=dwm-build /build/dwm /usr/local/bin/dwm

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
