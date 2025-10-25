FROM tabletop/ros-base AS build-modules

WORKDIR /tabletop

# Build git submodules
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src/ros/modules,target=src/ros/modules \
    --mount=type=bind,source=bin,target=bin \
    --mount=type=bind,source=setup.bash,target=setup.bash <<EOT
source setup.bash
tt-build --only-modules --skip-uv
EOT

FROM build-modules AS build-tabletop

# Build tabletop source code
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/tmp/uv.cache \
    --mount=type=bind,source=src,target=src \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    --mount=type=bind,source=bin,target=bin \
    --mount=type=bind,source=setup.bash,target=setup.bash <<EOT
export UV_CACHE_DIR=/tmp/uv.cache
source setup.bash
tt-build
EOT

# Update .bashrc and make ~/.config if it doesn't exist
RUN <<EOT
set -ex
echo "source ~/tabletop/setup.bash" >>  ~/.bashrc
mkdir -p ~/.config
EOT
