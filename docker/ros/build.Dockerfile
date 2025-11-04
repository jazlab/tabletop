FROM base_image

WORKDIR /tabletop

ARG USER_UID
ARG USER_GID

ENV CCACHE_DIR=/tmp/ccache

# Build git submodules
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/tmp/ccache,uid=$USER_UID,gid=$USER_GID \
    --mount=type=cache,target=build,uid=$USER_UID,gid=$USER_GID \
    --mount=type=cache,target=install,uid=$USER_UID,gid=$USER_GID \
    --mount=type=bind,source=src/ros/modules,target=src/ros/modules \
    --mount=type=bind,source=bin,target=bin \
    --mount=type=bind,source=setup.bash,target=setup.bash <<EOT
set -e
source setup.bash
tt-build --only-modules --skip-uv
rsync -av build install /tmp/colcon
EOT

# Build tabletop source code
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/tmp/ccache,uid=$USER_UID,gid=$USER_GID \
    --mount=type=cache,target=build,uid=$USER_UID,gid=$USER_GID \
    --mount=type=cache,target=install,uid=$USER_UID,gid=$USER_GID \
    --mount=type=bind,source=src/ros,target=src/ros \
    --mount=type=bind,source=bin,target=bin \
    --mount=type=bind,source=setup.bash,target=setup.bash <<EOT
set -e
source setup.bash
tt-build --skip-uv
rsync -av build install /tmp/colcon
EOT

RUN cd /tmp/colcon && rsync -av build install /tabletop

ARG USE_NVIDIA
ARG CUDA_VERSION

RUN --mount=type=cache,target=/tmp/uv.cache,uid=$USER_UID,gid=$USER_GID \
    --mount=type=bind,source=src/tabletop_py,target=src/tabletop_py \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml <<EOT
set -ex
export UV_CACHE_DIR=/tmp/uv.cache
if [[ $USE_NVIDIA = true ]] ; then
    UV_EXTRA="--extra cu$CUDA_VERSION"
else
    UV_EXTRA="--extra cpu"
fi
uv sync --locked $UV_EXTRA
EOT

# Update .bashrc and make ~/.config if it doesn't exist
RUN <<EOT
set -ex
echo "source /tabletop/setup.bash" >>  ~/.bashrc
mkdir -p ~/.config
EOT
