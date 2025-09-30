# Set build variables
export COMPOSE_BAKE=true
export USER_NAME=$(id -un)
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export INSTALL_CUDA= # Used for building isaac cumotion library, if applicable

# Build
if command -v nvidia-smi >/dev/null 2>&1; then
    export USE_NVIDIA=true
    export CONTAINER_RUNTIME=nvidia
    export NVIDIA_VISIBLE_DEVICES=all
    export NVIDIA_DRIVER_CAPABILITIES=all
else
    export USE_NVIDIA=
    export CONTAINER_RUNTIME=runc
    export NVIDIA_VISIBLE_DEVICES=
    export NVIDIA_DRIVER_CAPABILITIES=
fi

export BIND_CONSISTENCY=cached
# if [[ $(uname) = Darwin ]]; then
#     export BIND_CONSISTENCY=cached
# else
#     export BIND_CONSISTENCY=consistent
# fi

if command -v pactl >/dev/null 2>&1; then
    export PULSE_SERVER_HOST=$(pactl --format=json info | jq -r '.server_string')
    if [[ -S $PULSE_SERVER_HOST ]]; then
        export PULSE_COOKIE_HOST=~/.config/pulse/cookie
    else
        export PULSE_SERVER_HOST=/tmp/pulseaudio-empty.socket
        export PULSE_COOKIE_HOST=/tmp/pulseaudio-empty.cookie
    fi
elif [ -S "$XDG_RUNTIME_DIR/pulse/native" ]; then
    export PULSE_SERVER_HOST=$XDG_RUNTIME_DIR/pulse/native
    export PULSE_COOKIE_HOST=~/.config/pulse/cookie
else
    export PULSE_SERVER_HOST=/tmp/pulseaudio-empty.socket
    export PULSE_COOKIE_HOST=/tmp/pulseaudio-empty.cookie
fi
