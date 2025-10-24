# Build variables
export BUILDKIT_PROGRESS=plain
export COMPOSE_BAKE=true
export COMPOSE_REMOVE_ORPHANS=true
export USER_NAME=$(id -un)
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export INSTALL_CUDA= # Used for building isaac cumotion library, if applicable

# Nvidia build and runtime variables
export CUDA_VERSION=${CUDA_VERSION:-129}
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

# Bind mount consistency
export BIND_CONSISTENCY=cached
# if [[ $(uname) = Darwin ]]; then
#     export BIND_CONSISTENCY=cached
# else
#     export BIND_CONSISTENCY=consistent
# fi

# Collect absolute targets of symlinks in /dev/flir into a bash array, then make a JSON-style string
FLIR_DEV_PATHS=()
if [ -d /dev/flir ]; then
    # iterate entries (non-recursive)
    while IFS= read -r -d '' entry; do
        if [ -L "$entry" ]; then
            target=$(readlink -f "$entry" 2>/dev/null)
            if [ -n "$target" ]; then
                FLIR_DEV_PATHS+=("$target")
            fi
        fi
    done < <(find /dev/flir -maxdepth 1 -mindepth 1 -print0 2>/dev/null)
fi

# printf -v FLIR_DEVS "%s, " "${FLIR_DEV_PATHS[@]}"
# export FLIR_DEVS="[ ${FLIR_DEVS%, } ]"

export FLIR_DEV_0="${FLIR_DEV_PATHS[0]}"
export FLIR_DEV_1="${FLIR_DEV_PATHS[1]}"
export FLIR_DEV_2="${FLIR_DEV_PATHS[2]}"
export FLIR_DEV_3="${FLIR_DEV_PATHS[3]}"
export FLIR_DEV_4="${FLIR_DEV_PATHS[4]}"
export FLIR_DEV_5="${FLIR_DEV_PATHS[5]}"

# # Build a JSON-style string from the array (properly escape backslashes and quotes)
# FLIR_DEV_JSON="[]"
# if [ ${#FLIR_DEV_PATHS[@]} -gt 0 ]; then
#     json='['
#     first=true
#     for p in "${FLIR_DEV_PATHS[@]}"; do
#         esc=${p//\\/\\\\}
#         esc=${esc//\"/\\\"}
#         if $first; then
#             json+="\"$esc\""
#             first=false
#         else
#             json+=",\"$esc\""
#         fi
#     done
#     json+=']'
#     FLIR_DEV_JSON=$json
# fi

# export FLIR_DEV_JSON
# # FLIR_DEV_PATHS remains a shell array for in-shell use

# Pulse audio
export PULSE_SERVER_MNT=/dev/null:/dev/null
export PULSE_COOKIE_MNT=/dev/null:/dev/null
export PULSE_SERVER_CONTAINER=

if command -v pactl >/dev/null 2>&1; then
    PULSE_SERVER_HOST=$(pactl --format=json info | jq -r '.server_string')
    if [[ ! -S $PULSE_SERVER_HOST ]]; then
        unset PULSE_SERVER_HOST
    fi
elif [ -S "$XDG_RUNTIME_DIR/pulse/native" ]; then
    PULSE_SERVER_HOST=$XDG_RUNTIME_DIR/pulse/native
fi

if [[ $PULSE_SERVER_HOST ]]; then
    export PULSE_SERVER_MNT=$PULSE_SERVER_HOST:/tmp/pulseaudio.socket
    export PULSE_COOKIE_MNT=~/.config/pulse/cookie:/home/$USER_NAME/.config/pulse/cookie
    export PULSE_SERVER_CONTAINER=unix:/tmp/pulseaudio.socket
fi

export KITTY_LISTEN_ON_MNT=/dev/null:/dev/null
export KITTY_LISTEN_ON_CONTAINER=
if [[ $KITTY_LISTEN_ON = unix* ]]; then
    export KITTY_LISTEN_ON_MNT=${KITTY_LISTEN_ON#unix:}:/tmp/kitty
    export KITTY_LISTEN_ON_CONTAINER=unix:/tmp/kitty
elif [[ $KITTY_LISTEN_ON = tcp* ]]; then
    export KITTY_LISTEN_ON_CONTAINER=$KITTY_LISTEN_ON
fi
