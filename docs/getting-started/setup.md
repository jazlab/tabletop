# Setup

TableTop runs in Docker, so the only hard host requirement is Docker itself.
GPU, audio, and firmware tooling are optional. For the full real-hardware
walkthrough (robot network, URCaps, remote control), see
[Real Hardware Setup](real-hardware.md).

## Requirements

| Requirement | Needed for | Install |
| --- | --- | --- |
| [Docker](https://docs.docker.com/engine/install/) | everything | official docs |
| [Bash](https://www.gnu.org/software/bash/) (recent, ≥ 4) | the `tt-*` scripts | preinstalled on Linux; Can be [installed via `brew`](https://formulae.brew.sh/formula/bash) on macOS, which ships with version 3.2 by default |
| [uv](https://docs.astral.sh/uv/) | host Python env + `tt-env-gen` | official docs |
| [VS Code](https://code.visualstudio.com/) | Dev Container development | optional |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | GPU in containers | optional |
| PipeWire/PulseAudio | reward sounds | see [Audio](#audio-reward-sounds) |
| [PlatformIO](https://platformio.org/install/) | Teensy/Flic firmware | preinstalled in the container; only needed on the host to build firmware outside the container |

!!! tip "Apple Silicon"
    Enable **Use Rosetta for x86/amd64 emulation** in Docker Desktop to run the
    `ursim` container.

## Minimal install

```bash
# 1. Clone and pull submodules
git clone --recurse-submodules https://github.com/jazlab/tabletop.git
cd tabletop

# If you forgot to use `--recurse-submodules` when cloning,
# you can use the following command to pull the submodules
git submodule update --init --recursive

# 2. Source the environment (consider adding to ~/.bashrc)
source setup.bash

# 3. Sync the host Python environment (required by tt-env-gen).
#    Add `--extra cu130` to also pull the CUDA 13.0 wheels for PyTorch
#    GPU work (or whatever CUDA version your driver supports, see
#    `pyproject.toml` for available dependency groups, e.g. `--extra cu128`
#    for CUDA 12.8, or `--extra cpu` if you do not have a CUDA-capable GPU).
#    This is only necessary if you want to run the gaze estimation
#    calibration pipeline on the host machine and need support for a
#    different CUDA version than the default one available through PyPI.
#    Otherwise you can run it in the Devcontainer or using the commander
#    container, both of which user the NVIDIA Container Runtime if available.
uv sync            # or: uv sync --extra cu130

# 4. Generate the .env file (detects GPU / FLIR cameras / PulseAudio)
tt-env-gen

# 5. Edit .env to fill in your personal preferences (e.g. noVNC / Foxglove ports)
nano .env          # or whatever editor you prefer

# 6. Build the Docker images + ROS 2 workspace
tt-compose build
```

`setup.bash` is the single source of environment truth: it detects host vs.
container, activates the right uv virtualenv, sources ROS, and puts the
`tt-*` commands on your `PATH`.

`.env` is used **exclusively** for variable substitution in `compose.yaml`
(device paths, ports, GPU runtime, PulseAudio mounts); Docker Compose reads it
and may pass selected values into the containers. Keeping these substitutions
in `.env` — rather than in your shell — is what lets the Dev Container be built
directly from VS Code while still resolving every `$VAR` in `compose.yaml`.
Re-run `tt-env-gen` whenever you plug or unplug hardware, since device paths are
baked into `.env`.

## Optional host setup

These configure the host for real hardware and are run **by path** (they make
persistent, privileged changes, so they are intentionally not on `PATH`):

```bash
./scripts/configure/udev-configure.sh            # configure Teensy and Flir device udev rules
./scripts/configure/usbfs-configure.sh           # increase USB buffer size for FLIR
./scripts/configure/robot-network.sh             # configure robot subnet interface
./scripts/configure/scp-urcaps.sh                # copy urcaps to robot control boxes over SSH
./scripts/configure/cpu-speed-scaling-disable.sh # enable 'performance' CPU governor for real-time robot control
```

### Audio (reward sounds)

Reward sounds need PipeWire/PulseAudio on the host so audio can pass through to
the container. Install per your OS/distro-specific instructions. For Ubuntu and
MacOS, the following will typically suffice:

```bash
# Ubuntu (PipeWire as the modern PulseAudio replacement)
# This uninstalls the deprecated `pipewire-media-session` package if it was
# previously installed and installs `wireplumber`, which supersedes it.
# Remove `pipewire-media-session-` from the install command below if you need
# this package for any reason (you probably don't, since `wireplumber` should
# be a drop-in replacement).
sudo apt update
sudo apt install -y pipewire-pulse wireplumber pipewire-audio-client-libraries pipewire-media-session-
systemctl --user --now enable wireplumber.service

# macOS
brew install pulseaudio
brew services start pulseaudio
```

### Firmware

Firmware (Teensy + Flic micro-controllers) is built/flashed via the container
(this also works from the Dev Container):

```bash
tt-microros-build
```

!!! tip "First upload usually fails"
    `tt-microros-build` retries the build/upload twice. The first attempt
    almost always builds successfully but fails to *upload*; the second attempt
    almost always succeeds. This is a quirk of the PlatformIO upload tooling.

!!! note "Flic Micro firmware is incomplete"
    The Teensy is the only currently supported micro-controller.
    The Flic Micro firmware is **incomplete** and has no launch file,
    `tt-launch` entry, or Docker compose service; if you need it, you
    can add a launch file nearly identical to `teensy.launch.py` with
    a different micro-ROS agent node name and device path, then create
    an entry in `bin/container/tt-launch` and a service in `compose.yaml`.
    See [Architecture §5.1](../architecture.md).

## Next steps

- [Usage](usage.md) — build, start containers, run a task.
- [CLI & Tooling](../guide/cli.md) — the full `tt-*` command set.
