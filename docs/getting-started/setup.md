# Setup

TableTop runs in Docker, so the only hard host requirement is Docker itself.
GPU, audio, and firmware tooling are optional. For the exhaustive hardware
walkthrough (robot network, URCaps, Teensy), see the
[README](https://github.com/jazlab/tabletop#setup).

## Requirements

| Requirement | Needed for | Install |
|---|---|---|
| [Docker](https://docs.docker.com/engine/install/) | everything | official docs |
| [VS Code](https://code.visualstudio.com/) | Dev Container development | optional |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | GPU in containers | optional |
| PipeWire/PulseAudio | reward sounds | `scripts/install/pulse.sh` |
| [PlatformIO](https://platformio.org/install/) | building firmware on the host | preinstalled in the container |

!!! tip "Apple Silicon"
    Enable **Use Rosetta for x86/amd64 emulation** in Docker Desktop to run the
    `ursim` container.

## Minimal install

```bash
# 1. Clone and pull submodules
git clone https://github.com/jazlab/tabletop.git
cd tabletop
git submodule update --init --recursive --remote

# 2. Source the environment (consider adding to ~/.bashrc)
source setup.bash

# 3. Generate the .env file (detects GPU / FLIR cameras / PulseAudio)
tt-env-gen

# 4. Build the Docker images + ROS 2 workspace
tt-compose build
```

`setup.bash` is the single source of environment truth: it detects host vs.
container, activates the right uv virtualenv, sources ROS, and puts the
`tt-*` commands on your `PATH`. Re-run `tt-env-gen` whenever you plug or
unplug hardware, since device paths are baked into `.env`.

## Optional host setup

These configure the host for real hardware and are run **by path** (they make
persistent, privileged changes, so they are intentionally not on `PATH`):

```bash
./scripts/configure/udev-configure.sh            # Teensy / device udev rules
./scripts/configure/usbfs-configure.sh           # USB buffer size for FLIR
./scripts/configure/robot-network.sh             # robot subnet interface
./scripts/configure/cpu-speed-scaling-disable.sh # real-time control
```

Firmware (Teensy + Flic micro-controllers) is built/flashed via the container:

```bash
tt-microros-build
```

## Next steps

- [Usage](usage.md) — build, start containers, run a task.
- [CLI & Tooling](../guide/cli.md) — the full `tt-*` command set.
