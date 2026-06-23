# Real Hardware Setup

This page covers the one-time configuration needed to drive the **real**
hardware (the network, the UR5e robot, and the rig computers). If you only
intend to run in simulation (`robot_mode:=mock`), you can skip this page
entirely.

Before starting, make sure you have completed the [Setup](setup.md) steps
(clone, `source setup.bash`, `tt-env-gen`).

!!! note "Tested on Ubuntu 24.04"
    The real hardware has only been tested with a host machine running
    **Ubuntu 24.04**. The command-line examples on this page are written for
    Ubuntu 24.04; on another distribution use the equivalent tools (or the
    graphical settings). Where an approach differs across systems it is called
    out inline.

!!! note "Teensy firmware"
    Building and flashing the Teensy micro-controller firmware is covered in
    [Setup → Firmware](setup.md#firmware).

## Host configuration (Ubuntu 24.04)

These steps prepare the host for device access and real-time control. They are
the concrete commands that work on Ubuntu 24.04; on another distribution use the
equivalent mechanism (the udev rules themselves are distro-independent).

### Device access (udev rules)

The Teensy / Flic Micro and the FLIR cameras need udev rules so the containers
can open them — and so each FLIR camera gets a stable `/dev/flir/<serial>`
symlink. Create the rule files under `/etc/udev/rules.d/` and reload:

```bash
# Teensy / Flic Micro (vendor 16c0, plus the 1fc9 BLE dongle)
sudo tee /etc/udev/rules.d/00-teensy.rules > /dev/null <<'EOF'
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789a]*", ENV{MTP_NO_PROBE}="1"
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666", RUN:="/bin/stty -F /dev/%k raw -echo"
KERNEL=="hidraw*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", MODE:="0666"
EOF

# FLIR cameras (vendor 1e10 / 1724; creates /dev/flir/<serial> symlinks)
sudo tee /etc/udev/rules.d/40-flir.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", MODE="0666" SYMLINK+="flir/%s{serial}"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1724", MODE="0666" SYMLINK+="flir/%s{serial}"
EOF

# Reload and apply
sudo udevadm control --reload
sudo udevadm trigger
```

PlatformIO also ships
[generic board udev rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html);
the custom Teensy rule above replaces them. After plugging or unplugging a
device, re-run `tt-env-gen` so Docker re-maps it.

### USB buffer size (FLIR cameras)

FLIR cameras stream large frames and need a bigger USBFS buffer than the kernel
default. Raise `usbcore.usbfs_memory_mb` for the current session and persist it
via GRUB:

```bash
# Current session (effective immediately)
sudo sh -c 'echo 5000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

# Persist across reboots (GRUB)
sudo tee /etc/default/grub.d/99-usbfs-memory.cfg > /dev/null <<'EOF'
GRUB_CMDLINE_LINUX_DEFAULT="$GRUB_CMDLINE_LINUX_DEFAULT usbcore.usbfs_memory_mb=5000"
EOF
sudo update-grub
```

On a distro without `update-grub`, add `usbcore.usbfs_memory_mb=5000` to the
kernel command line through your bootloader's mechanism instead.

### CPU governor (real-time control)

For deterministic real-time robot control, disable CPU frequency scaling by
pinning the CPU to the `performance` governor — the setup
[recommended by the `ur_robot_driver` real-time guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html).
Any mechanism that pins the governor works; the
[`cpufrequtils`](https://manpages.ubuntu.com/manpages/noble/man1/cpufreq-info.1.html)
approach below (listed under [Requirements](setup.md#requirements)) is just one
example, shown for Ubuntu/Debian:

```bash
sudo apt install -y cpufrequtils
sudo systemctl disable ondemand || true
echo 'GOVERNOR=performance' | sudo tee /etc/default/cpufrequtils
sudo systemctl enable --now cpufrequtils
```

Tools with equivalent functionality on other distributions include:

- [`cpupower`](https://www.kernel.org/doc/html/latest/admin-guide/pm/cpufreq.html)
  — `sudo cpupower frequency-set -g performance`; ships with the kernel tools
  package (`linux-tools-common`/`linux-tools-$(uname -r)` on Debian/Ubuntu,
  `kernel-tools` on Fedora/RHEL).
- [TuneD](https://tuned-project.org/) — apply a real-time-oriented profile such
  as `latency-performance` (`sudo tuned-adm profile latency-performance`),
  common on Fedora/RHEL.

### Real-time kernel (optional, recommended)

The same
[`ur_robot_driver` real-time guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html)
recommends running a real-time or low-latency kernel for the best control
performance. On Ubuntu 24.04 the low-latency kernel is a drop-in install:

```bash
sudo apt install -y linux-lowlatency
sudo reboot
```

Confirm the running kernel after rebooting with `uname -a` (it should report
`lowlatency`). For a fully preemptible `PREEMPT_RT` kernel, see
[Ubuntu's real-time kernel](https://ubuntu.com/real-time); on other
distributions install that distro's real-time or low-latency kernel package.

## The TableTop network

The host, the UR5e robot control box(es), and the other rig computers (the
EyeLink and OptiTrack hosts) all communicate over a single wired LAN — the
**TableTop network** (it carries more than just the robots, so it is the local
network for the whole rig, not a "robot network").

### Addressing

Every device on the network gets a static IPv4 address in the same `/24` subnet.
The defaults (overridable via `setup.bash`) put everything on `192.168.13.0/24`:

| Device | Variable (in `setup.bash`) | Default |
| --- | --- | --- |
| Host (this machine) | `REVERSE_IP` | `192.168.13.10` |
| Robot / right arm | `ROBOT_IP` / `RIGHT_ROBOT_IP` | `192.168.13.20` |
| Left arm | `LEFT_ROBOT_IP` | `192.168.13.21` |
| EyeLink / OptiTrack hosts | — | pick free addresses in the same subnet |

The host address (`REVERSE_IP`) and each robot address (`ROBOT_IP`) **must share
the same first three octets** — i.e. live in the same `/24` subnet — or the UR
driver cannot reach the robot. The simulator uses a separate `192.168.12.0/24`
subnet (the `SIM_*` variables) and needs no physical network.

### Connecting the devices (network switch)

This configuration assumes an **Ethernet switch** bridges all the devices into
one LAN. Any unmanaged gigabit switch is sufficient. With a standard Ethernet
cable each, connect:

- the host machine,
- each UR5e control box,
- the EyeLink host PC,
- the OptiTrack host PC,

to a port on the switch. The switch simply forwards traffic between the devices;
no router or DHCP server is required, because every device uses a static
address.

### Assigning the host's static IP

**Graphical (GNOME / Ubuntu Settings).** Open **Settings → Network**, click the
gear next to the wired connection, open the **IPv4** tab, set **Method** to
**Manual**, and add the address `192.168.13.10` with netmask `255.255.255.0`
(leave the gateway blank). Apply, then toggle the connection off and on. Any
network-settings UI on your OS can do the same thing.

**Command line (Ubuntu, non-persistent).** Add the address directly to the
ethernet interface (find yours with `ip link`, then replace `<iface>`):

```bash
sudo ip addr add 192.168.13.10/24 dev <iface>
```

This lasts until the next reboot. To make it persistent, use the graphical
settings above,
[`nmcli`](https://networkmanager.dev/docs/api/latest/nmcli.html), or a
[netplan](https://netplan.io/) file under `/etc/netplan/`.

### Firewall

If you run a host firewall (e.g. [`ufw`](https://help.ubuntu.com/community/UFW)),
make sure it does not block traffic on the TableTop subnet — the UR driver and
the rig computers must reach the host on several ports. Either allow the subnet
or disable the firewall while on the rig network:

```bash
sudo ufw allow from 192.168.13.0/24   # allow the local subnet
# or, to turn the firewall off entirely:
sudo ufw disable
```

## UR5e robot

### Setting the robot IP address

With the network in place, set a static IP on the robot via the Teach Pendant:

1. Click the "hamburger" (menu) icon in the top-right corner of the window.
2. Click **Settings**.
3. Go to **System → Network**.
4. Change the network method to **Static Address**.
5. Fill out the fields with the following values (leave the rest at default):
    * **IP Address**: `ROBOT_IP` (e.g. `192.168.13.20`)
    * **Subnet Mask**: `255.255.255.0`
6. Click **Apply**.

### Installing and configuring the `external_control` URCap

The `external_control` URCap is required to command the robot from the host
machine (and therefore from the Docker containers). Copy the `*.urcap` file in
`share/` to the robot's `/programs` directory over SSH (this needs SSH access to
the robot as `root`):

```bash
scp share/*.urcap root@$LEFT_ROBOT_IP:/programs
scp share/*.urcap root@$RIGHT_ROBOT_IP:/programs
```

Install the copied URCap on the robot using the Teach Pendant:

1. In the **Settings** menu, go to **System → URCaps**.
2. Click the **+** icon and select the URCap file to install (e.g.
    `external_control.urcap`).
3. Click **Restart**. This restarts the robot and loads the new URCap.

*Repeat for each URCap you wish to use.*

Now configure the URCap with the appropriate IP settings:

1. In the **Installation** tab, go to **URCaps → External Control**.
2. Fill out the fields with the following values:
    * **Host IP**: `REVERSE_IP`
    * **Custom Port**: `50002`
    * **Host Name**: `REVERSE_IP`

Finally, create a program that uses the URCap:

1. Click **New → Program** at the top of the window to open the **Program** tab.
2. Click **URCaps → External Control** in the left sidebar to add the
    `external_control` URCap to the program.

Save the program and installation with **Save → Save All**. Save the program
as `external_control.urp` and the installation as `default.installation`, so
the commander loads the correct program and installation when the rig starts.

### Enabling Remote Control Mode

**Remote Control Mode** must be enabled on the Teach Pendant in order to
command the robot through the `external_control` URCap:

1. In the **Settings** menu, go to **System → Remote Control**.
2. Click **Enable**.
3. Click **Exit** to leave the settings menu.
4. Click the **Local** button in the top-right corner of the window.
5. Select **Remote Control** from the dropdown.

*You do not need to do this for the simulator.*

## EyeLink and OptiTrack computers

The EyeLink eye tracker and the OptiTrack motion-capture system each run on
their own dedicated host computer with vendor software; the TableTop host
connects to them over the network and reads their data through ROS nodes. This
is an overview — follow the vendor documentation for the authoritative setup.

### OptiTrack (Motive)

- Run [Motive](https://docs.optitrack.com/) on the OptiTrack host PC and attach
  that PC to the [TableTop network](#the-tabletop-network).
- In Motive, enable **data streaming** (NatNet), stream the rigid body you want
  to track (the rig expects one named `ground` by default), and note the
  server's IP and command/data ports.
- Point the rig's `optitrack_driver` at the server in
  `tabletop_rig/config/optitrack.yaml` (`server_address`,
  `server_command_port`, `server_data_port`, `connection_type`,
  `rigid_body_name`). The defaults expect Motive at `192.168.13.40` on NatNet
  ports `1510`/`1511`.
- See the OptiTrack [Motive](https://docs.optitrack.com/) and
  [NatNet SDK](https://docs.optitrack.com/developer-tools/natnet-sdk)
  documentation for camera calibration and streaming details.

### EyeLink

- Run the SR Research **EyeLink Host PC** software on the EyeLink host computer.
  The rig's `eyelink` node connects to it through `pylink` (from the EyeLink
  Developers Kit).
- By SR Research convention the Host PC is reached over a dedicated Ethernet
  link, typically at `100.1.1.1` with the connecting (Display) PC at
  `100.1.1.2`; configure the relevant host interface accordingly for your setup.
- Install the **EyeLink Developers Kit**, which provides `pylink` and the
  `edf2asc` converter used by the gaze tools. See the
  [SR Research support site](https://www.sr-research.com/support/) for the
  installation packages and EyeLink documentation (account required).
