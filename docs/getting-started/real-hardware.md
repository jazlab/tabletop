# Real Hardware Setup

This page covers the one-time configuration needed to drive the **real**
hardware (the network, the UR5e robot, and the rig computers). If you only
intend to run in simulation (`robot_mode:=mock` or `ursim`), you can skip this
page entirely.

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
settings above, `nmcli`, or a netplan file under `/etc/netplan/`.

### Firewall

If you run a host firewall (e.g. `ufw`), make sure it does not block traffic on
the TableTop subnet — the UR driver and the rig computers must reach the host on
several ports. Either allow the subnet or disable the firewall while on the rig
network:

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
machine (and therefore from the Docker containers). Copy the `*.urcap` files in
`ur_robot/programs/` to the robot's `/programs` directory over SSH (this needs
SSH access to the robot as `root`):

```bash
scp ur_robot/programs/*.urcap root@$LEFT_ROBOT_IP:/programs
scp ur_robot/programs/*.urcap root@$RIGHT_ROBOT_IP:/programs
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
