# Real Hardware Setup

This page covers the one-time configuration needed to drive the **real**
hardware (the UR5e robot in particular). If you only intend to run in
simulation (`robot_mode:=mock` or `ursim`), you can skip this page entirely.

Before starting, make sure you have completed the [Setup](setup.md) steps
(clone, `source setup.bash`, `tt-env-gen`) and any relevant
[optional host setup](setup.md#optional-host-setup) (udev rules, USB buffer
size, CPU scaling).

!!! note "Teensy firmware"
    Building and flashing the Teensy micro-controller firmware is covered in
    [Setup → Firmware](setup.md#firmware). The rest of this page is about the
    UR5e robot.

## UR5e robot

### Creating the local network

The host, the robot control boxes, and the other rig computers communicate
over a dedicated wired network. To bring up the host side of that network,
run:

```bash
./scripts/configure/robot-network.sh
```

This adds the host's reverse IP (`REVERSE_IP`) to the first detected ethernet
interface, using a `/24` subnet derived from the robot IP. The relevant
addresses — `ROBOT_IP`, `REVERSE_IP`, and their per-arm/sim variants — are
defined in `setup.bash` (not in a separate env file); `robot-network.sh`
sources `setup.bash` to read them. These two addresses are referenced
throughout the rest of this page.

### Setting the robot IP address

With the network created, set a static IP on the robot via the Teach Pendant:

1. Click the "hamburger" (menu) icon in the top-right corner of the window.
2. Click **Settings**.
3. Go to **System → Network**.
4. Change the network method to **Static Address**.
5. Fill out the fields with the following values (leave the rest at default):
    * **IP Address**: `ROBOT_IP`
    * **Subnet Mask**: `255.255.255.0`
6. Click **Apply**.

### Installing and configuring the `external_control` URCap

The `external_control` URCap is required to command the robot from the host
machine (and therefore from the Docker containers). To copy it to the robot,
run:

```bash
./scripts/configure/scp-urcaps.sh
```

This copies any `*.urcap` files in `ur_robot/programs/` to the robot's
`/programs` directory over SSH (it needs SSH access to the robot as `root`).

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
