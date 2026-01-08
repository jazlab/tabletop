# Musings
My thoughts/frustrations/notes as I work on the TableTop project (now
organized as troubleshooting guide!).

## Quick Reference

Before diving into troubleshooting, here are the most common commands:

```bash
# Host machine commands
source setup.bash           # Set up environment (add to .bashrc)
tt-env-gen                   # Generate .env file (run after hardware changes)
tt-compose build             # Build Docker images
tt-compose up ursim_novnc rig_novnc  # Start simulator and rig

# Inside container
tt-build                     # Build tabletop packages
tt-build --clean             # Clean rebuild
tt-launch rig robot_mode:=mock  # Launch with mock hardware
tt-launch tasks task:=foraging_ordered  # Run a task
```

## Docker

1. If you have issues building the docker container (like this one time we
    were trying to build the `rig` container and even though all the `rig`
    containers extend `rig_base`, they were all building independently,
    absolutely maddening), you should start first by stopping any running
    containers (`docker compose down` and/or `docker container stop $(docker container ls -q)`)
    and then call:
    ```bash
    ./scripts/docker_prune.sh -a
    ```
    This will remove all containers, networks, images, and build cache associated
    with the TableTop project (except the ursim image). After that, trying building
    the containers again. If that doesn't work, wipe your computer, reinstall
    ubuntu, and try again (that's what I did anyway)

## Build

1. If your computer freezes while building and you consider "quitting your life
   [recte job]" (courtesy of CursorAI), modify the build command as follows:
   ```bash
   ./scripts/build.sh --workers 1
   ```
   The freezing issues is likely due to your computer running out of memory,
   which is exacerbated by the fact it is trying to build multiple packages
   at once using multiple workers for each package.
   The `--workers 1` flag will force the build to use only one thread, which
   will prevent your computer from freezing (although it will take a stupid
   amount of time if you're building from scratch, so consider `2` at risk of
   freezing again).

## Editor

1. If you get the error:
    ```
    Unable to connect to VS Code server: Error in request.
    Error: connect ENOENT
    ```
    when trying to open another directory in the Dev Container using the CLI
    command:
    ```bash
    code <path>
    ```
    or
    ```bash
    cursor <path>
    ```
    simply open a new terminal and run the command again.

2. Don't mess with the VSCode debugger. You already figured it out bro. Stop
    forgetting you spent at least 10 hours figuring it out. No need for `gdb -ex`
    this and `build_debug` that. Just use the debugger and be happy about it.
    If you *really* need some info, check out this [video](https://www.youtube.com/watch?v=PBbEhRf8QjE&list=PL2dJBq8ig-vihvDVw-D5zAYOArTMIX0FA&index=1).

3. As a follow up to 2, if you forget why something is somewhere (or not
    somewhere), you spent at least 10 hours determining that the presence (or
    lack thereof) of that something at that somewhere was the optimal solution.
    Don't re-invent the wheel. You already invented it 5 times.


## ROS

1. If you want to visualize the URDF, you can run:
    ```bash
    ros2 launch tabletop_description view_robot.launch.py
    ```

2. If you want to run a demo with mock hardware, you can run:
    ```bash
    ros2 launch tabletop_rig rig.launch.py robot_mode:=mock use_mock_teensy:=true simulate_flic:=true
    ```

3. Don't try to use the `AIOExecutor` to run the `commander` node. You will be
    stuck trying to figure out why your little ROS services aren't responding
    and why you keep going through intermittent periods of successful ROS node
    execution and complete radio silence. It is because ROS (I forget what I
    was going to say here. Something unsavory I'm sure).

4. Additionally, for some reason you must run the rclpy executor in a separate
    thread from the asyncio thread. Something to do with the backwards way
    rclpy handles tasks. I'm not sure why this is the case, but it is.

## Flic

This gets its own section because it's a pain in the ass.

1. To disable and stop bluetooth on host machine, run:
    ```bash
    sudo systemctl disable bluetooth
    sudo systemctl stop bluetooth
    ```
    To stop it from restarting on boot, edit the `/etc/bluetooth/main.conf` file and set `AutoEnable=false`.

2. Getting the `flicd` server running in the `rig` container is a
    bitch. Forget about it, don't try to do it again. It's not worth it. Just run
    it in its own container, that's hard enough as it is.

3. Speaking of which, to connect to the `flicd` server from another container,
    you need to connect via the host machine's IP address (with respect to the
    docker internal network). E.g.
    ```bash
    ./flic_client/simple_client 172.17.0.1 [5551]
    ```
    *Port `5551` is the default and therefore optional.*

    Or
    ```bash
    ros2 run tabletop_rig <flic_connect|flic_delete> [--host 172.17.0.1] [--port 5551]
    ```
    *Both `host` and `port` are optional and default to `172.17.0.1` and `5551` respectively.*


4. The `--wait-for-hci` flag in the `flic` container command is necessary to fix the
    `Error: No HCI devices are available` error. It waits for a bluetooth device
    to be made available.

5. If your shit still isn't working (specifically you're getting a bluetooth
    device not found error, or your flic client refuses to connect to the server),
    just restart your computer. Maybe even twice.

6. "If you're ever stressed out, have a panic attack" (courtesy of CursorAI)

7. If you're actually ever stressed out, have a go at the FlicPiano. On your
    host machine, create a virtual environment and install the dependencies in
    `requirements-dev.txt` (namely the `mingus` package). Then, spin up the `flic`
    container (`docker compose up -d flic`), and run the following:
    ```bash
    ./scripts/piano.sh [--host <host>] [--port <port>] [--soundfont <soundfont>] [--key <key>] [--octave <octave>] [--scale <scale>]
    ```

8. If you want to run `flicd` on the host machine, you must first run:
    ```bash
    sudo setcap cap_net_admin=ep flic_server/flicd
    ```
    This will grant the `flicd` daemon the necessary permissions to access the
    bluetooth device.

9. Some more fun bluetooth commands to try if all else fails:
    ```bash
    sudo modprobe btusb # load the btusb module
    sudo modprobe -r btusb # unload the btusb module
    hciconfig # list bluetooth devices
    sudo hciconfig hci1 up # bring up the bluetooth device
    sudo hciconfig hci1 down # bring down the bluetooth device
    sudo hciconfig hci1 reset # reset the bluetooth device
    rfkill list # list blocked devices
    rfkill unblock bluetooth # unblock the bluetooth devices
    lsmod | grep bluetooth # list bluetooth modules
    btmgmt info # list bluetooth device info
    sudo btmgmt auto-power
    ```

10. This little bluetooth dongle sucks. $10 poorly spent.


## Teensy

1. If you change the teensy code, make sure the `executor` is initialized with
    the correct number of handles and that `colcon.meta` is updated with the
    correct number of each type of handle. Then rebuild using:
    ```bash
    tt-teensy-build
    ```
2. When in doubt, try unplugging it and plugging it back in.

3. To connect to the Teensy serial port for debugging:
    ```bash
    tt-teensy-connect
    ```

4. Make sure to regenerate your `.env` file after plugging in the Teensy so
   Docker can mount the device:
    ```bash
    tt-env-gen
    ```


## Environment and Configuration

1. If commands like `tt-build` or `tt-launch` aren't found, make sure you've
   sourced `setup.bash`:
    ```bash
    source /path/to/tabletop/setup.bash
    ```
    Better yet, add it to your `.bashrc`.

2. If Docker can't find hardware devices (FLIR cameras, Teensy, etc.),
   regenerate your `.env` file:
    ```bash
    tt-env-gen --clean
    ```
    This detects connected hardware and configures Docker mounts.

3. If you're switching between noVNC and X11 displays inside the container:
    ```bash
    tt-display-set novnc    # Use noVNC display
    tt-display-set x11      # Use host X11 display
    ```
    Then open a new terminal for changes to take effect.

4. If your `.env` file is missing variables or seems corrupted:
    ```bash
    tt-env-gen --clean      # Regenerate from .env.example
    ```

5. The `tt-compose` and `tt-docker` commands are wrappers that automatically
   set up the correct environment. Always prefer them over raw `docker` and
   `docker compose` commands.


## Gaze Estimation

1. The gaze calibration pipeline has multiple steps. Run them individually
   for debugging:
    ```bash
    gaze-preprocess -d /path/to/session --visualize  # Check preprocessing
    gaze-train -d /path/to/session                   # Train model
    gaze-visualize -d /path/to/session               # Visualize results
    ```

2. If training seems stuck or loss isn't decreasing, check that:
    - Preprocessing completed successfully (check for NaN values)
    - EyeLink and OptiTrack data are properly synchronized
    - The config file has reasonable hyperparameters

3. To convert EyeLink EDF files to CSV:
    ```bash
    python -m tabletop_py.gaze.edf recording.edf -o recording.csv
    ```
    Note: Requires the `edf2asc` utility from EyeLink Developers Kit.


## FLIR Cameras

1. Before using FLIR cameras, configure the USB filesystem:
    ```bash
    tt-usbfs-configure
    ```

2. Make sure udev rules are set up:
    ```bash
    tt-udev-configure
    ```

3. After plugging in cameras, regenerate the `.env` file:
    ```bash
    tt-env-gen
    ```


## Performance

1. For real-time robot control, disable CPU frequency scaling:
    ```bash
    tt-cpu-speed-scaling-disable
    ```

2. If builds are freezing your computer, limit parallelism:
    ```bash
    tt-build --workers 1    # Single threaded (slow but safe)
    tt-build --workers 2    # Compromise
    ```

3. To clean up disk space from old builds and logs:
    ```bash
    tt-clean-ws             # Clean tabletop build artifacts
    tt-clean-ws --all       # Clean everything including moveit2
    tt-clean-logs           # Clean log files
    ```
