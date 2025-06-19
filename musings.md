# Musings
My thoughts/frustrations/notes as I work on the TableTop project (now
organized as troubleshooting guide!).

## Docker

1. If you have issues building the docker container (like this one time we
    were trying to build the `server` container and even though all the `server`
    containers extend `server_base`, they were all building independently,
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
    ros2 launch tabletop_server server.launch.py robot_mode:=mock use_mock_teensy:=true simulate_flic:=true
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

2. Getting the `flicd` server running in the `tabletop_server` container is a
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
    ros2 run tabletop_server <flic_connect|flic_delete> [--host 172.17.0.1] [--port 5551]
    ```
    *Both `host` and `port` are optional and default to `172.17.0.1` and `5551` respectively.*


4. The `-w` flag in the `flic` container command is necessary to fix the
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
    sudo modprobe bluetooth # load the bluetooth module
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
    ./scripts/teensy_build.sh
    ```
2. When in doubt, try unplugging it and plugging it back in.
