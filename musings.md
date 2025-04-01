# Musings
My thoughts/frustrations/notes as I work on the TableTop project.

1. If you get the error:
```
Unable to connect to VS Code server: Error in request.
Error: connect ENOENT
```
when trying to open another directory in the dev container using the cli
command:
```bash
code <path>
```
or
```bash
cursor <path>
```
simply open a new terminal and run the command again.

2. If you want to visualize the urdf, you can run:
```bash
ros2 launch tabletop_description view_robot.launch.py
```

3. If you want to run the demo, you can run:
```bash
ros2 launch tabletop_tasks run_tasks.launch.py
```

4. If you change the teensy code, make sure the `executor` is initialized with
the correct number of handles and that `colcon.meta` is updated with the
correct number of each type of handle. Then rebuild using:
```bash
./scripts/teensy_build.sh
```
5. To disable and stop bluetooth on host machine, run:
```bash
sudo systemctl disable bluetooth
sudo systemctl stop bluetooth
```
To stop it from restarting on boot, edit the `/etc/bluetooth/main.conf` file and set `AutoEnable=false`.

6. Don't mess with the VSCode debugger. You already figured it out bro. Stop
forgetting you spent fucking hours figuring it out. No need for `gdb -ex`
this and `build_debug` that. Just use the debugger and be happy about it.
If you *really* need some info, check out this [video](https://www.youtube.com/watch?v=PBbEhRf8QjE&list=PL2dJBq8ig-vihvDVw-D5zAYOArTMIX0FA&index=1).

7. As a follow up to 6, if you forget why something is somewhere (or not
somewhere), you spent fucking hours determining that the presence (or lack
thereof) of that something at that somewhere was the optimal solution. Don't
re-invent the wheel. You already invented it 5 times.

8. Getting the `flicd` server running in the `tabletop_server` container is a
bitch. Forget about it, don't try to do it again. It's not worth it. Just run
it in its own container, that's hard enough as it is.

9. Speaking of which, to connect to the `flicd` server from another container,
you need to connect via the host machine's IP address (with respect to the
docker internal network). E.g.
```bash
./flic_client/simple_client 172.17.0.1 [5551]
```
(port `5551` is the default and therefore optional).
