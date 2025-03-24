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
