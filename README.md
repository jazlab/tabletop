# TableTop

## Usage

Set up by following these steps:
1. Clone this repo locally.
2. Create a virtual environment to manage dependencies.
3. Install requirements by navigating the this directory and running
   `pip install -r requirements.txt`.

To begin, run the demo script `$ python run_demo.py`. This will launch a gui
with "start", "pause", and "stop" buttons that uses mock I/O modules. You can
press "start" and watch it play some trials.

## Workflow

Before you make a git commit, make sure your changes satisfy the pre-commits by
running `$ pre-commit run --all-files` and addressing any issues raised. In many
cases the pre-commits will automatically modify files, so be sure to add the new
changes to your stages files (an re-run the pre-commits to check for remaining
issues).

Notes:

To ssh in to vnc server on docker:
$ ssh -L 6080:192.168.13.10:6080 tabletop.valmikikothare.com -N
$ http://localhost:6080/vnc.html?host=localhost&port=6080