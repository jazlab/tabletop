# Virtual TableTop Model

## Introduction

This directory has code to create a virtual model of the TableTop environment in
Mujoco. It is currently in draft mode and there are several things to do going
forward:
1. Flesh out the rig model with all the parts of the rig.
2. Allow this virtual tabletop to load logs from a session and simulate the
robot, rig, and monkey based on those logs.
3. Adapt the monkey arm model to the geometry of our monkeys.

## Usage

Set up by following these steps:
1. Clone this repo locally.
2. Create a virtual environment to manage dependencies.
3. Install requirements by navigating the this directory and running
   `$ pip install -r requirements.txt`.
4. Run the virtual tabletop model with `$ python run_tabletop.py`.
