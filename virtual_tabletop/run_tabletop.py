"""Entry point for running the virtual tabletop.

Running this file will launch a virtual tabletop environment with a robot arm
and a monkey arm. Your mouse controls the position of the monkey arm's palm.
"""

import time
import mujoco
import mujoco.viewer
import numpy as np


def main():
    """Run the virtual tabletop."""
    # Load the model and data
    model = mujoco.MjModel.from_xml_path("tabletop_v0.xml")
    data = mujoco.MjData(model)

    # Control visibility
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        body_mass = model.body_mass[i]
        print(f"Body ID: {i}, Name: {body_name}, Mass: {body_mass:.4f}")

    # Launch the viewer gui
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.
        viewer.cam.lookat = np.array([0., 0., 0.7])
        viewer.cam.elevation = -30.0
        viewer.cam.azimuth = -90.0

        # Iterate forever to run the simulation
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)


if __name__ == "__main__":
    main()
