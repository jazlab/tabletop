"""Entry point for running the virtual tabletop.

Running this file will launch a virtual tabletop environment with a robot arm
and a monkey arm. Your mouse controls the position of the monkey arm's palm.
"""

import inverse_kinematics
import mujoco
import numpy as np
import pyautogui


class RobotPolicy:
    """Robot policy class to control the motion of the robot."""

    def __init__(
        self, model, data, movement_duration=1000, trial_duration=2000
    ):
        """Constructor.

        Args:
            model: Mujoco model object.
            data: Mujoco data object associated with the model.
            movement_duration: Number of steps for the robot to move from the
                current position to the start position and from the end position
                to the start position.
            trial_duration: Number of steps for the robot to stay at the end
                position.
        """
        self._model = model
        self._data = data
        self._movement_duration = movement_duration
        self._trial_duration = trial_duration
        self._site_name = "attachment_site"
        self._site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, self._site_name
        )
        self._traj = []

    def _sample_start_pos(self):
        pos_x = np.random.uniform(0.1, 0.2)
        pos_y = np.random.uniform(1.1, 1.2)
        pos_z = np.random.uniform(0.1, 0.2)
        return np.array([pos_x, pos_y, pos_z])

    def _sample_end_pos(self):
        pos_x = np.random.uniform(0.3, 0.2)
        pos_y = np.random.uniform(0.2, 0.3)
        pos_z = np.random.uniform(0.4, 0.5)
        return np.array([pos_x, pos_y, pos_z])

    def _sample_traj(self):
        current_pos = self._data.site_xpos[self._site_id]
        start_pos = self._sample_start_pos()
        end_pos = self._sample_end_pos()
        traj = []

        # Move from current_pos to start_pos
        for x in np.linspace(1, 0, self._movement_duration):
            traj.append(x * current_pos + (1 - x) * start_pos)

        # Move from start_pos to end_pos
        for x in np.linspace(1, 0, self._movement_duration):
            traj.append(x * start_pos + (1 - x) * end_pos)

        # Trial
        for _ in range(self._trial_duration):
            traj.append(end_pos)

        # Move from end_pos to start_pos
        for x in np.linspace(1, 0, self._movement_duration):
            traj.append(x * end_pos + (1 - x) * start_pos)

        return traj

    def __call__(self):
        """Take one step of robot control."""
        if len(self._traj) == 0:
            # Sample a new trajectory
            self._traj = self._sample_traj()
        target_pos = self._traj.pop(0)
        _ = inverse_kinematics.qpos_from_site_pose(
            model=self._model,
            data=self._data,
            site_name="attachment_site",
            target_pos=target_pos,
        )


class ArmPolicy:
    """Arm policy class to control the motion of the arm via the mouse."""

    def __init__(self, model, data):
        """Constructor.

        Args:
            model: Mujoco model object.
            data: Mujoco data object associated with the model.
        """
        self._model = model
        self._data = data
        self._site_name = "ext_digitorum-P4"
        self._site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, self._site_name
        )

    def __call__(self):
        """Take one step of arm control."""
        x, y = pyautogui.position()
        # Coordinate transformation from screen to Mujoco
        velocity_x = (x - 800) / 2000
        velocity_y = (y - 500) / 2000
        _ = inverse_kinematics.qpos_from_site_pose(
            model=self._model,
            data=self._data,
            site_name=self._site_name,
            target_pos=[velocity_x, -velocity_y, 0.5 - velocity_y],
            max_steps=20,
        )


def main():
    """Run the virtual tabletop."""
    # Load the model, data, and control policies
    model = mujoco.MjModel.from_xml_path("tabletop_v0.xml")
    data = mujoco.MjData(model)
    robot_policy = RobotPolicy(model, data)
    arm_policy = ArmPolicy(model, data)

    # Launch the viewer gui
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.5
        viewer.cam.lookat = np.array([0.0, 0.4, 0.4])
        viewer.cam.elevation = -30.0

        # Iterate forever to run the simulation
        while viewer.is_running():
            robot_policy()
            arm_policy()
            viewer.cam.azimuth += 0.02  # rotate camera
            mujoco.mj_forward(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
