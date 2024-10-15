"""Demo script.

Run this to demo the TableTop app using mock I/O modules.
"""

from tabletop import app, influx, io, tasks, trial_generators


def main():
    """Main function."""
    # Create trial generator
    trial_generator = trial_generators.MockBlockStructuredAffordance(
        affordance_to_object_ids={"twist": [0, 1, 2], "pull": [3, 4, 5, 6]},
        trials_per_block=10,
    )

    # Create I/O modules
    robot = io.MockRobot()
    reward_button = io.MockRewardButton()
    juice_tube = io.MockJuiceTube()
    hand_fixation = io.MockHandFixation()
    smartglass = io.MockSmartGlass()
    arm_door = io.MockArmDoor()
    eye_tracker = io.MockEyelink()

    # Create task
    task = tasks.ButtonSearch(
        trial_generator=trial_generator,
        robot=robot,
        reward_button=reward_button,
        juice_tube=juice_tube,
        hand_fixation=hand_fixation,
        smartglass=smartglass,
        arm_door=arm_door,
    )

    # Create InfluxDB client
    influx_client = influx.Influx(tags={"subject": "nick"})

    # Create TableTop app
    io_modules = [
        robot,
        reward_button,
        juice_tube,
        hand_fixation,
        smartglass,
        arm_door,
        eye_tracker,
    ]
    app.TableTopApp(
        task=task,
        io_modules=io_modules,
        influx_client=influx_client,
    )


if __name__ == "__main__":
    main()
