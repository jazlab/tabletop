"""Demo script.

Run this to demo the TableTop app using mock I/O modules.
"""

import influx as influx_lib
import io_modules as io_modules_lib
import tasks as tasks_module
import trial_generators as trial_generators_module

import tabletop_app


def main():
    """Main function."""

    # Create I/O modules
    robot = io_modules_lib.MockRobot()
    reward_button = io_modules_lib.MockRewardButton()
    juice_tube = io_modules_lib.MockJuiceTube()
    hand_fixation = io_modules_lib.MockHandFixation()
    smartglass = io_modules_lib.MockSmartGlass()
    arm_door = io_modules_lib.MockArmDoor()
    eye_tracker = io_modules_lib.MockEyelink()

    # Create task
    task = tasks_module.ButtonSearch(
        trial_generator=trial_generators_module.MockTrialGenerator(),
        robot=robot,
        reward_button=reward_button,
        juice_tube=juice_tube,
        hand_fixation=hand_fixation,
        smartglass=smartglass,
        arm_door=arm_door,
    )
    
    # Create InfluxDB client
    influx_client = influx_lib.Influx(tags={'subject': 'nick'})

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
    tabletop_app.TableTopApp(
        task=task,
        io_modules=io_modules,
        influx_client=influx_client,
    )


if __name__ == "__main__":
    main()
