from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "ur_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": "192.168.13.10",
            "use_mock_hardware": "false",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    ur_robot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "test_joint_trajectory_controller.launch.py",
                    ]
                )
            ]
        ),
    )
    set_check_starting_point = SetParameter(
        name="/publisher_joint_trajectory_controller", value="true"
    )

    return LaunchDescription(
        [ur_robot_driver, ur_robot_controller, set_check_starting_point]
    )
