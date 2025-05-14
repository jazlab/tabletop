from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # launch_logging_config.level = logging.DEBUG
    # UR Robot Driver
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
            "robot_ip": "192.168.12.20",
            "reverse_ip": "192.168.12.10",
            "use_mock_hardware": "false",
            "controller_spawner_timeout": "120",
            "launch_rviz": "false",
            "kinematics_params_file": PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "config",
                    "ursim_calibration.yaml",
                ]
            ),
            "description_launchfile": PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            "description_file": PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "urdf",
                    "tabletop_control.urdf.xacro",
                ]
            ),
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_moveit_config"),
                        "launch",
                        "moveit.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription([ur_robot_driver, moveit])
