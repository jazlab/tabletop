import logging
import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchService,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import SetROSLogDir
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur5e",
            description="Robot name for SRDF",
        ),
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
        # Commander
        DeclareLaunchArgument(
            "commander_launch",
            default_value="true",
            choices=["true", "false"],
            description="Launch Commander?",
        ),
        DeclareLaunchArgument(
            "commander_log_level",
            default_value="INFO",
            description="Commander log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "commander_output",
            default_value="both",
            description="Commander output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # UR Driver
        DeclareLaunchArgument(
            "ur_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch UR Driver?",
        ),
        DeclareLaunchArgument(
            "ur_log_level",
            default_value="INFO",
            description="UR driver log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "ur_output",
            default_value="own_log",
            description="UR output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Teensy
        DeclareLaunchArgument(
            "teensy_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Teensy?",
        ),
        DeclareLaunchArgument(
            "teensy_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated teensy node",
        ),
        DeclareLaunchArgument(
            "teensy_log_level",
            default_value="INFO",
            description="Teensy log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "teensy_output",
            default_value="both",
            description="Teensy output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Flic
        DeclareLaunchArgument(
            "flic_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Flic?",
        ),
        DeclareLaunchArgument(
            "flic_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Simulate flic button presses",
        ),
        DeclareLaunchArgument(
            "flic_log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flic_output",
            default_value="both",
            description="Flic output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Flir
        DeclareLaunchArgument(
            "flir_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Flir?",
        ),
        DeclareLaunchArgument(
            "flir_log_level",
            default_value="INFO",
            description="Flir log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flir_output",
            default_value="both",
            description="Flir output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Optitrack
        DeclareLaunchArgument(
            "optitrack_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Optitrack?",
        ),
        DeclareLaunchArgument(
            "optitrack_log_level",
            default_value="INFO",
            description="Optitrack log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "optitrack_output",
            default_value="both",
            description="Optitrack output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Eyelink
        DeclareLaunchArgument(
            "eyelink_launch",
            default_value="true",
            choices=["true", "false"],
            description="Launch Eyelink?",
        ),
        DeclareLaunchArgument(
            "eyelink_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Force simulation of eyelink, even if Eyelink SDK is available",
        ),
        DeclareLaunchArgument(
            "eyelink_log_level",
            default_value="INFO",
            description="Eyelink log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "eyelink_output",
            default_value="both",
            description="Eyelink output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # RViz
        DeclareLaunchArgument(
            "rviz_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_log_level",
            default_value="INFO",
            description="RViz log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "rviz_output",
            default_value="own_log",
            description="RViz output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Bag
        DeclareLaunchArgument(
            "rosbag",
            default_value="false",
            choices=["true", "false"],
            description="Record rosbag?",
        ),
        DeclareLaunchArgument(
            "rosbag_output",
            default_value="both",
            description="Bag output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


#
# def add_launch_file(name: str, extra_launch_arguments: dict[str, SomeSubstitutionsType]) -> list[LaunchDescriptionEntity]:
#     launch_arguments = [
#         DeclareLaunchArgument(
#             f"{name}_log_level",
#             default_value="INFO",
#             description=f"{name.title()} log level",
#             choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
#         ),
#         DeclareLaunchArgument(
#             f"{name}_output",
#             default_value="both",
#             description="Eyelink output",
#             choices=["log", "both", "screen", "own_log"],
#         ),
#     ]
#     launch_description = GroupAction(
#         [
#             SetEnvironmentVariable(
#                 name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
#                 value=LaunchConfiguration(f"{name}_output"),
#             ),
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     [
#                         PathJoinSubstitution(
#                             [
#                                 FindPackageShare("tabletop_rig"),
#                                 "launch",
#                                 "teensy.launch.py",
#                             ]
#                         )
#                     ]
#                 ),
#                 launch_arguments={
#                     "simulate": LaunchConfiguration("teensy_simulate"),
#                     "log_level": LaunchConfiguration("teensy_log_level"),
#                     "use_sim_time": LaunchConfiguration("use_sim_time"),
#                 }.items(),
#             ),
#         ],
#         scoped=True,
#         forwarding=True,
#         condition=IfCondition(LaunchConfiguration("teensy_launch")),
#     )


def generate_launch_description():
    # Set ROS Log Directory and use_sim_time parameter for all nodes
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Create a new bag directory for the session and symlink to it
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    dirname = f"session_{timestamp}"
    session_bag_dir = os.path.join(os.environ["ROS_BAG_DIR"], dirname)
    symlink_path = os.path.join(os.environ["ROS_BAG_DIR"], "latest")

    def _create_session_bag_dir(_: LaunchContext):
        os.makedirs(session_bag_dir, exist_ok=True)
        try:
            os.remove(symlink_path)
        except FileNotFoundError:
            pass
        os.symlink(dirname, symlink_path)

    create_session_bag_dir = OpaqueFunction(
        function=_create_session_bag_dir,
        condition=IfCondition(LaunchConfiguration("rosbag")),
    )

    # Launch Files

    commander = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("commander_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "commander.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_name": LaunchConfiguration("robot_name"),
                    "robot_mode": LaunchConfiguration("robot_mode"),
                    "commander_log_level": LaunchConfiguration(
                        "commander_log_level"
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("commander_launch")),
    )

    ur = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("ur_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "ur.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_mode": LaunchConfiguration("robot_mode"),
                    "log_level": LaunchConfiguration("ur_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("ur_launch")),
    )

    teensy = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("teensy_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "teensy.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("teensy_simulate"),
                    "log_level": LaunchConfiguration("teensy_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("teensy_launch")),
    )

    flic = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("flic_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "flic.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("flic_simulate"),
                    "log_level": LaunchConfiguration("flic_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("flic_launch")),
    )

    optitrack = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("optitrack_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "optitrack.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("optitrack_simulate"),
                    "log_level": LaunchConfiguration("optitrack_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("optitrack_launch")),
    )

    flir = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("flir_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "flir.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "camera": "all",
                    "log_level": LaunchConfiguration("flir_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("flir_launch")),
    )

    rviz = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("rviz_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "rviz.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_name": LaunchConfiguration("robot_name"),
                    "log_level": LaunchConfiguration("rviz_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("rviz_launch")),
    )

    eyelink = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("eyelink_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "eyelink.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("eyelink_simulate"),
                    "initial_bag_dir": IfElseSubstitution(
                        LaunchConfiguration("rosbag"),
                        session_bag_dir,
                        "null",
                    ),
                    "log_level": LaunchConfiguration("eyelink_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("eyelink_launch")),
    )

    # Bag Recorder and Converter
    interfaces_config_file = os.path.join(
        get_package_share_directory("tabletop_rig"),
        "config",
        "rosbag_interfaces.yaml",
    )
    with open(interfaces_config_file, "r") as f:
        interfaces_config = yaml.safe_load(f)
    args = []
    if interfaces_config["all"]:
        args.append("--all")
    else:
        if "topics" in interfaces_config:
            args.extend(["--topics", *interfaces_config["topics"]])
        if "services" in interfaces_config:
            args.extend(["--services", *interfaces_config["services"]])
    rig_bag_dir = os.path.join(session_bag_dir, "rig")

    bag_recorder = ExecuteProcess(
        name="rosbag_recorder",
        cmd=["ros2", "bag", "record", "-o", rig_bag_dir, *args],
        output=LaunchConfiguration("rosbag_output"),
        condition=IfCondition(LaunchConfiguration("rosbag")),
        on_exit=[Shutdown()],
    )
    bag_converter = ExecuteProcess(
        name="bag_converter",
        cmd=[
            "ros2",
            "run",
            "tabletop_rig",
            "rosbag_to_csv",
            "-d",
            session_bag_dir,
        ],
        shell=True,
        output=LaunchConfiguration("rosbag_output"),
    )
    bag_converter_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[bag_converter], handle_once=True),
        condition=IfCondition(LaunchConfiguration("rosbag")),
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        create_session_bag_dir,
        commander,
        ur,
        teensy,
        flic,
        optitrack,
        eyelink,
        flir,
        rviz,
        bag_recorder,
        bag_converter_handler,
    ]

    return LaunchDescription(launch_actions)


def main():
    launch_config.log_dir = os.path.join(os.environ["ROS_LOG_DIR"], "rig")
    launch_config.level = logging.DEBUG
    ls = LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
