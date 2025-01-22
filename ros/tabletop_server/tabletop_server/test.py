import rclpy
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def main(args=None):
    rclpy.init(args=args)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/tabletop.srdf.xacro", mappings={"name": "ur5e"}
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
        .to_dict()
    )
    moveit_py = MoveItPy(config_dict=moveit_config)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
