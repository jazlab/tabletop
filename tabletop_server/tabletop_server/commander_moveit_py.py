import rclpy
from geometry_msgs.msg import PoseStamped
from moveit import MoveItPy
from rclpy.node import Node


class Commander(Node):
    def __init__(self):
        super().__init__(
            "commander", automatically_declare_parameters_from_overrides=True
        )

        # Initialize MoveItPy
        self.moveit_py = MoveItPy("moveit_py")

        # Get the planning component for your robot
        self.planning_component = self.moveit_py.get_planning_component(
            "ur_manipulator"
        )

        self.trajectory_execution_manager = (
            self.moveit_py.get_trajactory_execution_manager()
        )

        self.goals = []

        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = 0.5
        goal.pose.position.y = 0.5
        goal.pose.position.z = 0.5
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goals.append(goal)

        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = -0.5
        goal.pose.position.y = -0.5
        goal.pose.position.z = -0.5
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = -1.0

        self.goals.append(goal)

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        # self.timer = self.create_timer(self.timer_sec, self.timer_callback)
        self.i = 0

        self.get_logger().info("Commander initialized")

        self.plan_and_execute()

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.get_logger().info("Planning trajectory")

        # Set start state
        self.planning_component.set_start_state_to_current_state()

        # Set goal state
        goal = self.goals[self.i]

        self.planning_component.set_goal_state(
            pose_stamped=goal, pose_link="tool0"
        )

        # Plan
        if multi_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = self.planning_component.plan()

        # Execute plan
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.trajectory_execution_manager.push(robot_trajectory)
            self.trajectory_execution_manager.execute(
                self.execution_callback, blocking=False
            )
        else:
            self.get_logger.error("Planning failed, moving on to next goal...")
            self.i += 1 % len(self.goals)
            self.plan_and_execute()

    def execution_callback(self):
        self.get_logger().info("Execution finished")

        status = self.trajectory_execution_manager.get_last_execution_status()
        if status == "SUCCEEDED":
            self.get_logger().info("Execution succeeded")
        else:
            self.get_logger().warn("Execution failed with status: %s" % status)

        self.get_logger().info("Moving on to next goal...")

        self.i += 1 % len(self.goals)
        self.plan_and_execute()


def main(args=None):
    rclpy.init(args=args)
    # node = Node(
    #     "commander",
    #     automatically_declare_parameters_from_overrides=True,
    # )
    # params = node.get_parameters_by_prefix("")
    # node.get_logger().info("type(params): %s" % type(params))
    # node.get_logger().info("Commander started")

    # for name, param in params.items():
    #     node.get_logger().info("type(param): %s" % type(param))
    #     node.get_logger().info(f"{name}: {param.name}: {param.value}")

    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.spin()
    # commander = Commander(moveit_py)
    commander.destroy_node()
    rclpy.shutdown()
