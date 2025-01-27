from tabletop_server.commander import Commander


def main(tasks):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    rig_node = RigNode()
    moveit_py = MoveitpyWrapper(executor)

    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()
    commander = Commander()


if __name__ == "__main__":
    main()
