from tabletop_server.commander import Commander

import rclpy

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    commander = Commander(executor)

    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
