import rclpy
from tabletop_msgs.msg import TeensySensor

from tabletop_server.base import BaseNode


class SensorReader(BaseNode):
    def __init__(self):
        super().__init__("server")

        self.sensor_sub = self.create_subscription(
            TeensySensor, "teensy_sensor", self.sensor_callback, 1000
        )

    def sensor_callback(self, msg: TeensySensor):
        self.sensor_data.append(msg)


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_reader = SensorReader()
    executor.add_node(sensor_reader)

    try:
        executor.spin()
    finally:
        sensor_reader.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
