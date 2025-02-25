import rclpy
from tabletop_msgs.msg import TeensySensor

from tabletop_server.nodes import BaseNode


class Sensor(BaseNode):
    def __init__(self):
        super().__init__("server")

        self.sensor_sub = self.create_subscription(
            TeensySensor, "teensy_sensor", self.sensor_callback, 1000
        )
        self.sensor_data: list[TeensySensor] = []

    def sensor_callback(self, msg: TeensySensor):
        self.sensor_data.append(msg)


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()  # type: ignore
    sensor = Sensor()
    executor.add_node(sensor)

    try:
        executor.spin()
    finally:
        sensor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
