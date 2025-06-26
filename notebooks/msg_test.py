# %%
import rclpy
from rclpy.node import Node
from tabletop_interfaces.msg import TeensySensor


# %%
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.sensor_sub = self.create_subscription(
            TeensySensor,
            "/tabletop/teensy_sensor",
            self.sensor_callback,
            10,
        )

    def sensor_callback(self, msg: TeensySensor):
        print(msg)


# %%
rclpy.init()
node = MyNode()
rclpy.spin(node)
rclpy.shutdown()

# %%
