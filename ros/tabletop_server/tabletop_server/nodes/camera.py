import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image

from tabletop_server.nodes import BaseNode


class Camera(BaseNode):
    default_params = {
        "camera_topic": "camera/image",
        "camera_frame": "camera",
        "camera_fps": 30,
    }

    def __init__(self):
        super().__init__("camera_reader_writer")

        self.publisher = self.create_publisher(Image, "camera/image", 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        timer_period = 1 / (2 * self.get_parameter_wrapper("camera_fps"))
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)

    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        camera = Camera()
        executor.add_node(camera)

        try:
            executor.spin()
        finally:
            print("Shutting down camera")
            camera.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
