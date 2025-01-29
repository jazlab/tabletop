import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from tabletop_server.base import BaseNode


class CameraReaderWriter(BaseNode):
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
        self.create_timer(
            1 / self.get_parameter("camera_fps").value, self.timer_callback
        )

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
        executor = rclpy.executors.MultiThreadedExecutor()
        camera_reader_writer = CameraReaderWriter(executor)
        executor.add_node(camera_reader_writer)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            camera_reader_writer.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
