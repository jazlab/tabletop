import cv2
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.publisher = self.create_publisher(Image, "camera/image", 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.create_timer(0.1, self.timer_callback)
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri="camera_data", storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        self.bag_writer.open(storage_options, converter_options)
        topic_info = rosbag2_py.TopicMetadata(
            name="camera/image",
            type="sensor_msgs/msg/Image",
            serialization_format="cdr",
        )
        self.bag_writer.create_topic(topic_info)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)

    def __del__(self):
        self.cap.release()
        self.bag_writer.close()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
