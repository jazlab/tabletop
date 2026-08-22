"""One bounded, reduced-resolution camera feed for always-on monitoring."""

from time import monotonic_ns

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CAMERA_NAMES = (
    "left_front_top_cam",
    "right_front_top_cam",
    "left_back_top_cam",
    "right_back_top_cam",
    "left_bottom_cam",
    "right_bottom_cam",
)


class CameraPreview(Node):
    """Decode one camera in its own DDS participant at a bounded rate."""

    def __init__(self) -> None:
        super().__init__("camera_preview")
        camera_name = str(self.declare_parameter("camera_name", "").value)
        rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        self._max_width = int(self.declare_parameter("max_width", 640).value)
        self._publish_color = bool(
            self.declare_parameter("publish_color", False).value
        )
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"Unknown camera_name: {camera_name}")
        if rate_hz <= 0.0 or self._max_width <= 0:
            raise ValueError("rate_hz and max_width must be greater than zero")

        self._camera_name = camera_name
        self._period_ns = int(1_000_000_000 / rate_hz)
        self._last_publish_ns = None
        self._publisher = self.create_publisher(
            Image,
            f"/cam_preview/{camera_name}/image_raw",
            qos_profile_sensor_data,
        )
        self._color_publisher = None
        if self._publish_color:
            self._color_publisher = self.create_publisher(
                Image,
                f"/cam_preview/{camera_name}/image_color",
                qos_profile_sensor_data,
            )
        self._subscription = self.create_subscription(
            CompressedImage,
            f"/cam_sync/{camera_name}/image_raw/compressed",
            self._receive,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Previewing {camera_name} at {rate_hz:g} Hz, maximum width "
            f"{self._max_width}, color={'on' if self._publish_color else 'off'}"
        )

    def _resize(self, frame: np.ndarray) -> np.ndarray:
        if frame.shape[1] <= self._max_width:
            return frame
        scale = self._max_width / frame.shape[1]
        return cv2.resize(
            frame,
            (self._max_width, round(frame.shape[0] * scale)),
            interpolation=cv2.INTER_AREA,
        )

    @staticmethod
    def _image_message(frame: np.ndarray, header) -> Image:
        frame = np.ascontiguousarray(frame)
        preview = Image()
        preview.header = header
        preview.height, preview.width = frame.shape[:2]
        if frame.ndim == 2:
            preview.encoding = "mono8"
            channels = 1
        elif frame.shape[2] == 3:
            preview.encoding = "bgr8"
            channels = 3
        elif frame.shape[2] == 4:
            preview.encoding = "bgra8"
            channels = 4
        else:
            raise ValueError(f"Unsupported decoded frame shape {frame.shape}")
        preview.step = preview.width * channels
        preview.data = frame.tobytes()
        return preview

    def _receive(self, message: CompressedImage) -> None:
        now_ns = monotonic_ns()
        if (
            self._last_publish_ns is not None
            and now_ns - self._last_publish_ns < self._period_ns
        ):
            return

        frame = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if frame is None:
            self.get_logger().error(
                f"Could not decode a frame from {self._camera_name}",
                throttle_duration_sec=5.0,
            )
            return
        try:
            preview = self._image_message(self._resize(frame), message.header)
            if self._color_publisher is not None:
                if frame.ndim == 2:
                    # The synchronized driver publishes BayerRG8. Demosaic at
                    # source resolution before scaling so the Bayer pattern is
                    # not corrupted by interpolation.
                    # The compressed Bayer payload decodes with red and blue
                    # sites opposite OpenCV's RG conversion convention.
                    color_frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                else:
                    color_frame = frame
                color_preview = self._image_message(
                    self._resize(color_frame), message.header
                )
        except (ValueError, cv2.error) as error:
            self.get_logger().error(
                f"Could not prepare a preview from {self._camera_name}: {error}",
                throttle_duration_sec=5.0,
            )
            return
        self._publisher.publish(preview)
        if self._color_publisher is not None:
            self._color_publisher.publish(color_preview)
        self._last_publish_ns = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
