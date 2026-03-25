from dataclasses import dataclass
import threading
import time

import cv2
import msgpack
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import tyro
import zmq

from decoupled_wbc.control.sensor.sensor_server import CameraMountPosition, ImageMessageSchema
from decoupled_wbc.control.utils.cv_bridge import CvBridge
from decoupled_wbc.data.constants import RS_VIEW_CAMERA_HEIGHT, RS_VIEW_CAMERA_WIDTH


@dataclass
class ROS2ZMQCameraBridgeConfig:
    topic: str = "/camera/camera/color/image_raw"
    """ROS2 image topic to subscribe to."""

    compressed: bool = False
    """Subscribe to sensor_msgs/CompressedImage instead of sensor_msgs/Image."""

    port: int = 5555
    """ZMQ PUB port used by the existing camera viewer."""

    fps: float = 20.0
    """Maximum publish rate for the ZMQ bridge."""

    camera_name: str = CameraMountPosition.EGO_VIEW.value
    """Image key used in the ZMQ payload."""

    output_width: int = RS_VIEW_CAMERA_WIDTH
    """Output image width for the ZMQ stream."""

    output_height: int = RS_VIEW_CAMERA_HEIGHT
    """Output image height for the ZMQ stream."""

    node_name: str = "ros2_zmq_camera_bridge"
    """ROS2 node name."""


class ROS2ZMQCameraBridge(Node):
    def __init__(self, config: ROS2ZMQCameraBridgeConfig):
        super().__init__(config.node_name)
        self.config = config
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_timestamp = None
        self.frames_received = 0
        self.frames_published = 0
        self.messages_dropped = 0

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.setsockopt(zmq.SNDHWM, 20)
        self.zmq_socket.setsockopt(zmq.LINGER, 0)
        self.zmq_socket.bind(f"tcp://*:{config.port}")

        if config.compressed:
            self.subscription = self.create_subscription(
                CompressedImage, config.topic, self._compressed_callback, 1
            )
        else:
            self.subscription = self.create_subscription(Image, config.topic, self._raw_callback, 1)

        self.get_logger().info(
            f"Bridge started: topic={config.topic} compressed={config.compressed} "
            f"-> tcp://*:{config.port} as '{config.camera_name}' "
            f"at {config.output_width}x{config.output_height}"
        )

    def _resize_image(self, image):
        target_size = (self.config.output_width, self.config.output_height)
        if image.shape[1] == self.config.output_width and image.shape[0] == self.config.output_height:
            return image
        return cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)

    def _raw_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image = self._resize_image(image)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.latest_image = image
            self.latest_timestamp = timestamp
            self.frames_received += 1

    def _compressed_callback(self, msg: CompressedImage):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image = self._resize_image(image)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.latest_image = image
            self.latest_timestamp = timestamp
            self.frames_received += 1

    def _publish(self, payload: dict):
        try:
            packed = msgpack.packb(payload, use_bin_type=True)
            self.zmq_socket.send(packed, flags=zmq.NOBLOCK)
        except zmq.Again:
            self.messages_dropped += 1

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        interval = 1.0 / self.config.fps
        last_log_time = time.monotonic()

        try:
            while rclpy.ok():
                with self.lock:
                    image = None if self.latest_image is None else self.latest_image.copy()
                    timestamp = self.latest_timestamp

                if image is not None and timestamp is not None:
                    payload = ImageMessageSchema(
                        timestamps={self.config.camera_name: timestamp},
                        images={self.config.camera_name: image},
                    ).serialize()
                    self._publish(payload)
                    self.frames_published += 1

                now = time.monotonic()
                if now - last_log_time >= 5.0:
                    self.get_logger().info(
                        f"frames_received={self.frames_received} "
                        f"frames_published={self.frames_published} "
                        f"messages_dropped={self.messages_dropped}"
                    )
                    last_log_time = now

                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        finally:
            self.zmq_socket.close()
            self.zmq_context.term()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    config = tyro.cli(ROS2ZMQCameraBridgeConfig)
    rclpy.init(args=None)
    bridge = ROS2ZMQCameraBridge(config)
    bridge.run()


if __name__ == "__main__":
    main()
