"""Detect the red cargo box on the robot's deck with the deck camera.

Subscribes to /deck_camera (RGB image), finds red pixels in the part of the
image where the deck is, and publishes

    /cargo_present      std_msgs/Bool, latched. True while the cargo is there.
    /cargo/debug_image  sensor_msgs/Image, the camera image with the result.

The state only changes after `confirm_frames` frames in a row agree, so one
bad frame does not flip it.

    ros2 run delivery_robot_perception cargo_detector
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (qos_profile_sensor_data, QoSDurabilityPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def red_mask(rgb):
    """Pixels that are strongly red (hue near 0, saturated, not dark)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((hue <= 8) | (hue >= 172)) & (sat >= 150) & (val >= 100)


class CargoDetector(Node):
    """Publish whether the cargo box is on the deck."""

    def __init__(self):
        super().__init__(
            'cargo_detector',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('min_pixels', 1500)
        self.declare_parameter('roi_top_row', 45)
        self.declare_parameter('confirm_frames', 3)
        self.min_pixels = self.get_parameter('min_pixels').value
        self.roi_top = self.get_parameter('roi_top_row').value
        self.confirm = self.get_parameter('confirm_frames').value

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.present_pub = self.create_publisher(Bool, '/cargo_present', latched)
        self.debug_pub = self.create_publisher(Image, '/cargo/debug_image', 1)
        self.create_subscription(Image, '/deck_camera', self.on_image,
                                 qos_profile_sensor_data)
        self.present = None            # unknown until the first decision
        self.streak = 0                # frames in a row that disagree

    def on_image(self, msg):
        """Look for the cargo in one frame."""
        rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(
            msg.height, msg.width, 3)
        mask = red_mask(rgb)
        mask[:self.roi_top] = False
        pixels = int(mask.sum())
        seen = pixels >= self.min_pixels

        if self.present is None:
            self.set_state(seen)
        elif seen != self.present:
            self.streak += 1
            if self.streak >= self.confirm:
                self.set_state(seen)
        else:
            self.streak = 0

        if self.debug_pub.get_subscription_count():
            self.publish_debug(rgb, mask, seen, pixels, msg.header)

    def set_state(self, present):
        """Publish a new cargo state."""
        self.present = present
        self.streak = 0
        self.present_pub.publish(Bool(data=present))
        self.get_logger().info(
            'Cargo on deck' if present else 'Deck is empty')

    def publish_debug(self, rgb, mask, seen, pixels, header):
        """Camera image with the red area outlined and a text label."""
        img = rgb.copy()
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > 50:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = f'CARGO ({pixels} px)' if seen else f'empty ({pixels} px)'
        cv2.putText(img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if seen else (255, 255, 0), 1, cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = img.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    """Run the detector."""
    rclpy.init(args=args)
    node = CargoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
