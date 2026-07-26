#!/usr/bin/env python3
"""Synthetic reference-workload feed for the zero-loss E2E harness (#249).

Publishes to the 14 `/dc/e2e/synth/synthNN` topics that
tools/e2e/params/e2e_params.yaml's `synthNN` StringStamped Measurement instances
subscribe to and republish verbatim (dc_measurements::StringStamped::collect() returns
the last received message unmodified), and to the camera Measurement's image topic.

Each synth topic carries a strictly monotonic per-topic `value` counter, so
tools/e2e/scripts/verify_zero_loss.py can assert both "no gaps" (a missing value is
data loss) and "no duplicates" (a repeated value is a double-delivery) independently of
timestamp precision.
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from dc_interfaces.msg import StringStamped

NUM_SYNTH_TOPICS = 14
SYNTH_TOPIC_PREFIX = "/dc/e2e/synth/synth"
CAMERA_TOPIC = "/dc/e2e/camera/image_raw"
CAMERA_WIDTH = 64
CAMERA_HEIGHT = 64


def _solid_bgr8_image(width: int, height: int, value: int) -> Image:
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = bytes([value % 256]) * (msg.step * height)
    return msg


class WorkloadGenerator(Node):
    def __init__(self) -> None:
        super().__init__("e2e_workload_generator")
        self.declare_parameter("num_synth_topics", NUM_SYNTH_TOPICS)
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("camera_period_s", 15.0)

        num_topics = self.get_parameter("num_synth_topics").value
        rate_hz = self.get_parameter("rate_hz").value
        camera_period_s = self.get_parameter("camera_period_s").value

        self._names = [f"synth{i:02d}" for i in range(num_topics)]
        self._publishers = {
            name: self.create_publisher(StringStamped, f"{SYNTH_TOPIC_PREFIX}{i:02d}", 10)
            for i, name in enumerate(self._names)
        }
        self._counters = {name: 0 for name in self._names}
        self._camera_pub = self.create_publisher(Image, CAMERA_TOPIC, 10)
        self._camera_tick = 0

        self.create_timer(1.0 / rate_hz, self._tick_synth)
        self.create_timer(camera_period_s, self._tick_camera)
        self.get_logger().info(
            f"e2e workload generator: {num_topics} synth topics @ {rate_hz} Hz, "
            f"camera every {camera_period_s}s"
        )

    def _tick_synth(self) -> None:
        now = self.get_clock().now()
        for name, pub in self._publishers.items():
            msg = StringStamped()
            msg.header.stamp = now.to_msg()
            msg.group_key = name
            msg.data = json.dumps({"value": self._counters[name], "source": name})
            pub.publish(msg)
            self._counters[name] += 1

    def _tick_camera(self) -> None:
        self._camera_pub.publish(_solid_bgr8_image(CAMERA_WIDTH, CAMERA_HEIGHT, self._camera_tick))
        self._camera_tick += 1


def main() -> None:
    rclpy.init()
    node = WorkloadGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
