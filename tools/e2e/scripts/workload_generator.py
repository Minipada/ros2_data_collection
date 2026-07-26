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
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image

from dc_interfaces.msg import StringStamped

# How long to wait for the DC pipeline to subscribe before giving up and publishing
# anyway. The harness's own <10s startup gate (run.sh) trips first if the stack never
# comes up, so this is only a safety net against hanging, not a real timing knob.
PIPELINE_READY_TIMEOUT_S = 30.0

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
        # NB: not `self._publishers` — that name is rclpy.Node's own internal publisher
        # list, and shadowing it with a dict breaks the next create_publisher() call.
        self._synth_pubs = {
            name: self.create_publisher(StringStamped, f"{SYNTH_TOPIC_PREFIX}{i:02d}", 10)
            for i, name in enumerate(self._names)
        }
        self._counters = {name: 0 for name in self._names}
        self._camera_pub = self.create_publisher(Image, CAMERA_TOPIC, 10)
        self._camera_tick = 0
        self._rate_hz = rate_hz
        self._camera_period_s = camera_period_s

    def wait_for_pipeline(self, timeout_s: float = PIPELINE_READY_TIMEOUT_S) -> None:
        """Block until the DC pipeline has subscribed to every topic we publish.

        measurement_server only subscribes once the lifecycle manager activates it,
        which happens *after* dc_bridge's readiness gate (ADR-0006). Publishing the
        first values before then drops them (no subscriber yet) and shows up as data
        loss (missing value 0). Waiting for a subscriber on each publisher — the exact
        "pipeline is listening" signal, checked at the source rather than by parsing
        `ros2 topic info` — guarantees value 0 is actually delivered.
        """
        pubs = list(self._synth_pubs.values()) + [self._camera_pub]
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while rclpy.ok():
            if all(pub.get_subscription_count() > 0 for pub in pubs):
                self.get_logger().info("DC pipeline subscribed to all workload topics; starting")
                return
            if self.get_clock().now() >= deadline:
                self.get_logger().warning(
                    f"timed out after {timeout_s}s waiting for the DC pipeline to subscribe; "
                    "starting anyway (the harness startup gate will catch a dead pipeline)"
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def start(self) -> None:
        """Begin publishing. Call only after wait_for_pipeline() so value 0 is delivered."""
        self.create_timer(1.0 / self._rate_hz, self._tick_synth)
        self.create_timer(self._camera_period_s, self._tick_camera)
        self.get_logger().info(
            f"e2e workload generator: {len(self._names)} synth topics @ {self._rate_hz} Hz, "
            f"camera every {self._camera_period_s}s"
        )

    def _tick_synth(self) -> None:
        now = self.get_clock().now()
        for name, pub in self._synth_pubs.items():
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
        node.wait_for_pipeline()
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
