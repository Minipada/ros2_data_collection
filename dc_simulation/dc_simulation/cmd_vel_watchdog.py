#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Keep the simulator's command stream alive so silence can't latch a twist.

Nav2's command chain (controller_server -> velocity_smoother -> collision_monitor)
ends every motion episode with a few zero commands and then falls permanently
silent: velocity_smoother stops publishing once it has reached zero, and
collision_monitor ceases after stop_pub_timeout. gz-sim's DiffDrive has no
command timeout -- it applies the last twist it received forever. On a loaded
runner DDS drops messages (GitHub's rmem_max is 1 MiB; CycloneDDS logs that it
wants 8), and when one of those few terminal zeros is dropped the robot keeps
its cruise velocity: it has driven out of its aisle and wedged against the
warehouse boundary for the rest of a daily sim run, skipping every remaining
waypoint while the pass still reports success.

This node sits between the collision monitor and the bridge. It forwards every
command and, on a timer, re-publishes the latest one -- or a zero once the
stream has been quiet for longer than --timeout-seconds. The gz side therefore
sees a continuous stream: a dropped message costs one cycle instead of latching
a twist, and a genuinely dead chain stops the robot instead of running it away.
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelWatchdog(Node):
    """Re-stream cmd_vel into the bridge at a fixed rate."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_gz")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("timeout_seconds", 2.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        rate = self.get_parameter("publish_rate").value
        timeout = self.get_parameter("timeout_seconds").value

        self.last_cmd = None
        self.last_received = self.get_clock().now()

        # Wall clock, like the bridge this feeds: the liveness of the command
        # chain is a wall-clock fact, and /clock itself dies when the bridge does.
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(TwistStamped, input_topic, self.on_cmd_vel, qos)
        self.pub = self.create_publisher(TwistStamped, output_topic, qos)
        self.timer = self.create_timer(1.0 / rate, self.restream)
        self.timeout_sec = timeout

    def on_cmd_vel(self, msg: TwistStamped):
        self.last_cmd = msg
        self.last_received = self.get_clock().now()

    def restream(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if self.last_cmd is not None:
            quiet = (self.get_clock().now() - self.last_received).nanoseconds / 1e9
            if quiet <= self.timeout_sec:
                out.twist = self.last_cmd.twist
        self.pub.publish(out)


def main(args=None):
    """Start the watchdog."""
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
