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

This node sits between the collision monitor and the bridge. It re-publishes
the latest command while the stream is fresh, and zeros from the first quiet
moment past the timeout. Which timeout applies depends on whether nav2 is
mid-goal: while follow_path is executing, stalls are transient (costmap waits,
executor congestion) and worth bridging with the last command -- with the short
window the robot stop-and-gos through every stall, approaches swing wide and
miss the 10 cm goal tolerance. Once no goal is active, quiet means the goal
ended -- an aborted goal ends the stream on a cruise velocity with no terminal
zeros, and replaying that walks the robot off-route (one full-pass verification
lost 47 stations to 15 m of cumulative creep across 169 aborted attempts), so
zeros start after a few control periods.
"""

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelWatchdog(Node):
    """Bridge dropped commands with the latest one while nav2 is mid-goal; zero on quiet."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_gz")
        self.declare_parameter("status_topic", "/follow_path/_action/status")
        self.declare_parameter("publish_rate", 20.0)
        # A few control periods: an aborted goal's cruise command must not
        # outlive the abort by more than this.
        self.declare_parameter("idle_timeout_seconds", 0.15)
        # Long enough to bridge a mid-goal stall (the controller waiting on a
        # costmap update, an executor hiccup), far shorter than the sim pass
        # between two goals.
        self.declare_parameter("active_timeout_seconds", 2.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        status_topic = self.get_parameter("status_topic").value
        rate = self.get_parameter("publish_rate").value
        self.active_timeout_sec = self.get_parameter("active_timeout_seconds").value
        self.idle_timeout_sec = self.get_parameter("idle_timeout_seconds").value

        self.last_cmd = None
        self.last_received = self.get_clock().now()
        self.nav_active = False

        # Wall clock, like the bridge this feeds: the liveness of the command
        # chain is a wall-clock fact, and /clock itself dies when the bridge does.
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(TwistStamped, input_topic, self.on_cmd_vel, qos)
        self.pub = self.create_publisher(TwistStamped, output_topic, qos)
        # The action server republishes the whole status array on every
        # transition, so the latest message alone says whether a goal is live.
        self.status_sub = self.create_subscription(
            GoalStatusArray, status_topic, self.on_goal_status, qos
        )
        self.timer = self.create_timer(1.0 / rate, self.restream)

    def on_goal_status(self, msg: GoalStatusArray):
        # ACCEPTED/EXECUTING/CANCELING: a goal the controller is still serving.
        self.nav_active = bool(msg.status_list) and msg.status_list[-1].status <= 3

    def on_cmd_vel(self, msg: TwistStamped):
        self.last_cmd = msg
        self.last_received = self.get_clock().now()

    def restream(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if self.last_cmd is not None:
            quiet = (self.get_clock().now() - self.last_received).nanoseconds / 1e9
            timeout = self.active_timeout_sec if self.nav_active else self.idle_timeout_sec
            if quiet <= timeout:
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
