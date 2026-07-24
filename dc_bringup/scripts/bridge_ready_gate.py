#!/usr/bin/env python3
"""Readiness gate for the DC bringup startup sequence (ADR-0006).

Polls the Bridge's readiness service (``std_srvs/Trigger``, default
``dc_bridge/ready``) until it answers ``success=True``, then exits 0.
``dc_bringup.launch.py`` starts the lifecycle manager — and therefore the
activation of the collection nodes — only on this process's successful
exit, so no Record can be emitted before the Bridge and the Vector
shipper it supervises are able to accept it. Exits 1 if the deadline
passes first, which the launch file turns into a full shutdown.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node("bridge_ready_gate")
    service = node.declare_parameter("service", "dc_bridge/ready").value
    timeout_s = node.declare_parameter("timeout_s", 120.0).value
    poll_interval_s = node.declare_parameter("poll_interval_s", 0.5).value

    client = node.create_client(Trigger, service)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if not client.wait_for_service(timeout_sec=min(poll_interval_s, remaining)):
            continue
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=min(poll_interval_s * 2, remaining)
        )
        if future.done():
            response = future.result()
            if response is not None and response.success:
                node.get_logger().info(f"Bridge is ready: {response.message}")
                node.destroy_node()
                rclpy.shutdown()
                return 0
        else:
            client.remove_pending_request(future)
        time.sleep(poll_interval_s)

    node.get_logger().error(f"Bridge did not report ready on '{service}' within {timeout_s}s")
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # Launch shutdown (SIGINT) while still polling: exit quietly.
        sys.exit(130)
