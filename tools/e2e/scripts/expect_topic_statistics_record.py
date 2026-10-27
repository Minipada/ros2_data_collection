#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
#
# SPDX-License-Identifier: MPL-2.0

"""The pass/fail gate of the TopicStatistics E2E smoke (tools/e2e/scripts/run_pulse.sh).

Exits 0 only when a Record carrying both `average` and `sample_count` arrives on the
Measurement's topic_output before the deadline — the proof that a live pulse_bridge
MetricsMessage was flattened into a Record. Anything short of that is a non-zero exit.
"""

import argparse
import json
import sys
import time

import rclpy

from dc_interfaces.msg import StringStamped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/dc/measurement/topic_statistics")
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("pulse_smoke_record_waiter")
    record = {}

    def on_record(msg: StringStamped) -> None:
        if "record" in record:
            return
        try:
            parsed = json.loads(msg.data)
        except ValueError:
            node.get_logger().warning(f"non-JSON payload on {args.topic}: {msg.data[:200]!r}")
            return
        if "average" in parsed and "sample_count" in parsed:
            record["record"] = parsed
        else:
            node.get_logger().info(f"Record without average/sample_count: {sorted(parsed)}")

    node.create_subscription(StringStamped, args.topic, on_record, 10)

    deadline = time.monotonic() + args.timeout_seconds
    while rclpy.ok() and "record" not in record:
        if time.monotonic() >= deadline:
            print(
                f"FAIL: no Record with average and sample_count on {args.topic} within "
                f"{args.timeout_seconds}s",
                flush=True,
            )
            node.destroy_node()
            rclpy.shutdown()
            return 1
        rclpy.spin_once(node, timeout_sec=0.2)

    print("PASS: TopicStatistics Record:", json.dumps(record["record"], sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
