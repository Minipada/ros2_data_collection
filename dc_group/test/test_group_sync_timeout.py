"""Tests for the Group node's sync-timeout drop/emit_partial behaviour (#126).

Every test drives a real `GroupServer` through a real executor against real DDS topics —
`message_filters` has no timeout of its own, so what is under test is precisely the
interaction between the synchroniser, the deadline timer and the subscriptions, which a
mocked-out unit test would not exercise.
"""

import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from dc_group.group_server import GroupServer
from dc_interfaces.msg import StringStamped

GROUP = "test_group"
INPUT_A = "/dc/test/a"
INPUT_B = "/dc/test/b"
OUTPUT = "/dc/group/test_group"

# Short enough to keep the suite fast, long enough that a loaded CI machine still gets a
# Record of a complete set through the synchroniser before the deadline fires.
SYNC_TIMEOUT = 0.5
# Generous upper bounds: every wait below is `spin_until`, so a passing test returns as
# soon as the condition holds and only a genuine failure burns the whole budget.
SETTLE = 3.0


@pytest.fixture
def ros():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


class GroupHarness:
    """A `GroupServer` plus a client node publishing its inputs and collecting its output."""

    def __init__(self, inputs=(INPUT_A, INPUT_B), **group_params):
        overrides = [
            Parameter("groups", Parameter.Type.STRING_ARRAY, [GROUP]),
            Parameter(f"{GROUP}.inputs", Parameter.Type.STRING_ARRAY, list(inputs)),
            Parameter(f"{GROUP}.output", Parameter.Type.STRING, OUTPUT),
        ]
        for name, value in group_params.items():
            overrides.append(Parameter(f"{GROUP}.{name}", value=value))

        self.inputs = list(inputs)
        self.server = GroupServer("group_server", parameter_overrides=overrides)
        self.client = Node("group_test_client")
        self.input_publishers = [
            self.client.create_publisher(StringStamped, topic, 10) for topic in self.inputs
        ]
        self.records = []
        self.client.create_subscription(StringStamped, OUTPUT, self.records.append, 10)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.server)
        self.executor.add_node(self.client)

    def shutdown(self):
        self.executor.shutdown()
        self.client.destroy_node()
        self.server.destroy_node()

    def spin_until(self, predicate, timeout=SETTLE):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for_discovery(self):
        matched = self.spin_until(
            lambda: all(pub.get_subscription_count() > 0 for pub in self.input_publishers)
            and self.client.count_publishers(OUTPUT) > 0,
            timeout=10.0,
        )
        assert matched, "the Group node's subscriptions/publisher never matched the test node"

    def publish(self, index, data):
        msg = StringStamped()
        msg.data = json.dumps(data)
        msg.group_key = self.inputs[index].rsplit("/", 1)[-1]
        msg.header.stamp = self.server.get_clock().now().to_msg()
        self.input_publishers[index].publish(msg)

    def wait_for_records(self, count, timeout=SETTLE):
        return self.spin_until(lambda: len(self.records) >= count, timeout=timeout)

    def payloads(self):
        return [json.loads(record.data) for record in self.records]


@pytest.fixture
def harness(ros):
    built = {}

    def build(**kwargs):
        built["harness"] = GroupHarness(**kwargs)
        built["harness"].wait_for_discovery()
        return built["harness"]

    yield build
    if "harness" in built:
        built["harness"].shutdown()


def test_complete_set_still_publishes_one_unmarked_record(harness):
    """The deadline timer must not disturb the normal path: a complete set stays complete."""
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="emit_partial")
    group.publish(0, {"used": 12.0})
    group.publish(1, {"free": 34.0})

    assert group.wait_for_records(1), "no Record published for a complete set"
    # Nothing else may follow: the set was consumed, so the deadline has nothing left to fire on.
    group.spin_for(2 * SYNC_TIMEOUT)
    assert len(group.records) == 1

    payload = group.payloads()[0]
    assert payload["a"] == {"used": 12.0}
    assert payload["b"] == {"free": 34.0}
    assert payload["name"] == GROUP
    # A complete Record's shape is byte-for-byte what DC published before #126.
    assert "partial" not in payload
    assert "missing_inputs" not in payload


def test_silent_input_publishes_nothing_by_default(harness):
    """Default (no sync_timeout): an input that never publishes stalls the group, as before."""
    group = harness()
    assert group.server.params[GROUP]["sync_timeout"] == 0.0
    assert group.server.params[GROUP]["on_sync_timeout"] == "drop"

    group.publish(0, {"used": 12.0})
    group.spin_for(4 * SYNC_TIMEOUT)

    assert group.records == []
    # No deadline timer exists at all, so the synchroniser keeps holding the lone Record —
    # exactly the pre-#126 behaviour.
    assert group.server.sync_timers[GROUP] is None
    assert group.server.time_synchronizer[GROUP].queues[0] != {}


def test_drop_discards_the_incomplete_set_on_timeout(harness):
    """`drop` with a timeout publishes nothing *and* actively clears the held Record."""
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="drop")
    group.publish(0, {"used": 12.0})

    dropped = group.spin_until(
        lambda: group.server.time_synchronizer[GROUP].queues[0] == {},
        timeout=SETTLE,
    )
    assert dropped, "the incomplete set was never dropped"
    assert group.records == []

    group.spin_for(2 * SYNC_TIMEOUT)
    assert group.records == []


def test_emit_partial_publishes_one_partial_record(harness):
    """`emit_partial`: the Records that did arrive are published, the silent input is named."""
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="emit_partial")
    group.publish(0, {"used": 12.0})

    assert group.wait_for_records(1), "no partial Record published for the incomplete set"
    # A timed-out set is emitted once, not once per deadline tick.
    group.spin_for(4 * SYNC_TIMEOUT)
    assert len(group.records) == 1

    payload = group.payloads()[0]
    assert payload["a"] == {"used": 12.0}
    # The missing input's key is omitted, not present-and-null.
    assert "b" not in payload
    assert payload["partial"] is True
    assert payload["missing_inputs"] == [INPUT_B]
    # Group-level fields are unaffected by partialness.
    assert payload["name"] == GROUP
    assert "tags" in payload


def test_partial_record_is_not_republished_when_the_late_input_arrives(harness):
    """A Record emitted in a partial set must not be matched again by its late partner."""
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="emit_partial")
    group.publish(0, {"used": 12.0})
    assert group.wait_for_records(1), "no partial Record published"

    group.publish(1, {"free": 34.0})
    group.spin_for(4 * SYNC_TIMEOUT)

    # The late Record is alone in its own set, so it times out into its own partial Record —
    # never a second Record built from the already-emitted `a`.
    assert len(group.records) == 2
    late = group.payloads()[1]
    assert late["b"] == {"free": 34.0}
    assert "a" not in late
    assert late["missing_inputs"] == [INPUT_A]


def test_group_recovers_and_publishes_complete_records_after_a_partial(harness):
    """Purging the synchroniser on timeout must not wedge it for subsequent sets."""
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="emit_partial")
    group.publish(0, {"used": 12.0})
    assert group.wait_for_records(1), "no partial Record published"

    group.publish(0, {"used": 56.0})
    group.publish(1, {"free": 78.0})
    assert group.wait_for_records(2), "no Record published for the complete set after a partial"

    complete = group.payloads()[1]
    assert complete["a"] == {"used": 56.0}
    assert complete["b"] == {"free": 78.0}
    assert "partial" not in complete


def test_emit_partial_without_a_positive_timeout_falls_back_to_drop(harness):
    """`emit_partial` has no deadline to fire on without a `sync_timeout`; refuse it loudly."""
    group = harness(on_sync_timeout="emit_partial")
    assert group.server.params[GROUP]["on_sync_timeout"] == "drop"

    group.publish(0, {"used": 12.0})
    group.spin_for(4 * SYNC_TIMEOUT)
    assert group.records == []


def test_unknown_policy_falls_back_to_drop(harness):
    group = harness(sync_timeout=SYNC_TIMEOUT, on_sync_timeout="keep_everything")
    assert group.server.params[GROUP]["on_sync_timeout"] == "drop"

    group.publish(0, {"used": 12.0})
    group.spin_for(4 * SYNC_TIMEOUT)
    assert group.records == []
