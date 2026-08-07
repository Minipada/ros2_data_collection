import functools
import json
import sys

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy_message_converter import message_converter

from dc_interfaces.msg import StringStamped

from .flatten import flatten, unflatten_list

# Values accepted by the per-group `on_sync_timeout` parameter. `message_filters` has no
# timeout of its own, so DC runs a timer alongside the synchroniser (see
# https://answers.ros.org/question/410138/) and applies one of these when it fires.
ON_SYNC_TIMEOUT_DROP = "drop"
ON_SYNC_TIMEOUT_EMIT_PARTIAL = "emit_partial"
ON_SYNC_TIMEOUT_POLICIES = (ON_SYNC_TIMEOUT_DROP, ON_SYNC_TIMEOUT_EMIT_PARTIAL)


class GroupServer(Node):
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.subscribers = {}
        self.publishers_ = {}
        self.time_synchronizer = {}
        # Per group: {input index: latest Record not yet consumed by the synchroniser}, and
        # the timer measuring how long that incomplete set has been waiting.
        self.pending = {}
        self.sync_timers = {}
        self.group_msg_type_cls = None
        self.group_exclude_keys = None
        self.group_measurement_plugins = None
        self.params = {}
        self.init_parameters()
        self.init_subscribers()
        self.init_publishers()
        self.init_time_synchronizer()
        self.init_sync_timeouts()

    def callback(self, *args, group):
        """Synchroniser callback: every input of `group` produced a Record in the same window."""
        self.pending[group] = {}
        self.cancel_sync_timeout(group)
        self.publish_record(group, list(args), missing_inputs=[])

    def publish_record(self, group, measurements, missing_inputs):
        """Merge `measurements` into a single Record and publish it on the group's output.

        Args:
            group (str): Name of the group being published
            measurements (list): Records to merge, ordered by the input they came from
            missing_inputs (list): Input topics that contributed no Record. Empty for a
                complete Record, non-empty for a partial one
        """
        data_dict = {}
        plugins_list = []
        collected_time = self.get_clock().now()
        for measurement in measurements:
            # https://github.com/ros2/rosidl_python/blob/0f5c8f360be92566ad86f4b29f3db1febfca2242/rosidl_generator_py/resource/_msg.py.em#L187-L189
            measurement_dict = message_converter.convert_ros_message_to_dictionary(measurement)
            m_data = json.loads(measurement.data)
            m_data.pop("tags", None)
            if self.group_measurement_plugins:
                if "plugin" in m_data:
                    plugins_list.append(m_data["plugin"])
            tmp_data_dict = flatten(
                nested_dict={measurement_dict["group_key"]: m_data}, separator="."
            )
            data_dict = data_dict | tmp_data_dict

        for exclude_key in self.params[group]["exclude_keys"]:
            if exclude_key != "":
                if "*" not in exclude_key:
                    data_dict = {
                        k: v for k, v in data_dict.items() if not k.startswith(exclude_key)
                    }
                else:
                    # Split by *
                    data_dict = {
                        k: v
                        for k, v in data_dict.items()
                        if not all(x in k for x in exclude_key.split("*"))
                    }
        if self.params[group]["nested_data"]:
            data_dict = unflatten_list(flat_dict=data_dict, separator=".")
        data_dict["tags"] = self.params[group]["tags"]
        if self.group_measurement_plugins and plugins_list:
            data_dict["plugins"] = plugins_list

        if self.params[group]["include_group_name"]:
            data_dict["name"] = group

        # A Record assembled by the sync timeout rather than by the synchroniser is marked
        # so consumers can tell it apart. A complete Record carries neither key, keeping its
        # shape identical to what DC published before the timeout existed. The keys of the
        # inputs that never showed up are simply absent: the Group node only knows a missing
        # input's *topic*, never its `group_key` or its fields (both come from the Record
        # itself), so it cannot synthesise a correctly shaped null placeholder for it.
        if missing_inputs:
            data_dict["partial"] = True
            data_dict["missing_inputs"] = list(missing_inputs)

        msg = StringStamped()
        msg.data = json.dumps(data_dict)
        msg.header.stamp = collected_time.to_msg()
        msg.group_key = self.params[group]["group_key"]
        self.publishers_[group].publish(msg)

    def init_parameters(self):
        # Mandatory. Declared by type rather than by a typed ParameterDescriptor: since
        # Jazzy, rclpy overwrites a descriptor's `type` with the type inferred from the
        # default value, so the descriptor form raises on a parameter that has no default.
        self.declare_parameter("groups", Parameter.Type.STRING_ARRAY)
        self.group_measurement_plugins = self.declare_parameter("group_measurement_plugins", True)
        try:
            self.params["groups"] = (
                self.get_parameter("groups").get_parameter_value().string_array_value
            )
        except rclpy.exceptions.ParameterUninitializedException:
            self.get_logger().error("groups parameter not set. Exiting Node")
            sys.exit(1)

        for group in self.params["groups"]:
            self.declare_parameter(f"{group}.inputs", Parameter.Type.STRING_ARRAY)
            # Optionals
            self.declare_parameter(f"{group}.output", f"/dc/group/{group}")
            self.declare_parameter(f"{group}.exclude_keys", [""])
            self.declare_parameter(f"{group}.sync_delay", 5.0)
            self.declare_parameter(f"{group}.sync_timeout", 0.0)
            self.declare_parameter(f"{group}.on_sync_timeout", ON_SYNC_TIMEOUT_DROP)
            self.declare_parameter(f"{group}.group_key", group)
            self.declare_parameter(f"{group}.tags", [""])
            self.declare_parameter(f"{group}.nested_data", True)
            self.declare_parameter(f"{group}.include_group_name", True)
            self.params[group] = {
                "inputs": self.get_parameter(f"{group}.inputs")
                .get_parameter_value()
                .string_array_value,
                "exclude_keys": self.get_parameter(f"{group}.exclude_keys")
                .get_parameter_value()
                .string_array_value,
                "output": self.get_parameter(f"{group}.output").get_parameter_value().string_value,
                "sync_delay": self.get_parameter(f"{group}.sync_delay")
                .get_parameter_value()
                .double_value,
                "sync_timeout": self.get_parameter(f"{group}.sync_timeout")
                .get_parameter_value()
                .double_value,
                "on_sync_timeout": self.get_parameter(f"{group}.on_sync_timeout")
                .get_parameter_value()
                .string_value,
                "group_key": self.get_parameter(f"{group}.group_key")
                .get_parameter_value()
                .string_value,
                "tags": self.get_parameter(f"{group}.tags")
                .get_parameter_value()
                .string_array_value,
                "nested_data": self.get_parameter(f"{group}.nested_data")
                .get_parameter_value()
                .bool_value,
                "include_group_name": self.get_parameter(f"{group}.include_group_name")
                .get_parameter_value()
                .bool_value,
            }
            self.validate_sync_timeout_parameters(group)

    def validate_sync_timeout_parameters(self, group):
        """Reject an unusable drop/emit_partial configuration, falling back to `drop`."""
        policy = self.params[group]["on_sync_timeout"]
        if policy not in ON_SYNC_TIMEOUT_POLICIES:
            self.get_logger().error(
                f"Group '{group}': on_sync_timeout '{policy}' is not one of "
                f"{list(ON_SYNC_TIMEOUT_POLICIES)}, falling back to "
                f"'{ON_SYNC_TIMEOUT_DROP}'"
            )
            self.params[group]["on_sync_timeout"] = ON_SYNC_TIMEOUT_DROP
            policy = ON_SYNC_TIMEOUT_DROP

        if policy == ON_SYNC_TIMEOUT_EMIT_PARTIAL and self.params[group]["sync_timeout"] <= 0.0:
            self.get_logger().error(
                f"Group '{group}': on_sync_timeout '{ON_SYNC_TIMEOUT_EMIT_PARTIAL}' needs a "
                f"positive sync_timeout to know when a set is late, falling back to "
                f"'{ON_SYNC_TIMEOUT_DROP}'"
            )
            self.params[group]["on_sync_timeout"] = ON_SYNC_TIMEOUT_DROP

    def sync_timeout_enabled(self, group):
        return self.params[group]["sync_timeout"] > 0.0

    def init_subscribers(self):
        for group in self.params["groups"]:
            self.subscribers[group] = [
                Subscriber(self, StringStamped, topic) for topic in self.params[group]["inputs"]
            ]
            self.pending[group] = {}
            if not self.sync_timeout_enabled(group):
                continue
            # Registered before the synchroniser connects to the same Subscribers so that a
            # Record lands in `pending` *before* it can complete a set: the synchroniser's
            # callback then runs and clears it, instead of leaving it behind as a phantom
            # incomplete set that would time out later.
            for index, subscriber in enumerate(self.subscribers[group]):
                subscriber.registerCallback(
                    functools.partial(self.on_input, group=group, index=index)
                )

    def init_publishers(self):
        for group in self.params["groups"]:
            self.publishers_[group] = self.create_publisher(
                StringStamped, self.params[group]["output"], 10
            )

    def init_time_synchronizer(self):
        for group in self.params["groups"]:
            self.time_synchronizer[group] = ApproximateTimeSynchronizer(
                self.subscribers[group],
                10,
                self.params[group]["sync_delay"],
            )
            self.time_synchronizer[group].registerCallback(
                functools.partial(self.callback, group=group)
            )

    def init_sync_timeouts(self):
        """Create the per-group deadline timer, idle until an incomplete set shows up."""
        for group in self.params["groups"]:
            self.sync_timers[group] = None
            if not self.sync_timeout_enabled(group):
                continue
            timer = self.create_timer(
                self.params[group]["sync_timeout"],
                functools.partial(self.on_sync_timeout, group=group),
            )
            timer.cancel()
            self.sync_timers[group] = timer

    def on_input(self, msg, group, index):
        """Remember a Record and start the deadline if this opens a new incomplete set."""
        self.pending[group][index] = msg
        timer = self.sync_timers[group]
        # Only start the countdown on the first Record of a set: the deadline is on the set
        # as a whole, so later inputs must not push it back.
        if timer is not None and timer.is_canceled():
            timer.reset()

    def on_sync_timeout(self, group):
        """Deadline reached: the set the synchroniser is holding will never complete."""
        pending = self.pending[group]
        self.pending[group] = {}
        self.cancel_sync_timeout(group)
        if not pending:
            return

        inputs = self.params[group]["inputs"]
        missing_inputs = [topic for index, topic in enumerate(inputs) if index not in pending]
        # Ordered by input, not by arrival, so a partial Record's keys are deterministic.
        measurements = [pending[index] for index in sorted(pending)]

        if self.params[group]["on_sync_timeout"] == ON_SYNC_TIMEOUT_EMIT_PARTIAL:
            self.get_logger().warning(
                f"Group '{group}': no complete set after "
                f"{self.params[group]['sync_timeout']}s, emitting a partial Record "
                f"(no Record from: {missing_inputs})"
            )
            self.publish_record(group, measurements, missing_inputs=missing_inputs)
        else:
            self.get_logger().warning(
                f"Group '{group}': no complete set after "
                f"{self.params[group]['sync_timeout']}s, dropping the incomplete set "
                f"(no Record from: {missing_inputs})"
            )

        self.purge_synchronizer(group)

    def cancel_sync_timeout(self, group):
        timer = self.sync_timers.get(group)
        if timer is not None:
            timer.cancel()

    def purge_synchronizer(self, group):
        """Forget the Records the synchroniser is still holding for `group`.

        Without this, a Record already emitted in a partial set could be matched again once
        its late partner finally arrives, publishing it twice.

        Args:
            group (str): Name of the group whose synchroniser queues are cleared
        """
        synchronizer = self.time_synchronizer[group]
        with synchronizer.lock:
            for queue in synchronizer.queues:
                queue.clear()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    try:
        group_node = GroupServer(node_name="group_server")
        rclpy.spin(group_node)
    except KeyboardInterrupt:
        print("Group data collection stopped")  # noqa: T201
        group_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
