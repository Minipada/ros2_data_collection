import functools
import json
import sys

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy_message_converter import message_converter

from dc_interfaces.msg import StringStamped

from .flatten import flatten, unflatten_list


class GroupServer(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.subscribers = {}
        self.publishers_ = {}
        self.time_synchronizer = {}
        self.group_msg_type_cls = None
        self.group_exclude_keys = None
        self.group_measurement_plugins = None
        self.params = {}
        self.init_parameters()
        self.init_subscribers()
        self.init_publishers()
        self.init_time_synchronizer()

    def callback(self, *args, group):
        data_dict = {}
        plugins_list = []
        collected_time = self.get_clock().now()
        for measurement in args:
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
                        if not all([x in k for x in exclude_key.split("*")])
                    }
        if self.params[group]["nested_data"]:
            data_dict = unflatten_list(flat_dict=data_dict, separator=".")
        data_dict["tags"] = self.params[group]["tags"]
        if self.group_measurement_plugins and plugins_list:
            data_dict["plugins"] = plugins_list

        msg = StringStamped()
        msg.data = json.dumps(data_dict)
        msg.header.stamp = collected_time.to_msg()
        msg.group_key = self.params[group]["group_key"]
        self.publishers_[group].publish(msg)

    def init_parameters(self):
        # Mandatory
        self.declare_parameter(
            name="groups",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
            ),
        )
        self.group_measurement_plugins = self.declare_parameter("group_measurement_plugins", True)
        try:
            self.params["groups"] = (
                self.get_parameter("groups").get_parameter_value().string_array_value
            )
        except rclpy.exceptions.ParameterUninitializedException:
            self.get_logger().error("groups parameter not set. Exiting Node")
            sys.exit(1)

        for group in self.params["groups"]:
            self.declare_parameter(
                name=f"{group}.inputs",
                descriptor=ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                ),
            )
            # Optionals
            self.declare_parameter(f"{group}.output", f"/dc/group/{group}")
            self.declare_parameter(f"{group}.exclude_keys", [""])
            self.declare_parameter(f"{group}.sync_delay", 5.0)
            self.declare_parameter(f"{group}.group_key", group)
            self.declare_parameter(f"{group}.tags", [""])
            self.declare_parameter(f"{group}.nested_data", True)
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
                "group_key": self.get_parameter(f"{group}.group_key")
                .get_parameter_value()
                .string_value,
                "tags": self.get_parameter(f"{group}.tags")
                .get_parameter_value()
                .string_array_value,
                "nested_data": self.get_parameter(f"{group}.nested_data")
                .get_parameter_value()
                .bool_value,
            }

    def init_subscribers(self):
        for group in self.params["groups"]:
            self.subscribers[group] = [
                Subscriber(self, StringStamped, topic) for topic in self.params[group]["inputs"]
            ]

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
