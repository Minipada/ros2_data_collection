// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MEASUREMENT_CONFIG_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_CONFIG_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dc_core/measurement.hpp"
#include "dc_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dc_measurements
{

// Every per-Measurement parameter name and default, in one place. The server-level fields of the
// returned MeasurementConfig (save paths, run id, custom keys) are the caller's to fill.
inline dc_core::MeasurementConfig read_measurement_config(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                          const std::string& measurement_name)
{
  dc_core::MeasurementConfig config;

  // Mandatory parameters
  config.measurement_plugin = dc_util::get_str_type_param(node, measurement_name, "plugin");

  // Optional parameters
  config.group_key = dc_util::get_str_type_param(node, measurement_name, "group_key", "");
  config.topic_output = dc_util::get_str_type_param(node, measurement_name, "topic_output",
                                                    std::string("/dc/measurement/") + measurement_name);
  config.polling_interval = dc_util::get_int_type_param(node, measurement_name, "polling_interval", 1000);
  config.debug = dc_util::get_bool_type_param(node, measurement_name, "debug", false);
  config.enable_validator = dc_util::get_bool_type_param(node, measurement_name, "enable_validator", true);
  config.json_schema_path = dc_util::get_str_type_param(node, measurement_name, "json_schema_path", "");
  config.tags = dc_util::get_str_array_type_param(node, measurement_name, "tags", std::vector<std::string>());
  config.init_collect = dc_util::get_bool_type_param(node, measurement_name, "init_collect", true);
  config.init_max_measurements = dc_util::get_int_type_param(node, measurement_name, "init_max_measurements", 0);
  config.condition_max_measurements =
      dc_util::get_int_type_param(node, measurement_name, "condition_max_measurements", 0);
  config.include_measurement_name =
      dc_util::get_bool_type_param(node, measurement_name, "include_measurement_name", true);
  config.include_measurement_plugin =
      dc_util::get_bool_type_param(node, measurement_name, "include_measurement_plugin", false);
  config.if_all_conditions =
      dc_util::get_str_array_type_param(node, measurement_name, "if_all_conditions", std::vector<std::string>());
  config.if_any_conditions =
      dc_util::get_str_array_type_param(node, measurement_name, "if_any_conditions", std::vector<std::string>());
  config.if_none_conditions =
      dc_util::get_str_array_type_param(node, measurement_name, "if_none_conditions", std::vector<std::string>());
  config.gate_condition = dc_util::get_str_type_param(node, measurement_name, "gate_condition", "");
  config.remote_keys =
      dc_util::get_str_array_type_param(node, measurement_name, "remote_keys", std::vector<std::string>());
  config.remote_prefixes =
      dc_util::get_str_array_type_param(node, measurement_name, "remote_prefixes", std::vector<std::string>());
  config.nested = dc_util::get_bool_type_param(node, measurement_name, "nested", false);
  config.flatten = dc_util::get_bool_type_param(node, measurement_name, "flatten", false);
  config.buffer_duration_sec = dc_util::get_double_type_param(node, measurement_name, "buffer_duration_sec", 0.0);
  config.post_roll_duration_sec = dc_util::get_double_type_param(node, measurement_name, "post_roll_duration_sec", 0.0);
  config.cooldown_sec = dc_util::get_double_type_param(node, measurement_name, "cooldown_sec", 0.0);
  config.max_flush_rate_hz = dc_util::get_double_type_param(node, measurement_name, "max_flush_rate_hz", 0.0);
  config.flush_topic = dc_util::get_str_type_param(node, measurement_name, "flush_topic", std::string("/dc/flush"));

  return config;
}

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MEASUREMENT_CONFIG_HPP_
