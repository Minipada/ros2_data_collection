#include "dc_measurements/plugins/measurements/diagnostics.hpp"

#include <algorithm>

namespace dc_measurements
{

Diagnostics::Diagnostics() : dc_measurements::Measurement()
{
}

Diagnostics::~Diagnostics() = default;

uint8_t Diagnostics::levelThresholdFromString(const std::string& level_threshold)
{
  if (level_threshold == "OK")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  if (level_threshold == "WARN")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  if (level_threshold == "ERROR")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (level_threshold == "STALE")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }

  RCLCPP_ERROR_STREAM(logger_, "Unknown diagnostics level_threshold '" << level_threshold << "', defaulting to OK");
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

void Diagnostics::onConfigure()
{
  auto node = getNode();
  topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/diagnostics");
  level_threshold_str_ = dc_util::get_str_type_param(node, measurement_name_, "level_threshold", "OK");
  names_ = dc_util::get_str_array_type_param(node, measurement_name_, "names", std::vector<std::string>{});

  level_threshold_ = levelThresholdFromString(level_threshold_str_);

  subscription_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topic_, rclcpp::SystemDefaultsQoS(), std::bind(&Diagnostics::diagnosticsCb, this, std::placeholders::_1));
}

void Diagnostics::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "diagnostics.json");
  }
}

void Diagnostics::diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray& msg)
{
  json statuses = json::array();

  for (const auto& status : msg.status)
  {
    if (status.level < level_threshold_)
    {
      continue;
    }

    if (!names_.empty() && std::find(names_.begin(), names_.end(), status.name) == names_.end())
    {
      continue;
    }

    json values = json::object();
    for (const auto& key_value : status.values)
    {
      values[key_value.key] = key_value.value;
    }

    json status_json;
    status_json["name"] = status.name;
    status_json["message"] = status.message;
    status_json["hardware_id"] = status.hardware_id;
    status_json["level"] = static_cast<int>(status.level);
    status_json["values"] = values;

    statuses.push_back(status_json);
  }

  // Nothing matched the filters this cycle: leave the previously cached data alone rather
  // than overwriting it with an empty array, mirroring how a Measurement with no subscription
  // update simply has nothing new for collect() to report.
  if (statuses.empty())
  {
    return;
  }

  json data_json;
  data_json["statuses"] = statuses;

  dc_interfaces::msg::StringStamped pub_msg;
  pub_msg.group_key = group_key_;
  pub_msg.data = data_json.dump(-1, ' ', true);
  auto node = getNode();
  pub_msg.header.stamp = node->get_clock()->now();
  last_data_ = pub_msg;
}

dc_interfaces::msg::StringStamped Diagnostics::collect()
{
  dc_interfaces::msg::StringStamped msg = last_data_;
  last_data_ = dc_interfaces::msg::StringStamped();

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Diagnostics, dc_core::Measurement)
