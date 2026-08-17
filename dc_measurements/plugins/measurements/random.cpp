// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/random.hpp"

namespace dc_measurements
{

Random::Random() : dc_measurements::Measurement()
{
}

Random::~Random() = default;

void Random::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "random.json");
  }
}

void Random::onConfigure()
{
  auto node = getNode();
  value_type_ = dc_util::get_str_type_param(node, measurement_name_, "type", "integer");
  min_ = dc_util::get_double_type_param(node, measurement_name_, "min", 0.0);
  max_ = dc_util::get_double_type_param(node, measurement_name_, "max", 100.0);
  seed_ = dc_util::get_int_type_param(node, measurement_name_, "seed", -1);

  if (value_type_ != "integer" && value_type_ != "double")
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid Random type '" << value_type_ << "', expected 'integer' or 'double'");
    throw std::runtime_error{ "Invalid Random type: " + value_type_ };
  }

  if (min_ >= max_)
  {
    RCLCPP_ERROR_STREAM(logger_, "Random min (" << min_ << ") must be less than max (" << max_ << ")");
    throw std::runtime_error{ "Random min must be less than max" };
  }

  // A negative seed means "not set": seed from a real entropy source so runs differ. A
  // non-negative seed makes the generated sequence reproducible across runs (acceptance
  // criterion: optional seed parameter).
  if (seed_ >= 0)
  {
    generator_.seed(static_cast<uint64_t>(seed_));
  }
  else
  {
    std::random_device rd;
    generator_.seed(rd());
  }
}

dc_interfaces::msg::StringStamped Random::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;

  if (value_type_ == "integer")
  {
    std::uniform_int_distribution<int64_t> distribution(static_cast<int64_t>(min_), static_cast<int64_t>(max_));
    data_json["value"] = distribution(generator_);
  }
  else
  {
    std::uniform_real_distribution<double> distribution(min_, max_);
    data_json["value"] = distribution(generator_);
  }

  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Random, dc_core::Measurement)
