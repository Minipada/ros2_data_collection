// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/conditions/exist.hpp"

namespace dc_conditions
{

Exist::Exist() : dc_conditions::Condition()
{
}

void Exist::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
}

bool Exist::getState(dc_interfaces::msg::StringStamped msg)
{
  try
  {
    json data_json = json::parse(msg.data);
    json flat_json = data_json.flatten();
    std::string key_exact = std::string("/") + key_;
    std::string key_w_prefix = key_exact + "/";

    // A flattened key matches "key_" either exactly (key_ is itself a scalar leaf, e.g.
    // {"level": 5.5} with key_="level" flattens to exactly "/level") or as a "key_/..." prefix
    // (key_ names an object/array with descendants). Checking the prefix alone misses the
    // exact-match case entirely, since flatten() never emits "/level/" for a leaf value.
    bool found = false;
    for (auto& x : flat_json.items())
    {
      if (x.key() == key_exact || x.key().rfind(key_w_prefix, 0) == 0)
      {
        found = true;
        break;
      }
    }
    active_ = found;

    publishActive();
  }
  catch (json::parse_error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON (exist): " << msg.data);
    return false;
  }
  return active_;
}

Exist::~Exist() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::Exist, dc_core::Condition)
