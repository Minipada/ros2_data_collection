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
    std::string key_w_prefix = std::string("/") + key_ + "/";

    bool found = false;
    for (auto& x : flat_json.items())
    {
      if (x.key().rfind(key_w_prefix, 0) == 0)
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
