#include "dc_measurements/plugins/conditions/same_as_previous.hpp"

namespace dc_conditions
{

SameAsPrevious::SameAsPrevious() : dc_conditions::Condition()
{
}

void SameAsPrevious::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".keys", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".exclude", rclcpp::PARAMETER_STRING_ARRAY);
  node->get_parameter(condition_name_ + ".keys", keys_);
  node->get_parameter(condition_name_ + ".exclude", exclude_);

  for (std::size_t i = 0; i < keys_.size(); ++i)
  {
    keys_hash_.push_back("");
    previous_keys_hash_.push_back("");
  }
}

bool SameAsPrevious::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);
  data_json.erase("tags");
  previous_keys_hash_ = keys_hash_;

  // Exclude keys from keys_ first
  json flat_json = data_json.flatten();
  std::vector<std::string> keys;
  // Iterate through each key
  auto flat_json_tmp = flat_json;
  for (auto& x : flat_json.items())
  {
    // Iterate through each exclude field and check if the json key matches the exclude regex
    for (const auto& exclude_field : exclude_)
    {
      // The prefix / is added by nlohmann when flattening
      flat_json_tmp =
          dc_util::tojson::detail::remove_key_match_regex(flat_json_tmp, std::string("/") + exclude_field, x.key());
    }
  }
  flat_json = flat_json_tmp;

  // Iterate through each key key field and check if the json key matches the regex
  for (auto key = keys_.begin(); key != keys_.end(); ++key)
  {
    std::string key_w_prefix = std::string("/") + *key;
    int index = std::distance(keys_.begin(), key);
    if (!flat_json_tmp.contains(key_w_prefix) || !std::filesystem::exists(flat_json_tmp[key_w_prefix]))
    {
      keys_hash_[index] = "";
    }
    else if ((flat_json_tmp.contains(key_w_prefix)) && std::filesystem::exists(flat_json_tmp[key_w_prefix]))
    {
      auto new_hash = dc_util::getFileMd5(flat_json_tmp[key_w_prefix]);

      keys_hash_[index] = new_hash;
    }
    flat_json_tmp = dc_util::tojson::detail::remove_key_match_regex(flat_json_tmp, key_w_prefix, *key);
  }
  flat_json = flat_json_tmp;
  // If first data
  if (previous_json_.empty())
  {
    active_ = false;
  }
  // If hash comparison didn't determine the result, we compare the jsons
  else if (previous_json_ == flat_json.unflatten() &&
           ((previous_keys_hash_ == keys_hash_) || (previous_keys_hash_.empty())))
  {
    active_ = true;
  }
  else
  {
    active_ = false;
  }
  previous_json_ = flat_json.unflatten();

  publishActive();
  return active_;
}

SameAsPrevious::~SameAsPrevious() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::SameAsPrevious, dc_core::Condition)
