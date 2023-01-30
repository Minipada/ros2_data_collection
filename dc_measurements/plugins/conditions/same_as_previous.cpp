#include "dc_measurements/plugins/conditions/same_as_previous.hpp"

namespace dc_conditions
{

SameAsPrevious::SameAsPrevious() : dc_conditions::Condition()
{
}

void SameAsPrevious::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".topic", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".paths", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".exclude", rclcpp::PARAMETER_STRING_ARRAY);
  node->get_parameter(condition_name_ + ".topic", topic_);
  node->get_parameter(condition_name_ + ".paths", paths_);
  node->get_parameter(condition_name_ + ".exclude", exclude_);
  subscription_ = node->create_subscription<dc_interfaces::msg::StringStamped>(
      topic_.c_str(), 10, std::bind(&SameAsPrevious::dataCb, this, std::placeholders::_1));

  for (std::size_t i = 0; i < paths_.size(); ++i)
  {
    paths_hash_.push_back("");
    previous_paths_hash_.push_back("");
  }
}

void SameAsPrevious::dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg)
{
  json data_json = json::parse(msg->data);
  data_json.erase("tags");

  // Exclude paths from paths_ first
  json flat_json = data_json.flatten();
  std::vector<std::string> keys;
  // Iterate through each key
  auto flat_json_tmp = flat_json;
  for (auto& x : flat_json.items())
  {
    // Iterate through each exclude field and check if the json key matches the exclude regex
    for (auto exclude_field = exclude_.begin(); exclude_field != exclude_.end(); ++exclude_field)
    {
      // The prefix / is added by nlohmann when flattening
      flat_json_tmp =
          dc_util::tojson::detail::remove_key_match_regex(flat_json_tmp, std::string("/") + *exclude_field, x.key());
    }
  }
  flat_json = flat_json_tmp;

  // Iterate through each key path field and check if the json key matches the regex
  for (auto key_path = paths_.begin(); key_path != paths_.end(); ++key_path)
  {
    std::string key_path_w_prefix = std::string("/") + *key_path;
    int index = std::distance(paths_.begin(), key_path);
    if (!flat_json_tmp.contains(key_path_w_prefix) || !std::filesystem::exists(flat_json_tmp[key_path_w_prefix]))
    {
      paths_hash_[index] = "";
    }
    else if ((flat_json_tmp.contains(key_path_w_prefix)) && std::filesystem::exists(flat_json_tmp[key_path_w_prefix]))
    {
      auto new_hash = dc_util::getFileMd5(flat_json_tmp[key_path_w_prefix]);

      paths_hash_[index] = new_hash;
    }
    flat_json_tmp = dc_util::tojson::detail::remove_key_match_regex(flat_json_tmp, key_path_w_prefix, *key_path);
  }
  flat_json = flat_json_tmp;
  // If hash comparison didn't determine the result, we compare the jsons
  if (previous_json_ == flat_json.unflatten() && previous_paths_hash_ == paths_hash_)
  {
    active_ = true;
  }
  else
  {
    active_ = false;
  }
  previous_json_ = flat_json.unflatten();
  previous_paths_hash_ = paths_hash_;
}

SameAsPrevious::~SameAsPrevious() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::SameAsPrevious, dc_core::Condition)
