#include "dc_measurements/plugins/measurements/permissions.hpp"

namespace dc_measurements
{

Permissions::Permissions() : dc_measurements::Measurement()
{
}

Permissions::~Permissions() = default;

void Permissions::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".path", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".format", rclcpp::ParameterValue("int"));
  node->get_parameter(measurement_name_ + ".path", path_);
  node->get_parameter(measurement_name_ + ".format", permission_format_);

  full_path_ = dc_util::expand_env(path_);

  if (full_path_.empty())
  {
    throw std::runtime_error{ "Failed to configure permissions node. Set env or path parameter" };
  }
  else
  {
    RCLCPP_INFO(logger_, "Permissions path: %s", full_path_.c_str());
  }
}

void Permissions::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "permissions.json");
  }
}

struct stat Permissions::getOwner(const std::string& path)
{
  struct stat info;
  stat(path.c_str(), &info);

  return info;
}

std::string Permissions::formatPermissions(const mode_t& perm, std::string format)
{
  std::string modeval;
  if (dc_util::stringMatchesRegex("[rR][wW][xX]", format))
  {
    modeval.push_back((perm & S_IRUSR) ? 'r' : '-');
    modeval.push_back((perm & S_IWUSR) ? 'w' : '-');
    modeval.push_back((perm & S_IXUSR) ? 'x' : '-');
    modeval.push_back((perm & S_IRGRP) ? 'r' : '-');
    modeval.push_back((perm & S_IWGRP) ? 'w' : '-');
    modeval.push_back((perm & S_IXGRP) ? 'x' : '-');
    modeval.push_back((perm & S_IROTH) ? 'r' : '-');
    modeval.push_back((perm & S_IWOTH) ? 'w' : '-');
    modeval.push_back((perm & S_IXOTH) ? 'x' : '-');
  }
  else if (format == "int")
  {
    int users = 0;
    users += (perm & S_IRUSR) ? 4 : 0;
    users += (perm & S_IWUSR) ? 2 : 0;
    users += (perm & S_IXUSR) ? 1 : 0;
    modeval += std::to_string(users);

    int group = 0;
    group += (perm & S_IRGRP) ? 4 : 0;
    group += (perm & S_IWGRP) ? 2 : 0;
    group += (perm & S_IXGRP) ? 1 : 0;
    modeval += std::to_string(group);

    int others = 0;
    others += (perm & S_IROTH) ? 4 : 0;
    others += (perm & S_IWOTH) ? 2 : 0;
    others += (perm & S_IXOTH) ? 1 : 0;
    modeval += std::to_string(others);
  }
  return modeval;
}

dc_interfaces::msg::StringStamped Permissions::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  auto info = getOwner(full_path_);
  struct passwd* pw = getpwuid(info.st_uid);
  struct group* gr = getgrgid(info.st_gid);

  // Not recognized by the system
  // Can happen if file owned by host system and not by container
  if (pw != nullptr)
  {
    data_json["user"] = pw->pw_name;
  }
  if (gr != nullptr)
  {
    data_json["group"] = gr->gr_name;
  }

  data_json["uid"] = info.st_uid;
  data_json["gid"] = info.st_gid;
  data_json["exists"] = std::filesystem::exists(std::filesystem::path(full_path_));
  data_json["permissions"] = formatPermissions(info.st_mode, permission_format_);
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Permissions, dc_core::Measurement)
