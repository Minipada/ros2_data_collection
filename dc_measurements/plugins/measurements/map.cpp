#include "dc_measurements/plugins/measurements/map.hpp"

namespace dc_measurements
{

Map::Map() : dc_measurements::Measurement()
{
}

Map::~Map() = default;

void Map::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".topic", rclcpp::ParameterValue("/map"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_path",
                                               rclcpp::ParameterValue("map/%Y-%m-%dT%H:%M:%S"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_map_timeout",
                                               rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".quiet", rclcpp::ParameterValue(true));

  node->get_parameter(measurement_name_ + ".topic", map_topic_);
  node->get_parameter(measurement_name_ + ".save_path", save_path_);
  node->get_parameter(measurement_name_ + ".save_map_timeout", save_map_timeout_);
  node->get_parameter(measurement_name_ + ".quiet", quiet_);
}

void Map::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "map.json");
  }
}

std::string Map::getAbsolutePath(const std::string& param_reference, const rclcpp::Time& now)
{
  std::string file_save_path = getSavePath(param_reference, now);
  std::string local_file_save_path =
      std::filesystem::path(save_local_base_path_expanded_) / all_base_path_expanded_ / (file_save_path);
  local_file_save_path = dc_util::expand_time(local_file_save_path, now);

  return local_file_save_path;
}

std::string Map::getLocalPath(const std::string& param_reference, const rclcpp::Time& now)
{
  std::string file_save_path = getSavePath(param_reference, now);
  std::string local_file_save_path = std::filesystem::path(file_save_path);
  local_file_save_path = dc_util::expand_time(local_file_save_path, now);

  return local_file_save_path;
}

std::string Map::saveMap()
{
  auto node = getNode();
  auto now = node->get_clock()->now();
  std::string absolute_path = getAbsolutePath("save_path", now);
  auto dir = std::filesystem::path(absolute_path).parent_path().u8string();
  auto basename = std::filesystem::path(absolute_path).filename().u8string();

  std::filesystem::create_directories(dir);
  std::string command =
      std::string(std::string("cd ") + dir + " && ros2 run nav2_map_server map_saver_cli -t " + map_topic_ + " -f " +
                  basename + " --ros-args -p save_map_timeout:=" + std::to_string(save_map_timeout_));
  if (quiet_)
  {
    command += " >/dev/null 2>&1";
  }
  auto return_code = system(command.c_str());

  if (return_code != 0)
  {
    RCLCPP_ERROR(logger_, "Map was not saved");
    return "";
  }

  if (!std::filesystem::exists((absolute_path + ".pgm").c_str()) ||
      !std::filesystem::exists((absolute_path + ".yaml").c_str()))
  {
    // Delete in case only one of them was saved
    std::remove((absolute_path + ".pgm").c_str());
    std::remove((absolute_path + ".yaml").c_str());
    RCLCPP_ERROR(logger_, "Map was not saved");
    return "";
  }

  return absolute_path;
}

int Map::getMapSize(const std::string& path_map)
{
  std::ifstream infile(path_map.c_str());
  std::string line_data = "";

  getline(infile, line_data);  // Get first line.
  getline(infile, line_data);  // Get second line, where size is

  std::istringstream iss(line_data);
  std::string width;
  getline(iss, width, ' ');  // Get width, which is also equal to height
  return std::stoi(width);
}

void Map::saveRemoteKeys(json& data_json, const std::string& key, const std::string& relative_path,
                         const rclcpp::Time& now)
{
  for (auto it = remote_keys_.begin(); it != remote_keys_.end(); ++it)
  {
    int index = std::distance(remote_keys_.begin(), it);
    std::string new_key = *it;
    std::string new_value = std::filesystem::path(all_base_path_expanded_) / remote_prefixes_[index] / relative_path;

    // Erase leading slash
    if (new_value[0] == '/')
    {
      new_value.erase(0, 1);
    }
    // e.g data_json["remote_paths"]["minio"]["yaml"]
    data_json["remote_paths"][new_key][key] = dc_util::expand_time(new_value, now);
  }
}

dc_interfaces::msg::StringStamped Map::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  auto now = node->get_clock()->now();
  msg.header.stamp = now;
  msg.group_key = group_key_;
  json data_json;
  auto file_save_path = saveMap();
  if (!file_save_path.empty())
  {
    YAML::Node data_map = YAML::LoadFile(file_save_path + ".yaml");

    data_json["resolution"] = data_map["resolution"].as<float>();
    data_json["origin"]["x"] = data_map["origin"][0].as<float>();
    data_json["origin"]["y"] = data_map["origin"][1].as<float>();
    data_json["local_paths"]["yaml"] = (file_save_path + ".yaml").c_str();
    data_json["local_paths"]["pgm"] = (file_save_path + ".pgm").c_str();
    std::string relative_path = getLocalPath("save_path", now);
    saveRemoteKeys(data_json, "yaml", relative_path + ".yaml", now);
    saveRemoteKeys(data_json, "pgm", relative_path + ".pgm", now);
    int width = getMapSize(file_save_path + ".pgm");
    data_json["width"] = width;
    data_json["height"] = width;
  }
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Map, dc_core::Measurement)
