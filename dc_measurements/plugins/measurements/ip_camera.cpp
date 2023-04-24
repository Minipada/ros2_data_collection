#include "dc_measurements/plugins/measurements/ip_camera.hpp"

namespace dc_measurements
{

IpCamera::IpCamera() : dc_measurements::Measurement()
{
}

IpCamera::~IpCamera()
{
}

std::string IpCamera::getAbsolutePath(const std::string& param_reference)
{
  auto node = getNode();
  std::string file_save_path = node->get_parameter(measurement_name_ + "." + param_reference).as_string();
  std::string local_file_save_path = std::filesystem::path(save_local_base_path_) / "tmp" / (file_save_path);
  local_file_save_path = dc_util::expand_env(local_file_save_path);
  local_file_save_path = dc_util::expand_values(local_file_save_path, node);
  return local_file_save_path;
}

void IpCamera::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".input", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".video", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".audio", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".bitrate_video", rclcpp::ParameterValue("2M"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".bitrate_audio",
                                               rclcpp::ParameterValue("192k"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".segment", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".segment_time", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".ffmpeg_log_level",
                                               rclcpp::ParameterValue("info"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".ffmpeg_banner", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_path",
                                               rclcpp::ParameterValue("ffmpeg_%Y-%m-%dT%H:%M:%S"));

  node->get_parameter(measurement_name_ + ".input", input_);
  node->get_parameter(measurement_name_ + ".video", video_);
  node->get_parameter(measurement_name_ + ".audio", audio_);
  node->get_parameter(measurement_name_ + ".bitrate_video", bitrate_video_);
  node->get_parameter(measurement_name_ + ".bitrate_audio", bitrate_audio_);
  node->get_parameter(measurement_name_ + ".segment", segment_);
  node->get_parameter(measurement_name_ + ".segment_time", segment_time_);
  node->get_parameter(measurement_name_ + ".ffmpeg_log_level", ffmpeg_log_level_);
  node->get_parameter(measurement_name_ + ".ffmpeg_banner", ffmpeg_banner_);
  node->get_parameter(measurement_name_ + ".save_path", save_path_);

  // Validate parameters
  if (dc_util::stringMatchesRegex(save_path_, "^(?!%).+"))
  {
    throw std::runtime_error("Save path cannot start with a % character");
  }
  if (dc_util::stringMatchesRegex(bitrate_video_, "[0-9]+[kmKM]"))
  {
    throw std::runtime_error("Video birate must be an integer followed by k, K, m or M");
  }
  if (dc_util::stringMatchesRegex(bitrate_audio_, "[0-9]+[kmKM]"))
  {
    throw std::runtime_error("Audio birate must be an integer followed by k, K, m or M");
  }

  std::string absolute_path = getAbsolutePath("save_path");
  storage_dir_ = std::filesystem::path(absolute_path).parent_path().u8string();
  std::filesystem::create_directories(dc_util::expand_time(storage_dir_));
  std::string playlist_path = std::filesystem::path(storage_dir_) / "playlist.m3u8";
  RCLCPP_ERROR(logger_, "absolute_path:%s, storage_dir_: %s, save_path_:%s", absolute_path.c_str(),
               storage_dir_.c_str(), save_path_.c_str());

  // TODO do a healthcheck to ensure it's available.
  // Cameras don't support ICMP, see https://stackoverflow.com/questions/11502856/is-there-such-a-rtsp-ping
  std::string ffmpeg_path = "/usr/bin/ffmpeg";

  if (!std::filesystem::exists(ffmpeg_path))
  {
    throw std::runtime_error{ "ffmpeg is not installed!" };
  }

  std::string command = std::string(ffmpeg_path);

  if (!ffmpeg_banner_)
  {
    command += " -hide_banner";
  }

  command += " -loglevel " + ffmpeg_log_level_ + " -i " + input_;

  if (video_)
  {
    command += " -vcodec libx264 -x264-params keyint=1 -pix_fmt yuv420p -b:v " + bitrate_video_;
  }

  if (audio_)
  {
    command += " -acodec copy -b:a " + bitrate_audio_;
  }

  command += " -map 0";

  if (segment_)
  {
    command += " -strftime 1 -f hls -hls_time " + std::to_string(segment_time_) + " -hls_list_size " +
               std::to_string(segment_time_) + " -hls_flags delete_segments -hls_segment_filename";
  }

  command += " \"" + absolute_path + ".ts\" " + playlist_path;

  if (debug_)
  {
    RCLCPP_INFO(logger_, "FFMPEG command: '%s'", command.c_str());
  }

  av_log_set_level(AV_LOG_FATAL);

  // Start the ffmpeg command in a separate thread
  ffmpeg_thread_ = std::thread(
      [&](const std::string& command) {
        // Run the ffmpeg command
        if (std::system(command.c_str()) != 0)
        {
          RCLCPP_ERROR(logger_, "Error running command: %s", command.c_str());
        }
      },
      command);

  // Detach the thread so it runs in the background
  ffmpeg_thread_.detach();
}

void IpCamera::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "ip_camera.json");
  }
}

dc_interfaces::msg::StringStamped IpCamera::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;

  std::vector<std::string> recorded_videos;
  for (const auto& entry : std::filesystem::directory_iterator(storage_dir_))
  {
    std::string entry_path = entry.path().u8string();
    AVFormatContext* ifmt_ctx = nullptr;
    if (std::filesystem::path(entry_path).filename() != "playlist.m3u8" &&
        avformat_open_input(&ifmt_ctx, entry_path.c_str(), nullptr, nullptr) >= 0)
    {
      try
      {
        std::string collect_path = entry.path().u8string();
        collect_path.replace(collect_path.find("tmp/"), sizeof("tmp/") - 1, "");
        auto collect_dir = std::filesystem::path(collect_path).parent_path().u8string();
        std::filesystem::create_directories(collect_dir);
        std::filesystem::rename(entry_path, collect_path);
        data_json["local_path"] = collect_path;
        // Find the index of the '/' character that precedes the date and time string
        std::size_t index = collect_path.find_last_of('/');
        // Extract the substring starting from the '/' character
        std::string date_time_string = collect_path.substr(index + 1);
        // Find the index of the dot character
        std::size_t dot_index = date_time_string.find(".");
        // Remove the characters starting from the dot character
        date_time_string.erase(dot_index);

        data_json["timestamp"] = date_time_string;
        auto remote_path = collect_path;
        remote_path.replace(remote_path.find(save_local_base_path_expanded_), save_local_base_path_expanded_.size() - 1,
                            "");
        remote_path = remote_path.substr(1, remote_path.size() - 1);
        data_json["remote_path"] = remote_path;
        data_json["data_src"] = measurement_name_;
        msg.data = data_json.dump(-1, ' ', true);
        avformat_free_context(ifmt_ctx);
        return msg;
      }
      catch (std::filesystem::filesystem_error& e)
      {
        RCLCPP_ERROR(logger_, "Could not move file %s", entry_path.c_str());
      }
      avformat_free_context(ifmt_ctx);
    }
  }
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::IpCamera, dc_core::Measurement)
