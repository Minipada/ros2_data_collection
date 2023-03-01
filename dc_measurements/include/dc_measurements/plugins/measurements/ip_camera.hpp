
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__IP_CAMERA_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__IP_CAMERA_HPP_

#include <filesystem>

extern "C" {
#include <libavformat/avformat.h>
}

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace dc_measurements
{

class IpCamera : public dc_measurements::Measurement
{
public:
  IpCamera();
  ~IpCamera();
  dc_interfaces::msg::StringStamped collect() override;
  std::string getAbsolutePath(const std::string& param_reference);

private:
  std::string input_;
  bool video_;
  bool audio_;
  bool segment_;
  int segment_time_;
  std::string ffmpeg_log_level_;
  bool ffmpeg_banner_;
  std::string save_path_;
  bool print_ffmpeg_cmd_;
  std::string storage_dir_;
  std::thread ffmpeg_thread_;
  std::string bitrate_video_;
  std::string bitrate_audio_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void setValidationSchema() override;
  void onConfigure() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__IP_CAMERA_HPP_
