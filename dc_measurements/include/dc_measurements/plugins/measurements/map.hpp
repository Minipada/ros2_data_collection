#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MAP_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MAP_HPP_

#include <filesystem>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Map : public dc_measurements::Measurement
{
public:
  Map();
  ~Map() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  std::string saveMap();
  std::string getAbsolutePath(const std::string& param_reference, const rclcpp::Time& now);
  std::string getLocalPath(const std::string& param_reference, const rclcpp::Time& now);

  int getMapSize(const std::string& path_map);
  void saveRemoteKeys(json& data_json, const std::string& key, const std::string& relative_path,
                      const rclcpp::Time& now);
  std::string map_topic_;
  std::string save_path_;
  float save_map_timeout_;
  bool quiet_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MAP_HPP_
