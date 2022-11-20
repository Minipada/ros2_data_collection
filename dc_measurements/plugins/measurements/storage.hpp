
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STORAGE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STORAGE_HPP_

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
namespace dc_measurements
{

class Storage : public dc_measurements::Measurement
{
public:
  Storage();
  ~Storage();
  dc_interfaces::msg::StringStamped collect() override;

private:
  std::string path_;
  std::string full_path_;
  std::string env_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STORAGE_HPP_
