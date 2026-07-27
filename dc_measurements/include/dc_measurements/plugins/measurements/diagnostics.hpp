#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DIAGNOSTICS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DIAGNOSTICS_HPP_

#include <string>
#include <vector>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Diagnostics : public dc_measurements::Measurement
{
public:
  Diagnostics();
  ~Diagnostics() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray& msg);
  uint8_t levelThresholdFromString(const std::string& level_threshold);

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  dc_interfaces::msg::StringStamped last_data_;
  std::string topic_;
  std::string level_threshold_str_;
  uint8_t level_threshold_{ 0 };
  std::vector<std::string> names_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DIAGNOSTICS_HPP_
