
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"

namespace dc_measurements
{

class DrivingType : public dc_measurements::Measurement
{
public:
  DrivingType();
  ~DrivingType() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void modeTopicCb(const std_msgs::msg::String& msg);
  void velocitySourceCb(const std::string& mode);

  // Shape 1: a dedicated topic carrying a raw mode value, mapped through value_mapping_.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  std::string mode_topic_;
  std::vector<std::string> value_mapping_from_;
  std::vector<std::string> value_mapping_to_;
  std::map<std::string, std::string> value_mapping_;

  // Shape 2: infer the mode from whichever velocity source last published.
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> velocity_subscriptions_;
  std::vector<std::string> velocity_topics_;
  std::vector<std::string> velocity_modes_;
  double velocity_timeout_s_;
  bool velocity_observed_{ false };
  int64_t last_velocity_time_ns_{ 0 };

  std::string current_mode_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_
