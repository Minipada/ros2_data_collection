#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_

#include <math.h>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dc_conditions
{

class SameAsPrevious : public dc_conditions::Condition
{
public:
  SameAsPrevious();
  ~SameAsPrevious();

protected:
  std::vector<std::string> exclude_;
  std::vector<std::string> paths_;
  std::vector<std::string> paths_hash_;
  std::vector<std::string> previous_paths_hash_;
  std::string topic_;
  void dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg);
  void onConfigure();
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr subscription_;
  json previous_json_;
  bool file_hash_same_{ true };
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_
