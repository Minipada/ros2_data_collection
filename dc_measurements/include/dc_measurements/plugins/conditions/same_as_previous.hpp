#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_

#include <math.h>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class SameAsPrevious : public dc_conditions::Condition
{
public:
  SameAsPrevious();
  ~SameAsPrevious();

protected:
  std::vector<std::string> exclude_;
  std::vector<std::string> keys_;
  std::vector<std::string> keys_hash_;
  std::vector<std::string> previous_keys_hash_;
  std::string topic_;
  bool getState(dc_interfaces::msg::StringStamped msg);
  void onConfigure();
  json previous_json_;
  bool file_hash_same_{ true };
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__SAME_AS_PREVIOUS_HPP_
