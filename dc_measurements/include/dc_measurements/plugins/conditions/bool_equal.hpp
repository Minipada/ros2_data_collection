#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__BOOL_EQUAL_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__BOOL_EQUAL_HPP_

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class BoolEqual : public dc_conditions::Condition
{
public:
  BoolEqual();
  ~BoolEqual();

protected:
  std::string key_;
  double value_;
  bool getState(dc_interfaces::msg::StringStamped msg);
  void onConfigure();
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__BOOL_EQUAL_HPP_
