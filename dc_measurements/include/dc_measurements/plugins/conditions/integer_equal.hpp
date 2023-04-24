#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_EQUAL_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_EQUAL_HPP_

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class IntegerEqual : public dc_conditions::Condition
{
public:
  IntegerEqual();
  ~IntegerEqual() override;

protected:
  std::string key_;
  int value_;
  bool getState(dc_interfaces::msg::StringStamped msg) override;
  void onConfigure() override;
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_EQUAL_HPP_
