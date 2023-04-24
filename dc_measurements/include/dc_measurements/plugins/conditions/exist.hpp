#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__EXIST_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__EXIST_HPP_

#include <math.h>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class Exist : public dc_conditions::Condition
{
public:
  Exist();
  ~Exist() override;

protected:
  std::string key_;
  bool getState(dc_interfaces::msg::StringStamped msg) override;
  void onConfigure() override;
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__EXIST_HPP_
