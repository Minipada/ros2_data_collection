#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__STRING_MATCH_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__STRING_MATCH_HPP_

#include <regex>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class StringMatch : public dc_conditions::Condition
{
public:
  StringMatch();
  ~StringMatch();

protected:
  std::string key_;
  std::string regex_;
  bool getState(dc_interfaces::msg::StringStamped msg);
  void onConfigure();
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__STRING_MATCH_HPP_
