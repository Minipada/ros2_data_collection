#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__LIST_STRING_EQUAL_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__LIST_STRING_EQUAL_HPP_

#include <algorithm>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_conditions
{

class ListStringEqual : public dc_conditions::Condition
{
public:
  ListStringEqual();
  ~ListStringEqual() override;

protected:
  std::string key_;
  std::vector<std::string> value_;
  bool order_matters_;
  bool getState(dc_interfaces::msg::StringStamped msg) override;
  void onConfigure() override;
  static bool compareFunction(const std::string& a, const std::string& b);
  void sortStringVector(std::vector<std::string>& str_vector);
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__LIST_STRING_EQUAL_HPP_
