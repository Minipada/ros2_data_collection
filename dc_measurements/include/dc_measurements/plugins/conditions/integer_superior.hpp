// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_SUPERIOR_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_SUPERIOR_HPP_

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/string_utils.hpp"

namespace dc_conditions
{

class IntegerSuperior : public dc_conditions::Condition
{
public:
  IntegerSuperior();
  ~IntegerSuperior() override;

protected:
  std::string key_;
  int value_;
  bool include_value_;
  bool getState(dc_interfaces::msg::StringStamped msg) override;
  void onConfigure() override;
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__INTEGER_SUPERIOR_HPP_
