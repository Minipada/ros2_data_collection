// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__COMPARE_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__COMPARE_HPP_

#include <cstdint>
#include <regex>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/condition.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/string_utils.hpp"

namespace dc_conditions
{

// One plugin for the whole compare family (#486): `comparison` picks the operator and the type
// the user gives `value` picks the operand type -- the two axes the collapsed plugins each
// hard-coded.
class Compare : public dc_conditions::Condition
{
public:
  Compare();
  ~Compare() override;

protected:
  std::string key_;
  bool getState(dc_interfaces::msg::StringStamped msg) override;
  void onConfigure() override;

private:
  enum class Op
  {
    EQ,
    NE,
    GT,
    GE,
    LT,
    LE,
    MATCH,
    EXISTS
  };
  enum class Operand
  {
    NONE,
    BOOL,
    INT,
    DOUBLE,
    STRING,
    BOOL_ARRAY,
    INT_ARRAY,
    DOUBLE_ARRAY,
    STRING_ARRAY
  };

  // The comparisons are type-strict, so every operand kind keeps its own typed member and the
  // others stay unread.
  bool value_bool_{ false };
  int64_t value_int_{ 0 };
  double value_double_{ 0.0 };
  std::string value_string_;
  std::vector<bool> value_bool_array_;
  std::vector<int64_t> value_int_array_;
  std::vector<double> value_double_array_;
  std::vector<std::string> value_string_array_;

  Op op_{ Op::EQ };
  Operand operand_{ Operand::NONE };
  std::regex regex_;
  bool order_matters_{ true };
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__COMPARE_HPP_
