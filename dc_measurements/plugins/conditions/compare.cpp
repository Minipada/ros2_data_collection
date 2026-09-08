// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/conditions/compare.hpp"

#include <algorithm>
#include <stdexcept>

namespace dc_conditions
{

Compare::Compare() : dc_conditions::Condition()
{
}

void Compare::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");

  const std::string comparison = dc_util::get_str_type_param(node, condition_name_, "comparison");
  if (comparison == "eq")
  {
    op_ = Op::EQ;
  }
  else if (comparison == "ne")
  {
    op_ = Op::NE;
  }
  else if (comparison == "gt")
  {
    op_ = Op::GT;
  }
  else if (comparison == "ge")
  {
    op_ = Op::GE;
  }
  else if (comparison == "lt")
  {
    op_ = Op::LT;
  }
  else if (comparison == "le")
  {
    op_ = Op::LE;
  }
  else if (comparison == "match")
  {
    op_ = Op::MATCH;
  }
  else if (comparison == "exists")
  {
    op_ = Op::EXISTS;
  }
  else
  {
    throw std::runtime_error("Condition '" + condition_name_ + "': unknown comparison '" + comparison +
                             "' (expected eq, ne, gt, ge, lt, le, match or exists)");
  }

  if (op_ == Op::EXISTS)
  {
    return;
  }
  if (op_ == Op::MATCH)
  {
    // Compiled once here so a bad regex fails configure instead of throwing on every Record.
    regex_ = std::regex(dc_util::get_str_type_param(node, condition_name_, "regex"));
    return;
  }

  // `value` has no fixed parameter type -- the type the user supplies IS the operand type, so it
  // cannot go through dc_util's typed helpers. Declared from the node's parameter override when
  // the caller (a test) has not declared it itself.
  const std::string value_name = condition_name_ + ".value";
  if (!node->has_parameter(value_name))
  {
    const auto& overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    const auto it = overrides.find(value_name);
    if (it == overrides.end())
    {
      throw std::runtime_error("Condition '" + condition_name_ + "': comparison '" + comparison +
                               "' requires a 'value' parameter");
    }
    node->declare_parameter(value_name, it->second);
  }
  const rclcpp::Parameter value = node->get_parameter(value_name);

  switch (value.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      operand_ = Operand::BOOL;
      value_bool_ = value.as_bool();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      operand_ = Operand::INT;
      value_int_ = value.as_int();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      operand_ = Operand::DOUBLE;
      value_double_ = value.as_double();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      operand_ = Operand::STRING;
      value_string_ = value.as_string();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      operand_ = Operand::BOOL_ARRAY;
      value_bool_array_ = value.as_bool_array();
      order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      operand_ = Operand::INT_ARRAY;
      value_int_array_ = value.as_integer_array();
      order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      operand_ = Operand::DOUBLE_ARRAY;
      value_double_array_ = value.as_double_array();
      order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      operand_ = Operand::STRING_ARRAY;
      value_string_array_ = value.as_string_array();
      order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
      break;
    default:
      throw std::runtime_error("Condition '" + condition_name_ + "': 'value' parameter has no usable type");
  }

  // gt/ge/lt/le need an orderable operand: only the numeric scalars ever were, in the collapsed
  // plugins.
  const bool ordered = (op_ == Op::GT || op_ == Op::GE || op_ == Op::LT || op_ == Op::LE);
  const bool orderable = (operand_ == Operand::INT || operand_ == Operand::DOUBLE);
  if (ordered && !orderable)
  {
    throw std::runtime_error("Condition '" + condition_name_ + "': comparison '" + comparison +
                             "' needs an int or double 'value'");
  }
}

bool Compare::getState(dc_interfaces::msg::StringStamped msg)
{
  try
  {
    json data_json = json::parse(msg.data);

    if (op_ == Op::EXISTS)
    {
      json flat_json = data_json.flatten();
      std::string key_exact = std::string("/") + key_;
      std::string key_w_prefix = key_exact + "/";

      // A flattened key matches "key_" either exactly (key_ is itself a scalar leaf, e.g.
      // {"level": 5.5} with key_="level" flattens to exactly "/level") or as a "key_/..." prefix
      // (key_ names an object/array with descendants). Checking the prefix alone misses the
      // exact-match case entirely, since flatten() never emits "/level/" for a leaf value.
      bool found = false;
      for (auto& x : flat_json.items())
      {
        if (x.key() == key_exact || x.key().rfind(key_w_prefix, 0) == 0)
        {
          found = true;
          break;
        }
      }
      active_ = found;
      publishActive();
      return active_;
    }

    if (op_ == Op::MATCH)
    {
      json flat_json = data_json.flatten();
      std::string key_w_prefix = std::string("/") + key_;

      if (!flat_json.contains(key_w_prefix))
      {
        RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
        active_ = false;
        publishActive();
        return active_;
      }

      active_ = std::regex_match(flat_json[key_w_prefix].get<std::string>(), regex_);
      publishActive();
      return active_;
    }

    if (operand_ == Operand::BOOL_ARRAY || operand_ == Operand::INT_ARRAY || operand_ == Operand::DOUBLE_ARRAY ||
        operand_ == Operand::STRING_ARRAY)
    {
      // json::json_pointer navigates the unflattened data_json directly: flatten() explodes
      // arrays into indexed keys ("/key/0", "/key/1", ...), so an array-valued "/key" would
      // never be found by that pattern -- exactly the value type compared here.
      json::json_pointer key_ptr(std::string("/") + key_);

      if (!data_json.contains(key_ptr))
      {
        RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
        active_ = false;
        publishActive();
        return active_;
      }

      const json& field = data_json.at(key_ptr);

      if (field.type() != json::value_t::array)
      {
        RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not an array");
        active_ = false;
        publishActive();
        return active_;
      }

      bool elements_ok = false;
      switch (operand_)
      {
        case Operand::BOOL_ARRAY:
          elements_ok = std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_boolean(); });
          break;
        case Operand::INT_ARRAY:
          elements_ok = std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_number_integer(); });
          break;
        case Operand::DOUBLE_ARRAY:
          elements_ok = std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_number_float(); });
          break;
        case Operand::STRING_ARRAY:
          elements_ok = std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_string(); });
          break;
        default:
          break;
      }
      if (!elements_ok)
      {
        RCLCPP_WARN_STREAM(logger_, "All values are not of the expected type in key " << key_);
        active_ = false;
        publishActive();
        return active_;
      }

      switch (operand_)
      {
        case Operand::BOOL_ARRAY:
        {
          std::vector<bool> data = field.get<std::vector<bool>>();
          if (order_matters_)
          {
            active_ = (data == value_bool_array_);
          }
          else
          {
            // vector<bool> has no < to sort on; compare as ints, as ListBoolEqual did.
            std::vector<int> data_int(data.begin(), data.end());
            std::vector<int> value_int(value_bool_array_.begin(), value_bool_array_.end());
            std::sort(data_int.begin(), data_int.end());
            std::sort(value_int.begin(), value_int.end());
            active_ = (data_int == value_int);
          }
          break;
        }
        case Operand::INT_ARRAY:
        {
          std::vector<int64_t> data = field.get<std::vector<int64_t>>();
          if (!order_matters_)
          {
            std::sort(data.begin(), data.end());
            auto value_sorted = value_int_array_;
            std::sort(value_sorted.begin(), value_sorted.end());
            active_ = (data == value_sorted);
          }
          else
          {
            active_ = (data == value_int_array_);
          }
          break;
        }
        case Operand::DOUBLE_ARRAY:
        {
          std::vector<double> data = field.get<std::vector<double>>();
          if (!order_matters_)
          {
            std::sort(data.begin(), data.end());
            auto value_sorted = value_double_array_;
            std::sort(value_sorted.begin(), value_sorted.end());
            active_ = (data == value_sorted);
          }
          else
          {
            active_ = (data == value_double_array_);
          }
          break;
        }
        case Operand::STRING_ARRAY:
        {
          std::vector<std::string> data = field.get<std::vector<std::string>>();
          if (!order_matters_)
          {
            std::sort(data.begin(), data.end());
            auto value_sorted = value_string_array_;
            std::sort(value_sorted.begin(), value_sorted.end());
            active_ = (data == value_sorted);
          }
          else
          {
            active_ = (data == value_string_array_);
          }
          break;
        }
        default:
          break;
      }
      if (op_ == Op::NE)
      {
        active_ = !active_;
      }
      publishActive();
      return active_;
    }

    json flat_json = data_json.flatten();
    std::string key_w_prefix = std::string("/") + key_;

    if (!flat_json.contains(key_w_prefix))
    {
      RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
      active_ = false;
      publishActive();
      return active_;
    }

    const json& field = flat_json[key_w_prefix];

    // Type-strict by operand kind: an integer JSON literal is not a double operand and vice
    // versa, matching the collapsed plugins. is_number_integer(), not a strict type()==
    // number_integer check: nlohmann::json parses non-negative integer literals (the common
    // case) as number_unsigned, and is_number_integer() is the one that correctly treats both
    // as "an integer".
    switch (operand_)
    {
      case Operand::BOOL:
        if (field.type() != json::value_t::boolean)
        {
          RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not a boolean");
          active_ = false;
          publishActive();
          return active_;
        }
        break;
      case Operand::INT:
        if (!field.is_number_integer())
        {
          RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not an integer");
          active_ = false;
          publishActive();
          return active_;
        }
        break;
      case Operand::DOUBLE:
        if (field.type() != json::value_t::number_float)
        {
          RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not a double");
          active_ = false;
          publishActive();
          return active_;
        }
        break;
      case Operand::STRING:
        if (field.type() != json::value_t::string)
        {
          RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not a string");
          active_ = false;
          publishActive();
          return active_;
        }
        break;
      default:
        break;
    }

    switch (op_)
    {
      case Op::EQ:
        switch (operand_)
        {
          case Operand::BOOL:
            active_ = (field == value_bool_);
            break;
          case Operand::INT:
            active_ = (field == value_int_);
            break;
          case Operand::DOUBLE:
            active_ = (field == value_double_);
            break;
          case Operand::STRING:
            active_ = (field == value_string_);
            break;
          default:
            break;
        }
        break;
      case Op::NE:
        switch (operand_)
        {
          case Operand::BOOL:
            active_ = (field != value_bool_);
            break;
          case Operand::INT:
            active_ = (field != value_int_);
            break;
          case Operand::DOUBLE:
            active_ = (field != value_double_);
            break;
          case Operand::STRING:
            active_ = (field != value_string_);
            break;
          default:
            break;
        }
        break;
      case Op::GT:
        active_ = (operand_ == Operand::INT) ? (field > value_int_) : (field > value_double_);
        break;
      case Op::GE:
        active_ = (operand_ == Operand::INT) ? (field >= value_int_) : (field >= value_double_);
        break;
      case Op::LT:
        active_ = (operand_ == Operand::INT) ? (field < value_int_) : (field < value_double_);
        break;
      case Op::LE:
        active_ = (operand_ == Operand::INT) ? (field <= value_int_) : (field <= value_double_);
        break;
      default:
        break;
    }
    publishActive();
    return active_;
  }
  catch (json::parse_error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON (compare): " << msg.data);
    active_ = false;
    publishActive();
    return active_;
  }
}

Compare::~Compare() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::Compare, dc_core::Condition)
