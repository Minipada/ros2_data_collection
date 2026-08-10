
#ifndef DC_UTIL__NODE_UTILS_HPP_
#define DC_UTIL__NODE_UTILS_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_util
{

template <typename T>
std::vector<T> remove_duplicates(std::vector<T> v)
{
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
  return v;
}

template <typename T>
std::vector<T> flatten(std::vector<std::vector<T>> const& vec)
{
  std::vector<T> flattened;
  for (auto const& v : vec)
  {
    flattened.insert(flattened.end(), v.begin(), v.end());
  }
  return flattened;
}

namespace detail
{

// Shared implementation behind every dc_util::get_*_type_param()/get_*_param() helper below:
// declare `full_name` via nav2_util's idempotent declare-if-not-declared, then read it back,
// logging and exiting fatally if the value can't be retrieved. This is the single place in
// dc_util (and, by convention, in dc_measurements/dc_group) that talks to nav2_util directly —
// see docs/adr/0008-dc-util-owns-parameter-declaration.md.
template <typename T, typename NodeT>
T get_param_or_fatal(NodeT node, const std::string& full_name)
{
  T value{};
  try
  {
    if (!node->get_parameter(full_name, value))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value", full_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined", full_name.c_str());
    exit(-1);
  }

  return value;
}

template <typename T, typename NodeT>
T declare_and_get(NodeT node, const std::string& full_name, const rclcpp::ParameterType& param_type)
{
  nav2_util::declare_parameter_if_not_declared(node, full_name, param_type);
  return get_param_or_fatal<T>(node, full_name);
}

template <typename T, typename NodeT>
T declare_and_get(NodeT node, const std::string& full_name, const T& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, full_name, rclcpp::ParameterValue(default_value));
  return get_param_or_fatal<T>(node, full_name);
}

}  // namespace detail

// ---------------------------------------------------------------------------------------------
// Plugin-scoped helpers: declare and read "<plugin_name>.<param_name>". This is the sanctioned
// way for a Measurement/Condition plugin to declare its own parameters — see
// doc/src/dc/contributing.md "Declaring plugin parameters".
// ---------------------------------------------------------------------------------------------

template <typename NodeT>
std::string get_str_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<std::string>(node, plugin_name + "." + param_name, rclcpp::PARAMETER_STRING);
}

template <typename NodeT>
std::string get_str_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                               const std::string& default_value)
{
  return detail::declare_and_get<std::string>(node, plugin_name + "." + param_name, default_value);
}

template <typename NodeT>
std::vector<std::string> get_str_array_type_param(NodeT node, const std::string& plugin_name,
                                                  const std::string& param_name)
{
  return detail::declare_and_get<std::vector<std::string>>(node, plugin_name + "." + param_name,
                                                           rclcpp::PARAMETER_STRING_ARRAY);
}

template <typename NodeT>
std::vector<std::string> get_str_array_type_param(NodeT node, const std::string& plugin_name,
                                                  const std::string& param_name,
                                                  const std::vector<std::string>& default_value)
{
  return detail::declare_and_get<std::vector<std::string>>(node, plugin_name + "." + param_name, default_value);
}

template <typename NodeT>
bool get_bool_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<bool>(node, plugin_name + "." + param_name, rclcpp::PARAMETER_BOOL);
}

template <typename NodeT>
bool get_bool_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                         const bool& default_value)
{
  return detail::declare_and_get<bool>(node, plugin_name + "." + param_name, default_value);
}

template <typename NodeT>
std::vector<bool> get_bool_array_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<std::vector<bool>>(node, plugin_name + "." + param_name, rclcpp::PARAMETER_BOOL_ARRAY);
}

template <typename NodeT>
int get_int_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<int>(node, plugin_name + "." + param_name, rclcpp::PARAMETER_INTEGER);
}

template <typename NodeT>
int get_int_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                       const int& default_value)
{
  return detail::declare_and_get<int>(node, plugin_name + "." + param_name, default_value);
}

template <typename NodeT>
std::vector<int64_t> get_int_array_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<std::vector<int64_t>>(node, plugin_name + "." + param_name,
                                                       rclcpp::PARAMETER_INTEGER_ARRAY);
}

template <typename NodeT>
double get_double_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  return detail::declare_and_get<double>(node, plugin_name + "." + param_name, rclcpp::PARAMETER_DOUBLE);
}

template <typename NodeT>
double get_double_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                             const double& default_value)
{
  return detail::declare_and_get<double>(node, plugin_name + "." + param_name, default_value);
}

template <typename NodeT>
std::vector<double> get_double_array_type_param(NodeT node, const std::string& plugin_name,
                                                const std::string& param_name)
{
  return detail::declare_and_get<std::vector<double>>(node, plugin_name + "." + param_name,
                                                      rclcpp::PARAMETER_DOUBLE_ARRAY);
}

// ---------------------------------------------------------------------------------------------
// Node-level helpers: declare and read a top-level, unprefixed parameter name (e.g.
// "measurement_plugins", not "<something>.measurement_plugins"). Distinct names from the
// plugin-scoped helpers above, rather than overloads, since a 3-argument
// (node, plugin_name, param_name) call and a 3-argument (node, param_name, default_value) call
// are otherwise indistinguishable when both a plugin_name and a default happen to be strings.
// ---------------------------------------------------------------------------------------------

template <typename NodeT>
std::string get_str_param(NodeT node, const std::string& param_name, const std::string& default_value)
{
  return detail::declare_and_get<std::string>(node, param_name, default_value);
}

template <typename NodeT>
std::vector<std::string> get_str_array_param(NodeT node, const std::string& param_name,
                                             const std::vector<std::string>& default_value)
{
  return detail::declare_and_get<std::vector<std::string>>(node, param_name, default_value);
}

}  // namespace dc_util

#endif  // DC_UTIL__NODE_UTILS_HPP_
