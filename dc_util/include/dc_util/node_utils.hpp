
#ifndef DC_UTIL__NODE_UTILS_HPP_
#define DC_UTIL__NODE_UTILS_HPP_

#include <algorithm>
#include <string>

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

template <typename NodeT>
std::vector<std::string> get_str_array_type_param(NodeT node, const std::string& plugin_name,
                                                  const std::string& param_name)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name, rclcpp::PARAMETER_STRING_ARRAY);
  std::vector<std::string> str_arr_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, str_arr_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return str_arr_param;
}

template <typename NodeT>
std::vector<std::string> get_str_array_type_param(NodeT node, const std::string& plugin_name,
                                                  const std::string& param_name,
                                                  const std::vector<std::string>& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name,
                                               rclcpp::ParameterValue(default_value));
  std::vector<std::string> str_arr_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, str_arr_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return str_arr_param;
}

template <typename NodeT>
std::string get_str_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name, rclcpp::PARAMETER_STRING);
  std::string str_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, str_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return str_param;
}

template <typename NodeT>
std::string get_str_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                               const std::string& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name,
                                               rclcpp::ParameterValue(default_value));
  std::string str_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, str_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return str_param;
}

template <typename NodeT>
bool get_bool_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name, rclcpp::PARAMETER_BOOL);
  bool bool_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, bool_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return bool_param;
}

template <typename NodeT>
bool get_bool_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                         const bool& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name,
                                               rclcpp::ParameterValue(default_value));
  bool bool_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, bool_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return bool_param;
}

template <typename NodeT>
int get_int_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name, rclcpp::PARAMETER_INTEGER);
  int int_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, int_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return int_param;
}

template <typename NodeT>
int get_int_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                       const int& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name,
                                               rclcpp::ParameterValue(default_value));
  int int_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, int_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return int_param;
}

template <typename NodeT>
int get_float_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name, rclcpp::PARAMETER_DOUBLE);
  float float_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, float_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return float_param;
}

template <typename NodeT>
int get_float_type_param(NodeT node, const std::string& plugin_name, const std::string& param_name,
                         const float& default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + "." + param_name,
                                               rclcpp::ParameterValue(default_value));
  float float_param;
  try
  {
    if (!node->get_parameter(plugin_name + "." + param_name, float_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Can not get '%s' param value for %s", param_name.c_str(), plugin_name.c_str());
      exit(-1);
    }
  }
  catch (rclcpp::exceptions::ParameterUninitializedException& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "'%s' param not defined for %s", param_name.c_str(), plugin_name.c_str());
    exit(-1);
  }

  return float_param;
}

}  // namespace dc_util

#endif  // DC_UTIL__NODE_UTILS_HPP_
