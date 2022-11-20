#ifndef DC_UTIL__STRING_UTILS_HPP_
#define DC_UTIL__STRING_UTILS_HPP_

#include <iostream>
#include <numeric>
#include <regex>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dc_util
{
std::string expand_env(std::string text)
{
  static const std::regex env_re{ R"--(\$\{?([[:alpha:]]\w+)\}?)--" };
  std::smatch match;
  while (std::regex_search(text, match, env_re))
  {
    auto const from = match[0];
    auto const var_name = match[1].str().c_str();
    text.replace(from.first, from.second, std::getenv(var_name));
  }
  return text;
}
}  // namespace dc_util

#endif  // DC_UTIL__STRING_UTILS_HPP_
