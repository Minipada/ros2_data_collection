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

const char* boolToString(bool b)
{
  return b ? "true" : "false";
}

char* convert(const std::string& s)
{
  char* pc = new char[s.size() + 1];
  std::strcpy(pc, s.c_str());
  return pc;
}

std::string join(const std::vector<std::string>& vec, const std::string delim = ", ")
{
  if (vec.empty())
  {
    return std::string();
  }

  return std::accumulate(std::next(vec.begin()), vec.end(), vec[0],
                         [&delim](const std::string& a, const std::string& b) { return a + delim + b; });
}

std::string to_space_separated_string(std::vector<std::string> string_array)
{
  std::vector<char*> vc;
  std::transform(string_array.begin(), string_array.end(), std::back_inserter(vc), convert);
  std::string formatted_string;
  for (size_t i = 0; i < vc.size(); i++)
  {
    formatted_string += vc[i];
    if (i != vc.size() - 1)
    {
      formatted_string += " ";
    }
  }
  return formatted_string;
}

std::string expand_time(const std::string& s)
{
  auto now = rclcpp::Clock().now();
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
      now.nanoseconds()) };
  std::time_t newt = std::chrono::system_clock::to_time_t(t);
  std::stringstream trans_time;

  trans_time << std::put_time(localtime(&newt), s.c_str());
  std::string fmt_time = trans_time.str();

  return fmt_time;
}

std::string expand_time(const std::string& s, const rclcpp::Time& now)
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
      now.nanoseconds()) };
  std::time_t newt = std::chrono::system_clock::to_time_t(t);
  std::stringstream trans_time;

  trans_time << std::put_time(localtime(&newt), s.c_str());
  std::string fmt_time = trans_time.str();

  return fmt_time;
}

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

template <typename NodeT>
std::string expand_values(std::string text, NodeT node)
{
  static const std::regex env_re{ R"--(\=\{?([[:alpha:]]\w+)\}?)--" };
  std::smatch match;
  while (std::regex_search(text, match, env_re))
  {
    auto const from = match[0];
    auto const var_name = match[1].str().c_str();
    text.replace(from.first, from.second, node->get_parameter(var_name).as_string());
  }
  return text;
}

}  // namespace dc_util

#endif  // DC_UTIL__STRING_UTILS_HPP_
