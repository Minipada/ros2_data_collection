#ifndef DC_UTIL__STRING_UTILS_HPP_
#define DC_UTIL__STRING_UTILS_HPP_

#include <algorithm>
#include <boost/algorithm/hex.hpp>
#include <boost/uuid/detail/md5.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <regex>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_util
{
using boost::uuids::detail::md5;

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

std::string md5toString(const md5::digest_type& digest)
{
  const auto intDigest = reinterpret_cast<const int*>(&digest);
  std::string result;
  boost::algorithm::hex(intDigest, intDigest + (sizeof(md5::digest_type) / sizeof(int)), std::back_inserter(result));
  return result;
}

std::string getFileMd5(std::string file_path)
{
  std::string s;
  std::ifstream infile(file_path);
  std::string result;

  while (std::getline(infile, s))
  {
    md5 hash;
    md5::digest_type digest;

    hash.process_bytes(s.data(), s.size());
    hash.get_digest(digest);

    result = md5toString(digest);
  }

  return result;
}

std::string join(const std::vector<std::string>& vec, const std::string& delim = ", ")
{
  if (vec.empty())
  {
    return std::string();
  }

  return std::accumulate(std::next(vec.begin()), vec.end(), vec[0],
                         [&delim](const std::string& a, const std::string& b) { return a + delim + b; });
}

std::vector<std::string> split(const std::string& str, const std::string& regex_str = "/")
{
  std::regex regexz(regex_str);
  return { std::sregex_token_iterator(str.begin(), str.end(), regexz, -1), std::sregex_token_iterator() };
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

    text.replace(from.first, from.second, std::getenv(match[1].str().c_str()));
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
    text.replace(from.first, from.second, node->get_parameter(match[1].str().c_str()).as_string());
  }
  return text;
}

bool stringMatchesRegex(std::string regex, std::string value)
{
  const std::regex base_regex(regex);

  return std::regex_match(value, base_regex);
}

}  // namespace dc_util

#endif  // DC_UTIL__STRING_UTILS_HPP_
