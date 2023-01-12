#ifndef DC_UTIL__FILESYSTEM_UTILS_HPP_
#define DC_UTIL__FILESYSTEM_UTILS_HPP_

#include <fstream>
#include <iostream>
#include <numeric>
#include <regex>
#include <vector>

namespace dc_util
{

std::string get_file_content(const std::string& path, const bool& strip_end_new_line = true)
{
  std::ifstream ifs(path);
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  if (strip_end_new_line)
  {
    if (!content.empty() && content[content.length() - 1] == '\n')
    {
      content.erase(content.length() - 1);
    }
  }

  return content;
}

void write_str_file(const std::string& path, const std::string& write_str, const bool& strip_end_new_line = true)
{
  std::string content = write_str;
  if (strip_end_new_line)
  {
    if (!content.empty() && content[content.length() - 1] == '\n')
    {
      content.erase(content.length() - 1);
    }
  }
  // Open the file
  std::ofstream w_file(path);
  // Write to the file
  w_file << content;
  // Close the file
  w_file.close();
}

}  // namespace dc_util

#endif  // DC_UTIL__FILESYSTEM_UTILS_HPP_
