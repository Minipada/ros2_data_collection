#include "dc_bridge/vector_binary.hpp"

#include <filesystem>
#include <sstream>

namespace dc_bridge
{

std::optional<std::string> find_vector_binary(const std::string& ament_prefix_path)
{
  std::stringstream ss(ament_prefix_path);
  std::string prefix;
  while (std::getline(ss, prefix, ':'))
  {
    if (prefix.empty())
    {
      continue;
    }
    std::filesystem::path candidate = std::filesystem::path(prefix) / "lib" / "vector_vendor" / "vector";
    std::error_code ec;
    if (std::filesystem::is_regular_file(candidate, ec))
    {
      return candidate.string();
    }
  }
  return std::nullopt;
}

}  // namespace dc_bridge
