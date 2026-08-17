// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/topic_config.hpp"

namespace dc_bridge
{

std::string TopicConfig::derive_tag(const std::string& topic)
{
  std::string stripped = topic;
  if (!stripped.empty() && stripped.front() == '/')
  {
    stripped.erase(stripped.begin());
  }
  if (stripped.empty())
  {
    return "dc";
  }
  for (char& c : stripped)
  {
    if (c == '/')
    {
      c = '.';
    }
  }
  return stripped;
}

TopicConfig TopicConfig::make(const std::string& topic, std::optional<std::string> tag)
{
  return TopicConfig{ topic, tag.has_value() ? *tag : derive_tag(topic) };
}

}  // namespace dc_bridge
