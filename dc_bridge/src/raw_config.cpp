// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/raw_config.hpp"

#include <sstream>

namespace dc_bridge
{

namespace
{

std::vector<std::regex> compile(const std::vector<std::string>& patterns, const char* what)
{
  std::vector<std::regex> compiled;
  compiled.reserve(patterns.size());
  for (const auto& pattern : patterns)
  {
    try
    {
      compiled.emplace_back(pattern, std::regex::ECMAScript | std::regex::optimize);
    }
    catch (const std::regex_error& e)
    {
      throw RawConfigError(std::string("raw.") + what + ": '" + pattern + "' is not a valid regex: " + e.what());
    }
  }
  return compiled;
}

bool matches_any(const std::vector<std::regex>& patterns, const std::string& value)
{
  for (const auto& pattern : patterns)
  {
    if (std::regex_search(value, pattern))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

std::vector<std::string> default_raw_exclude()
{
  return { "^/rosout$", "^/parameter_events$", "^/dc/measurement/", "^/dc/group/" };
}

std::vector<std::string> default_raw_exclude_types()
{
  return { "^sensor_msgs/msg/(Image|CompressedImage|PointCloud|PointCloud2|LaserScan)$", "^tf2_msgs/msg/TFMessage$" };
}

const char* to_string(RawSkip skip)
{
  switch (skip)
  {
    case RawSkip::Accepted:
      return "accepted";
    case RawSkip::NotIncluded:
      return "no include pattern matched";
    case RawSkip::Excluded:
      return "excluded by raw.exclude";
    case RawSkip::ExcludedType:
      return "excluded by raw.exclude_types";
  }
  return "unknown";
}

RawTopicFilter::RawTopicFilter(const RawConfig& config)
  : include_(compile(config.include, "include"))
  , exclude_(compile(config.exclude, "exclude"))
  , exclude_types_(compile(config.exclude_types, "exclude_types"))
{
}

RawSkip RawTopicFilter::evaluate(const std::string& topic, const std::string& type) const
{
  // An empty include list means "nothing", not "everything": raw mode's default include
  // is an explicit `^/` (see the parameter defaults), so an operator who deliberately
  // empties the list gets silence rather than the whole graph.
  if (!matches_any(include_, topic))
  {
    return RawSkip::NotIncluded;
  }
  if (matches_any(exclude_, topic))
  {
    return RawSkip::Excluded;
  }
  if (matches_any(exclude_types_, type))
  {
    return RawSkip::ExcludedType;
  }
  return RawSkip::Accepted;
}

std::string raw_tag(const std::string& tag_prefix, const std::string& topic)
{
  std::string out = tag_prefix;
  std::size_t i = 0;
  if (i < topic.size() && topic[i] == '/')
  {
    ++i;  // leading '/' is the namespace root, not a separator
  }
  for (; i < topic.size(); ++i)
  {
    const char c = topic[i];
    if (c == '/')
    {
      out.push_back('.');
    }
    else if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '.')
    {
      out.push_back(c);
    }
    else
    {
      out.push_back('_');
    }
  }
  return out;
}

RawRateLimiter::RawRateLimiter(double max_rate_hz)
{
  if (max_rate_hz <= 0.0)
  {
    min_interval_ = Clock::duration::zero();
    return;
  }
  const double seconds = 1.0 / max_rate_hz;
  min_interval_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

bool RawRateLimiter::allow(const std::string& topic, Clock::time_point now)
{
  if (unlimited())
  {
    return true;
  }
  auto [it, inserted] = last_pass_.emplace(topic, now);
  if (inserted)
  {
    return true;  // first message on this topic always passes
  }
  if (now - it->second < min_interval_)
  {
    return false;
  }
  it->second = now;
  return true;
}

std::string RawStats::summary() const
{
  std::ostringstream out;
  out << "raw: " << subscribed_topics.load() << " topic(s), " << forwarded.load() << " forwarded, dropped "
      << dropped_rate.load() << " rate / " << dropped_oversize.load() << " oversize / " << dropped_shipper.load()
      << " shipper / " << dropped_undecodable.load() << " undecodable";
  return out.str();
}

}  // namespace dc_bridge
