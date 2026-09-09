// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/record_dispatch.hpp"

#include <algorithm>
#include <set>
#include <utility>

#include "dc_bridge/topic_config.hpp"

namespace dc_bridge
{

nlohmann::json RecordDispatcher::parse_payload(const std::string& data)
{
  try
  {
    return nlohmann::json::parse(data);
  }
  catch (const nlohmann::json::exception&)
  {
    return nlohmann::json(data);
  }
}

RecordDispatcher::RecordDispatcher(Topics topics, Deps deps) : deps_(std::move(deps))
{
  std::set<std::string> all;
  all.insert(topics.records.begin(), topics.records.end());
  all.insert(topics.files.begin(), topics.files.end());
  for (const auto& topic : all)
  {
    Routing routing;
    routing.tag = TopicConfig::derive_tag(topic);
    routing.forward = std::find(topics.records.begin(), topics.records.end(), topic) != topics.records.end();
    routing.enqueue_intent = std::find(topics.files.begin(), topics.files.end(), topic) != topics.files.end();
    routing_by_topic_.emplace(topic, std::move(routing));
    subscribed_topics_.push_back(topic);
  }
}

void RecordDispatcher::dispatch(const IncomingRecord& incoming) const
{
  const auto it = routing_by_topic_.find(incoming.topic);
  if (it == routing_by_topic_.end())
  {
    // Only topics from subscribed_topics() are ever subscribed; anything else has no
    // Tag and no Destination to reach.
    return;
  }
  const Routing& routing = it->second;

  nlohmann::json payload = parse_payload(incoming.data);

  if (routing.enqueue_intent)
  {
    // Durable enqueue (#265): the intent lands on disk before this returns, so it
    // survives a Bridge crash/restart (or the separate dc_uploader process, #446, never
    // having been up at all). dc_uploader discovers it by rescanning the queue directory;
    // there is no in-process wake-up signal to a different OS process.
    deps_.intent_queue->enqueue(routing.tag, payload);
  }

  if (!routing.forward)
  {
    return;
  }

  Record record;
  record.tag = routing.tag;
  record.timestamp_secs = static_cast<std::uint64_t>(std::max<std::int32_t>(0, incoming.stamp_secs));
  record.timestamp_nanos = incoming.stamp_nanos;
  record.payload = std::move(payload);
  try
  {
    std::lock_guard<std::mutex> lock(*deps_.forwarder_mutex);
    deps_.forwarder->send(record);
  }
  catch (const ForwarderError& e)
  {
    if (deps_.on_warning)
    {
      deps_.on_warning("failed to forward record on tag '" + routing.tag + "': " + e.what());
    }
  }
}

}  // namespace dc_bridge
