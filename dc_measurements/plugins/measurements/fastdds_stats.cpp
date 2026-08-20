// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/fastdds_stats.hpp"

#include <string>
#include <vector>

namespace dc_measurements
{

namespace
{
using eprosima::statistics_backend::DataKind;
using eprosima::statistics_backend::EntityId;
using eprosima::statistics_backend::EntityKind;
using eprosima::statistics_backend::StatisticKind;
using eprosima::statistics_backend::StatisticsBackend;
using eprosima::statistics_backend::StatisticsData;

double mean(const std::vector<StatisticsData>& data)
{
  double sum = 0.0;
  for (const auto& point : data)
  {
    sum += point.second;
  }
  return sum / static_cast<double>(data.size());
}

double sum(const std::vector<StatisticsData>& data)
{
  double total = 0.0;
  for (const auto& point : data)
  {
    total += point.second;
  }
  return total;
}

// Entities of `kind` discovered under `root` (the monitor's domain entity), reported by name --
// the "physical-layer data" the acceptance criteria asks for: which host/user/process is
// actually running each participant.
json namesOf(EntityKind kind, const EntityId& root)
{
  json names = json::array();
  for (const auto& id : StatisticsBackend::get_entities(kind, root))
  {
    // Info is a plain nlohmann::json object; every entity kind carries at least "name".
    names.push_back(StatisticsBackend::get_info(id).value("name", std::string()));
  }
  return names;
}
}  // namespace

FastddsStats::FastddsStats() : dc_measurements::Measurement()
{
}

FastddsStats::~FastddsStats() = default;

void FastddsStats::onConfigure()
{
  auto node = getNode();
  domain_id_ = static_cast<uint32_t>(dc_util::get_int_type_param(node, measurement_name_, "domain_id", 0));

  // One Monitor per Measurement instance, on this Measurement's own domain -- init_monitor()
  // returns the EntityId of the domain entity it starts tracking, which every entity query below
  // is scoped under.
  monitor_id_ = StatisticsBackend::init_monitor(static_cast<eprosima::statistics_backend::DomainId>(domain_id_));
  last_poll_ = std::chrono::system_clock::now();
}

void FastddsStats::onCleanup()
{
  if (monitor_id_.is_valid())
  {
    StatisticsBackend::stop_monitor(monitor_id_);
    monitor_id_.invalidate();
  }
}

void FastddsStats::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "fastdds_stats.json");
  }
}

dc_interfaces::msg::StringStamped FastddsStats::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;
  msg.header.stamp = node->get_clock()->now();

  const auto poll_end = std::chrono::system_clock::now();
  const auto poll_start = last_poll_;
  last_poll_ = poll_end;

  const auto participants = StatisticsBackend::get_entities(EntityKind::PARTICIPANT, monitor_id_);
  const auto datawriters = StatisticsBackend::get_entities(EntityKind::DATAWRITER, monitor_id_);
  const auto datareaders = StatisticsBackend::get_entities(EntityKind::DATAREADER, monitor_id_);

  json data;
  data["event"] = "sample";
  data["domain_id"] = domain_id_;
  data["participant_count"] = participants.size();
  data["datawriter_count"] = datawriters.size();
  data["datareader_count"] = datareaders.size();

  // Every DataKind query below is scoped to the window since the previous poll, so consecutive
  // Records report disjoint data rather than an ever-growing running average.
  if (!datawriters.empty() && !datareaders.empty())
  {
    const auto latency = StatisticsBackend::get_data(DataKind::FASTDDS_LATENCY, datawriters, datareaders, 1, poll_start,
                                                     poll_end, StatisticKind::MEAN);
    if (!latency.empty())
    {
      // Fast DDS's Statistics Module reports write-to-notification latency in nanoseconds.
      data["latency_ns_mean"] = mean(latency);
    }
  }
  if (!datawriters.empty())
  {
    const auto throughput = StatisticsBackend::get_data(DataKind::PUBLICATION_THROUGHPUT, datawriters, 1, poll_start,
                                                        poll_end, StatisticKind::MEAN);
    if (!throughput.empty())
    {
      data["publication_throughput_bytes_per_sec_mean"] = mean(throughput);
    }
  }
  if (!datareaders.empty())
  {
    const auto throughput = StatisticsBackend::get_data(DataKind::SUBSCRIPTION_THROUGHPUT, datareaders, 1, poll_start,
                                                        poll_end, StatisticKind::MEAN);
    if (!throughput.empty())
    {
      data["subscription_throughput_bytes_per_sec_mean"] = mean(throughput);
    }
  }
  if (!participants.empty())
  {
    const auto sent = StatisticsBackend::get_data(DataKind::RTPS_PACKETS_SENT, participants, 1, poll_start, poll_end,
                                                  StatisticKind::SUM);
    if (!sent.empty())
    {
      data["rtps_packets_sent"] = static_cast<int64_t>(sum(sent));
    }
    const auto lost = StatisticsBackend::get_data(DataKind::RTPS_PACKETS_LOST, participants, 1, poll_start, poll_end,
                                                  StatisticKind::SUM);
    if (!lost.empty())
    {
      data["rtps_packets_lost"] = static_cast<int64_t>(sum(lost));
    }
  }

  json participants_json = json::array();
  for (const auto& id : participants)
  {
    const auto info = StatisticsBackend::get_info(id);
    json entry;
    entry["name"] = info.value("name", std::string());
    entry["guid"] = info.value("guid", std::string());
    participants_json.push_back(entry);
  }
  data["participants"] = participants_json;

  data["hosts"] = namesOf(EntityKind::HOST, monitor_id_);
  data["users"] = namesOf(EntityKind::USER, monitor_id_);
  // Named process_names, not processes: cpu.cpp already owns a top-level "processes" column
  // (its total process count) in the shared dc table, and Vector's postgres sink maps a
  // Record's top-level JSON keys straight onto columns of the same name.
  data["process_names"] = namesOf(EntityKind::PROCESS, monitor_id_);

  msg.data = data.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::FastddsStats, dc_core::Measurement)
