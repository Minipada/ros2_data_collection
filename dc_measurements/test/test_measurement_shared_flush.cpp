// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <map>
#include <string>
#include <vector>

#include "dc_interfaces/msg/flush_event.hpp"
#include "measurement_test_bench.hpp"

// A non-default topic, so the test also covers several Measurements being pointed at a shared
// flush topic of the deployment's choosing.
namespace
{
const char* const kFlushTopic = "/dc/incident_flush";
const char* const kFast = "dummy_fast";
const char* const kSlow = "dummy_slow";
const char* const kLive = "dummy_live";
const int kFastPollingMs = 50;
const int kSlowPollingMs = 150;
}  // namespace

// The fan-out half of the flush contract (#345): one FlushEvent, several Measurements, one
// incident_id. Two Measurements buffer on the same flush topic at different polling rates; a
// third has no buffer_duration_sec at all and so subscribes to nothing.
class MeasurementSharedFlushTest : public MeasurementBench
{
protected:
  MeasurementSharedFlushTest() : MeasurementBench(std::vector<std::string>{ kFast, kSlow, kLive })
  {
    flush_pub_ = ms_node_->create_publisher<dc_interfaces::msg::FlushEvent>(kFlushTopic, rclcpp::QoS(10));

    declareMeasurement(kFast, kFastPollingMs);
    declareMeasurement(kSlow, kSlowPollingMs);
    declareMeasurement(kLive, kFastPollingMs);

    for (const std::string& name : { std::string(kFast), std::string(kSlow) })
    {
      ms_node_->declare_parameter(name + ".buffer_duration_sec", 10.0);
      ms_node_->declare_parameter(name + ".flush_topic", std::string(kFlushTopic));
      // data_pub_ is KeepLast(1): pacing the release keeps every Record of the window deliverable
      // rather than only the last one of a synchronous burst (#289).
      ms_node_->declare_parameter(name + ".max_flush_rate_hz", 20.0);
    }
  }

  // Raw Record strings per Measurement, in arrival order -- buffering shapes the *content* of
  // each released Record, which is what the assertions inspect.
  void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg) override
  {
    received_[measurement].push_back(msg.data);
    incident_ids_[measurement].push_back(msg.incident_id);
  }

  void declareMeasurement(const std::string& name, const int polling_interval)
  {
    const std::string topic = "/dc/measurement/" + name;
    ms_node_->declare_parameter(name + ".plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter(name + ".topic_output", topic);
    ms_node_->declare_parameter(name + ".record", nlohmann::json({ { "message", name } }).dump());
    ms_node_->declare_parameter(name + ".polling_interval", polling_interval);
  }

  void publishFlush(const std::string& incident_id)
  {
    dc_interfaces::msg::FlushEvent flush_msg;
    flush_msg.incident_id = incident_id;
    flush_pub_->publish(flush_msg);
  }

  // How many of this Measurement's Records carry exactly this incident_id on the envelope.
  int countTaggedWith(const std::string& name, const std::string& incident_id)
  {
    int count = 0;
    for (const auto& id : incident_ids_[name])
    {
      if (id == incident_id)
      {
        count++;
      }
    }
    return count;
  }

  // Every Record this Measurement published so far, for a failure message that names names.
  std::string dumpOf(const std::string& name)
  {
    std::string out;
    for (const auto& data : received_[name])
    {
      out += "\n  " + data;
    }
    return out;
  }

  // How many of this Measurement's Records carry an incident_id at all, whatever its value.
  int countCarryingAnIncident(const std::string& name)
  {
    int count = 0;
    for (const auto& id : incident_ids_[name])
    {
      if (!id.empty())
      {
        count++;
      }
    }
    return count;
  }

  rclcpp::Publisher<dc_interfaces::msg::FlushEvent>::SharedPtr flush_pub_;

public:
  std::map<std::string, std::vector<std::string>> received_;
  std::map<std::string, std::vector<std::string>> incident_ids_;
};

// Acceptance criterion: a single FlushEvent releases the buffered window of every Measurement
// listening on the shared topic, and every Record of both windows carries the same incident_id --
// time-correlated context across streams, not just one. Their polling intervals differ, so the
// windows hold different Record counts under that one id.
TEST_F(MeasurementSharedFlushTest, OneFlushEventReleasesEveryBufferedWindowUnderOneIncidentId)
{
  startLifecycleNode();

  // Both buffer silently over the same wall-clock window, each at its own rate.
  spinFor(600);
  ASSERT_TRUE(received_[kFast].empty()) << kFast << " published while buffering:" << dumpOf(kFast);
  ASSERT_TRUE(received_[kSlow].empty()) << kSlow << " published while buffering:" << dumpOf(kSlow);

  const std::string incident_id = "incident-9f3c1a-shared";
  publishFlush(incident_id);
  ASSERT_TRUE(spinUntil([this] { return received_[kFast].size() >= 8u && received_[kSlow].size() >= 3u; }, 5000))
      << kFast << " released " << received_[kFast].size() << ", " << kSlow << " released " << received_[kSlow].size();
  // Let the tail of both windows out.
  spinFor(500);

  for (const std::string& name : { std::string(kFast), std::string(kSlow) })
  {
    EXPECT_EQ(countTaggedWith(name, incident_id), static_cast<int>(received_[name].size()))
        << name << " released a Record not tagged with the incident_id the FlushEvent carried";
  }
  // Different polling intervals, so different Record counts -- one incident either way.
  EXPECT_GT(received_[kFast].size(), received_[kSlow].size());
}

// Acceptance criterion: a Measurement with no buffer_duration_sec subscribes to no flush topic,
// so the same FlushEvent leaves it publishing live and its Records carry no incident_id.
TEST_F(MeasurementSharedFlushTest, AMeasurementWithoutBufferingIsUnaffectedByTheFlush)
{
  startLifecycleNode();

  // Publishing live all along, while the two buffering Measurements stay silent.
  ASSERT_TRUE(spinUntil([this] { return received_[kLive].size() >= 3u; }, 3000));
  ASSERT_TRUE(received_[kFast].empty()) << kFast << " published while buffering:" << dumpOf(kFast);
  const size_t before_flush = received_[kLive].size();

  publishFlush("incident-live-untouched");
  spinFor(400);

  EXPECT_GT(received_[kLive].size(), before_flush) << "the FlushEvent should not have interrupted live publishing";
  EXPECT_EQ(countCarryingAnIncident(kLive), 0) << "a Measurement that never buffers has no incident to report";
}

DC_MEASUREMENT_TEST_MAIN()
