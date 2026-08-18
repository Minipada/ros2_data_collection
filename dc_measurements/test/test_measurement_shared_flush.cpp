// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"

using json = nlohmann::json;

namespace
{
// A non-default topic, so the test also covers several Measurements being pointed at a shared
// flush topic of the deployment's choosing.
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
class MeasurementSharedFlushTest : public ::testing::Test
{
protected:
  MeasurementSharedFlushTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    received_.clear();
    subs_.clear();
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ kFast, kSlow, kLive });
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

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareMeasurement(const std::string& name, const int polling_interval)
  {
    const std::string topic = "/dc/measurement/" + name;
    ms_node_->declare_parameter(name + ".plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter(name + ".topic_output", topic);
    ms_node_->declare_parameter(name + ".record", json({ { "message", name } }).dump());
    ms_node_->declare_parameter(name + ".polling_interval", polling_interval);
    subs_.push_back(ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this, name](const dc_interfaces::msg::StringStamped& msg) { received_[name].push_back(msg.data); }));
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void publishFlush(const std::string& incident_id)
  {
    dc_interfaces::msg::FlushEvent flush_msg;
    flush_msg.incident_id = incident_id;
    flush_pub_->publish(flush_msg);
  }

  // How many of this Measurement's Records carry exactly this incident_id.
  int countTaggedWith(const std::string& name, const std::string& incident_id)
  {
    int count = 0;
    for (const auto& data : received_[name])
    {
      json data_json = json::parse(data);
      if (data_json.contains("incident_id") && data_json["incident_id"].get<std::string>() == incident_id)
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
    for (const auto& data : received_[name])
    {
      if (json::parse(data).contains("incident_id"))
      {
        count++;
      }
    }
    return count;
  }

  void spinFor(int milliseconds)
  {
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (
        (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
        milliseconds)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
  }

  // Spin until `done` holds, giving up after `timeout_ms` -- a bounded wait, so a behavior
  // regression fails the test loudly instead of hanging until the suite times out.
  bool spinUntil(const std::function<bool()>& done, int timeout_ms)
  {
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (!done())
    {
      if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time))
              .count() >= timeout_ms)
      {
        return false;
      }
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
    return true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  std::vector<rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr> subs_;
  rclcpp::Publisher<dc_interfaces::msg::FlushEvent>::SharedPtr flush_pub_;

public:
  std::map<std::string, std::vector<std::string>> received_;
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

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
