#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>

#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"

using json = nlohmann::json;

class MeasurementBufferingTest : public ::testing::Test
{
protected:
  MeasurementBufferingTest()
  {
    SetUp();
  }

  ~MeasurementBufferingTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementBufferingTest::dummyDataCallback, this, std::placeholders::_1));
    flush_pub_ = ms_node_->create_publisher<dc_interfaces::msg::FlushEvent>("/dc/flush", rclcpp::QoS(10));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"hello\"}"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void dummyDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    received_.push_back(msg.data);
    // When each Record actually landed, for the rate-limited release (#289).
    arrivals_.push_back(std::chrono::steady_clock::now());
  }

  // Milliseconds between the first and the last Record received so far.
  int receivedSpanMs() const
  {
    if (arrivals_.size() < 2)
    {
      return 0;
    }
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(arrivals_.back() - arrivals_.front()).count());
  }

  void publishFlush(const std::string& incident_id)
  {
    dc_interfaces::msg::FlushEvent flush_msg;
    flush_msg.incident_id = incident_id;
    flush_pub_->publish(flush_msg);
  }

  // How many received Records carry this incident_id.
  int countTaggedWith(const std::string& incident_id) const
  {
    int count = 0;
    for (const auto& data : received_)
    {
      json data_json = json::parse(data);
      if (data_json.contains("incident_id") && data_json["incident_id"] == incident_id)
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
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<dc_interfaces::msg::FlushEvent>::SharedPtr flush_pub_;
  int polling_interval_{ 50 };

public:
  std::vector<std::string> received_;
  std::vector<std::chrono::steady_clock::time_point> arrivals_;
};

// Acceptance criterion: buffer_duration_sec unset/zero preserves today's behavior exactly --
// live publishing on every tick, no buffering.
TEST_F(MeasurementBufferingTest, ZeroBufferDurationPublishesLiveAsBefore)
{
  ms_node_->declare_parameter("dummy.init_collect", true);

  startLifecycleNode();

  while (received_.empty())
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(received_.empty());
  json data_json = json::parse(received_.front());
  EXPECT_FALSE(data_json.contains("incident_id"));
}

// Acceptance criterion: while buffering is armed, samples are buffered silently -- nothing is
// published even after several polling ticks.
TEST_F(MeasurementBufferingTest, BuffersSamplesSilentlyUntilFlush)
{
  ms_node_->declare_parameter("dummy.buffer_duration_sec", 10.0);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);

  EXPECT_TRUE(received_.empty());
}

// Acceptance criterion: on a FlushEvent, exactly the buffered window is published, each Record
// tagged with the event's incident_id. With post_roll_duration_sec left at its default 0 this is
// pre-roll only -- the Measurement returns to buffering instead of publishing live from then on
// (the #287 behavior this replaces resumed live publishing for good).
TEST_F(MeasurementBufferingTest, FlushEventReleasesBufferedWindowThenReturnsToBuffering)
{
  ms_node_->declare_parameter("dummy.buffer_duration_sec", 10.0);
  ms_node_->declare_parameter("dummy.flush_topic", std::string("/dc/flush"));

  startLifecycleNode();

  // Let a few samples accumulate in the ring buffer, silently.
  spinFor(polling_interval_ * 3);
  ASSERT_TRUE(received_.empty());

  publishFlush("incident-42");
  ASSERT_TRUE(spinUntil([this] { return !received_.empty(); }, 1000));
  // Let the whole released window be delivered.
  spinFor(polling_interval_ * 2);

  // Every Record released from the buffer carries the incident_id from the FlushEvent.
  int released = static_cast<int>(received_.size());
  EXPECT_EQ(countTaggedWith("incident-42"), released);

  // Pre-roll only: no further Records once the window is out, since buffering resumed.
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(static_cast<int>(received_.size()), released);
}

// Acceptance criterion: the end-to-end cycle on a real Measurement -- buffer, flush, post-roll
// live publish, cooldown (a second FlushEvent ignored), re-armed and buffering again.
TEST_F(MeasurementBufferingTest, FullIncidentCycleBuffersFlushesPostRollsCoolsDownThenReArms)
{
  ms_node_->declare_parameter("dummy.buffer_duration_sec", 10.0);
  ms_node_->declare_parameter("dummy.post_roll_duration_sec", 0.6);
  ms_node_->declare_parameter("dummy.cooldown_sec", 0.6);

  startLifecycleNode();

  // Buffering: samples accumulate silently.
  spinFor(polling_interval_ * 3);
  ASSERT_TRUE(received_.empty());

  // Flushing: the buffered window comes out tagged with the incident.
  publishFlush("incident-1");
  ASSERT_TRUE(spinUntil([this] { return !received_.empty(); }, 1000));

  // PostRoll: 200ms in, the released window has certainly all been delivered, so any Record that
  // shows up in the *next* 200ms was published live rather than released from the buffer.
  spinFor(200);
  int after_release = static_cast<int>(received_.size());
  spinFor(200);
  EXPECT_GT(static_cast<int>(received_.size()), after_release) << "post-roll should publish live";
  // Released and post-roll Records alike belong to the same incident.
  EXPECT_EQ(countTaggedWith("incident-1"), static_cast<int>(received_.size()));

  // Cooldown: post-roll ends 600ms after the flush and cooldown runs 600ms past that. A further
  // FlushEvent inside either phase belongs to the incident already being captured and must be
  // ignored entirely -- once while still post-rolling, once during cooldown proper.
  publishFlush("ignored-during-post-roll");
  spinFor(400);
  publishFlush("ignored-during-cooldown");
  spinFor(400);
  EXPECT_EQ(countTaggedWith("ignored-during-post-roll"), 0) << "a FlushEvent during post-roll is ignored";
  EXPECT_EQ(countTaggedWith("ignored-during-cooldown"), 0) << "a FlushEvent during cooldown is ignored";

  // Back to Buffering: nothing more is published once post-roll is over.
  spinFor(600);
  int at_rearm = static_cast<int>(received_.size());
  spinFor(polling_interval_ * 4);
  EXPECT_EQ(static_cast<int>(received_.size()), at_rearm) << "buffering again, so nothing is published";

  // Re-armed: a fresh incident is captured with no manual intervention, and the samples buffered
  // during cooldown are its pre-roll.
  publishFlush("incident-3");
  EXPECT_TRUE(spinUntil([this] { return countTaggedWith("incident-3") > 0; }, 1000))
      << "the Measurement should have re-armed itself after cooldown";
  EXPECT_EQ(countTaggedWith("incident-1"), at_rearm);
}

// Acceptance criterion (#289): a large buffered window is not burst-published when
// max_flush_rate_hz is configured -- it is spread out at (at most) that rate, so a robot
// recovering from an incident doesn't also have to absorb the whole window at once.
TEST_F(MeasurementBufferingTest, MaxFlushRateSpreadsALargeBufferedWindowOverTime)
{
  const double max_flush_rate_hz = 10.0;  // one Record every 100ms
  const int release_interval_ms = 100;
  ms_node_->declare_parameter("dummy.buffer_duration_sec", 10.0);
  ms_node_->declare_parameter("dummy.max_flush_rate_hz", max_flush_rate_hz);

  startLifecycleNode();

  // A second of 50ms polling buffers ~20 samples -- a window far larger than one release
  // interval, which is what makes the rate limit observable.
  spinFor(1000);
  ASSERT_TRUE(received_.empty());

  publishFlush("incident-rate");
  ASSERT_TRUE(spinUntil([this] { return !received_.empty(); }, 1000));

  // Without the rate limit the whole window would be out by now; at 10Hz only a handful can be.
  spinFor(300);
  const int early = static_cast<int>(received_.size());
  EXPECT_LE(early, 8) << "burst: " << early << " Records in the first 300ms of a 10Hz release";

  // The whole window still makes it out, only spread over time.
  ASSERT_TRUE(spinUntil([this] { return received_.size() >= 15u; }, 5000))
      << "only " << received_.size() << " Records released";
  EXPECT_GT(static_cast<int>(received_.size()), early);
  EXPECT_EQ(countTaggedWith("incident-rate"), static_cast<int>(received_.size()));

  // n Records at that rate cannot have arrived in less than (n-1) intervals; 70% of that leaves
  // room for timer and delivery jitter while still failing loudly on a burst (span ~0).
  const int spacing_count = static_cast<int>(received_.size()) - 1;
  EXPECT_GE(receivedSpanMs(), (spacing_count * release_interval_ms * 7) / 10)
      << spacing_count + 1 << " Records arrived within " << receivedSpanMs() << "ms";
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
