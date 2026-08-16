#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>

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

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<dc_interfaces::msg::FlushEvent>::SharedPtr flush_pub_;
  int polling_interval_{ 50 };

public:
  std::vector<std::string> received_;
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
// tagged with the event's incident_id, and live publishing resumes immediately afterward.
TEST_F(MeasurementBufferingTest, FlushEventReleasesBufferedWindowThenResumesLivePublishing)
{
  ms_node_->declare_parameter("dummy.buffer_duration_sec", 10.0);
  ms_node_->declare_parameter("dummy.flush_topic", std::string("/dc/flush"));

  startLifecycleNode();

  // Let a few samples accumulate in the ring buffer, silently.
  spinFor(polling_interval_ * 3);
  ASSERT_TRUE(received_.empty());

  dc_interfaces::msg::FlushEvent flush_msg;
  flush_msg.incident_id = "incident-42";
  flush_pub_->publish(flush_msg);

  while (received_.empty())
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  // Every Record released from the buffer carries the incident_id from the FlushEvent.
  int count_after_flush = static_cast<int>(received_.size());
  for (const auto& data : received_)
  {
    json data_json = json::parse(data);
    ASSERT_TRUE(data_json.contains("incident_id"));
    EXPECT_EQ(data_json["incident_id"], "incident-42");
  }

  // Live publishing resumes: subsequent ticks publish without buffering, so more Records arrive
  // without needing another FlushEvent.
  while (static_cast<int>(received_.size()) <= count_after_flush)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GT(static_cast<int>(received_.size()), count_after_flush);

  // Records published live after the flush must not carry incident_id -- that only tags the
  // released buffered window.
  json last_json = json::parse(received_.back());
  EXPECT_FALSE(last_json.contains("incident_id"));
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
