// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MeasurementInterventionTest : public ::testing::Test
{
protected:
  MeasurementInterventionTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "intervention" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/intervention", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementInterventionTest::dataCallback, this, std::placeholders::_1));
    autonomous_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/autonomy/cmd_vel", rclcpp::SystemDefaultsQoS());
    teleop_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: one test stops collection mid-takeover on purpose, and TearDown must not then
  // drive the lifecycle node through a transition it is no longer in a state for.
  void stopCollection()
  {
    if (stopped_)
    {
      return;
    }
    stopped_ = true;
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("intervention.plugin", std::string("dc_measurements/Intervention"));
    ms_node_->declare_parameter("intervention.group_key", std::string("intervention"));
    ms_node_->declare_parameter("intervention.topic_output", std::string("/dc/measurement/intervention"));
    ms_node_->declare_parameter("intervention.polling_interval", 50);
  }

  void declareVelocitySources()
  {
    ms_node_->declare_parameter("intervention.velocity_topics",
                                std::vector<std::string>{ "/autonomy/cmd_vel", "/teleop/cmd_vel" });
    ms_node_->declare_parameter("intervention.velocity_modes", std::vector<std::string>{ "autonomous", "teleop" });
    ms_node_->declare_parameter("intervention.velocity_timeout_s", 30.0);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    data_json_ = nlohmann::json::parse(data_str);
    records_.push_back(data_json_);
    callback_active_ = true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void waitForSubscriber(const std::string& topic)
  {
    while (ms_node_->count_subscribers(topic) == 0)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
  }

  // Keeps a velocity source active long enough for the plugin to poll it at least once, so the
  // mode it implies is what the next poll compares against.
  void driveOn(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::Twist twist;
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(twist);
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Publishes on `pub` until a Record arrives, so a test never depends on how many polls the
  // transition takes to be seen.
  void driveUntilRecord(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub)
  {
    callback_active_ = false;
    geometry_msgs::msg::Twist twist;
    for (int i = 0; i < 2000 && !callback_active_; ++i)
    {
      pub->publish(twist);
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(callback_active_) << "No intervention Record was published within the timeout";
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autonomous_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_pub_;
  nlohmann::json data_json_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementInterventionTest, TakeoverProducesAStartThenAnEndRecord)
{
  declareCommonParameters();
  declareVelocitySources();

  startLifecycleNode();
  waitForSubscriber("/autonomy/cmd_vel");
  waitForSubscriber("/teleop/cmd_vel");

  // The robot drives itself for a while: no takeover, so nothing to report.
  driveOn(autonomous_vel_pub_, std::chrono::milliseconds(300));
  ASSERT_FALSE(callback_active_) << "Autonomous operation alone must not produce an intervention Record";

  driveUntilRecord(teleop_vel_pub_);
  const auto start = data_json_;
  EXPECT_EQ(start["event"], "start");
  EXPECT_EQ(start["open"], true);
  EXPECT_EQ(start["from_mode"], "autonomous");
  EXPECT_EQ(start["to_mode"], "teleop");
  EXPECT_GT(start["sequence"].get<uint64_t>(), 0u);
  EXPECT_GT(start["previous_duration"].get<double>(), 0.0);

  // Keep the takeover going, so the reported duration is a measurable one.
  driveOn(teleop_vel_pub_, std::chrono::milliseconds(300));

  driveUntilRecord(autonomous_vel_pub_);
  const auto end = data_json_;
  EXPECT_EQ(end["event"], "end");
  EXPECT_EQ(end["open"], false);
  EXPECT_EQ(end["from_mode"], "teleop");
  EXPECT_EQ(end["to_mode"], "autonomous");
  // No other mode change happens between the two boundaries in this scenario, so the sequence
  // advances by exactly one -- a gap here would mean a Record was dropped.
  EXPECT_EQ(end["sequence"].get<uint64_t>(), start["sequence"].get<uint64_t>() + 1);
  // On the end Record, previous_duration (the dwell of "teleop") is the takeover's own length.
  EXPECT_GT(end["previous_duration"].get<double>(), 0.0);

  EXPECT_EQ(records_.size(), 2u) << "Only the two takeover boundaries are Records";
}

TEST_F(MeasurementInterventionTest, InterventionOpenAtShutdownGetsNoClosingRecord)
{
  declareCommonParameters();
  declareVelocitySources();

  startLifecycleNode();
  waitForSubscriber("/autonomy/cmd_vel");
  waitForSubscriber("/teleop/cmd_vel");

  driveOn(autonomous_vel_pub_, std::chrono::milliseconds(300));
  driveUntilRecord(teleop_vel_pub_);
  ASSERT_EQ(data_json_["event"], "start");
  ASSERT_EQ(data_json_["open"], true);

  // Collection stops with the human still driving.
  stopCollection();
  spinFor(std::chrono::milliseconds(200));

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a closing Record";
  EXPECT_EQ(records_.back()["event"], "start");
  EXPECT_EQ(records_.back()["open"], true)
      << "An unterminated takeover's only Record must stay marked open, so it cannot be averaged as a zero";
}

TEST_F(MeasurementInterventionTest, ModeSignalThatNeverArrivesProducesNoRecords)
{
  declareCommonParameters();
  ms_node_->declare_parameter("intervention.velocity_topics", std::vector<std::string>{ "/never/cmd_vel" });
  ms_node_->declare_parameter("intervention.velocity_modes", std::vector<std::string>{ "teleop" });
  ms_node_->declare_parameter("intervention.velocity_timeout_s", 30.0);

  startLifecycleNode();
  spinFor(std::chrono::milliseconds(500));

  EXPECT_FALSE(callback_active_) << "An unknown mode is not a takeover";
  EXPECT_TRUE(records_.empty());
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
