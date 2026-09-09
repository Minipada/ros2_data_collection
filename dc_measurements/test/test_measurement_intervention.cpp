// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "measurement_test_bench.hpp"

class MeasurementInterventionTest : public MeasurementBench
{
protected:
  MeasurementInterventionTest() : MeasurementBench("intervention")
  {
    autonomous_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/autonomy/cmd_vel", rclcpp::SystemDefaultsQoS());
    teleop_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", rclcpp::SystemDefaultsQoS());
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

  // Keeps a velocity source active long enough for the plugin to poll it at least once, so the
  // mode it implies is what the next poll compares against.
  void driveOn(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::Twist twist;
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(twist);
      spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Publishes on `pub` until a Record arrives, so a test never depends on how many polls the
  // transition takes to be seen.
  void driveUntilRecord(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub)
  {
    callback_active_ = false;
    geometry_msgs::msg::Twist twist;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!callback_active_ && std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(twist);
      spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(callback_active_) << "No intervention Record was published within the timeout";
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autonomous_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_pub_;
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
  spinFor(200);

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
  spinFor(500);

  EXPECT_FALSE(callback_active_) << "An unknown mode is not a takeover";
  EXPECT_TRUE(records_.empty());
}

DC_MEASUREMENT_TEST_MAIN()
