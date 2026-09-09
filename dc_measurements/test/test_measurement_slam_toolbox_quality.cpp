// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <fstream>
#include <functional>
#include <nlohmann/json-schema.hpp>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "measurement_test_bench.hpp"
#include "std_msgs/msg/empty.hpp"

class MeasurementSlamToolboxQualityTest : public MeasurementBench
{
protected:
  MeasurementSlamToolboxQualityTest() : MeasurementBench("slam_quality")
  {
    pose_pub_ = ms_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/test/pose",
                                                                                          rclcpp::SensorDataQoS());
    // Stands in for whatever slam_toolbox itself eventually publishes: the plugin subscribes
    // generically and never decodes the message, so the type here doesn't have to match a
    // real slam_toolbox one -- only its *arrival* is exercised.
    loop_closure_pub_ =
        ms_node_->create_publisher<std_msgs::msg::Empty>("/test/loop_closure_event", rclcpp::SystemDefaultsQoS());
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("slam_quality.plugin", std::string("dc_measurements/SlamToolboxQuality"));
    ms_node_->declare_parameter("slam_quality.group_key", std::string("slam_quality"));
    ms_node_->declare_parameter("slam_quality.topic_output", std::string("/dc/measurement/slam_quality"));
    ms_node_->declare_parameter("slam_quality.pose_topic", std::string("/test/pose"));
    ms_node_->declare_parameter("slam_quality.loop_closure_topic", std::string("/test/loop_closure_event"));
    ms_node_->declare_parameter("slam_quality.polling_interval", 50);
    ms_node_->declare_parameter("slam_quality.init_collect", false);
  }

  static std::function<bool(const nlohmann::json&)> isEvent(const std::string& event)
  {
    return [event](const nlohmann::json& record) { return record.value("event", "") == event; };
  }

  // Republishes `msg` until a *new* Record matching `predicate` shows up, so a best-effort sample
  // lost before the plugin subscribed doesn't make the test flaky.
  nlohmann::json publishUntilRecord(const geometry_msgs::msg::PoseWithCovarianceStamped& msg,
                                    const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    nlohmann::json matched;
    if (!spinUntil(
            [&] {
              pose_pub_->publish(msg);
              for (size_t r = first_new; r < records_.size(); ++r)
              {
                if (predicate(records_[r]))
                {
                  matched = records_[r];
                  return true;
                }
              }
              return false;
            },
            4000))
    {
      ADD_FAILURE() << "No matching Record within the timeout";
      return nlohmann::json{};
    }
    return matched;
  }

  // A loop closure is single-shot: republishing it would report several occurrences instead of
  // one, so this only sends it once and waits for the Record it causes.
  nlohmann::json publishLoopClosureAndWaitForRecord()
  {
    const size_t first_new = records_.size();
    loop_closure_pub_->publish(std_msgs::msg::Empty());
    nlohmann::json matched;
    if (!spinUntil(
            [&] {
              for (size_t r = first_new; r < records_.size(); ++r)
              {
                if (isEvent("loop_closure")(records_[r]))
                {
                  matched = records_[r];
                  return true;
                }
              }
              return false;
            },
            4000))
    {
      ADD_FAILURE() << "No loop_closure Record within the timeout";
      return nlohmann::json{};
    }
    return matched;
  }

  // The schema the plugin itself validates against, applied here directly so a Record that only
  // half fills it fails the test rather than only logging.
  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/slam_toolbox_quality.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr loop_closure_pub_;
};

TEST_F(MeasurementSlamToolboxQualityTest, ReportsPoseAndCovarianceOnThePollingInterval)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/pose");

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.pose.pose.position.x = 1.5;
  msg.pose.pose.position.y = -2.25;
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.covariance[0] = 0.01;
  msg.pose.covariance[7] = 0.02;
  msg.pose.covariance[35] = 0.03;

  const auto record = publishUntilRecord(msg, isEvent("sample"));

  EXPECT_NEAR(record["x"].get<double>(), 1.5, 1e-6);
  EXPECT_NEAR(record["y"].get<double>(), -2.25, 1e-6);
  EXPECT_NEAR(record["yaw"].get<double>(), 0.0, 1e-6);
  EXPECT_NEAR(record["covariance_x"].get<double>(), 0.01, 1e-9);
  EXPECT_NEAR(record["covariance_y"].get<double>(), 0.02, 1e-9);
  EXPECT_NEAR(record["covariance_yaw"].get<double>(), 0.03, 1e-9);
  expectValidatesAgainstSchema(record);
}

TEST_F(MeasurementSlamToolboxQualityTest, ReportsNothingWhilePoseNeverPublishes)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/pose");

  // Several polling intervals with nothing on /pose: no Record at all beats a Record with no
  // localization data.
  spinFor(500);

  EXPECT_TRUE(records_.empty()) << records_.size() << " Record(s) published without any pose";
}

TEST_F(MeasurementSlamToolboxQualityTest, EmitsASingleShotRecordPerLoopClosure)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/pose");
  waitForSubscriber("/test/loop_closure_event");

  const auto record = publishLoopClosureAndWaitForRecord();

  EXPECT_EQ(record["event"], "loop_closure");
  // The source topic carries only a timestamp -- no localization data belongs in the Record
  // body. (Every Record picks up a few enrichment keys -- flattened/nested and friends -- from
  // the shared publish() pipeline regardless of Measurement, so this checks absence of the
  // sample-specific fields rather than an exact key count.)
  for (const auto& key : { "x", "y", "yaw", "covariance_x", "covariance_y", "covariance_yaw" })
  {
    EXPECT_FALSE(record.contains(key)) << key << " should be absent on a loop_closure Record";
  }
  expectValidatesAgainstSchema(record);
}

DC_MEASUREMENT_TEST_MAIN()
