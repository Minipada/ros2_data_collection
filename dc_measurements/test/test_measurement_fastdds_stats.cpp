// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// This Measurement has no input topic: it polls Fast-DDS-statistics-backend's static registry
// directly, so the only thing a test can drive is the polling interval itself -- unlike e.g.
// battery, there is nothing here to publish() to trigger a specific Record.
class MeasurementFastddsStatsTest : public ::testing::Test
{
protected:
  MeasurementFastddsStatsTest()
  {
    SetUp();
  }

  ~MeasurementFastddsStatsTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "fastdds_stats" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/fastdds_stats", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementFastddsStatsTest::dataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("fastdds_stats.plugin", std::string("dc_measurements/FastddsStats"));
    ms_node_->declare_parameter("fastdds_stats.group_key", std::string("fastdds_stats"));
    ms_node_->declare_parameter("fastdds_stats.topic_output", std::string("/dc/measurement/fastdds_stats"));
    // A domain unlikely to collide with any other DDS traffic on the test host/CI runner.
    ms_node_->declare_parameter("fastdds_stats.domain_id", 221);
    ms_node_->declare_parameter("fastdds_stats.polling_interval", 100);
    ms_node_->declare_parameter("fastdds_stats.init_collect", false);
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
    records_.push_back(nlohmann::json::parse(data_str));
  }

  nlohmann::json waitForRecord(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (records_.empty() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (records_.empty())
    {
      ADD_FAILURE() << "No Record within the timeout";
      return nlohmann::json{};
    }
    return records_.front();
  }

  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/fastdds_stats.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::vector<nlohmann::json> records_;
};

TEST_F(MeasurementFastddsStatsTest, ReportsASampleOnThePollingIntervalEvenWithNoOtherParticipants)
{
  declareCommonParameters();
  startLifecycleNode();

  // Nothing else is running on domain 221 for this test, so every count is expected at zero --
  // the point is that a sample is still emitted and still validates, the same way `uptime`
  // always reports something on every poll regardless of what else is going on.
  const auto record = waitForRecord(std::chrono::seconds(5));

  EXPECT_EQ(record["event"], "sample");
  EXPECT_EQ(record["domain_id"].get<int>(), 221);
  EXPECT_GE(record["participant_count"].get<int>(), 0);
  EXPECT_GE(record["datawriter_count"].get<int>(), 0);
  EXPECT_GE(record["datareader_count"].get<int>(), 0);
  expectValidatesAgainstSchema(record);
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
