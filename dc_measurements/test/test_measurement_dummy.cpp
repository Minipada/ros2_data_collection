// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "lifecycle_msgs/msg/state.hpp"
#include "measurement_test_bench.hpp"

class MeasurementDummyTest : public MeasurementBench
{
protected:
  MeasurementDummyTest() : MeasurementBench("dummy")
  {
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  }
};

TEST_F(MeasurementDummyTest, DummyDataCorrect)
{
  declareCommonParameters();

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["message"], "My message");
}

// A Record with custom keys names them, so the Bridge's Uploader can carry the same
// labelling onto the Measurement's File metadata Records (#419).
TEST_F(MeasurementDummyTest, CustomKeysAreDeclaredInTheRecord)
{
  declareCommonParameters();
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "site" });
  ms_node_->declare_parameter("custom_keys_str.site.name", std::string("site"));
  ms_node_->declare_parameter("custom_keys_str.site.value", std::string("warehouse-3"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["site"], "warehouse-3");
  EXPECT_EQ(data_json_["custom_keys"], nlohmann::json::array({ "site" }));
}

// robot_name is the one custom key whose resolution is flexible: literal, hostname, or a
// file's contents, in that order, with hostname as the default when nothing is set (#442).

TEST_F(MeasurementDummyTest, RobotNameLiteralValueIsUnchanged)
{
  declareCommonParameters();
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value", std::string("C3PO"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["robot_name"], "C3PO");
}

TEST_F(MeasurementDummyTest, RobotNameDefaultsToTheHostname)
{
  declareCommonParameters();
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  char hostname_buf[256] = { 0 };
  ASSERT_EQ(gethostname(hostname_buf, sizeof(hostname_buf) - 1), 0);
  EXPECT_EQ(data_json_["robot_name"], std::string(hostname_buf));
}

TEST_F(MeasurementDummyTest, RobotNameResolvesFromFile)
{
  auto robot_name_file = (std::filesystem::temp_directory_path() / "dc_measurement_dummy_robot_name_file").u8string();
  std::ofstream(robot_name_file) << "TB-42";

  declareCommonParameters();
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value_from_file", robot_name_file);

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["robot_name"], "TB-42");

  std::filesystem::remove(robot_name_file);
}

// rclcpp_lifecycle wraps every transition callback (on_configure here) in its own catch,
// converting an uncaught exception into a CallbackReturn::ERROR rather than letting it
// propagate to the caller of configure() (same behavior documented in
// test_measurement_compare.cpp's InvalidComparisonFailsToReachInactiveState).
// So the observable, end-to-end consequence of an unreadable value_from_file is that the
// whole MeasurementServer fails to reach the "inactive" state, with the resolveRobotName()
// error logged as the ERROR/FATAL "Original error" during the transition.
TEST_F(MeasurementDummyTest, RobotNameMissingFileFailsConfigureClearly)
{
  declareCommonParameters();
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value_from_file",
                              std::string("/nonexistent/dc_robot_name_that_does_not_exist"));

  auto result_state = ms_node_->configure();

  EXPECT_NE(result_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(MeasurementDummyTest, NoCustomKeysLeavesTheRecordUntouched)
{
  declareCommonParameters();

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_FALSE(data_json_.contains("custom_keys"));
}

TEST_F(MeasurementDummyTest, DummyDataIncorrect)
{
  int polling_interval = 50;
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.polling_interval", polling_interval);
  // Not a valid JSON
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\":"));

  startLifecycleNode();

  // Check no sample has been published in the polling interval time, because exception was
  // triggered and the message did not go through
  spinFor(polling_interval * 2);

  EXPECT_EQ(callback_count_, 0);
}

DC_MEASUREMENT_TEST_MAIN()
