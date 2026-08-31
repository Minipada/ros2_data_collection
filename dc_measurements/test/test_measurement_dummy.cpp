// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"

class MeasurementDummyTest : public ::testing::Test
{
protected:
  MeasurementDummyTest()
  {
    SetUp();
  }

  ~MeasurementDummyTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementDummyTest::dummyDataCallback, this, std::placeholders::_1));
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
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    dummy_record_ = data_json;
    dummy_message_ = data_json["message"].get<nlohmann::json::string_t>();
    dummy_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::string dummy_message_;
  nlohmann::json dummy_record_;

public:
  bool dummy_callback_{ false };
};

TEST_F(MeasurementDummyTest, DummyDataCorrect)
{
  auto record = std::string("{\"message\": \"My message\"}");
  auto message = nlohmann::json::parse(record)["message"];
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", record);

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(dummy_message_, message);
}

// A Record with custom keys names them, so the Bridge's Uploader can carry the same
// labelling onto the Measurement's File metadata Records (#419).
TEST_F(MeasurementDummyTest, CustomKeysAreDeclaredInTheRecord)
{
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "site" });
  ms_node_->declare_parameter("custom_keys_str.site.name", std::string("site"));
  ms_node_->declare_parameter("custom_keys_str.site.value", std::string("warehouse-3"));

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(dummy_record_["site"], "warehouse-3");
  EXPECT_EQ(dummy_record_["custom_keys"], nlohmann::json::array({ "site" }));
}

// robot_name is the one custom key whose resolution is flexible: literal, hostname, or a
// file's contents, in that order, with hostname as the default when nothing is set (#442).

TEST_F(MeasurementDummyTest, RobotNameLiteralValueIsUnchanged)
{
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value", std::string("C3PO"));

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(dummy_record_["robot_name"], "C3PO");
}

TEST_F(MeasurementDummyTest, RobotNameDefaultsToTheHostname)
{
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  char hostname_buf[256] = { 0 };
  ASSERT_EQ(gethostname(hostname_buf, sizeof(hostname_buf) - 1), 0);
  EXPECT_EQ(dummy_record_["robot_name"], std::string(hostname_buf));
}

TEST_F(MeasurementDummyTest, RobotNameResolvesFromFile)
{
  auto robot_name_file = (std::filesystem::temp_directory_path() / "dc_measurement_dummy_robot_name_file").u8string();
  std::ofstream(robot_name_file) << "TB-42";

  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value_from_file", robot_name_file);

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(dummy_record_["robot_name"], "TB-42");

  std::filesystem::remove(robot_name_file);
}

// rclcpp_lifecycle wraps every transition callback (on_configure here) in its own catch,
// converting an uncaught exception into a CallbackReturn::ERROR rather than letting it
// propagate to the caller of configure() (same behavior documented in
// test_measurement_bool_equal.cpp's MeasurementServerConfigureFailsToReachInactiveState).
// So the observable, end-to-end consequence of an unreadable value_from_file is that the
// whole MeasurementServer fails to reach the "inactive" state, with the resolveRobotName()
// error logged as the ERROR/FATAL "Original error" during the transition.
TEST_F(MeasurementDummyTest, RobotNameMissingFileFailsConfigureClearly)
{
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));
  ms_node_->declare_parameter("custom_key_str_list", std::vector<std::string>{ "robot_name" });
  ms_node_->declare_parameter("custom_keys_str.robot_name.name", std::string("robot_name"));
  ms_node_->declare_parameter("custom_keys_str.robot_name.value_from_file",
                              std::string("/nonexistent/dc_robot_name_that_does_not_exist"));

  auto result_state = ms_node_->configure();

  EXPECT_NE(result_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(MeasurementDummyTest, NoCustomKeysLeavesTheRecordUntouched)
{
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"My message\"}"));

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(dummy_record_.contains("custom_keys"));
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

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  // Check no sample has been published in the polling interval time, because exception was
  // triggered and the message did not go through
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         polling_interval * 2)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_TRUE(dummy_message_.empty());
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
