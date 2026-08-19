// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/msg/named_lifecycle_state.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"

class MeasurementRos2ControlStatusTest : public ::testing::Test
{
protected:
  MeasurementRos2ControlStatusTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(
        rclcpp::NodeOptions(), std::vector<std::string>{ "ros2_control_status" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/ros2_control_status", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementRos2ControlStatusTest::dataCallback, this, std::placeholders::_1));
    activity_pub_ = ms_node_->create_publisher<controller_manager_msgs::msg::ControllerManagerActivity>(
        "/test/controller_manager/activity", rclcpp::QoS(1).reliable().transient_local());
  }

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: one test stops collection mid-activation on purpose, and TearDown must not then
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
    ms_node_->declare_parameter("ros2_control_status.plugin", std::string("dc_measurements/Ros2ControlStatus"));
    ms_node_->declare_parameter("ros2_control_status.group_key", std::string("ros2_control_status"));
    ms_node_->declare_parameter("ros2_control_status.topic_output", std::string("/dc/measurement/ros2_control_status"));
    ms_node_->declare_parameter("ros2_control_status.topic", std::string("/test/controller_manager/activity"));
    ms_node_->declare_parameter("ros2_control_status.polling_interval", 50);
    ms_node_->declare_parameter("ros2_control_status.init_collect", false);
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
    callback_active_ = true;
  }

  static controller_manager_msgs::msg::NamedLifecycleState makeEntry(const std::string& name, uint8_t state_id,
                                                                     const std::string& label)
  {
    controller_manager_msgs::msg::NamedLifecycleState entry;
    entry.name = name;
    entry.state.id = state_id;
    entry.state.label = label;
    return entry;
  }

  void publishActivity(const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& controllers,
                       const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& hardware_components)
  {
    controller_manager_msgs::msg::ControllerManagerActivity msg;
    msg.header.stamp = ms_node_->get_clock()->now();
    msg.controllers = controllers;
    msg.hardware_components = hardware_components;
    activity_pub_->publish(msg);
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

  // The detector treats a component's very first observed sample as a baseline, not a
  // transition: no Record comes out of it, and the state it carries never fires again since
  // nothing is left to compare against. Every test that wants to observe an actual "start"
  // therefore has to establish a non-active baseline first.
  void establishInactiveBaseline(const std::string& name)
  {
    publishActivity({ makeEntry(name, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive") }, {});
    spinFor(std::chrono::milliseconds(100));
  }

  // Republishes `controllers`/`hardware_components` until a *new* Record matching `predicate`
  // shows up, so a best-effort sample lost before the plugin subscribed doesn't make the test
  // flaky.
  nlohmann::json
  publishUntilRecord(const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& controllers,
                     const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& hardware_components,
                     const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    for (int i = 0; i < 400; ++i)
    {
      publishActivity(controllers, hardware_components);
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      for (size_t r = first_new; r < records_.size(); ++r)
      {
        if (predicate(records_[r]))
        {
          return records_[r];
        }
      }
    }
    ADD_FAILURE() << "No matching Record within the timeout";
    return nlohmann::json{};
  }

  static std::function<bool(const nlohmann::json&)> isEvent(const std::string& event)
  {
    return [event](const nlohmann::json& record) { return record.value("event", "") == event; };
  }

  // The schema the plugin itself validates against, applied here directly so a Record that only
  // half fills it fails the test rather than only logging.
  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/ros2_control_status.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr activity_pub_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementRos2ControlStatusTest, EnteringActiveRaisesAStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/controller_manager/activity");

  // Staying in the same non-active state must not produce a Record.
  publishActivity(
      { makeEntry("diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured") },
      {});
  spinFor(std::chrono::milliseconds(200));
  ASSERT_TRUE(records_.empty()) << "A first-seen sample is a baseline, not a transition";

  const std::vector<controller_manager_msgs::msg::NamedLifecycleState> active = { makeEntry(
      "diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active") };
  const auto start = publishUntilRecord(active, {}, isEvent("start"));

  EXPECT_EQ(start["seq"], 1);
  EXPECT_EQ(start["component_type"], "controller");
  EXPECT_EQ(start["component"], "diff_drive_controller");
  EXPECT_EQ(start["from_state"], "unconfigured");
  EXPECT_EQ(start["to_state"], "active");
  EXPECT_TRUE(start["open"].get<bool>());
  EXPECT_GE(start["previous_state_duration_s"].get<double>(), 0.0);
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementRos2ControlStatusTest, LeavingActiveRaisesAnEndRecordWithTheActiveDuration)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/controller_manager/activity");
  establishInactiveBaseline("diff_drive_controller");

  const std::vector<controller_manager_msgs::msg::NamedLifecycleState> active = { makeEntry(
      "diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active") };
  publishUntilRecord(active, {}, isEvent("start"));

  const std::vector<controller_manager_msgs::msg::NamedLifecycleState> inactive = { makeEntry(
      "diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive") };
  const auto end = publishUntilRecord(inactive, {}, isEvent("end"));

  EXPECT_EQ(end["seq"], 2);
  EXPECT_EQ(end["from_state"], "active");
  EXPECT_EQ(end["to_state"], "inactive");
  EXPECT_FALSE(end["open"].get<bool>());
  EXPECT_GT(end["previous_state_duration_s"].get<double>(), 0.0);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementRos2ControlStatusTest, TransitionBetweenTwoNonActiveStatesProducesNoRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/controller_manager/activity");
  establishInactiveBaseline("diff_drive_controller");

  publishActivity(
      { makeEntry("diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured") },
      {});
  spinFor(std::chrono::milliseconds(200));

  EXPECT_TRUE(records_.empty()) << "inactive -> unconfigured crosses no `active` boundary";
}

TEST_F(MeasurementRos2ControlStatusTest, HardwareComponentsAreTrackedIndependentlyOfControllers)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/controller_manager/activity");

  publishActivity({}, { makeEntry("arm", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured") });
  spinFor(std::chrono::milliseconds(200));
  ASSERT_TRUE(records_.empty());

  const auto start = publishUntilRecord(
      {}, { makeEntry("arm", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active") }, isEvent("start"));

  EXPECT_EQ(start["component_type"], "hardware_component");
  EXPECT_EQ(start["component"], "arm");
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementRos2ControlStatusTest, ComponentActiveAtShutdownStaysOpenWithNoSyntheticEndRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/controller_manager/activity");
  establishInactiveBaseline("diff_drive_controller");

  const std::vector<controller_manager_msgs::msg::NamedLifecycleState> active = { makeEntry(
      "diff_drive_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active") };
  publishUntilRecord(active, {}, isEvent("start"));

  // Collection stops with the controller still active.
  stopCollection();
  spinFor(std::chrono::milliseconds(200));

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a closing Record";
  EXPECT_EQ(records_.back()["event"], "start");
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
