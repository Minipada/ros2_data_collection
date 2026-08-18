// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class MeasurementBatteryTest : public ::testing::Test
{
protected:
  MeasurementBatteryTest()
  {
    SetUp();
  }

  ~MeasurementBatteryTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "battery" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/battery", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementBatteryTest::dataCallback, this, std::placeholders::_1));
    battery_pub_ =
        ms_node_->create_publisher<sensor_msgs::msg::BatteryState>("/test/battery_state", rclcpp::SensorDataQoS());
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("battery.plugin", std::string("dc_measurements/Battery"));
    ms_node_->declare_parameter("battery.group_key", std::string("battery"));
    ms_node_->declare_parameter("battery.topic_output", std::string("/dc/measurement/battery"));
    ms_node_->declare_parameter("battery.topic", std::string("/test/battery_state"));
    ms_node_->declare_parameter("battery.polling_interval", 50);
    ms_node_->declare_parameter("battery.init_collect", false);
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

  // An "unmeasured" pack: sensor_msgs/BatteryState signals every optional field with NaN.
  static sensor_msgs::msg::BatteryState unmeasuredBatteryState()
  {
    sensor_msgs::msg::BatteryState msg;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.voltage = nan;
    msg.temperature = nan;
    msg.current = nan;
    msg.charge = nan;
    msg.capacity = nan;
    msg.design_capacity = nan;
    msg.percentage = nan;
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    msg.present = true;
    return msg;
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

  // Republishes `msg` until a *new* Record matching `predicate` shows up, so a best-effort sample
  // lost before the plugin subscribed doesn't make the test flaky, and an earlier Record of the
  // same shape isn't mistaken for the one this step is waiting on.
  nlohmann::json publishUntilRecord(const sensor_msgs::msg::BatteryState& msg,
                                    const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    for (int i = 0; i < 400; ++i)
    {
      battery_pub_->publish(msg);
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
    const std::string path =
        ament_index_cpp::get_package_share_directory("dc_measurements") + "/plugins/measurements/json/battery.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  std::vector<nlohmann::json> records_;
};

TEST_F(MeasurementBatteryTest, ReportsPercentageVoltageAndCurrentOnThePollingInterval)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/battery_state");

  auto msg = unmeasuredBatteryState();
  msg.percentage = 0.62F;
  msg.voltage = 48.4F;
  msg.current = -12.5F;
  msg.capacity = 42.0F;
  msg.design_capacity = 50.0F;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.serial_number = "PACK-A";

  const auto record = publishUntilRecord(msg, isEvent("sample"));

  EXPECT_NEAR(record["percentage"].get<double>(), 62.0, 1e-4);
  EXPECT_NEAR(record["voltage"].get<double>(), 48.4, 1e-4);
  EXPECT_NEAR(record["current"].get<double>(), -12.5, 1e-4);
  EXPECT_EQ(record["power_supply_status"], "discharging");
  // Health, and design-versus-current capacity, exactly as the hardware reported them.
  EXPECT_EQ(record["power_supply_health"], "good");
  EXPECT_NEAR(record["health_percentage"].get<double>(), 84.0, 1e-4);
  EXPECT_EQ(record["serial_number"], "PACK-A");
  expectValidatesAgainstSchema(record);
}

TEST_F(MeasurementBatteryTest, ReportsNothingWhileTheTopicNeverPublishes)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/battery_state");

  // Several polling intervals with nothing on the input topic: no Record at all beats a Record
  // of absent fields.
  spinFor(std::chrono::milliseconds(500));

  EXPECT_TRUE(records_.empty()) << records_.size() << " Record(s) published without any BatteryState";
}

TEST_F(MeasurementBatteryTest, AlmostEmptyBatteryStateStillProducesAValidRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/battery_state");

  auto msg = unmeasuredBatteryState();
  msg.voltage = 24.0F;

  const auto record = publishUntilRecord(msg, isEvent("sample"));

  EXPECT_NEAR(record["voltage"].get<double>(), 24.0, 1e-4);
  EXPECT_EQ(record["power_supply_status"], "unknown");
  // Absent, not null-filled.
  for (const auto& key : { "percentage", "current", "charge", "capacity", "design_capacity", "temperature",
                           "health_percentage", "power_supply_health", "power_supply_technology", "serial_number" })
  {
    EXPECT_FALSE(record.contains(key)) << key << " should be absent when the hardware doesn't report it";
  }
  expectValidatesAgainstSchema(record);
}

TEST_F(MeasurementBatteryTest, MarksTheStartAndTheEndOfAChargingSession)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/battery_state");

  auto discharging = unmeasuredBatteryState();
  discharging.percentage = 0.93F;
  discharging.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  publishUntilRecord(discharging, isEvent("sample"));

  auto low = discharging;
  low.percentage = 0.31F;
  publishUntilRecord(low, [](const nlohmann::json& record) {
    return record.value("event", "") == "sample" && record.value("percentage", 100.0) < 40.0;
  });

  auto charging = low;
  charging.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  const auto started = publishUntilRecord(charging, isEvent("charge_session_start"));

  EXPECT_EQ(started["session_id"].get<uint64_t>(), 1u);
  EXPECT_NEAR(started["percentage"].get<double>(), 31.0, 1e-4);
  EXPECT_NEAR(started["discharge_depth_percent"].get<double>(), 62.0, 1e-4);
  expectValidatesAgainstSchema(started);

  auto full = charging;
  full.percentage = 0.97F;
  full.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  const auto ended = publishUntilRecord(full, isEvent("charge_session_end"));

  EXPECT_EQ(ended["session_id"].get<uint64_t>(), 1u);
  EXPECT_GT(ended["duration_sec"].get<double>(), 0.0);
  EXPECT_NEAR(ended["start_percentage"].get<double>(), 31.0, 1e-4);
  EXPECT_NEAR(ended["end_percentage"].get<double>(), 97.0, 1e-4);
  EXPECT_NEAR(ended["charged_percent"].get<double>(), 66.0, 1e-4);
  expectValidatesAgainstSchema(ended);

  // The discharge that fed the session (62 points) is counted as wear but is not a whole cycle
  // yet -- BatteryCycleAccumulator's own tests cover the accumulation itself.
  const auto sample = publishUntilRecord(full, [](const nlohmann::json& record) {
    return record.value("event", "") == "sample" && record.contains("completed_cycles");
  });
  EXPECT_EQ(sample["completed_cycles"].get<uint64_t>(), 0u);
}

// A robot with two packs runs one Measurement per pack, each on its own input topic.
class MeasurementTwoBatteriesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(
        rclcpp::NodeOptions(), std::vector<std::string>{ "battery_left", "battery_right" });
    for (const auto& pack : { "left", "right" })
    {
      const std::string name = std::string("battery_") + pack;
      ms_node_->declare_parameter(name + ".plugin", std::string("dc_measurements/Battery"));
      ms_node_->declare_parameter(name + ".group_key", name);
      ms_node_->declare_parameter(name + ".topic_output", "/dc/measurement/" + name);
      ms_node_->declare_parameter(name + ".topic", "/test/" + name);
      ms_node_->declare_parameter(name + ".polling_interval", 50);
      ms_node_->declare_parameter(name + ".init_collect", false);

      subs_.push_back(ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
          "/dc/measurement/" + name, rclcpp::SystemDefaultsQoS(),
          [this, name](const dc_interfaces::msg::StringStamped& msg) {
            std::string data_str = msg.data.c_str();
            boost::replace_all(data_str, "'", "\"");
            records_[name].push_back(nlohmann::json::parse(data_str));
          }));
      pubs_.push_back(
          ms_node_->create_publisher<sensor_msgs::msg::BatteryState>("/test/" + name, rclcpp::SensorDataQoS()));
    }
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  std::vector<rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr> pubs_;
  std::map<std::string, std::vector<nlohmann::json>> records_;
};

TEST_F(MeasurementTwoBatteriesTest, EachPackReportsItsOwnRecords)
{
  ms_node_->configure();
  ms_node_->activate();

  while (ms_node_->count_subscribers("/test/battery_left") == 0 ||
         ms_node_->count_subscribers("/test/battery_right") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  sensor_msgs::msg::BatteryState left;
  left.percentage = 0.20F;
  left.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  left.serial_number = "PACK-LEFT";
  sensor_msgs::msg::BatteryState right = left;
  right.percentage = 0.80F;
  right.serial_number = "PACK-RIGHT";

  for (int i = 0; i < 400 && (records_["battery_left"].empty() || records_["battery_right"].empty()); ++i)
  {
    pubs_[0]->publish(left);
    pubs_[1]->publish(right);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_FALSE(records_["battery_left"].empty());
  ASSERT_FALSE(records_["battery_right"].empty());
  EXPECT_EQ(records_["battery_left"].back()["serial_number"], "PACK-LEFT");
  EXPECT_NEAR(records_["battery_left"].back()["percentage"].get<double>(), 20.0, 1e-4);
  EXPECT_EQ(records_["battery_right"].back()["serial_number"], "PACK-RIGHT");
  EXPECT_NEAR(records_["battery_right"].back()["percentage"].get<double>(), 80.0, 1e-4);
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
