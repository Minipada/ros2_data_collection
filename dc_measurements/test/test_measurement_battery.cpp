// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <nlohmann/json-schema.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "measurement_test_bench.hpp"

class MeasurementBatteryTest : public MeasurementBench
{
protected:
  MeasurementBatteryTest() : MeasurementBench("battery")
  {
    battery_pub_ =
        ms_node_->create_publisher<sensor_msgs::msg::BatteryState>("/test/battery_state", rclcpp::SensorDataQoS());
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

  // Republishes `msg` until a *new* Record matching `predicate` shows up, so a best-effort sample
  // lost before the plugin subscribed doesn't make the test flaky, and an earlier Record of the
  // same shape isn't mistaken for the one this step is waiting on.
  nlohmann::json publishUntilRecord(const sensor_msgs::msg::BatteryState& msg,
                                    const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < deadline)
    {
      battery_pub_->publish(msg);
      spinOnce();
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

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
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
  spinFor(500);

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
class MeasurementTwoBatteriesTest : public MeasurementBench
{
protected:
  MeasurementTwoBatteriesTest() : MeasurementBench(std::vector<std::string>{ "battery_left", "battery_right" })
  {
    for (const auto& pack : { "left", "right" })
    {
      const std::string name = std::string("battery_") + pack;
      ms_node_->declare_parameter(name + ".plugin", std::string("dc_measurements/Battery"));
      ms_node_->declare_parameter(name + ".group_key", name);
      ms_node_->declare_parameter(name + ".topic_output", "/dc/measurement/" + name);
      ms_node_->declare_parameter(name + ".topic", "/test/" + name);
      ms_node_->declare_parameter(name + ".polling_interval", 50);
      ms_node_->declare_parameter(name + ".init_collect", false);

      pubs_.push_back(
          ms_node_->create_publisher<sensor_msgs::msg::BatteryState>("/test/" + name, rclcpp::SensorDataQoS()));
    }
  }

  // Each pack's Records bucketed by its Measurement's name, so the test can tell them apart.
  void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg) override
  {
    records_by_measurement_[measurement].push_back(parseRecord(msg));
  }

  std::vector<rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr> pubs_;

public:
  std::map<std::string, std::vector<nlohmann::json>> records_by_measurement_;
};

TEST_F(MeasurementTwoBatteriesTest, EachPackReportsItsOwnRecords)
{
  ms_node_->configure();
  ms_node_->activate();

  // The plugins' subscriptions on their input topics only exist once activated.
  waitForSubscriber("/test/battery_left");
  waitForSubscriber("/test/battery_right");

  sensor_msgs::msg::BatteryState left;
  left.percentage = 0.20F;
  left.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  left.serial_number = "PACK-LEFT";
  sensor_msgs::msg::BatteryState right = left;
  right.percentage = 0.80F;
  right.serial_number = "PACK-RIGHT";

  ASSERT_TRUE(spinUntil(
      [&] {
        pubs_[0]->publish(left);
        pubs_[1]->publish(right);
        return !records_by_measurement_["battery_left"].empty() && !records_by_measurement_["battery_right"].empty();
      },
      5000))
      << "no Record ever arrived for both packs";

  const auto& left_records = records_by_measurement_["battery_left"];
  const auto& right_records = records_by_measurement_["battery_right"];
  EXPECT_EQ(left_records.back()["serial_number"], "PACK-LEFT");
  EXPECT_NEAR(left_records.back()["percentage"].get<double>(), 20.0, 1e-4);
  EXPECT_EQ(right_records.back()["serial_number"], "PACK-RIGHT");
  EXPECT_NEAR(right_records.back()["percentage"].get<double>(), 80.0, 1e-4);
}

DC_MEASUREMENT_TEST_MAIN()
