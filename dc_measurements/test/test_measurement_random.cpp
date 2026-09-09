// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementRandomTest : public MeasurementBench
{
protected:
  MeasurementRandomTest() : MeasurementBench("random")
  {
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("random.plugin", std::string("dc_measurements/Random"));
    ms_node_->declare_parameter("random.group_key", std::string("random"));
    ms_node_->declare_parameter("random.topic_output", std::string("/dc/measurement/random"));
  }
};

TEST_F(MeasurementRandomTest, DefaultIntegerWithinRange)
{
  declareCommonParameters();

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_GE(data_json_["value"].get<double>(), 0);
  EXPECT_LE(data_json_["value"].get<double>(), 100);
}

TEST_F(MeasurementRandomTest, DoubleWithinConfiguredRange)
{
  declareCommonParameters();
  ms_node_->declare_parameter("random.type", std::string("double"));
  ms_node_->declare_parameter("random.min", 5.0);
  ms_node_->declare_parameter("random.max", 6.0);

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_GE(data_json_["value"].get<double>(), 5.0);
  EXPECT_LE(data_json_["value"].get<double>(), 6.0);
}

TEST_F(MeasurementRandomTest, SameSeedProducesSameSequence)
{
  declareCommonParameters();
  ms_node_->declare_parameter("random.min", 0.0);
  ms_node_->declare_parameter("random.max", 1000000.0);
  ms_node_->declare_parameter("random.seed", 42);

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";
  double first_run_value = data_json_["value"].get<double>();

  // A second, independently-configured node with the same `seed` (remapped to its own node
  // name/topic so it can't collide with ms_node_'s rosout publisher, lifecycle services, or
  // Record subscription) must reproduce the exact same first emitted value.
  rclcpp::NodeOptions second_node_options;
  second_node_options.arguments({ "--ros-args", "-r", "__node:=measurement_server_2" });
  auto ms_node_2 = std::make_shared<measurement_server::MeasurementServer>(second_node_options,
                                                                           std::vector<std::string>{ "random" });
  bool second_callback = false;
  double second_run_value = 0;
  auto sub_data_2 = ms_node_2->create_subscription<dc_interfaces::msg::StringStamped>(
      "/dc/measurement/random_2", rclcpp::SystemDefaultsQoS(), [&](const dc_interfaces::msg::StringStamped& msg) {
        second_run_value = MeasurementBench::parseRecord(msg)["value"].get<double>();
        second_callback = true;
      });
  ms_node_2->declare_parameter("random.plugin", std::string("dc_measurements/Random"));
  ms_node_2->declare_parameter("random.group_key", std::string("random"));
  ms_node_2->declare_parameter("random.topic_output", std::string("/dc/measurement/random_2"));
  ms_node_2->declare_parameter("random.min", 0.0);
  ms_node_2->declare_parameter("random.max", 1000000.0);
  ms_node_2->declare_parameter("random.seed", 42);
  ms_node_2->configure();
  ms_node_2->activate();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!second_callback && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(ms_node_2->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(second_callback) << "the second server never published a Record";
  ms_node_2->deactivate();
  ms_node_2->cleanup();

  EXPECT_EQ(first_run_value, second_run_value);
}

DC_MEASUREMENT_TEST_MAIN()
