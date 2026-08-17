// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementMemoryTest : public ::testing::Test
{
protected:
  MeasurementMemoryTest()
  {
    SetUp();
  }

  ~MeasurementMemoryTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "memory" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/memory", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMemoryTest::memoryDataCallback, this, std::placeholders::_1));
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

  void memoryDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    used_ = data_json["used"];
    memory_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  float used_;

public:
  bool memory_callback_{ false };
};

TEST_F(MeasurementMemoryTest, MemoryUsedIsAPercentage)
{
  ms_node_->declare_parameter("memory.plugin", std::string("dc_measurements/Memory"));
  ms_node_->declare_parameter("memory.group_key", std::string("memory"));
  ms_node_->declare_parameter("memory.topic_output", std::string("/dc/measurement/memory"));

  startLifecycleNode();

  while (!memory_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_GE(used_, 0.0);
  EXPECT_LE(used_, 100.0);
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
