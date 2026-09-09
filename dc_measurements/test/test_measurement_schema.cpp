// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"

// The schema a Measurement validates against defaults to one file per plugin type, named after
// it (#498). Every plugin the package registers therefore needs a matching file installed, or
// configure() throws "Enabled validation but didn't configure schema!" for that whole type.

std::vector<std::string> registeredPluginTypes()
{
  std::ifstream xml(ament_index_cpp::get_package_share_directory("dc_measurements") + "/measurement_plugin.xml");
  std::string content((std::istreambuf_iterator<char>(xml)), std::istreambuf_iterator<char>());

  std::vector<std::string> types;
  const std::string needle = "class name=\"";
  for (size_t pos = content.find(needle); pos != std::string::npos; pos = content.find(needle, pos))
  {
    pos += needle.size();
    types.push_back(content.substr(pos, content.find('"', pos) - pos));
  }
  return types;
}

std::string schemaPath(const std::string& plugin_type)
{
  return ament_index_cpp::get_package_share_directory("dc_measurements") + "/plugins/measurements/json/" +
         dc_measurements::Measurement::defaultSchemaFile(plugin_type);
}

TEST(MeasurementSchemaTest, SnakeCaseKeepsAcronymsWithTheWordTheyPrecede)
{
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("OS"), "os");
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("TCPHealth"), "tcp_health");
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("IpCamera"), "ip_camera");
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("Ros2ControlStatus"), "ros2_control_status");
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("MissionNav2ThroughPoses"), "mission_nav2_through_poses");
  EXPECT_EQ(dc_measurements::Measurement::snakeCase("StringStamped"), "string_stamped");
}

TEST(MeasurementSchemaTest, DefaultSchemaFileTakesTheTypeNotTheWholeLookupName)
{
  // The package part is not a CamelCase word: converting the whole lookup name would nest it
  // under json/ as a directory, where nothing is installed.
  EXPECT_EQ(dc_measurements::Measurement::defaultSchemaFile("dc_measurements/Battery"), "battery.json");
  EXPECT_EQ(dc_measurements::Measurement::defaultSchemaFile("dc_measurements/TCPHealth"), "tcp_health.json");
  EXPECT_EQ(dc_measurements::Measurement::defaultSchemaFile("dc_demos/UptimeCustom"), "uptime_custom.json");
}

TEST(MeasurementSchemaTest, EveryRegisteredPluginHasItsOwnSchemaFile)
{
  // StringStamped republishes another Measurement's Record verbatim, so there is no shape to
  // constrain: it is the one type with no schema, and a deployment of it must say so with
  // enable_validator: false.
  const std::set<std::string> schemaless = { "dc_measurements/StringStamped" };

  const std::vector<std::string> types = registeredPluginTypes();
  ASSERT_FALSE(types.empty());

  std::set<std::string> schema_names;
  for (const std::string& type : types)
  {
    if (schemaless.count(type) != 0)
    {
      EXPECT_FALSE(std::filesystem::exists(schemaPath(type)))
          << type << " gained a schema; drop it from the schemaless list";
      continue;
    }
    EXPECT_TRUE(std::filesystem::exists(schemaPath(type)))
        << type << " derives a default schema of " << schemaPath(type) << ", which is not installed";
    schema_names.insert(dc_measurements::Measurement::defaultSchemaFile(type));
  }

  // Two plugins deriving the same file name would make one of them validate against a schema
  // written for the other.
  EXPECT_EQ(schema_names.size(), types.size() - schemaless.size());
}

// The schema follows the plugin type, not the instance id: a Measurement may be named whatever
// the deployment likes and still find its plugin's schema.
class MeasurementAliasedSchemaTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "mem_alias" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/mem_alias", rclcpp::SystemDefaultsQoS(), [this](const dc_interfaces::msg::StringStamped& msg) {
          nlohmann::json data_json = nlohmann::json::parse(msg.data);
          used_ = data_json["used"];
          callback_ = true;
        });
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  float used_{ 0.0 };

public:
  bool callback_{ false };
};

TEST_F(MeasurementAliasedSchemaTest, ConfiguresAndPublishesWithTheDefaultSchema)
{
  ms_node_->declare_parameter("mem_alias.plugin", std::string("dc_measurements/Memory"));
  ms_node_->declare_parameter("mem_alias.topic_output", std::string("/dc/measurement/mem_alias"));
  // enable_validator is left at its default of true: configuring at all is what proves the
  // schema was found.

  ms_node_->configure();
  ms_node_->activate();

  const auto start = std::chrono::steady_clock::now();
  while (!callback_ && std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_TRUE(callback_);
  EXPECT_GE(used_, 0.0);
  EXPECT_LE(used_, 100.0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
