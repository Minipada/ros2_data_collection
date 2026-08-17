// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <grp.h>
#include <gtest/gtest.h>
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementPermissionsTest : public ::testing::Test
{
protected:
  MeasurementPermissionsTest()
  {
    SetUp();
  }

  ~MeasurementPermissionsTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "permissions" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/permissions", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementPermissionsTest::permissionsDataCallback, this, std::placeholders::_1));

    test_file_ = (std::filesystem::temp_directory_path() / "dc_measurement_permissions_test_file").u8string();
    std::ofstream(test_file_) << "test";
    chmod(test_file_.c_str(), 0644);
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
    std::filesystem::remove(test_file_);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void permissionsDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;
  std::string test_file_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementPermissionsTest, ReportsOwnerAndPermissionsAsInt)
{
  ms_node_->declare_parameter("permissions.plugin", std::string("dc_measurements/Permissions"));
  ms_node_->declare_parameter("permissions.group_key", std::string("permissions"));
  ms_node_->declare_parameter("permissions.topic_output", std::string("/dc/measurement/permissions"));
  ms_node_->declare_parameter("permissions.path", test_file_);
  ms_node_->declare_parameter("permissions.format", std::string("int"));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_TRUE(data_json_["exists"].get<bool>());
  EXPECT_EQ(data_json_["permissions"].get<std::string>(), "644");
  EXPECT_EQ(data_json_["uid"].get<uid_t>(), getuid());
  EXPECT_EQ(data_json_["gid"].get<gid_t>(), getgid());

  struct passwd* pw = getpwuid(getuid());
  if (pw != nullptr)
  {
    EXPECT_EQ(data_json_["user"].get<std::string>(), pw->pw_name);
  }
}

TEST_F(MeasurementPermissionsTest, ReportsPermissionsAsRwxString)
{
  ms_node_->declare_parameter("permissions.plugin", std::string("dc_measurements/Permissions"));
  ms_node_->declare_parameter("permissions.group_key", std::string("permissions"));
  ms_node_->declare_parameter("permissions.topic_output", std::string("/dc/measurement/permissions"));
  ms_node_->declare_parameter("permissions.path", test_file_);
  ms_node_->declare_parameter("permissions.format", std::string("rwx"));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  // 0644 = rw-r--r--
  EXPECT_EQ(data_json_["permissions"].get<std::string>(), "rw-r--r--");
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
