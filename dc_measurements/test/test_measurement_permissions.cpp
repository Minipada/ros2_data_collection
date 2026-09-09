// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "measurement_test_bench.hpp"

class MeasurementPermissionsTest : public MeasurementBench
{
protected:
  MeasurementPermissionsTest() : MeasurementBench("permissions")
  {
    test_file_ = (std::filesystem::temp_directory_path() / "dc_measurement_permissions_test_file").u8string();
    std::ofstream(test_file_) << "test";
    chmod(test_file_.c_str(), 0644);
  }

  ~MeasurementPermissionsTest() override
  {
    std::filesystem::remove(test_file_);
  }

  std::string test_file_;
};

TEST_F(MeasurementPermissionsTest, ReportsOwnerAndPermissionsAsInt)
{
  ms_node_->declare_parameter("permissions.plugin", std::string("dc_measurements/Permissions"));
  ms_node_->declare_parameter("permissions.group_key", std::string("permissions"));
  ms_node_->declare_parameter("permissions.topic_output", std::string("/dc/measurement/permissions"));
  ms_node_->declare_parameter("permissions.path", test_file_);
  ms_node_->declare_parameter("permissions.format", std::string("int"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

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

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  // 0644 = rw-r--r--
  EXPECT_EQ(data_json_["permissions"].get<std::string>(), "rw-r--r--");
}

DC_MEASUREMENT_TEST_MAIN()
