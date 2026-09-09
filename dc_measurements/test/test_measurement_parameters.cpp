// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <sys/utsname.h>

#include "measurement_test_bench.hpp"

class MeasurementParametersTest : public MeasurementBench
{
protected:
  MeasurementParametersTest() : MeasurementBench("os")
  {
  }
};

TEST_F(MeasurementParametersTest, PollingIntervalOneMeasurementWithinAFewPercent)
{
  int polling_interval = 30;
  int count_measurement = 90;
  // A 1% window fired exactly count_measurement times proved flaky: executor jitter on a loaded
  // (or coverage-instrumented) runner delays a fire past the window, off by one. The test exists
  // to catch a mis-applied polling_interval, not to bound scheduler latency.
  int error = 3;
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.plugin", rclcpp::ParameterValue("dc_measurements/OS"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.group_key", rclcpp::ParameterValue("os"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.topic_output",
                                               rclcpp::ParameterValue("/dc/measurement/os"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.polling_interval",
                                               rclcpp::ParameterValue(polling_interval));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.init_collect", rclcpp::ParameterValue(false));

  startLifecycleNode();

  // Verify parameters are set properly
  EXPECT_EQ(std::vector<std::string>({ "os" }), ms_node_->get_parameter("measurement_plugins").as_string_array());
  EXPECT_EQ("dc_measurements/OS", ms_node_->get_parameter("os.plugin").as_string());
  EXPECT_EQ("os", ms_node_->get_parameter("os.group_key").as_string());
  EXPECT_EQ("/dc/measurement/os", ms_node_->get_parameter("os.topic_output").as_string());
  EXPECT_EQ(polling_interval, static_cast<int>(ms_node_->get_parameter("os.polling_interval").as_int()));
  EXPECT_FALSE(ms_node_->get_parameter("os.init_collect").as_bool());

  // Verify that in a certain amount of time, roughly that rate of samples are collected
  spinFor(polling_interval * (count_measurement + error));

  EXPECT_NEAR(callback_count_, count_measurement, error);
}

DC_MEASUREMENT_TEST_MAIN()
