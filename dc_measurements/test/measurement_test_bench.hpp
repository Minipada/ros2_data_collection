// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__TEST__MEASUREMENT_TEST_BENCH_HPP_
#define DC_MEASUREMENTS__TEST__MEASUREMENT_TEST_BENCH_HPP_

#include <gtest/gtest.h>

#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "rclcpp/rclcpp.hpp"

// The shared bench for this package's suites (#504): the MeasurementServer fixture, the Record
// capture and the bounded waits in one place, so a suite is its fixture line plus its
// assertions. Every wait is bounded -- a behavior regression fails the test loudly instead of
// hanging until the suite's outer timeout.
class MeasurementBench : public ::testing::Test
{
protected:
  explicit MeasurementBench(std::string measurement, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : MeasurementBench(std::vector<std::string>{ std::move(measurement) }, options)
  {
  }

  MeasurementBench(std::vector<std::string> measurements, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : measurements_(std::move(measurements))
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, measurements_);
    for (const std::string& name : measurements_)
    {
      subs_.push_back(ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
          "/dc/measurement/" + name, rclcpp::SystemDefaultsQoS(),
          [this, name](const dc_interfaces::msg::StringStamped& msg) { onRecord(name, msg); }));
    }
  }

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: suites that stop collection mid-test on purpose call this too, and TearDown must
  // not then drive the lifecycle node through a transition it is no longer in a state for.
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

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  // The Record capture. The default logs, normalizes the quotes and files the Record away;
  // suites that capture differently (a count, raw strings, per-measurement buckets) override it.
  virtual void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg)
  {
    (void)measurement;
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << normalize(msg));
    data_json_ = parseRecord(msg);
    records_.push_back(data_json_);
    callback_active_ = true;
    callback_count_++;
  }

  // Suites with helper nodes that must be spun alongside the server override this.
  virtual void spinExtra()
  {
  }

  void spinOnce()
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    spinExtra();
  }

  void spinFor(int milliseconds)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < deadline)
    {
      spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Spin until `done` holds, giving up after `timeout_ms`. A `done` that also publishes (the
  // publish-until-a-Record-shows-up idiom) fits the same helper.
  bool spinUntil(const std::function<bool()>& done, int timeout_ms = 5000)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!done())
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        return false;
      }
      spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  void waitForSubscriber(const std::string& topic, int timeout_ms = 5000)
  {
    ASSERT_TRUE(spinUntil([&] { return ms_node_->count_subscribers(topic) > 0; }, timeout_ms))
        << "no subscriber ever appeared on " << topic;
  }

  static std::string normalize(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data;
    boost::replace_all(data_str, "'", "\"");
    return data_str;
  }

  static nlohmann::json parseRecord(const dc_interfaces::msg::StringStamped& msg)
  {
    return nlohmann::json::parse(normalize(msg));
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  std::vector<std::string> measurements_;
  std::vector<rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr> subs_;
  nlohmann::json data_json_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };

public:
  bool callback_active_{ false };
  int callback_count_{ 0 };
};

// The one main for every suite in this package: gtest plus rclcpp around RUN_ALL_TESTS.
#define DC_MEASUREMENT_TEST_MAIN()                                                                                     \
  int main(int argc, char** argv)                                                                                      \
  {                                                                                                                    \
    ::testing::InitGoogleTest(&argc, argv);                                                                            \
    rclcpp::init(argc, argv);                                                                                          \
    const bool all_successful = RUN_ALL_TESTS() == 0;                                                                  \
    rclcpp::shutdown();                                                                                                \
    return all_successful ? 0 : 1;                                                                                     \
  }

#endif  // DC_MEASUREMENTS__TEST__MEASUREMENT_TEST_BENCH_HPP_
