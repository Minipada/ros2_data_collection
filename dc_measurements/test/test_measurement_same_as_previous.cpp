// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

// Exercises dc_conditions::SameAsPrevious through the if_all_conditions gating path: given a
// fixed Record (the dummy Measurement's static "record" JSON) and a condition config (no "keys"
// configured, so it compares the whole flattened Record rather than hashing specific file
// fields), assert whether collection is activated or suppressed.
//
// A Dummy Measurement's "record" param is static for the node's lifetime (read once in
// onConfigure(), never re-read per poll), so this fixture can only observe "the Record never
// changes" -- SameAsPrevious's active_=false transition when a *changed* Record arrives is not
// covered here; it would need a Record source whose content can vary between polls, which no
// existing Measurement plugin exposes as a live-updatable parameter.
class MeasurementSameAsPreviousTest : public MeasurementBench
{
protected:
  MeasurementSameAsPreviousTest()
    : MeasurementBench("dummy", rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter(
                                    "condition_plugins", std::vector<std::string>{ "same_prev" }) }))
  {
    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "same_prev" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("same_prev.plugin", std::string("dc_conditions/SameAsPrevious"));
    ms_node_->declare_parameter("same_prev.keys", std::vector<std::string>());
    ms_node_->declare_parameter("same_prev.exclude", std::vector<std::string>());
  }

  void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg) override
  {
    (void)measurement;
    (void)msg;
    callback_count_++;
  }

  // Deliberately wider than the other test files here: the very first poll is guaranteed
  // suppressed (previous_json_ starts empty) while the second poll is guaranteed to activate
  // (the record never changes), so distinguishing "before poll #2" from "at/after poll #2" needs
  // a window with real margin instead of racing a tight timer.
  int polling_interval_{ 200 };
};

// Acceptance criterion: the very first Record has nothing to compare against
// (previous_json_.empty()) -> the condition is inactive and collection is suppressed. Checked
// well inside the first polling_interval_ window, before a second poll could activate it.
TEST_F(MeasurementSameAsPreviousTest, FirstCollectionNeverActivates)
{
  startLifecycleNode();

  spinFor(polling_interval_ - 100);
  EXPECT_EQ(callback_count_, 0);
}

// Acceptance criterion: once a previous Record exists, an identical subsequent Record activates
// the condition -- and since the dummy Record never changes, it keeps activating from then on.
TEST_F(MeasurementSameAsPreviousTest, IdenticalSubsequentRecordsActivateFromSecondCollectionOnward)
{
  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_count_ > 0; }, 10000)) << "No Record was ever published";
  const int first_count = callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(callback_count_, first_count);
}

DC_MEASUREMENT_TEST_MAIN()
