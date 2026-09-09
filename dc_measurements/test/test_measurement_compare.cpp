// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_core/condition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "measurement_test_bench.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// One suite for the whole collapsed compare family (#486): every operator x operand-type
// combination the fourteen deleted plugins covered, through the same if_all_conditions gating
// path a real Measurement uses. Given a fixed Record (the dummy Measurement's static "record"
// JSON) and a condition config (key/comparison/value), assert whether collection is activated
// or suppressed.
class MeasurementCompareTest : public MeasurementBench
{
protected:
  MeasurementCompareTest()
    // condition_plugins is declared by the constructor itself, so it must be supplied as a
    // parameter override rather than via ms_node_->declare_parameter afterwards.
    : MeasurementBench("dummy", rclcpp::NodeOptions().parameter_overrides(
                                    { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "cmp" }) }))
  {
    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "cmp" });
    // Route collection entirely through if_all_conditions: disable the unconditional
    // init-publish path (-1) and allow unlimited publishes once the condition is on (0).
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("cmp.plugin", std::string("dc_conditions/Compare"));
  }

  void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg) override
  {
    (void)measurement;
    (void)msg;
    callback_count_++;
  }

  // A condition that should activate but never does fails on the EXPECT instead of hanging CI.
  void awaitFirstPublish()
  {
    ASSERT_TRUE(spinUntil([this] { return callback_count_ > 0; }, 5000)) << "no Record was ever published";
  }

  int polling_interval_{ 50 };
};

// -------------------------------------------------------------------------------------------------
// eq / ne, boolean operand (was BoolEqual -- whose `value` member was declared double, so any
// boolean value made configure throw; that known bug is not carried over).
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, BoolEqValueEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flag\": true}"));
  ms_node_->declare_parameter("cmp.key", std::string("flag"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", true);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, BoolEqValueDifferingNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flag\": true}"));
  ms_node_->declare_parameter("cmp.key", std::string("flag"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", false);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, BoolNeValueDifferingActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flag\": true}"));
  ms_node_->declare_parameter("cmp.key", std::string("flag"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", false);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, BoolNeValueEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flag\": true}"));
  ms_node_->declare_parameter("cmp.key", std::string("flag"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", true);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// eq / ne, double operand (was DoubleEqual)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, DoubleEqValueEqualActivatesAndKeepsPublishing)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5.5);

  startLifecycleNode();
  awaitFirstPublish();
  int first_count = callback_count_;

  // if_all_conditions keeps being satisfied every poll since the Record never changes.
  spinFor(polling_interval_ * 3);
  EXPECT_GT(callback_count_, first_count);
}

TEST_F(MeasurementCompareTest, DoubleEqValueDifferingNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 3.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleEqMissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5.5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// Gotcha: the comparison is type-strict, not just numeric. A Record field written as an integer
// literal (no decimal point) is numerically equal to the configured double but parses to
// number_unsigned, not number_float, so it never matches.
TEST_F(MeasurementCompareTest, DoubleEqIntegerLiteralInRecordNeverMatchesDespiteNumericEquality)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleNeValueDifferingActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", 3.0);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleNeValueEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", 5.5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// eq / ne, integer operand (was IntegerEqual)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, IntegerEqValueEqualActivatesAndKeepsPublishing)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  awaitFirstPublish();
  int first_count = callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(callback_count_, first_count);
}

TEST_F(MeasurementCompareTest, IntegerEqValueDifferingNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 3);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerEqMissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// Gotcha (mirror of the double case): a Record field written as a float literal parses to
// number_float, not an integer type, so it never matches an integer operand.
TEST_F(MeasurementCompareTest, IntegerEqDoubleLiteralInRecordNeverMatchesDespiteNumericEquality)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerNeValueDifferingActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", 3);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

// -------------------------------------------------------------------------------------------------
// eq / ne, string operand (new with Compare: the family had no scalar string plugin)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, StringEqValueEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::string("ok"));

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, StringEqValueDifferingNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::string("error"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, StringEqMissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": \"ok\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::string("ok"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, StringNeValueDifferingActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ne"));
  ms_node_->declare_parameter("cmp.value", std::string("error"));

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

// -------------------------------------------------------------------------------------------------
// le / lt / ge / gt, double operand (was DoubleInferior + DoubleSuperior, whose include_value
// parameter chose strict vs inclusive)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, DoubleLeBelowConfiguredValueActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 8.0);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleLeAboveConfiguredValueNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 3.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleLeBoundaryEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 5.0);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleLtBoundaryEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("lt"));
  ms_node_->declare_parameter("cmp.value", 5.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleGeAboveConfiguredValueActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 3.0);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleGeBelowConfiguredValueNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 8.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleGeBoundaryEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 5.0);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleGtBoundaryEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("gt"));
  ms_node_->declare_parameter("cmp.value", 5.0);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// le / lt / ge / gt, integer operand (was IntegerInferior + IntegerSuperior)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, IntegerLeBelowConfiguredValueActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 8);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerLeAboveConfiguredValueNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 3);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerLeBoundaryEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("le"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerLtBoundaryEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("lt"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerGeAboveConfiguredValueActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 3);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerGeBelowConfiguredValueNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 8);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerGeBoundaryEqualActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("ge"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerGtBoundaryEqualNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("gt"));
  ms_node_->declare_parameter("cmp.value", 5);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// eq, array operands (was ListBoolEqual / ListDoubleEqual / ListIntegerEqual /
// ListStringEqual, whose order_matters parameter chose exact-order vs sorted comparison)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, BoolArrayExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": [true, false, true]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ true, false, true });

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, BoolArrayReorderedNeverActivatesWhenOrderMatters)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": [true, false, true]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ false, true, true });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, BoolArrayReorderedActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": [true, false, true]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ false, true, true });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, BoolArrayDifferentElementsNeverActivateWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": [true, false, true]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ false, false, false });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, BoolArrayOfIntegersNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": [1, 2, 3]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ true, false, true });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleArrayExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"levels\": [1.5, 2.5]}"));
  ms_node_->declare_parameter("cmp.key", std::string("levels"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<double>{ 1.5, 2.5 });

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleArrayReorderedNeverActivatesWhenOrderMatters)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"levels\": [1.5, 2.5]}"));
  ms_node_->declare_parameter("cmp.key", std::string("levels"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<double>{ 2.5, 1.5 });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, DoubleArrayReorderedActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"levels\": [1.5, 2.5]}"));
  ms_node_->declare_parameter("cmp.key", std::string("levels"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<double>{ 2.5, 1.5 });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, DoubleArrayDifferentElementsNeverActivateWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"levels\": [1.5, 2.5]}"));
  ms_node_->declare_parameter("cmp.key", std::string("levels"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<double>{ 9.9, 9.9 });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// Mirror of the scalar type strictness: integer literals are not double elements.
TEST_F(MeasurementCompareTest, DoubleArrayOfIntegerLiteralsNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"levels\": [1, 2]}"));
  ms_node_->declare_parameter("cmp.key", std::string("levels"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<double>{ 1.5, 2.5 });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerArrayExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"counts\": [1, 2, 3]}"));
  ms_node_->declare_parameter("cmp.key", std::string("counts"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<int64_t>{ 1, 2, 3 });

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerArrayReorderedNeverActivatesWhenOrderMatters)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"counts\": [1, 2, 3]}"));
  ms_node_->declare_parameter("cmp.key", std::string("counts"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<int64_t>{ 3, 2, 1 });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerArrayReorderedActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"counts\": [1, 2, 3]}"));
  ms_node_->declare_parameter("cmp.key", std::string("counts"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<int64_t>{ 3, 2, 1 });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, IntegerArrayDifferentElementsNeverActivateWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"counts\": [1, 2, 3]}"));
  ms_node_->declare_parameter("cmp.key", std::string("counts"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<int64_t>{ 9, 9, 9 });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, IntegerArrayOfDoubleLiteralsNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"counts\": [1.0, 2.0, 3.0]}"));
  ms_node_->declare_parameter("cmp.key", std::string("counts"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<int64_t>{ 1, 2, 3 });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, StringArrayExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"names\": [\"a\", \"b\", \"c\"]}"));
  ms_node_->declare_parameter("cmp.key", std::string("names"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<std::string>{ "a", "b", "c" });

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, StringArrayReorderedNeverActivatesWhenOrderMatters)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"names\": [\"a\", \"b\", \"c\"]}"));
  ms_node_->declare_parameter("cmp.key", std::string("names"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<std::string>{ "c", "b", "a" });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, StringArrayReorderedActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"names\": [\"a\", \"b\", \"c\"]}"));
  ms_node_->declare_parameter("cmp.key", std::string("names"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<std::string>{ "c", "b", "a" });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, StringArrayDifferentElementsNeverActivateWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"names\": [\"a\", \"b\", \"c\"]}"));
  ms_node_->declare_parameter("cmp.key", std::string("names"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<std::string>{ "x", "y", "z" });
  ms_node_->declare_parameter("cmp.order_matters", false);

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, ArrayScalarKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"flags\": true}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ true });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, ArrayMissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": [true]}"));
  ms_node_->declare_parameter("cmp.key", std::string("flags"));
  ms_node_->declare_parameter("cmp.comparison", std::string("eq"));
  ms_node_->declare_parameter("cmp.value", std::vector<bool>{ true });

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// match (was StringMatch)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, MatchRegexFullMatchActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("match"));
  ms_node_->declare_parameter("cmp.regex", std::string("ok-[0-9]+"));

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

TEST_F(MeasurementCompareTest, MatchRegexNotMatchingNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("match"));
  ms_node_->declare_parameter("cmp.regex", std::string("error-[0-9]+"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// std::regex_match requires a *full* match, not a substring search.
TEST_F(MeasurementCompareTest, MatchPartialSubstringNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("match"));
  ms_node_->declare_parameter("cmp.regex", std::string("ok"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

TEST_F(MeasurementCompareTest, MatchMissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": \"ok-123\"}"));
  ms_node_->declare_parameter("cmp.key", std::string("status"));
  ms_node_->declare_parameter("cmp.comparison", std::string("match"));
  ms_node_->declare_parameter("cmp.regex", std::string("ok-[0-9]+"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// -------------------------------------------------------------------------------------------------
// exists (was Exist)
// -------------------------------------------------------------------------------------------------

TEST_F(MeasurementCompareTest, ExistsKeyPresentActivatesAndKeepsPublishing)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("exists"));

  startLifecycleNode();
  awaitFirstPublish();
  int first_count = callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(callback_count_, first_count);
}

TEST_F(MeasurementCompareTest, ExistsKeyAbsentNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": 5.5}"));
  ms_node_->declare_parameter("cmp.key", std::string("level"));
  ms_node_->declare_parameter("cmp.comparison", std::string("exists"));

  startLifecycleNode();
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(callback_count_, 0);
}

// A nested key (found via the flattened "/parent/child" prefix match, not an exact top-level
// key) still counts as existing.
TEST_F(MeasurementCompareTest, ExistsNestedKeyActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"parent\": {\"child\": 1}}"));
  ms_node_->declare_parameter("cmp.key", std::string("parent"));
  ms_node_->declare_parameter("cmp.comparison", std::string("exists"));

  startLifecycleNode();
  awaitFirstPublish();
  EXPECT_GE(callback_count_, 1);
}

// -------------------------------------------------------------------------------------------------
// Configuration errors: loaded directly (bypassing the Measurement/if_all_conditions harness)
// so the failure surfaces as a plain, isolated exception from configure() instead of being
// swallowed by MeasurementServer's lifecycle-transition error handling.
// -------------------------------------------------------------------------------------------------

TEST(CompareDirectTest, UnknownComparisonThrowsOnConfigure)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("compare_direct_test_node");
  node->declare_parameter("cmp.key", std::string("level"));
  node->declare_parameter("cmp.comparison", std::string("foo"));
  node->declare_parameter("cmp.value", 5.5);

  pluginlib::ClassLoader<dc_core::Condition> loader("dc_core", "dc_core::Condition");
  auto condition = loader.createUniqueInstance("dc_conditions/Compare");

  EXPECT_THROW(condition->configure(node, "cmp"), std::runtime_error);
}

// Through the real MeasurementServer wiring, the same exception is thrown from inside
// loadConditionPlugins()'s plugin-creation loop, which only catches pluginlib exceptions --
// but rclcpp_lifecycle's transition machinery wraps every registered transition callback
// (on_configure here) in its own catch, converting an uncaught exception into a
// CallbackReturn::ERROR rather than letting it propagate to the caller of configure(). So no
// exception crosses this test's own call to configure(); the observable, end-to-end
// consequence of a bad `comparison` value is that the whole MeasurementServer fails to reach
// the "inactive" state.
TEST(MeasurementCompareServerTest, InvalidComparisonFailsToReachInactiveState)
{
  auto options = rclcpp::NodeOptions().parameter_overrides(
      { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "cmp" }) });
  auto ms_node = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });

  ms_node->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node->declare_parameter("cmp.plugin", std::string("dc_conditions/Compare"));
  ms_node->declare_parameter("cmp.key", std::string("level"));
  ms_node->declare_parameter("cmp.comparison", std::string("foo"));
  ms_node->declare_parameter("cmp.value", 5.5);

  auto result_state = ms_node->configure();
  EXPECT_NE(result_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST(CompareDirectTest, OrderedComparisonWithStringValueThrowsOnConfigure)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("compare_direct_test_node");
  node->declare_parameter("cmp.key", std::string("status"));
  node->declare_parameter("cmp.comparison", std::string("gt"));
  node->declare_parameter("cmp.value", std::string("x"));

  pluginlib::ClassLoader<dc_core::Condition> loader("dc_core", "dc_core::Condition");
  auto condition = loader.createUniqueInstance("dc_conditions/Compare");

  EXPECT_THROW(condition->configure(node, "cmp"), std::runtime_error);
}

TEST(CompareDirectTest, MissingValueThrowsOnConfigure)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("compare_direct_test_node");
  node->declare_parameter("cmp.key", std::string("level"));
  node->declare_parameter("cmp.comparison", std::string("eq"));

  pluginlib::ClassLoader<dc_core::Condition> loader("dc_core", "dc_core::Condition");
  auto condition = loader.createUniqueInstance("dc_conditions/Compare");

  EXPECT_THROW(condition->configure(node, "cmp"), std::runtime_error);
}

DC_MEASUREMENT_TEST_MAIN()
