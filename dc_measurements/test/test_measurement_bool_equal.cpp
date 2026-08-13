#include <gtest/gtest.h>

#include <chrono>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// dc_conditions::BoolEqual has a known, already-flagged bug: its `value_` member is declared
// `double` (dc_measurements/plugins/conditions/bool_equal.hpp) even though the "value" parameter
// is declared PARAMETER_BOOL. bool_equal.cpp's onConfigure() carries a NOTE explaining this was
// deliberately left alone during #178 (a parameter-declaration-unification pass) as "worth its
// own follow-up" rather than fixed inline. Given that documented, deliberate deferral, this test
// file documents the *actual* current behavior rather than silently fixing or working around it.
//
// Loads the plugin directly (bypassing the Measurement/if_all_conditions harness used elsewhere
// in this directory) so the failure surfaces as a plain, isolated exception from configure()
// instead of being swallowed by MeasurementServer's own lifecycle-transition error handling.
TEST(BoolEqualDirectTest, ConfiguringWithABooleanValueThrowsDueToKnownTypeMismatch)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bool_eq_direct_test_node");
  node->declare_parameter("bool_eq.key", std::string("flag"));
  node->declare_parameter("bool_eq.value", true);

  pluginlib::ClassLoader<dc_core::Condition> loader("dc_core", "dc_core::Condition");
  auto condition = loader.createUniqueInstance("dc_conditions/BoolEqual");

  // rclcpp::Node::get_parameter(name, double&) on a PARAMETER_BOOL-typed parameter throws
  // rclcpp::exceptions::InvalidParameterTypeException -- BoolEqual::onConfigure() does not catch
  // it, so it propagates straight out of configure().
  EXPECT_THROW(condition->configure(node, "bool_eq"), rclcpp::exceptions::InvalidParameterTypeException);
}

// Through the real MeasurementServer wiring, the same exception is thrown from inside
// loadConditionPlugins()'s plugin-creation loop, which only catches
// pluginlib::PluginlibException -- but rclcpp_lifecycle's transition machinery wraps every
// registered transition callback (on_configure here) in its own catch, converting an uncaught
// exception into a CallbackReturn::ERROR rather than letting it propagate to the caller of
// configure() (confirmed empirically: "Callback returned ERROR during the transition: configure").
// So unlike the direct-load case above, no exception crosses this test's own call to configure();
// the observable, end-to-end consequence for anyone who configures a `dc_conditions/BoolEqual`
// condition with a "value" parameter at all is instead that the whole MeasurementServer fails to
// reach the "inactive" state.
TEST(BoolEqualDirectTest, MeasurementServerConfigureFailsToReachInactiveState)
{
  auto options = rclcpp::NodeOptions().parameter_overrides(
      { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "bool_eq" }) });
  auto ms_node = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });

  ms_node->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));

  ms_node->declare_parameter("bool_eq.plugin", std::string("dc_conditions/BoolEqual"));
  ms_node->declare_parameter("bool_eq.key", std::string("flag"));
  ms_node->declare_parameter("bool_eq.value", true);

  auto result_state = ms_node->configure();
  EXPECT_NE(result_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
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
