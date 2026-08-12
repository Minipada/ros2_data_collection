// Bond timeout handling and respawn behaviour of `lifecycle_manager_dc`.
//
// A managed node is made to look dead by stopping its executor: it stays discoverable,
// but stops answering heartbeats, which is what the manager's bond actually watches.
// Spinning is restored afterwards so the manager can drive the node through the hard
// reset it triggers — a permanently dead node would block the manager's own (untimed)
// change_state call rather than exercise the recovery path.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dc_lifecycle_manager/lifecycle_manager_client.hpp"
#include "managed_node_harness.hpp"

using dc_lifecycle_manager::SystemStatus;
using dc_lifecycle_manager_test::contains;
using dc_lifecycle_manager_test::countOf;
using dc_lifecycle_manager_test::DummyManagedNode;
using dc_lifecycle_manager_test::managerOptions;
using dc_lifecycle_manager_test::TestableLifecycleManager;
using dc_lifecycle_manager_test::TransitionRecorder;
using namespace std::chrono_literals;  // NOLINT

namespace
{
/// Heartbeat timeout the manager is configured with in these tests. Also the DC
/// default: the manager gives a bond half of it to form, which a shorter value can
/// lose the race against.
constexpr double kBondTimeout = 4.0;
/// How long to let a fresh bond beat before silencing the node behind it. bondcpp arms
/// its heartbeat-timeout timer from the *second* heartbeat it receives (the first only
/// moves the bond out of "waiting for sister"), so a node silenced the instant its bond
/// forms is never declared dead at all.
constexpr auto kBondSettleDuration = 2s;
/// How long a node is silenced for: past kBondTimeout plus the bond's own disconnect
/// timeout, so the manager's side of the bond is conclusively dead before the node
/// starts answering again.
constexpr auto kSilenceDuration = 8s;
}  // namespace

class LifecycleManagerBondTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    recorder_ = std::make_shared<TransitionRecorder>();
  }

  void TearDown() override
  {
    client_.reset();
    client_node_.reset();
    manager_thread_.reset();
    manager_.reset();
    nodes_.clear();
  }

  void spawnManagedNodes(const std::vector<std::string>& names)
  {
    for (const auto& name : names)
    {
      auto node = std::make_shared<DummyManagedNode>(name, recorder_, true);
      node->startSpinning();
      nodes_.push_back(node);
    }
  }

  /// Start a manager named after the running test, and a client wired to it.
  void startManager(const std::vector<std::string>& node_names, bool attempt_respawn_reconnection)
  {
    const std::string manager_name = managerName();
    manager_ = std::make_shared<dc_lifecycle_manager::LifecycleManager>(managerOptions(
        manager_name, node_names, { "configure", "activate" }, true, kBondTimeout, attempt_respawn_reconnection, 60.0));
    manager_thread_ = std::make_unique<nav2_util::NodeThread>(manager_->get_node_base_interface());
    client_node_ = std::make_shared<rclcpp::Node>(manager_name + "_client");
    client_ = std::make_unique<dc_lifecycle_manager::LifecycleManagerClient>(manager_name, client_node_);
  }

  /// A manager node name unique to the running test.
  static std::string managerName()
  {
    return std::string("lifecycle_manager_") + ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  /// Stop a managed node's heartbeats for long enough that the manager's bond times out.
  void silenceManagedNode(std::size_t index)
  {
    std::this_thread::sleep_for(kBondSettleDuration);
    nodes_[index]->stopSpinning();
    std::this_thread::sleep_for(kSilenceDuration);
    nodes_[index]->startSpinning();
  }

  std::shared_ptr<TransitionRecorder> recorder_;
  std::vector<std::shared_ptr<DummyManagedNode>> nodes_;
  std::shared_ptr<dc_lifecycle_manager::LifecycleManager> manager_;
  std::unique_ptr<nav2_util::NodeThread> manager_thread_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<dc_lifecycle_manager::LifecycleManagerClient> client_;
};

// One managed node losing its bond takes the whole managed set down; with respawn
// reconnection enabled the manager then brings the set back up on its own.
TEST_F(LifecycleManagerBondTest, BondTimeoutTakesSystemDownAndRespawnBringsItBack)
{
  const std::vector<std::string> names{ "bond_node_a", "bond_node_b" };
  spawnManagedNodes(names);
  startManager(names, true);

  ASSERT_TRUE(recorder_->waitForCount(4, 30s)) << "managed nodes were never brought up";
  ASSERT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);

  silenceManagedNode(1);

  ASSERT_TRUE(recorder_->waitFor(
      [](const std::vector<std::string>& entries) {
        return contains(entries, "bond_node_a:cleanup") && contains(entries, "bond_node_b:cleanup");
      },
      60s))
      << "a broken bond did not take the managed nodes down";

  ASSERT_TRUE(recorder_->waitFor(
      [](const std::vector<std::string>& entries) {
        return countOf(entries, "bond_node_a:activate") == 2 && countOf(entries, "bond_node_b:activate") == 2;
      },
      60s))
      << "the managed nodes were never brought back up after respawn";

  const auto entries = recorder_->entries();
  EXPECT_EQ(countOf(entries, "bond_node_a:configure"), 2u)
      << "respawn should re-run the full bring-up, not just activate";
  EXPECT_EQ(countOf(entries, "bond_node_b:configure"), 2u);
  EXPECT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);
}

// With attempt_respawn_reconnection off, the same broken bond takes the system down and
// leaves it down — recovery is the operator's (or the process supervisor's) call.
TEST_F(LifecycleManagerBondTest, BondTimeoutLeavesSystemDownWhenRespawnIsDisabled)
{
  const std::vector<std::string> names{ "bond_norespawn_node_a", "bond_norespawn_node_b" };
  spawnManagedNodes(names);
  startManager(names, false);

  ASSERT_TRUE(recorder_->waitForCount(4, 30s)) << "managed nodes were never brought up";
  ASSERT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);

  silenceManagedNode(1);

  ASSERT_TRUE(recorder_->waitFor(
      [](const std::vector<std::string>& entries) {
        return contains(entries, "bond_norespawn_node_a:cleanup") && contains(entries, "bond_norespawn_node_b:cleanup");
      },
      60s))
      << "a broken bond did not take the managed nodes down";

  // Give a respawn attempt (a 1 s timer) every chance to happen before ruling it out.
  std::this_thread::sleep_for(5s);
  const auto entries = recorder_->entries();
  EXPECT_EQ(countOf(entries, "bond_norespawn_node_a:activate"), 1u) << "nodes were re-activated despite respawn "
                                                                       "reconnection being disabled";
  EXPECT_EQ(countOf(entries, "bond_norespawn_node_b:activate"), 1u);
  EXPECT_EQ(client_->is_active(5s), SystemStatus::INACTIVE);
}

// A managed node that never comes back is retried until bond_respawn_max_duration is
// up, then given up on. The node here is deliberately never spun: its services are
// advertised, so the manager's state query reaches it and times out, exactly as it
// would against a server that came back up but is not answering.
TEST_F(LifecycleManagerBondTest, RespawnStopsRetryingAfterMaxDuration)
{
  const std::vector<std::string> names{ "bond_unresponsive_node" };
  auto unresponsive = std::make_shared<DummyManagedNode>(names.front(), recorder_, false);

  auto manager = std::make_shared<TestableLifecycleManager>(
      managerOptions(managerName(), names, { "configure", "activate" }, false, kBondTimeout, true, 60.0));
  manager->createLifecycleServiceClients();

  ASSERT_EQ(manager->bondRespawnStartTime().nanoseconds(), 0);

  manager->checkBondRespawnConnection();
  EXPECT_NE(manager->bondRespawnStartTime().nanoseconds(), 0) << "the first respawn check should open the retry window";

  manager->setBondRespawnMaxDuration(0.0);
  manager->checkBondRespawnConnection();
  EXPECT_EQ(manager->bondRespawnStartTime().nanoseconds(), 0) << "the retry window should close once the maximum "
                                                                 "duration has elapsed";
  EXPECT_TRUE(recorder_->entries().empty()) << "an unresponsive node must not be transitioned";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
