// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Startup/teardown ordering of the nodes `lifecycle_manager_dc` manages.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "dc_lifecycle_manager/lifecycle_manager_client.hpp"
#include "managed_node_harness.hpp"

using dc_lifecycle_manager::SystemStatus;
using dc_lifecycle_manager_test::DummyManagedNode;
using dc_lifecycle_manager_test::managerOptions;
using dc_lifecycle_manager_test::TransitionRecorder;
using namespace std::chrono_literals;  // NOLINT

namespace
{
/// The entries recorded from `from` onwards, so a phase can be compared on its own.
std::vector<std::string> tail(const std::vector<std::string>& entries, std::size_t from)
{
  if (from >= entries.size())
  {
    return {};
  }
  return std::vector<std::string>(entries.begin() + static_cast<std::ptrdiff_t>(from), entries.end());
}
}  // namespace

class LifecycleManagerStartupTest : public ::testing::Test
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
      // Bonds are covered by test_lifecycle_manager_bond; these tests run with
      // bond_timeout 0.0, which is the manager's own "no bonds" configuration.
      auto node = std::make_shared<DummyManagedNode>(name, recorder_, false);
      node->startSpinning();
      nodes_.push_back(node);
    }
  }

  /// Start a manager named after the running test, and a client wired to it.
  void startManager(const std::vector<std::string>& node_names, bool autostart)
  {
    const std::string manager_name = managerName();
    manager_ = std::make_shared<dc_lifecycle_manager::LifecycleManager>(
        managerOptions(manager_name, node_names, { "configure", "activate" }, autostart, 0.0));
    manager_thread_ = std::make_unique<nav2_util::NodeThread>(manager_->get_node_base_interface());
    client_node_ = std::make_shared<rclcpp::Node>(manager_name + "_client");
    client_ = std::make_unique<dc_lifecycle_manager::LifecycleManagerClient>(manager_name, client_node_);
  }

  /// A manager node name unique to the running test.
  static std::string managerName()
  {
    return std::string("lifecycle_manager_") + ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  std::shared_ptr<TransitionRecorder> recorder_;
  std::vector<std::shared_ptr<DummyManagedNode>> nodes_;
  std::shared_ptr<dc_lifecycle_manager::LifecycleManager> manager_;
  std::unique_ptr<nav2_util::NodeThread> manager_thread_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<dc_lifecycle_manager::LifecycleManagerClient> client_;
};

// Every managed node is configured before any is activated, and within each transition
// the nodes are driven in the order `node_names` declares them.
TEST_F(LifecycleManagerStartupTest, ConfiguresThenActivatesEveryNodeInDeclaredOrder)
{
  const std::vector<std::string> names{ "startup_node_a", "startup_node_b", "startup_node_c" };
  spawnManagedNodes(names);
  startManager(names, true);

  ASSERT_TRUE(recorder_->waitForCount(6, 30s)) << "managed nodes were never brought up";
  EXPECT_EQ(recorder_->entries(), (std::vector<std::string>{ "startup_node_a:configure", "startup_node_b:configure",
                                                             "startup_node_c:configure", "startup_node_a:activate",
                                                             "startup_node_b:activate", "startup_node_c:activate" }));
  EXPECT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);
}

// Regression: every declared transition is applied to every managed node. Pairing the
// two lists index-wise silently dropped each transition past the node count, leaving a
// single managed node configured but never activated.
TEST_F(LifecycleManagerStartupTest, AppliesEveryTransitionToASingleManagedNode)
{
  const std::vector<std::string> names{ "startup_solo_node" };
  spawnManagedNodes(names);
  startManager(names, true);

  ASSERT_TRUE(recorder_->waitForCount(2, 30s)) << "single managed node was not fully brought up";
  EXPECT_EQ(recorder_->entries(),
            (std::vector<std::string>{ "startup_solo_node:configure", "startup_solo_node:activate" }));
  EXPECT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);
}

// With autostart off nothing moves until a STARTUP request arrives on `manage_nodes`.
TEST_F(LifecycleManagerStartupTest, StartupServiceBringsUpNodesWhenAutostartIsDisabled)
{
  const std::vector<std::string> names{ "startup_manual_node_a", "startup_manual_node_b" };
  spawnManagedNodes(names);
  startManager(names, false);

  ASSERT_EQ(client_->is_active(5s), SystemStatus::INACTIVE);
  EXPECT_TRUE(recorder_->entries().empty()) << "nodes transitioned without a startup request";

  ASSERT_TRUE(client_->startup(30s));
  EXPECT_EQ(recorder_->entries(),
            (std::vector<std::string>{ "startup_manual_node_a:configure", "startup_manual_node_b:configure",
                                       "startup_manual_node_a:activate", "startup_manual_node_b:activate" }));
  EXPECT_EQ(client_->is_active(5s), SystemStatus::ACTIVE);
}

// Teardown runs in the reverse of the bring-up order, so a node is never left running
// while something it came up before it is already gone.
TEST_F(LifecycleManagerStartupTest, ResetTransitionsNodesInReverseOrder)
{
  const std::vector<std::string> names{ "startup_reset_node_a", "startup_reset_node_b", "startup_reset_node_c" };
  spawnManagedNodes(names);
  startManager(names, true);
  ASSERT_TRUE(recorder_->waitForCount(6, 30s)) << "managed nodes were never brought up";

  ASSERT_TRUE(client_->reset(30s));
  EXPECT_EQ(tail(recorder_->entries(), 6),
            (std::vector<std::string>{ "startup_reset_node_c:deactivate", "startup_reset_node_b:deactivate",
                                       "startup_reset_node_a:deactivate", "startup_reset_node_c:cleanup",
                                       "startup_reset_node_b:cleanup", "startup_reset_node_a:cleanup" }));
  EXPECT_EQ(client_->is_active(5s), SystemStatus::INACTIVE);
}

// Shutdown deactivates, cleans up and finalizes — each phase in reverse order.
TEST_F(LifecycleManagerStartupTest, ShutdownTransitionsNodesInReverseOrder)
{
  const std::vector<std::string> names{ "startup_shutdown_node_a", "startup_shutdown_node_b" };
  spawnManagedNodes(names);
  startManager(names, true);
  ASSERT_TRUE(recorder_->waitForCount(4, 30s)) << "managed nodes were never brought up";

  ASSERT_TRUE(client_->shutdown(30s));
  EXPECT_EQ(tail(recorder_->entries(), 4),
            (std::vector<std::string>{ "startup_shutdown_node_b:deactivate", "startup_shutdown_node_a:deactivate",
                                       "startup_shutdown_node_b:cleanup", "startup_shutdown_node_a:cleanup",
                                       "startup_shutdown_node_b:shutdown", "startup_shutdown_node_a:shutdown" }));
  EXPECT_EQ(client_->is_active(5s), SystemStatus::INACTIVE);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
