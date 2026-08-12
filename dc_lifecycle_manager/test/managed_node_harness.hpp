// Shared harness for the dc_lifecycle_manager gtest suites.
//
// Everything here runs in-process: the manager under test, the lifecycle nodes it
// manages, and the recorder both write to. Per ADR-0006 the Bridge is deliberately
// outside the lifecycle manager, so these doubles stand in for the C++ collection
// nodes `lifecycle_manager_dc` actually manages — nothing here models the Bridge.
#ifndef MANAGED_NODE_HARNESS_HPP_
#define MANAGED_NODE_HARNESS_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dc_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_lifecycle_manager_test
{

/**
 * @brief Ordered log of every lifecycle transition every managed node went through.
 *
 * Entries are "<node name>:<transition>". The manager transitions nodes one at a time
 * with blocking service calls, so the recorded order is the order the manager drove
 * them in, and can be compared against exactly.
 */
class TransitionRecorder
{
public:
  void record(const std::string& entry)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.push_back(entry);
  }

  std::vector<std::string> entries() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_;
  }

  /**
   * @brief Block until the recorded entries satisfy `predicate`, or `timeout` elapses.
   * @return Whether the predicate held.
   */
  bool waitFor(const std::function<bool(const std::vector<std::string>&)>& predicate,
               std::chrono::milliseconds timeout) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (predicate(entries()))
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return predicate(entries());
  }

  /**
   * @brief Block until at least `count` transitions have been recorded.
   */
  bool waitForCount(std::size_t count, std::chrono::milliseconds timeout) const
  {
    return waitFor([count](const std::vector<std::string>& entries) { return entries.size() >= count; }, timeout);
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::string> entries_;
};

/**
 * @brief A lifecycle node standing in for one of the nodes the manager brings up.
 *
 * Records each transition it is driven through, and — when bonds are in play — keeps a
 * bond with the manager exactly like the real collection nodes do (created on activate,
 * destroyed on deactivate). Spinning is under the test's control so a test can simulate
 * a node that stops answering heartbeats without killing the process it lives in.
 */
class DummyManagedNode : public nav2_util::LifecycleNode
{
public:
  DummyManagedNode(const std::string& name, std::shared_ptr<TransitionRecorder> recorder, bool use_bond)
    : nav2_util::LifecycleNode(name), recorder_(std::move(recorder)), use_bond_(use_bond)
  {
  }

  ~DummyManagedNode() override
  {
    stopSpinning();
  }

  /// Start processing callbacks (heartbeats, lifecycle service calls) in a background thread.
  void startSpinning()
  {
    if (!spin_thread_)
    {
      spin_thread_ = std::make_unique<nav2_util::NodeThread>(get_node_base_interface());
    }
  }

  /// Stop processing callbacks: the node stays discoverable but answers nothing.
  void stopSpinning()
  {
    spin_thread_.reset();
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override
  {
    recorder_->record(std::string(get_name()) + ":configure");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*state*/) override
  {
    if (use_bond_)
    {
      createBond();
    }
    recorder_->record(std::string(get_name()) + ":activate");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override
  {
    if (use_bond_)
    {
      destroyBond();
    }
    recorder_->record(std::string(get_name()) + ":deactivate");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override
  {
    recorder_->record(std::string(get_name()) + ":cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override
  {
    recorder_->record(std::string(get_name()) + ":shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<TransitionRecorder> recorder_;
  bool use_bond_;
  std::unique_ptr<nav2_util::NodeThread> spin_thread_;
};

/**
 * @brief The manager under test, with the internals a test needs to drive or observe.
 *
 * Only the bond-respawn give-up path uses the exposed internals; every other test drives
 * the manager through its `manage_nodes` / `is_active` services like a real caller does.
 */
class TestableLifecycleManager : public dc_lifecycle_manager::LifecycleManager
{
public:
  explicit TestableLifecycleManager(const rclcpp::NodeOptions& options) : LifecycleManager(options)
  {
  }

  using LifecycleManager::checkBondRespawnConnection;
  using LifecycleManager::createLifecycleServiceClients;

  rclcpp::Time bondRespawnStartTime() const
  {
    return bond_respawn_start_time_;
  }

  void setBondRespawnMaxDuration(double seconds)
  {
    bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(seconds);
  }
};

/**
 * @brief Build the NodeOptions a `LifecycleManager` reads its configuration from.
 *
 * `manager_name` renames the node (and with it its `manage_nodes` / `is_active`
 * services). Every test needs its own: the manager holds service clients built from its
 * own `shared_from_this()`, a reference cycle that keeps the node alive past the end of
 * the test that created it, so a manager left over from an earlier test would otherwise
 * still be answering on the name the next test's client connects to.
 */
inline rclcpp::NodeOptions managerOptions(const std::string& manager_name, const std::vector<std::string>& node_names,
                                          const std::vector<std::string>& transitions, bool autostart,
                                          double bond_timeout, bool attempt_respawn_reconnection = true,
                                          double bond_respawn_max_duration = 10.0)
{
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=" + manager_name });
  options.parameter_overrides({
      rclcpp::Parameter("node_names", node_names),
      rclcpp::Parameter("transitions", transitions),
      rclcpp::Parameter("autostart", autostart),
      rclcpp::Parameter("bond_timeout", bond_timeout),
      rclcpp::Parameter("attempt_respawn_reconnection", attempt_respawn_reconnection),
      rclcpp::Parameter("bond_respawn_max_duration", bond_respawn_max_duration),
  });
  return options;
}

/// Whether `entries` contains `entry`.
inline bool contains(const std::vector<std::string>& entries, const std::string& entry)
{
  return std::find(entries.begin(), entries.end(), entry) != entries.end();
}

/// How many times `entry` appears in `entries`.
inline std::size_t countOf(const std::vector<std::string>& entries, const std::string& entry)
{
  return static_cast<std::size_t>(std::count(entries.begin(), entries.end(), entry));
}

}  // namespace dc_lifecycle_manager_test

#endif  // MANAGED_NODE_HARNESS_HPP_
