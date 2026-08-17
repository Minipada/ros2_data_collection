// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Supervisor: spawns a program (the vendored Vector binary in production) and restarts
// it if it dies. Deliberately agnostic to what it supervises (see tests) so the restart
// logic is exercised without a real Vector binary.
#ifndef DC_BRIDGE__SUPERVISOR_HPP_
#define DC_BRIDGE__SUPERVISOR_HPP_

#include <sys/types.h>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace dc_bridge
{

struct SupervisorConfig
{
  std::string program;
  std::vector<std::string> args;
  /// Minimum time to wait after an exit before respawning, to avoid a restart storm if
  /// the supervised process keeps failing immediately.
  std::chrono::milliseconds restart_backoff{ 500 };

  /// Builds the `vector --config <p1> --config <p2> ...` invocation. `config_paths` is
  /// the rendered config plus any custom_config_files passthrough snippets (ADR-0003) —
  /// Vector merges multiple `--config` files natively.
  static SupervisorConfig vector(const std::string& binary, const std::vector<std::string>& config_paths);
};

/// Spawns and restarts a child process. Not copyable (owns a child pid).
class Supervisor
{
public:
  explicit Supervisor(SupervisorConfig config) : config_(std::move(config))
  {
  }
  ~Supervisor();

  Supervisor(const Supervisor&) = delete;
  Supervisor& operator=(const Supervisor&) = delete;

  /// Spawns the supervised process. A no-op once stop() has been called, so a signal
  /// handler installed *before* the first start() can never race it into spawning a
  /// process nobody will ever stop.
  ///
  /// On Linux the child gets PR_SET_PDEATHSIG(SIGKILL), so it cannot outlive the Bridge
  /// even when the Bridge dies by SIGKILL/crash (paths no signal handler can cover).
  /// Throws std::runtime_error if the spawn fails.
  void start();

  /// Whether the supervised child is currently alive (reaps it if it has exited).
  bool is_running();

  /// Pid of the currently supervised process, if one has been spawned and not reaped.
  std::optional<pid_t> child_id() const
  {
    return child_pid_;
  }

  /// Checks whether the supervised process has exited and, if so, restarts it (subject
  /// to restart_backoff). Returns true if a restart happened. A no-op returning false
  /// once stop() has been called — prevents the respawn-after-stop race a background
  /// poll loop would otherwise hit against a concurrent stop().
  bool poll_restart();

  /// Stops the supervised process and permanently disables future respawns from
  /// poll_restart (there is no "un-stop": a Supervisor is only ever stopped once, right
  /// before the owning process exits).
  void stop();

private:
  // Returns true if the child has exited (reaping it), false if still running or if
  // there is no child.
  bool reap_if_exited();

  SupervisorConfig config_;
  std::optional<pid_t> child_pid_;
  std::optional<std::chrono::steady_clock::time_point> last_exit_;
  bool stopped_{ false };
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__SUPERVISOR_HPP_
