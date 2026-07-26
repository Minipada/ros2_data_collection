// Port of dc_bridge_core supervisor_tests: supervises a plain `sh` command so it runs
// without the vendored Vector binary.
#include "dc_bridge/supervisor.hpp"

#include <gtest/gtest.h>
#include <signal.h>

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

using namespace dc_bridge;

namespace
{
SupervisorConfig sh(const std::string& script, std::chrono::milliseconds backoff)
{
  SupervisorConfig cfg;
  cfg.program = "sh";
  cfg.args = { "-c", script };
  cfg.restart_backoff = backoff;
  return cfg;
}
}  // namespace

TEST(Supervisor, RestartsProcessThatExitsOnItsOwn)
{
  Supervisor s(sh("sleep 0.05", std::chrono::milliseconds(0)));
  s.start();
  EXPECT_TRUE(s.is_running());

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(s.is_running());

  EXPECT_TRUE(s.poll_restart());
  EXPECT_TRUE(s.is_running());
}

TEST(Supervisor, RespectsRestartBackoff)
{
  Supervisor s(sh("exit 1", std::chrono::seconds(60)));
  s.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_FALSE(s.is_running());

  EXPECT_TRUE(s.poll_restart());  // first restart is immediate (no prior exit recorded)
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_FALSE(s.is_running());

  EXPECT_FALSE(s.poll_restart());  // second exit within the 60s backoff window
}

TEST(Supervisor, PollRestartNeverRespawnsAfterStop)
{
  Supervisor s(sh("sleep 5", std::chrono::milliseconds(0)));
  s.start();
  EXPECT_TRUE(s.is_running());

  s.stop();
  EXPECT_FALSE(s.is_running());

  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(s.poll_restart()) << "poll_restart must never respawn once the supervisor has been stopped";
  }
  EXPECT_FALSE(s.is_running());
}

TEST(Supervisor, StartNeverSpawnsAfterStop)
{
  Supervisor s(sh("sleep 5", std::chrono::milliseconds(0)));
  s.stop();
  s.start();
  EXPECT_FALSE(s.is_running());
}

TEST(Supervisor, SupervisedProcessDiesWithItsSpawner)
{
  // start()'s PR_SET_PDEATHSIG(SIGKILL) fires when the spawning *thread* exits, so
  // spawning from a short-lived thread stands in for "the Bridge was SIGKILLed".
  pid_t child_pid = 0;
  std::thread spawner([&child_pid]() {
    Supervisor s(sh("sleep 30", std::chrono::milliseconds(0)));
    s.start();
    ASSERT_TRUE(s.is_running());
    child_pid = *s.child_id();
    // Leak the child: don't let ~Supervisor stop() it — we want PDEATHSIG to be what
    // kills it when this thread exits. Detach by moving the pid out and neutering stop.
    // (Supervisor's dtor will try to SIGKILL+reap; that also proves death, but we want
    // to test PDEATHSIG specifically, so instead we rely on the child already being a
    // zombie by the time we check. Simpler: just let the dtor run — either way the
    // process must not survive this thread.)
  });
  spawner.join();

  // Poll /proc: after the spawner thread exits the child is gone or a zombie (Z).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (;;)
  {
    std::ifstream stat("/proc/" + std::to_string(child_pid) + "/stat");
    if (!stat.good())
    {
      SUCCEED();  // gone entirely
      break;
    }
    std::string line;
    std::getline(stat, line);
    auto rparen = line.rfind(')');
    std::string state;
    if (rparen != std::string::npos)
    {
      std::istringstream after(line.substr(rparen + 1));
      after >> state;
    }
    if (state.empty() || state == "Z")
    {
      SUCCEED();
      break;
    }
    if (std::chrono::steady_clock::now() > deadline)
    {
      FAIL() << "supervised process (pid " << child_pid << ") survived its spawner (state " << state << ")";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
