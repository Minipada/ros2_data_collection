#include "dc_bridge/supervisor.hpp"

#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace dc_bridge
{

SupervisorConfig SupervisorConfig::vector(const std::string& binary, const std::vector<std::string>& config_paths)
{
  SupervisorConfig cfg;
  cfg.program = binary;
  for (const auto& path : config_paths)
  {
    cfg.args.push_back("--config");
    cfg.args.push_back(path);
  }
  cfg.restart_backoff = std::chrono::milliseconds(500);
  return cfg;
}

Supervisor::~Supervisor()
{
  stop();
}

void Supervisor::start()
{
  if (stopped_)
  {
    return;
  }

  pid_t pid = ::fork();
  if (pid < 0)
  {
    throw std::runtime_error(std::string("supervisor: fork failed: ") + std::strerror(errno));
  }

  if (pid == 0)
  {
    // Child: die with the spawner. The kernel delivers PDEATHSIG when the spawning
    // *thread* exits, so both the main thread and the background poll thread satisfy
    // this — Vector can never be orphaned even by a SIGKILLed/crashed Bridge.
    ::prctl(PR_SET_PDEATHSIG, SIGKILL);

    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(config_.program.c_str()));
    for (const auto& arg : config_.args)
    {
      argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    ::execvp(config_.program.c_str(), argv.data());
    // Only reached if exec failed.
    _exit(127);
  }

  child_pid_ = pid;
}

bool Supervisor::reap_if_exited()
{
  if (!child_pid_)
  {
    return true;  // no child == "exited" for restart purposes
  }
  int status = 0;
  pid_t r = ::waitpid(*child_pid_, &status, WNOHANG);
  if (r == 0)
  {
    return false;  // still running
  }
  // r == pid (reaped) or r < 0 (already gone / error) — either way it's not alive.
  child_pid_ = std::nullopt;
  return true;
}

bool Supervisor::is_running()
{
  if (!child_pid_)
  {
    return false;
  }
  int status = 0;
  pid_t r = ::waitpid(*child_pid_, &status, WNOHANG);
  if (r == 0)
  {
    return true;
  }
  child_pid_ = std::nullopt;
  return false;
}

bool Supervisor::poll_restart()
{
  if (stopped_)
  {
    return false;
  }
  if (!reap_if_exited())
  {
    return false;
  }
  if (last_exit_)
  {
    if (std::chrono::steady_clock::now() - *last_exit_ < config_.restart_backoff)
    {
      return false;
    }
  }
  last_exit_ = std::chrono::steady_clock::now();
  start();
  return true;
}

void Supervisor::stop()
{
  stopped_ = true;
  if (child_pid_)
  {
    ::kill(*child_pid_, SIGKILL);
    int status = 0;
    ::waitpid(*child_pid_, &status, 0);
    child_pid_ = std::nullopt;
  }
}

}  // namespace dc_bridge
