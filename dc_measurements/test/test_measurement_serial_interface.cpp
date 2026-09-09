// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "measurement_test_bench.hpp"

extern char** environ;

// Spawns `socat` to bridge two virtual pty devices at fixed, well-known paths (the `link=`
// address option), so the test can point a SerialInterface Measurement at one end and write
// fixture lines from the other, entirely without hardware. Killing and restarting a pair
// against the *same* link paths simulates a device unplug/replug cycle.
class SocatPtyPair
{
public:
  SocatPtyPair(std::string dev_path, std::string peer_path)
    : dev_path_(std::move(dev_path)), peer_path_(std::move(peer_path))
  {
  }

  ~SocatPtyPair()
  {
    stop();
  }

  bool start()
  {
    ::unlink(dev_path_.c_str());
    ::unlink(peer_path_.c_str());

    // posix_spawn(p), not fork()+exec(): this test binary links rclcpp/DDS, which run their
    // own internal threads, and calling raw fork() from a multithreaded process can deadlock
    // the child (or even the parent, inside libc's fork()) if another thread holds a lock
    // (e.g. malloc's arena lock) at the instant of the fork. posix_spawn is specified to be
    // safe to call from a multithreaded program, which fork() is not.
    std::string dev_arg = "pty,raw,echo=0,link=" + dev_path_;
    std::string peer_arg = "pty,raw,echo=0,link=" + peer_path_;
    char* argv[] = { const_cast<char*>("socat"), const_cast<char*>(dev_arg.c_str()),
                     const_cast<char*>(peer_arg.c_str()), nullptr };

    pid_t pid = -1;
    int rc = ::posix_spawnp(&pid, "socat", nullptr, nullptr, argv, environ);
    if (rc != 0)
    {
      return false;
    }
    pid_ = pid;

    for (int i = 0; i < 400; ++i)
    {
      if (::access(dev_path_.c_str(), F_OK) == 0 && ::access(peer_path_.c_str(), F_OK) == 0)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    stop();
    return false;
  }

  void stop()
  {
    if (pid_ > 0)
    {
      ::kill(pid_, SIGTERM);
      // Bounded wait (SIGKILL fallback) rather than a plain blocking waitpid(): this must
      // never be able to hang the test harness, no matter how socat behaves.
      bool reaped = false;
      for (int i = 0; i < 100; ++i)
      {
        int status;
        pid_t result = ::waitpid(pid_, &status, WNOHANG);
        if (result == pid_ || result < 0)
        {
          reaped = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!reaped)
      {
        ::kill(pid_, SIGKILL);
        int status;
        ::waitpid(pid_, &status, 0);
      }
      pid_ = -1;
    }
    ::unlink(dev_path_.c_str());
    ::unlink(peer_path_.c_str());
  }

  const std::string& devPath() const
  {
    return dev_path_;
  }
  const std::string& peerPath() const
  {
    return peer_path_;
  }

private:
  std::string dev_path_;
  std::string peer_path_;
  pid_t pid_{ -1 };
};

static void writeLine(const std::string& path, const std::string& line)
{
  int fd = ::open(path.c_str(), O_WRONLY | O_NOCTTY);
  if (fd < 0)
  {
    return;
  }
  std::string data = line + "\n";
  ssize_t written = ::write(fd, data.c_str(), data.size());
  (void)written;
  ::close(fd);
}

// Directly allocates a POSIX pty pair (no subprocess) and symlinks its slave device to a
// fixed path, so a SerialInterface Measurement can be pointed at a stable path while the
// underlying pty is torn down and recreated to simulate an unplug/replug cycle. Used by the
// reconnect test in place of a second `socat` subprocess: this needs synchronous control over
// exactly when the slave device becomes openable (grantpt()/unlockpt() complete before
// open() returns), which coordinating with a second external process could not reliably
// guarantee. Closing the master here reproduces the same read()-error-on-unplug condition a
// real USB-serial adapter hangup produces on the slave side.
class DirectPty
{
public:
  explicit DirectPty(std::string link_path) : link_path_(std::move(link_path))
  {
  }

  ~DirectPty()
  {
    teardown();
  }

  bool open()
  {
    teardown();
    master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd_ < 0)
    {
      return false;
    }
    if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0)
    {
      teardown();
      return false;
    }
    const char* slave_name = ::ptsname(master_fd_);
    if (slave_name == nullptr)
    {
      teardown();
      return false;
    }
    ::unlink(link_path_.c_str());
    if (::symlink(slave_name, link_path_.c_str()) != 0)
    {
      teardown();
      return false;
    }
    return true;
  }

  void teardown()
  {
    if (master_fd_ >= 0)
    {
      ::close(master_fd_);
      master_fd_ = -1;
    }
    ::unlink(link_path_.c_str());
  }

  void writeLine(const std::string& line) const
  {
    if (master_fd_ < 0)
    {
      return;
    }
    std::string data = line + "\n";
    ssize_t written = ::write(master_fd_, data.c_str(), data.size());
    (void)written;
  }

private:
  std::string link_path_;
  int master_fd_{ -1 };
};

class MeasurementSerialInterfaceTest : public MeasurementBench
{
protected:
  MeasurementSerialInterfaceTest() : MeasurementBench("serial")
  {
    std::string suffix = std::to_string(::getpid());
    dev_path_ = "/tmp/dc_test_serial_dev_" + suffix;
    peer_path_ = "/tmp/dc_test_serial_peer_" + suffix;
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("serial.plugin", std::string("dc_measurements/SerialInterface"));
    ms_node_->declare_parameter("serial.group_key", std::string("serial"));
    ms_node_->declare_parameter("serial.topic_output", std::string("/dc/measurement/serial"));
    ms_node_->declare_parameter("serial.polling_interval", 30);
    ms_node_->declare_parameter("serial.init_collect", false);
  }

  // Keeps writing `line` to `path_to_write` (and spinning) until a Record shows up, so a write
  // that lands before the Measurement opened its end of the pty is retried rather than lost.
  void spinUntilCallback(const std::string& path_to_write, const std::string& line)
  {
    spinUntilCallback([&] { writeLine(path_to_write, line); });
  }

  void spinUntilCallback(const std::function<void()>& write_fn)
  {
    ASSERT_TRUE(spinUntil(
        [&] {
          write_fn();
          return callback_active_;
        },
        10000))
        << "No Record received within the timeout";
  }

  std::string dev_path_;
  std::string peer_path_;
};

TEST_F(MeasurementSerialInterfaceTest, ActivatesSuccessfullyWithMissingPort)
{
  declareCommonParameters();
  ms_node_->declare_parameter("serial.port", std::string("/tmp/dc_test_serial_does_not_exist"));

  // Must not throw: an unplugged/missing device should not fail activation.
  ASSERT_NO_THROW(startLifecycleNode());

  spinFor(100);

  SUCCEED();
}

TEST_F(MeasurementSerialInterfaceTest, DelimiterParsingProducesNamedFields)
{
  SocatPtyPair pty(dev_path_, peer_path_);
  ASSERT_TRUE(pty.start()) << "socat is required to run this test (virtual pty pair)";

  declareCommonParameters();
  ms_node_->declare_parameter("serial.port", pty.devPath());
  ms_node_->declare_parameter("serial.parsing_type", std::string("delimiter"));
  ms_node_->declare_parameter("serial.delimiter", std::string(","));
  ms_node_->declare_parameter("serial.fields", std::vector<std::string>{ "temperature", "humidity" });

  startLifecycleNode();

  spinUntilCallback(pty.peerPath(), "23.5,60");

  EXPECT_EQ(data_json_["raw"], "23.5,60");
  EXPECT_EQ(data_json_["fields"]["temperature"], "23.5");
  EXPECT_EQ(data_json_["fields"]["humidity"], "60");
}

TEST_F(MeasurementSerialInterfaceTest, RegexParsingProducesNamedFields)
{
  SocatPtyPair pty(dev_path_, peer_path_);
  ASSERT_TRUE(pty.start()) << "socat is required to run this test (virtual pty pair)";

  declareCommonParameters();
  ms_node_->declare_parameter("serial.port", pty.devPath());
  ms_node_->declare_parameter("serial.parsing_type", std::string("regex"));
  ms_node_->declare_parameter("serial.regex", std::string("^T:(\\d+\\.\\d+) H:(\\d+)$"));
  ms_node_->declare_parameter("serial.fields", std::vector<std::string>{ "temperature", "humidity" });

  startLifecycleNode();

  spinUntilCallback(pty.peerPath(), "T:21.0 H:55");

  EXPECT_EQ(data_json_["fields"]["temperature"], "21.0");
  EXPECT_EQ(data_json_["fields"]["humidity"], "55");
}

TEST_F(MeasurementSerialInterfaceTest, ReconnectsAfterDisconnect)
{
  // Direct POSIX pty allocation, not socat, for this test specifically: it needs precise,
  // synchronous control over exactly when the device is torn down and re-created (see
  // DirectPty's comment above).
  DirectPty pty(dev_path_);
  ASSERT_TRUE(pty.open()) << "failed to allocate a virtual pty";

  declareCommonParameters();
  ms_node_->declare_parameter("serial.port", dev_path_);
  ms_node_->declare_parameter("serial.parsing_type", std::string("delimiter"));
  ms_node_->declare_parameter("serial.delimiter", std::string(","));
  ms_node_->declare_parameter("serial.fields", std::vector<std::string>{ "value" });

  startLifecycleNode();

  spinUntilCallback([&] { pty.writeLine("1"); });
  ASSERT_EQ(data_json_["fields"]["value"], "1");

  // Simulate an unplug: tear down the pty out from under the already-open port. The
  // plugin's next read() on its still-open fd gets a real error (EIO/hangup), the same
  // condition a real USB-serial adapter unplug produces.
  pty.teardown();

  spinFor(600);

  // Simulate a replug: a fresh pty allocated and symlinked at the exact same path.
  ASSERT_TRUE(pty.open()) << "failed to reallocate the virtual pty";

  callback_active_ = false;
  spinUntilCallback([&] { pty.writeLine("2"); });

  EXPECT_EQ(data_json_["fields"]["value"], "2");
}

DC_MEASUREMENT_TEST_MAIN()
