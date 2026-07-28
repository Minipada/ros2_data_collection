#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

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

    pid_t pid = fork();
    if (pid == 0)
    {
      std::string dev_arg = "pty,raw,echo=0,link=" + dev_path_;
      std::string peer_arg = "pty,raw,echo=0,link=" + peer_path_;
      ::execlp("socat", "socat", dev_arg.c_str(), peer_arg.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    if (pid < 0)
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
      int status;
      ::waitpid(pid_, &status, 0);
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

class MeasurementSerialInterfaceTest : public ::testing::Test
{
protected:
  MeasurementSerialInterfaceTest()
  {
    SetUp();
  }

  ~MeasurementSerialInterfaceTest() override
  {
  }

  void SetUp() override
  {
    std::string suffix = std::to_string(::getpid());
    dev_path_ = "/tmp/dc_test_serial_dev_" + suffix;
    peer_path_ = "/tmp/dc_test_serial_peer_" + suffix;

    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "serial" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/serial", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementSerialInterfaceTest::dataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  void spinUntilCallback(const std::string& path_to_write, const std::string& line)
  {
    while (!callback_active_)
    {
      writeLine(path_to_write, line);
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;
  std::string dev_path_;
  std::string peer_path_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementSerialInterfaceTest, ActivatesSuccessfullyWithMissingPort)
{
  ms_node_->declare_parameter("serial.plugin", std::string("dc_measurements/SerialInterface"));
  ms_node_->declare_parameter("serial.group_key", std::string("serial"));
  ms_node_->declare_parameter("serial.topic_output", std::string("/dc/measurement/serial"));
  ms_node_->declare_parameter("serial.port", std::string("/tmp/dc_test_serial_does_not_exist"));
  ms_node_->declare_parameter("serial.polling_interval", 30);
  ms_node_->declare_parameter("serial.init_collect", false);

  // Must not throw: an unplugged/missing device should not fail activation.
  ASSERT_NO_THROW(startLifecycleNode());

  for (int i = 0; i < 5; ++i)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  SUCCEED();
}

TEST_F(MeasurementSerialInterfaceTest, DelimiterParsingProducesNamedFields)
{
  SocatPtyPair pty(dev_path_, peer_path_);
  ASSERT_TRUE(pty.start()) << "socat is required to run this test (virtual pty pair)";

  ms_node_->declare_parameter("serial.plugin", std::string("dc_measurements/SerialInterface"));
  ms_node_->declare_parameter("serial.group_key", std::string("serial"));
  ms_node_->declare_parameter("serial.topic_output", std::string("/dc/measurement/serial"));
  ms_node_->declare_parameter("serial.port", pty.devPath());
  ms_node_->declare_parameter("serial.polling_interval", 30);
  ms_node_->declare_parameter("serial.init_collect", false);
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

  ms_node_->declare_parameter("serial.plugin", std::string("dc_measurements/SerialInterface"));
  ms_node_->declare_parameter("serial.group_key", std::string("serial"));
  ms_node_->declare_parameter("serial.topic_output", std::string("/dc/measurement/serial"));
  ms_node_->declare_parameter("serial.port", pty.devPath());
  ms_node_->declare_parameter("serial.polling_interval", 30);
  ms_node_->declare_parameter("serial.init_collect", false);
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
  SocatPtyPair pty(dev_path_, peer_path_);
  ASSERT_TRUE(pty.start()) << "socat is required to run this test (virtual pty pair)";

  ms_node_->declare_parameter("serial.plugin", std::string("dc_measurements/SerialInterface"));
  ms_node_->declare_parameter("serial.group_key", std::string("serial"));
  ms_node_->declare_parameter("serial.topic_output", std::string("/dc/measurement/serial"));
  ms_node_->declare_parameter("serial.port", pty.devPath());
  ms_node_->declare_parameter("serial.polling_interval", 30);
  ms_node_->declare_parameter("serial.init_collect", false);
  ms_node_->declare_parameter("serial.parsing_type", std::string("delimiter"));
  ms_node_->declare_parameter("serial.delimiter", std::string(","));
  ms_node_->declare_parameter("serial.fields", std::vector<std::string>{ "value" });

  startLifecycleNode();

  spinUntilCallback(pty.peerPath(), "1");
  ASSERT_EQ(data_json_["fields"]["value"], "1");

  // Simulate an unplug: tear down the virtual pty pair out from under the open port.
  pty.stop();

  for (int i = 0; i < 20; ++i)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // Simulate a replug: a fresh socat pair at the exact same link paths.
  ASSERT_TRUE(pty.start()) << "failed to restart the virtual pty pair";

  callback_active_ = false;
  spinUntilCallback(pty.peerPath(), "2");

  EXPECT_EQ(data_json_["fields"]["value"], "2");
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
