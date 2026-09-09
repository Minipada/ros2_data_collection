// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <boost/asio.hpp>

#include "measurement_test_bench.hpp"

class MeasurementTCPHealthTest : public MeasurementBench
{
protected:
  MeasurementTCPHealthTest() : MeasurementBench("tcp_health")
  {
  }

  // Asks the OS for a free ephemeral port by binding to port 0.
  static unsigned short reserveFreePort()
  {
    boost::asio::io_service svc;
    boost::asio::ip::tcp::acceptor acceptor(svc);
    acceptor.open(boost::asio::ip::tcp::v4());
    acceptor.bind({ boost::asio::ip::tcp::v4(), 0 });
    return acceptor.local_endpoint().port();
  }
};

TEST_F(MeasurementTCPHealthTest, ActiveWhenPortIsListening)
{
  boost::asio::io_service svc;
  boost::asio::ip::tcp::acceptor acceptor(svc);
  acceptor.open(boost::asio::ip::tcp::v4());
  acceptor.bind({ boost::asio::ip::tcp::v4(), 0 });
  acceptor.listen();
  unsigned short port = acceptor.local_endpoint().port();

  ms_node_->declare_parameter("tcp_health.plugin", std::string("dc_measurements/TCPHealth"));
  ms_node_->declare_parameter("tcp_health.group_key", std::string("tcp_health"));
  ms_node_->declare_parameter("tcp_health.topic_output", std::string("/dc/measurement/tcp_health"));
  ms_node_->declare_parameter("tcp_health.name", std::string("test-service"));
  ms_node_->declare_parameter("tcp_health.port", static_cast<int>(port));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["port"].get<int>(), port);
  EXPECT_EQ(data_json_["server_name"].get<std::string>(), "test-service");
  EXPECT_TRUE(data_json_["active"].get<bool>());
}

TEST_F(MeasurementTCPHealthTest, InactiveWhenPortIsFree)
{
  unsigned short port = reserveFreePort();

  ms_node_->declare_parameter("tcp_health.plugin", std::string("dc_measurements/TCPHealth"));
  ms_node_->declare_parameter("tcp_health.group_key", std::string("tcp_health"));
  ms_node_->declare_parameter("tcp_health.topic_output", std::string("/dc/measurement/tcp_health"));
  ms_node_->declare_parameter("tcp_health.name", std::string("test-service"));
  ms_node_->declare_parameter("tcp_health.port", static_cast<int>(port));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_FALSE(data_json_["active"].get<bool>());
}

DC_MEASUREMENT_TEST_MAIN()
