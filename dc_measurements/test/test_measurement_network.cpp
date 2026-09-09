// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>

#include "measurement_test_bench.hpp"

class MeasurementNetworkTest : public MeasurementBench
{
protected:
  MeasurementNetworkTest() : MeasurementBench("network")
  {
  }
};

// The ICMP ping socket (SOCK_DGRAM + IPPROTO_ICMP) needs no root/CAP_NET_RAW, only a
// permissive net.ipv4.ping_group_range -- true by default for the root group this runs as
// in CI/containers, but not guaranteed on every host. Assert on the parts of the Record
// that are deterministic everywhere: the network interface list (every Linux host has at
// least loopback) and the shape/types of the ping fields, not their values.
TEST_F(MeasurementNetworkTest, ReportsLocalInterfacesAndPingShape)
{
  ms_node_->declare_parameter("network.plugin", std::string("dc_measurements/Network"));
  ms_node_->declare_parameter("network.group_key", std::string("network"));
  ms_node_->declare_parameter("network.topic_output", std::string("/dc/measurement/network"));
  // 127.0.0.1 rather than the 8.8.8.8 default so this doesn't depend on outbound internet access.
  ms_node_->declare_parameter("network.ping_address", std::string("127.0.0.1"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  ASSERT_TRUE(data_json_["interfaces"].is_array());
  EXPECT_NE(std::find(data_json_["interfaces"].begin(), data_json_["interfaces"].end(), "lo"),
            data_json_["interfaces"].end());
  ASSERT_TRUE(data_json_["ping"].is_number_integer());
  ASSERT_TRUE(data_json_["online"].is_boolean());
}

TEST_F(MeasurementNetworkTest, InvalidPingAddressDisablesTheMeasurement)
{
  ms_node_->declare_parameter("network.plugin", std::string("dc_measurements/Network"));
  ms_node_->declare_parameter("network.group_key", std::string("network"));
  ms_node_->declare_parameter("network.topic_output", std::string("/dc/measurement/network"));
  ms_node_->declare_parameter("network.ping_address", std::string("not-an-ip-address"));
  ms_node_->declare_parameter("network.polling_interval", 50);

  startLifecycleNode();

  spinFor(150);

  EXPECT_FALSE(callback_active_);
}

DC_MEASUREMENT_TEST_MAIN()
